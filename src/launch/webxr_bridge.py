#!/usr/bin/env python3
"""
WebXR transport bridge.

Modes:
    - direct: Quest browser connects here and ROS topics are published locally.
    - ingress: Quest browser connects here, packets are optionally published locally,
      and compact JSON packets are forwarded to a remote receiver.
    - remote-receiver: accepts forwarded packets near Isaac Sim and publishes ROS topics.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import ssl
import time

from src.config_loader import default_project_root, load_runtime_config
from src.quest_ingress import QuestPacket, TransportMetricsTracker, format_ros_frame_id

PROJECT_ROOT = default_project_root()


def get_websockets_module():
    try:
        import websockets
    except ImportError:
        print("Please install websockets: pip install websockets")
        raise SystemExit(1)
    return websockets


def import_ros_interfaces():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    return rclpy, Node, PoseStamped, Joy


def _teleop_qos(queue_size: int = 1):
    """Sensor-data QoS for live teleop topics: keep only the newest sample so a
    slow subscriber can never back-pressure the publisher, and BEST_EFFORT drops
    stale samples instead of head-of-line blocking. Falls back to a plain depth
    if rclpy.qos is unavailable."""
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(queue_size)),
        )
    except ImportError:
        return max(1, int(queue_size))


def build_ros_bridge_node_class(Node, PoseStamped, Joy):
    class WebXRROSBridge(Node):
        def __init__(self):
            super().__init__("webxr_ros_bridge")
            # Use low-latency (depth-1, BEST_EFFORT) QoS: the newest pose/input
            # sample is the only one that matters for live teleop, so we drop
            # stale samples instead of queuing them.
            pose_qos = _teleop_qos(1)
            self.pub_left_pose = self.create_publisher(PoseStamped, "/quest/left_hand/pose", pose_qos)
            self.pub_right_pose = self.create_publisher(PoseStamped, "/quest/right_hand/pose", pose_qos)
            self.pub_left_input = self.create_publisher(Joy, "/quest/left_hand/inputs", pose_qos)
            self.pub_right_input = self.create_publisher(Joy, "/quest/right_hand/inputs", pose_qos)

            self.get_logger().info("WebXR ROS Bridge initialized")
            self.get_logger().info("Publishing to: /quest/left_hand/pose, /quest/right_hand/pose")
            self.get_logger().info("Publishing to: /quest/left_hand/inputs, /quest/right_hand/inputs")

        def publish_packet(self, packet: QuestPacket) -> None:
            timestamp = self.get_clock().now().to_msg()
            frame_id = format_ros_frame_id(
                packet,
                ros_publish_epoch_ms=time.time() * 1000.0,
            )

            for hand in ("left", "right"):
                controller = packet.controllers.get(hand)
                if controller is None:
                    continue

                if controller.position_xyz is not None and controller.orientation_xyzw is not None:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = timestamp
                    pose_msg.header.frame_id = frame_id
                    pose_msg.pose.position.x = float(controller.position_xyz[0])
                    pose_msg.pose.position.y = float(controller.position_xyz[1])
                    pose_msg.pose.position.z = float(controller.position_xyz[2])
                    pose_msg.pose.orientation.x = float(controller.orientation_xyzw[0])
                    pose_msg.pose.orientation.y = float(controller.orientation_xyzw[1])
                    pose_msg.pose.orientation.z = float(controller.orientation_xyzw[2])
                    pose_msg.pose.orientation.w = float(controller.orientation_xyzw[3])
                    if hand == "left":
                        self.pub_left_pose.publish(pose_msg)
                    else:
                        self.pub_right_pose.publish(pose_msg)

                joy_msg = Joy()
                joy_msg.header.stamp = timestamp
                joy_msg.header.frame_id = frame_id
                joy_msg.axes = [
                    float(controller.trigger),
                    float(controller.squeeze),
                    float(controller.thumbstick_x),
                    float(controller.thumbstick_y),
                ]
                joy_msg.buttons = [
                    int(controller.button_a_x),
                    int(controller.button_b_y),
                    0,
                    int(controller.thumbstick_click),
                ]
                if hand == "left":
                    self.pub_left_input.publish(joy_msg)
                else:
                    self.pub_right_input.publish(joy_msg)

    return WebXRROSBridge


class UnifiedLogger:
    def __init__(self, ros_node=None):
        self._ros_node = ros_node
        if ros_node is None:
            logging.basicConfig(level=logging.INFO, format="%(message)s")
            self._logger = logging.getLogger("webxr_ros_bridge")
        else:
            self._logger = None

    def info(self, message: str) -> None:
        if self._ros_node is not None:
            self._ros_node.get_logger().info(message)
        else:
            self._logger.info(message)

    def warn(self, message: str) -> None:
        if self._ros_node is not None:
            self._ros_node.get_logger().warn(message)
        else:
            self._logger.warning(message)

    def error(self, message: str) -> None:
        if self._ros_node is not None:
            self._ros_node.get_logger().error(message)
        else:
            self._logger.error(message)


class ForwardingClient:
    def __init__(
        self,
        url: str,
        logger: UnifiedLogger,
        retry_interval_s: float = 2.0,
        queue_size: int = 64,
        ssl_context=None,
    ):
        self.url = url
        self.logger = logger
        self.retry_interval_s = max(0.25, float(retry_interval_s))
        self.ssl_context = ssl_context
        self._queue: asyncio.Queue[str | None] = asyncio.Queue(maxsize=max(1, int(queue_size)))
        self._task: asyncio.Task | None = None
        self._stop_requested = False
        self._queue_drops = 0

    @property
    def queue_drops(self) -> int:
        return self._queue_drops

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._run(), name="quest-forwarder")

    def submit(self, packet: QuestPacket) -> None:
        payload = json.dumps(packet.to_mapping(), separators=(",", ":"))
        if self._queue.full():
            try:
                self._queue.get_nowait()
                self._queue_drops += 1
            except asyncio.QueueEmpty:
                pass
        try:
            self._queue.put_nowait(payload)
        except asyncio.QueueFull:
            self._queue_drops += 1

    async def close(self) -> None:
        self._stop_requested = True
        try:
            self._queue.put_nowait(None)
        except asyncio.QueueFull:
            pass
        if self._task is not None:
            await self._task

    async def _run(self) -> None:
        websockets = get_websockets_module()
        while not self._stop_requested:
            try:
                self.logger.info(f"[Forward] Connecting to remote receiver: {self.url}")
                async with websockets.connect(self.url, ssl=self.ssl_context) as websocket:
                    self.logger.info(f"[Forward] Connected to remote receiver: {self.url}")
                    while not self._stop_requested:
                        payload = await self._queue.get()
                        if payload is None:
                            break
                        await websocket.send(payload)
            except Exception as exc:
                if self._stop_requested:
                    break
                self.logger.warn(
                    f"[Forward] Remote receiver unavailable ({exc}); retrying in "
                    f"{self.retry_interval_s:.1f}s"
                )
                await asyncio.sleep(self.retry_interval_s)


class BridgeRuntime:
    def __init__(
        self,
        mode: str,
        logger: UnifiedLogger,
        ros_node=None,
        forwarder: ForwardingClient | None = None,
        metrics_log_period_s: float = 3.0,
    ):
        self.mode = mode
        self.logger = logger
        self.ros_node = ros_node
        self.forwarder = forwarder
        self.metrics = TransportMetricsTracker(label=mode, log_period_s=metrics_log_period_s)

    def on_client_connected(self, client_addr) -> None:
        self.metrics.reset()
        self.logger.info(f"Client connected: {client_addr}")

    def on_client_disconnected(self, client_addr) -> None:
        self.logger.info(f"Client disconnected: {client_addr}")

    async def handle_packet(self, packet: QuestPacket, payload_size_bytes: int) -> None:
        receive_monotonic_s = time.monotonic()
        receive_epoch_ms = time.time() * 1000.0

        if self.mode == "ingress":
            packet.transport.ingress_receive_epoch_ms = receive_epoch_ms
        elif self.mode == "remote-receiver":
            packet.transport.remote_receive_epoch_ms = receive_epoch_ms

        self.metrics.record(
            packet,
            payload_size_bytes=payload_size_bytes,
            receive_monotonic_s=receive_monotonic_s,
            receive_epoch_ms=receive_epoch_ms,
        )
        snapshot = self.metrics.maybe_snapshot(receive_monotonic_s)
        if snapshot is not None:
            log_line = snapshot.format_log_line()
            if self.forwarder is not None and self.forwarder.queue_drops > 0:
                log_line += f" forward_queue_drop={self.forwarder.queue_drops}"
            self.logger.info(log_line)

        if self.ros_node is not None:
            self.ros_node.publish_packet(packet)

        if self.forwarder is not None:
            self.forwarder.submit(packet)


class WebSocketServer:
    def __init__(self, runtime: BridgeRuntime, host="0.0.0.0", port=9999, ssl_context=None):
        self.runtime = runtime
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self.clients = set()

    async def handler(self, websocket, path=None):
        websockets = get_websockets_module()
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        self.runtime.on_client_connected(client_addr)

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    packet = QuestPacket.from_mapping(data)
                    await self.runtime.handle_packet(
                        packet,
                        payload_size_bytes=len(message.encode("utf-8")),
                    )
                except json.JSONDecodeError as exc:
                    self.runtime.logger.warn(f"Invalid JSON: {exc}")
                except ValueError as exc:
                    self.runtime.logger.warn(f"Invalid WebXR packet: {exc}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            self.runtime.on_client_disconnected(client_addr)

    async def start(self):
        websockets = get_websockets_module()
        protocol = "wss" if self.ssl_context else "ws"
        self.runtime.logger.info(
            f"Starting WebSocket server in {self.runtime.mode} mode on "
            f"{protocol}://{self.host}:{self.port}"
        )

        if self.runtime.mode in ("direct", "ingress"):
            import socket

            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.connect(("8.8.8.8", 80))
                local_ip = sock.getsockname()[0]
                sock.close()
                self.runtime.logger.info(f"Enter this IP in Quest browser: {local_ip}")
            except Exception:
                pass

        async with websockets.serve(self.handler, self.host, self.port, ssl=self.ssl_context):
            await asyncio.Future()


async def ros_spin(rclpy, node):
    # Non-blocking pump: drain ready callbacks without waiting, then yield to the
    # event loop with sleep(0) so the WebSocket handler stays maximally responsive.
    # The old timeout_sec=0.01 + sleep(0.01) introduced up to ~20 ms of latency
    # and bursty publish jitter under load.
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0)


def build_server_ssl_context(cert_file: str | None, key_file: str | None, logger: UnifiedLogger):
    if not cert_file or not key_file:
        return None
    if not os.path.exists(cert_file) or not os.path.exists(key_file):
        logger.error(f"Cert/Key file not found: {cert_file}, {key_file}")
        raise FileNotFoundError(cert_file, key_file)

    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert_file, key_file)
    logger.info("[Transport] SSL enabled for incoming WebSocket server")
    return context


def build_forward_ssl_context(forward_url: str | None, forward_insecure: bool):
    if not forward_url or not forward_url.startswith("wss://"):
        return None
    if not forward_insecure:
        return ssl.create_default_context()

    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    return context


def resolve_publish_ros(args) -> bool:
    if args.publish_ros and args.no_publish_ros:
        raise ValueError("Choose only one of --publish-ros or --no-publish-ros")
    if args.publish_ros:
        return True
    if args.no_publish_ros:
        return False
    return args.mode != "ingress"


async def run_bridge(args) -> None:
    publish_ros = resolve_publish_ros(args)

    rclpy = None
    ros_node = None
    logger = UnifiedLogger()
    forwarder = None
    ros_started = False

    try:
        if publish_ros:
            rclpy, Node, PoseStamped, Joy = import_ros_interfaces()
            WebXRROSBridge = build_ros_bridge_node_class(Node, PoseStamped, Joy)
            rclpy.init()
            ros_started = True
            ros_node = WebXRROSBridge()
            logger = UnifiedLogger(ros_node)

        server_ssl_context = build_server_ssl_context(args.cert, args.key, logger)
        forward_ssl_context = build_forward_ssl_context(args.forward_url, args.forward_insecure)

        if args.forward_url:
            forwarder = ForwardingClient(
                url=args.forward_url,
                logger=logger,
                retry_interval_s=args.forward_retry_interval_s,
                queue_size=args.forward_queue_size,
                ssl_context=forward_ssl_context,
            )
            forwarder.start()

        runtime = BridgeRuntime(
            mode=args.mode,
            logger=logger,
            ros_node=ros_node,
            forwarder=forwarder,
            metrics_log_period_s=args.metrics_log_period_s,
        )
        ws_server = WebSocketServer(runtime, args.host, args.port, server_ssl_context)

        tasks = [asyncio.create_task(ws_server.start(), name="quest-websocket-server")]
        if publish_ros and rclpy is not None and ros_node is not None:
            tasks.append(asyncio.create_task(ros_spin(rclpy, ros_node), name="quest-ros-spin"))

        await asyncio.gather(*tasks)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if forwarder is not None:
            await forwarder.close()
        if ros_node is not None:
            ros_node.destroy_node()
        if ros_started and rclpy is not None:
            rclpy.shutdown()


def build_arg_parser(argv: list[str] | None = None) -> argparse.ArgumentParser:
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument("--config", dest="config_path")
    bootstrap_args, _ = bootstrap.parse_known_args(argv)
    runtime_config = load_runtime_config(
        config_path=bootstrap_args.config_path,
        project_root=PROJECT_ROOT,
    )
    server_config = runtime_config.main.get("server", {})
    transport_config = runtime_config.main.get("transport", {})
    cert_config = runtime_config.main.get("paths", {}).get("certs", {})

    parser = argparse.ArgumentParser(
        description="WebXR transport bridge",
        parents=[bootstrap],
    )
    parser.add_argument(
        "--mode",
        default="direct",
        choices=("direct", "ingress", "remote-receiver"),
        help="Bridge mode",
    )
    parser.add_argument("--host", default=server_config.get("host", "0.0.0.0"), help="Host to bind to")
    parser.add_argument(
        "--port",
        type=int,
        default=int(server_config.get("websocket_port", 9999)),
        help="WebSocket port",
    )
    parser.add_argument(
        "--cert",
        default=cert_config.get("cert"),
        help="Path to SSL certificate (cert.pem)",
    )
    parser.add_argument(
        "--key",
        default=cert_config.get("key"),
        help="Path to SSL key (key.pem)",
    )
    parser.add_argument(
        "--forward-url",
        help="Optional ws:// or wss:// receiver URL for forwarding compact controller packets",
    )
    parser.add_argument(
        "--forward-insecure",
        action="store_true",
        help="Disable TLS verification for the remote forwarding connection",
    )
    parser.add_argument(
        "--forward-queue-size",
        type=int,
        default=int(transport_config.get("forward_queue_size", 64)),
        help="Buffered packet count for forwarding mode",
    )
    parser.add_argument(
        "--forward-retry-interval-s",
        type=float,
        default=float(transport_config.get("forward_retry_interval_s", 2.0)),
        help="Reconnect delay when the remote forwarding target is unavailable",
    )
    parser.add_argument(
        "--metrics-log-period-s",
        type=float,
        default=float(transport_config.get("log_interval_s", 3.0)),
        help="Transport metrics logging interval",
    )
    parser.add_argument(
        "--publish-ros",
        action="store_true",
        help="Publish ROS topics locally even in ingress mode",
    )
    parser.add_argument(
        "--no-publish-ros",
        action="store_true",
        help="Disable local ROS topic publishing",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser(argv)
    args = parser.parse_args(argv)

    print(
        """
╔══════════════════════════════════════════════════════════════════╗
║                    WebXR Transport Bridge                       ║
╠══════════════════════════════════════════════════════════════════╣
║  direct          Quest -> this machine -> ROS                  ║
║  ingress         Quest -> this machine -> remote receiver      ║
║  remote-receiver local ingress -> this machine -> ROS          ║
╚══════════════════════════════════════════════════════════════════╝
        """
    )

    asyncio.run(run_bridge(args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
