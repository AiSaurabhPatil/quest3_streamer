#!/usr/bin/env python3
"""
WebXR to ROS Bridge

Receives controller data from Quest WebXR page via WebSocket
and publishes to ROS 2 topics.

Usage:
    python webxr_ros_bridge.py [--port 9090] [--host 0.0.0.0]
"""

import asyncio
import json
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

try:
    import websockets
except ImportError:
    print("Please install websockets: pip install websockets")
    exit(1)


class WebXRROSBridge(Node):
    def __init__(self):
        super().__init__('webxr_ros_bridge')
        
        # Publishers (same topics as OpenXR streamer for compatibility)
        self.pub_left_pose = self.create_publisher(PoseStamped, '/quest/left_hand/pose', 10)
        self.pub_right_pose = self.create_publisher(PoseStamped, '/quest/right_hand/pose', 10)
        self.pub_left_input = self.create_publisher(Joy, '/quest/left_hand/inputs', 10)
        self.pub_right_input = self.create_publisher(Joy, '/quest/right_hand/inputs', 10)
        
        self.get_logger().info("WebXR ROS Bridge initialized")
        self.get_logger().info("Publishing to: /quest/left_hand/pose, /quest/right_hand/pose")
        self.get_logger().info("Publishing to: /quest/left_hand/inputs, /quest/right_hand/inputs")
    
    def process_controller_data(self, data):
        """Process incoming WebXR controller data and publish to ROS"""
        timestamp = self.get_clock().now().to_msg()
        
        controllers = data.get('controllers', {})
        
        for hand in ['left', 'right']:
            ctrl = controllers.get(hand)
            if not ctrl:
                continue
            
            # Publish Pose
            if ctrl.get('position') and ctrl.get('orientation'):
                pose_msg = PoseStamped()
                pose_msg.header.stamp = timestamp
                pose_msg.header.frame_id = "quest_world"
                
                pose_msg.pose.position.x = float(ctrl['position']['x'])
                pose_msg.pose.position.y = float(ctrl['position']['y'])
                pose_msg.pose.position.z = float(ctrl['position']['z'])
                
                pose_msg.pose.orientation.x = float(ctrl['orientation']['x'])
                pose_msg.pose.orientation.y = float(ctrl['orientation']['y'])
                pose_msg.pose.orientation.z = float(ctrl['orientation']['z'])
                pose_msg.pose.orientation.w = float(ctrl['orientation']['w'])
                
                if hand == 'left':
                    self.pub_left_pose.publish(pose_msg)
                else:
                    self.pub_right_pose.publish(pose_msg)
            
            # Publish Inputs (Joy message format matching ros_interface.py)
            joy_msg = Joy()
            joy_msg.header.stamp = timestamp
            
            # Axes: [Trigger, Squeeze, StickX, StickY]
            joy_msg.axes = [
                float(ctrl.get('trigger', 0.0)),
                float(ctrl.get('squeeze', 0.0)),
                float(ctrl.get('thumbstick_x', 0.0)),
                float(ctrl.get('thumbstick_y', 0.0))
            ]
            
            # Buttons: [A/X, B/Y, Menu, StickClick]
            joy_msg.buttons = [
                int(ctrl.get('button_a_x', False)),
                int(ctrl.get('button_b_y', False)),
                0,  # Menu button (not easily accessible in WebXR)
                int(ctrl.get('thumbstick_click', False))
            ]
            
            if hand == 'left':
                self.pub_left_input.publish(joy_msg)
            else:
                self.pub_right_input.publish(joy_msg)


class WebSocketServer:
    def __init__(self, ros_node, host='0.0.0.0', port=9090):
        self.ros_node = ros_node
        self.host = host
        self.port = port
        self.clients = set()
    
    async def handler(self, websocket, path=None):
        """Handle incoming WebSocket connections"""
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        self.ros_node.get_logger().info(f"Client connected: {client_addr}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.ros_node.process_controller_data(data)
                except json.JSONDecodeError as e:
                    self.ros_node.get_logger().warn(f"Invalid JSON: {e}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            self.ros_node.get_logger().info(f"Client disconnected: {client_addr}")
    
    async def start(self):
        """Start the WebSocket server"""
        self.ros_node.get_logger().info(f"Starting WebSocket server on ws://{self.host}:{self.port}")
        
        # Get local IP for display
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            self.ros_node.get_logger().info(f"Enter this IP in Quest browser: {local_ip}")
        except:
            pass
        
        async with websockets.serve(self.handler, self.host, self.port):
            await asyncio.Future()  # Run forever


async def ros_spin(node):
    """Async ROS spin"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)


async def main(host, port):
    """Main async entry point"""
    rclpy.init()
    ros_node = WebXRROSBridge()
    ws_server = WebSocketServer(ros_node, host, port)
    
    try:
        await asyncio.gather(
            ws_server.start(),
            ros_spin(ros_node)
        )
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='WebXR to ROS Bridge')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=9090, help='WebSocket port')
    args = parser.parse_args()
    
    print("""
╔═══════════════════════════════════════════════════════════╗
║           WebXR to ROS Bridge                             ║
╠═══════════════════════════════════════════════════════════╣
║  1. Run this script on your PC                            ║
║  2. Open webxr_streamer.html on Quest browser             ║
║  3. Enter your PC's IP address in the config              ║
║  4. Click 'Start VR Session' and put on headset           ║
╚═══════════════════════════════════════════════════════════╝
    """)
    
    asyncio.run(main(args.host, args.port))
