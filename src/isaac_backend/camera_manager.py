from __future__ import annotations

from dataclasses import dataclass, field
import queue
import threading
import time

import numpy as np


@dataclass
class CameraManagerConfig:
    enabled: bool = True
    resolution: tuple[int, int] = (224, 224)
    publish_interval_frames: int = 2
    queue_size: int = 3
    log_errors: bool = True
    error_log_interval_s: float = 5.0

    @classmethod
    def from_mapping(cls, values: dict | None):
        values = values or {}
        resolution = values.get("resolution", (224, 224))
        return cls(
            enabled=bool(values.get("enabled", True)),
            resolution=(int(resolution[0]), int(resolution[1])),
            publish_interval_frames=max(1, int(values.get("publish_interval_frames", 2))),
            queue_size=max(1, int(values.get("queue_size", 3))),
            log_errors=bool(values.get("log_errors", True)),
            error_log_interval_s=float(values.get("error_log_interval_s", 5.0)),
        )


@dataclass
class CameraManagerDiagnostics:
    captured_frames: int = 0
    published_frames: int = 0
    dropped_frames: int = 0
    errors: dict[str, int] = field(default_factory=dict)


class CameraManager:
    def __init__(
        self,
        stage,
        camera_specs: dict,
        camera_publishers=None,
        viewport_cameras: list[tuple[str, str]] | None = None,
        config: CameraManagerConfig | dict | None = None,
    ):
        self.stage = stage
        self.camera_specs = dict(camera_specs or {})
        self.camera_publishers = camera_publishers
        self.viewport_cameras = list(viewport_cameras or [])
        self.config = (
            config if isinstance(config, CameraManagerConfig) else CameraManagerConfig.from_mapping(config)
        )
        self.diagnostics = CameraManagerDiagnostics()
        self._current_viewport_index = 0
        self._frame_counter = 0
        self._camera_queue = queue.Queue(maxsize=self.config.queue_size)
        self._thread_running = False
        self._publisher_thread = None
        self._annotators: dict[str, object] = {}
        self._render_products: dict[str, object] = {}
        self._last_error_log_s: dict[str, float] = {}

    @property
    def viewport_camera_names(self) -> list[str]:
        return [name for name, _ in self.viewport_cameras]

    def start(self):
        if not self.config.enabled or not self.camera_specs:
            return self

        import omni.replicator.core as rep

        for camera_name, camera_spec in self.camera_specs.items():
            camera_prim = self.stage.GetPrimAtPath(camera_spec.prim_path)
            if not camera_prim.IsValid():
                self._record_error(
                    "missing_camera_prim",
                    f"{camera_name}: {camera_spec.prim_path}",
                )
                continue

            render_product = rep.create.render_product(
                camera_spec.prim_path,
                self.config.resolution,
            )
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([render_product])
            self._render_products[camera_name] = render_product
            self._annotators[camera_name] = annotator

        if not self._annotators:
            return self

        if self.camera_publishers is not None:
            self._thread_running = True
            self._publisher_thread = threading.Thread(
                target=self._publish_loop,
                name="camera-publish-thread",
                daemon=True,
            )
            self._publisher_thread.start()
        return self

    def update(self, stamp=None, return_frames: bool = False, rendered: bool = True):
        if not self._annotators:
            return {} if return_frames else None

        # Only capture on iterations that actually rendered: a non-render physics
        # step produces no fresh pixels, so annotator.get_data() would return
        # stale data (or block on the GPU). This keeps capture cost proportional
        # to the render rate rather than the (faster) control rate once physics
        # and rendering are decoupled. When rendering every step (legacy
        # behavior) `rendered` is always True and this is a no-op gate.
        if not rendered:
            return {} if return_frames else None

        self._frame_counter += 1
        if self._frame_counter % self.config.publish_interval_frames != 0:
            return {} if return_frames else None

        captured_frames: dict[str, np.ndarray] = {}

        for camera_name, annotator in self._annotators.items():
            try:
                data = annotator.get_data()
                if data is None:
                    continue

                image_rgb = self._coerce_rgb(camera_name, data)
                if image_rgb is None:
                    continue

                captured_frames[camera_name] = image_rgb
                self.diagnostics.captured_frames += 1
                if self.camera_publishers is not None:
                    try:
                        self._camera_queue.put_nowait((camera_name, image_rgb, stamp))
                    except queue.Full:
                        self.diagnostics.dropped_frames += 1
                        self._record_error("camera_queue_full", camera_name)
            except Exception as exc:
                self._record_error("capture_error", f"{camera_name}: {exc}")

        if return_frames:
            return captured_frames
        return None

    def switch_viewport_camera_next(self):
        if not self.viewport_cameras:
            return None

        import omni.kit.viewport.utility

        self._current_viewport_index = (self._current_viewport_index + 1) % len(self.viewport_cameras)
        camera_name, camera_path = self.viewport_cameras[self._current_viewport_index]
        viewport = omni.kit.viewport.utility.get_active_viewport()
        if viewport is None:
            self._record_error("viewport_unavailable", camera_name)
            return None

        try:
            viewport.camera_path = camera_path
            return camera_name, camera_path
        except Exception as exc:
            self._record_error("viewport_switch_error", f"{camera_name}: {exc}")
            return None

    def close(self):
        self._thread_running = False
        if self._publisher_thread is not None:
            self._publisher_thread.join(timeout=1.0)
            self._publisher_thread = None

        for camera_name, annotator in self._annotators.items():
            render_product = self._render_products.get(camera_name)
            if render_product is None:
                continue
            try:
                annotator.detach([_render_product_detach_target(render_product)])
            except Exception as exc:
                if _is_known_replicator_detach_mismatch(exc):
                    continue
                self._record_error("camera_detach_error", f"{camera_name}: {exc}")

        self._annotators.clear()
        self._render_products.clear()

    def _publish_loop(self):
        while self._thread_running:
            try:
                camera_name, image_rgb, stamp = self._camera_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self.camera_publishers.publish_rgb(camera_name, image_rgb, stamp=stamp)
                self.diagnostics.published_frames += 1
            except Exception as exc:
                self._record_error("publish_error", f"{camera_name}: {exc}")
            finally:
                self._camera_queue.task_done()

    def _coerce_rgb(self, camera_name: str, data) -> np.ndarray | None:
        image_array = np.asarray(data)
        if image_array.ndim != 3 or image_array.shape[2] not in (3, 4):
            self._record_error(
                "unexpected_image_shape",
                f"{camera_name}: {image_array.shape}",
            )
            return None
        if image_array.shape[2] == 4:
            image_array = image_array[:, :, :3]
        return np.asarray(image_array, dtype=np.uint8, order="C").copy()

    def _record_error(self, key: str, detail: str):
        self.diagnostics.errors[key] = self.diagnostics.errors.get(key, 0) + 1
        if not self.config.log_errors:
            return

        now_s = time.monotonic()
        last_log_s = self._last_error_log_s.get(key, 0.0)
        if now_s - last_log_s < self.config.error_log_interval_s:
            return

        self._last_error_log_s[key] = now_s
        print(f"[Camera] {key}: {detail}")


def _render_product_detach_target(render_product):
    for attribute_name in ("path", "render_product_path"):
        value = getattr(render_product, attribute_name, None)
        if value is None:
            continue
        if callable(value):
            value = value()
        if value:
            return str(value)

    for method_name in ("get_path", "get_render_product_path"):
        method = getattr(render_product, method_name, None)
        if method is None:
            continue
        value = method()
        if value:
            return str(value)

    return render_product


def _is_known_replicator_detach_mismatch(exc: Exception) -> bool:
    return (
        isinstance(exc, AttributeError)
        and "HydraTexture" in str(exc)
        and "split" in str(exc)
    )
