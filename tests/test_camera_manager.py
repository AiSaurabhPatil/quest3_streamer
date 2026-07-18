import os
import sys
import unittest


TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.isaac_backend.camera_manager import CameraManager  # noqa: E402


class FakeAnnotator:
    def __init__(self):
        self.detached_targets = []

    def detach(self, targets):
        self.detached_targets.extend(targets)


class SplitExpectingAnnotator:
    def detach(self, targets):
        target = targets[0]
        target.split("/")


class FakeRenderProduct:
    path = "/Render/Product"


class FakeHydraTexture:
    pass


class CameraManagerTests(unittest.TestCase):
    def test_close_detaches_render_product_path_when_available(self):
        manager = CameraManager(stage=None, camera_specs={}, config={"log_errors": False})
        annotator = FakeAnnotator()
        manager._annotators["head"] = annotator
        manager._render_products["head"] = FakeRenderProduct()

        manager.close()

        self.assertEqual(annotator.detached_targets, ["/Render/Product"])
        self.assertEqual(manager.diagnostics.errors, {})

    def test_close_suppresses_known_hydra_texture_detach_mismatch(self):
        manager = CameraManager(stage=None, camera_specs={}, config={"log_errors": False})
        manager._annotators["head"] = SplitExpectingAnnotator()
        manager._render_products["head"] = FakeHydraTexture()

        manager.close()

        self.assertEqual(manager.diagnostics.errors, {})


if __name__ == "__main__":
    unittest.main()
