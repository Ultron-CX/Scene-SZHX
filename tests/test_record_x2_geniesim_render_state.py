import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).parents[1] / "admin" / "spark" / "record-x2-geniesim-render-state.py"
RUNNER_PATH = Path(__file__).parents[1] / "admin" / "spark" / "run-x2-geniesim-fixed-controller-gate.sh"
SPEC = importlib.util.spec_from_file_location("record_x2_render_state", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def stamp(seconds):
    sec = int(seconds)
    return SimpleNamespace(sec=sec, nanosec=round((seconds - sec) * 1e9))


def base(seconds):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(seconds)),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=0.8),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )),
    )


def tf(seconds):
    item = SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(seconds), frame_id="/x2"), child_frame_id="/x2/base_link",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.8),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    return SimpleNamespace(transforms=[item])


class RenderStateRecorderTests(unittest.TestCase):
    def test_pairing_uses_simulation_time(self):
        sampler = MODULE.Sampler(30.0, 0.05)
        self.assertIsNone(sampler.update_base(MODULE.base_from_message(base(10.0))))
        sample = sampler.update_transforms(MODULE.transforms_from_message(tf(10.01)))
        self.assertEqual(0, sample["sequence"])
        self.assertEqual("/x2/base_link", sample["transforms"][0]["child_path"])
        self.assertFalse(sample.get("real_robot_commands", False))

    def test_duplicate_child_path_is_rejected(self):
        message = tf(1.0)
        message.transforms.append(message.transforms[0])
        with self.assertRaises(ValueError):
            MODULE.transforms_from_message(message)

    def test_source_has_no_publish_or_physical_transport(self):
        source = MODULE_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("203.0.113.", source)
        self.assertNotIn("ssh", source)

    def test_atomic_receipt_is_not_left_private_to_container_root(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "receipt.json"
            MODULE.atomic_json(path, {"status": "RUNNING"})
            self.assertEqual({"status": "RUNNING"}, json.loads(path.read_text(encoding="utf-8")))
            # Windows may retain write bits for group/other, while Linux honors
            # the exact 0644 chmod.  The cross-platform contract is that every
            # reader can inspect the receipt from outside the root container.
            self.assertEqual(0o444, path.stat().st_mode & 0o444)

    def test_signal_handlers_are_installed_after_rclpy_init(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        init_index = source.index("    rclpy.init()")
        handler_index = source.index("    signal.signal(signal.SIGINT, stop)")
        self.assertLess(init_index, handler_index)

    def test_runner_stops_pid_one_from_the_host(self):
        source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertIn('docker kill --signal=SIGINT "$state_recorder_container"', source)
        function = source.split("stop_state_recorder() {", 1)[1].split("\n}", 1)[0]
        self.assertNotIn('docker exec "$state_recorder_container"', function)


if __name__ == "__main__":
    unittest.main()
