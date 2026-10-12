import json
import unittest

from sceneforge.x2_control import X2_CPGWALK_JOINT_NAMES
from sceneforge.x2_session import X2PhysicsState
from sceneforge.x2_sim_time_differential import (
    PROBE_SCHEMA,
    STATE_SEQUENCE_SCHEMA,
    extract_state_sequence_from_physx_trace,
    load_state_sequence_payload,
    physics_state_payload,
    run_sim_time_differential_probe,
)


class FakeWallClock:
    def __init__(self):
        self.now = 100.0

    def read(self):
        return self.now

    def sleep(self, seconds):
        self.now += float(seconds)


class DeterministicPolicy:
    def __call__(self, values):
        # Depend on replayed observations, but never on wall time.
        base = 0.01 * float(values[-65 + 6])
        return tuple(base + 0.001 * index for index in range(17))


class WallClockBoundPolicy:
    def __init__(self, clock):
        self.clock = clock

    def __call__(self, _values):
        return (0.001 * self.clock.read(),) * 17


def replay_state(index):
    sim_time = 1.0 + 0.02 * index
    return X2PhysicsState(
        source_sequence=100 + index,
        sim_time=sim_time,
        command_stamp_sim_time=sim_time,
        velocity_command=(0.3, 0.0, 0.0),
        joint_position=(-0.248, 0.0, 0.0, 0.5303, -0.2823, 0.0) * 2
        + (0.0,) * 5,
        joint_velocity=(0.01 * index,) * 17,
        base_angular_velocity=(0.0, 0.0, 0.001 * index),
        observation_gravity=(0.0, 0.0, -1.0),
        base_linear_velocity=(0.2, 0.0, 0.0),
        roll=0.001 * index,
        pitch=-0.001 * index,
    )


class SimTimeDifferentialProbeTests(unittest.TestCase):
    def test_extracts_actual_physx_state_at_requested_sim_rate(self):
        joint_names = list(X2_CPGWALK_JOINT_NAMES) + [
            f"extra_{index}" for index in range(14)
        ]
        physics = []
        bases = []
        for index in range(701):
            sim_time = 10.0 + 0.002 * index
            physics.append({
                "physics_sequence": 1000 + index,
                "sim_time": sim_time,
                "phase": "GAIT1_FORWARD",
                "position": [0.001 * index] * 31,
                "velocity": [0.01] * 31,
            })
            bases.append({
                "physics_sequence": 1000 + index,
                "sim_time": sim_time,
                "phase": "GAIT1_FORWARD",
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                "effective_velocity_command": [0.3, 0.0, 0.0],
                "base_angular_velocity_body_rps": [0.0, 0.0, 0.0],
                "base_linear_velocity_body_mps": [0.2, 0.0, 0.0],
            })
        payload = extract_state_sequence_from_physx_trace({
            "schema": "sceneforge/x2-physx-locomotion-samples/v2",
            "physics_authority": "genie_sim_isaac_physx",
            "joint_names": joint_names,
            "physics_samples": physics,
            "base_samples": bases,
            "real_robot_commands": False,
        }, start_offset_s=0.1, duration_s=1.0, sample_period_s=0.02)
        self.assertEqual(payload["schema"], STATE_SEQUENCE_SCHEMA)
        self.assertEqual(len(payload["states"]), 51)
        self.assertEqual(payload["states"][0]["source_sequence"], 1050)
        self.assertAlmostEqual(payload["states"][1]["sim_time"], 10.12)
        self.assertEqual(len(payload["states"][0]["joint_position"]), 17)
        self.assertEqual(
            load_state_sequence_payload(payload)[-1].source_sequence,
            1550,
        )

    def test_physx_state_extraction_rejects_unsafe_or_misaligned_trace(self):
        base = {
            "schema": "sceneforge/x2-physx-locomotion-samples/v2",
            "physics_authority": "genie_sim_isaac_physx",
            "joint_names": list(X2_CPGWALK_JOINT_NAMES),
            "physics_samples": [],
            "base_samples": [],
            "real_robot_commands": True,
        }
        with self.assertRaisesRegex(ValueError, "real_robot_commands=false"):
            extract_state_sequence_from_physx_trace(base)

    def test_wall_delay_does_not_change_next_command_or_complete_digest(self):
        clock = FakeWallClock()
        report = run_sim_time_differential_probe(
            tuple(replay_state(index) for index in range(8)),
            DeterministicPolicy,
            pause_before_index=4,
            wall_sleep_seconds=0.25,
            sleep_fn=clock.sleep,
            clock_fn=clock.read,
            policy_identity={"kind": "deterministic-test-policy"},
        )
        self.assertEqual(report["schema"], PROBE_SCHEMA)
        self.assertEqual(report["status"], "PASSED")
        self.assertFalse(report["qualification"])
        self.assertFalse(report["real_robot_commands"])
        self.assertFalse(report["serialization"]["allow_nan"])
        self.assertEqual(report["pause"]["observed_wall_pause_s"], 0.25)
        self.assertTrue(report["checks"]["next_command_exactly_equal"])
        self.assertTrue(
            report["checks"]["next_complete_state_digest_exactly_equal"]
        )
        self.assertTrue(
            report["checks"]["full_complete_state_digest_stream_exactly_equal"]
        )
        json.dumps(report, allow_nan=False)

    def test_probe_detects_a_policy_that_uses_wall_time(self):
        clock = FakeWallClock()
        report = run_sim_time_differential_probe(
            tuple(replay_state(index) for index in range(6)),
            lambda: WallClockBoundPolicy(clock),
            pause_before_index=3,
            wall_sleep_seconds=0.5,
            sleep_fn=clock.sleep,
            clock_fn=clock.read,
        )
        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(report["checks"]["next_command_exactly_equal"])
        self.assertFalse(
            report["checks"]["next_complete_state_digest_exactly_equal"]
        )
        self.assertFalse(report["real_robot_commands"])

    def test_replay_payload_requires_explicit_offline_safety_flag(self):
        states = [physics_state_payload(replay_state(index)) for index in range(2)]
        with self.assertRaisesRegex(ValueError, "real_robot_commands=false"):
            load_state_sequence_payload(
                {"schema": STATE_SEQUENCE_SCHEMA, "states": states}
            )
        with self.assertRaisesRegex(ValueError, "real_robot_commands=false"):
            load_state_sequence_payload(
                {
                    "schema": STATE_SEQUENCE_SCHEMA,
                    "states": states,
                    "real_robot_commands": True,
                }
            )
        loaded = load_state_sequence_payload(
            {
                "schema": STATE_SEQUENCE_SCHEMA,
                "states": states,
                "real_robot_commands": False,
            }
        )
        self.assertEqual(loaded, tuple(replay_state(index) for index in range(2)))

    def test_pause_must_follow_a_nonempty_prefix(self):
        states = tuple(replay_state(index) for index in range(2))
        with self.assertRaisesRegex(ValueError, "non-empty prefix"):
            run_sim_time_differential_probe(
                states,
                DeterministicPolicy,
                pause_before_index=0,
                wall_sleep_seconds=0.01,
            )


if __name__ == "__main__":
    unittest.main()
