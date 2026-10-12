import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from sceneforge.x2_locomotion_qualification import (
    CANONICAL_MEDIA_RECEIPT_SCHEMA,
    COMBINATION_VIDEO_OVERLAY_SCHEMA,
    EPISODE_EVIDENCE_SCHEMA_V2,
    SUITE_SCHEMA_V2,
    build_suite,
)
from tests.test_x2_locomotion_qualification import read, rewrite, write_episode


def digest(label: str) -> str:
    return hashlib.sha256(label.encode("utf-8")).hexdigest()


def make_v2_episodes(root: Path) -> list[Path]:
    episodes = [write_episode(root, index) for index in range(1, 6)]
    for path in episodes:
        payload = read(path)
        evidence = payload["qualification_evidence"]
        evidence["schema"] = EPISODE_EVIDENCE_SCHEMA_V2
        evidence["artifacts"] = [
            row for row in evidence["artifacts"]
            if row["role"] not in {"video", "video_receipt"}
        ]
        rewrite(path, payload)
        for name in ("video.mp4", "video_receipt.json"):
            (path.parent / name).unlink(missing_ok=True)
    return episodes


def make_canonical(
    root: Path,
    suite_id: str,
    episode: Path,
    *,
    review_status: str = "PASSED",
) -> tuple[Path, Path]:
    payload = read(episode)
    evidence = payload["qualification_evidence"]
    state_hash = next(
        row["sha256"] for row in evidence["artifacts"]
        if row["role"] == "state_command_trace"
    )
    video = root / "canonical.mp4"
    video.write_bytes(b"canonical-h264-fixture")
    frame_count = 900
    fields = [
        "scenario_phase_gait",
        "requested_and_effective_velocity",
        "measured_velocity",
        "left_and_right_foot_contact",
        "cpg_phase",
        "suspension",
        "sim_time",
        "global_lifecycle_rtf_estimate",
    ]
    receipt = root / "canonical-media-receipt.json"
    receipt.write_text(json.dumps({
        "schema": CANONICAL_MEDIA_RECEIPT_SCHEMA,
        "suite_id": suite_id,
        "status": "PASSED",
        "canonical_episode": {
            "episode_id": evidence["episode_id"],
            "result_sha256": hashlib.sha256(episode.read_bytes()).hexdigest(),
            "state_command_trace_sha256": state_hash,
        },
        "source_video_receipt_sha256": digest("renderer-receipt"),
        "human_review_attestation_sha256": digest("review-attestation"),
        "authority": {
            "source": "genie_sim_isaac_physx",
            "replay": "none_state_only_renderer",
            "renderer_advances_physics": False,
        },
        "video": {
            "size": video.stat().st_size,
            "sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
            "codec": "h264",
            "fps": 30.0,
            "width": 1280,
            "height": 720,
            "frame_count": frame_count,
        },
        "overlay": {
            "schema": COMBINATION_VIDEO_OVERLAY_SCHEMA,
            "position": "top_right",
            "frame_count": frame_count,
            "fields": fields,
        },
        "camera": {
            "mode": "robot_follow_full_body_profile_v1",
            "offset_xyz_m": [-2.2, -2.2, 1.45],
            "target_xyz_m": [0.15, 0.0, 0.68],
            "left_menu_visible": False,
            "visual_full_body_verification": "PASSED",
        },
        "qa_frames": [
            {"label": "first", "frame_index": 0, "sha256": digest("first")},
            {"label": "middle", "frame_index": 450, "sha256": digest("middle")},
            {"label": "last", "frame_index": 899, "sha256": digest("last")},
        ],
        "human_review": {
            "status": review_status,
            "method": "human_review_first_middle_last_full_body",
            "reviewer": "reviewer@example.test",
            "reviewed_at": "2026-08-20T06:30:00Z",
            "full_robot_visible": True,
            "head_visible": True,
            "hands_visible": True,
            "legs_and_feet_visible": True,
            "left_menu_hidden": True,
            "hud_readable": True,
        },
        "published_command_topics": [],
        "qualification": False,
        "real_robot_commands": False,
    }, indent=2), encoding="utf-8")
    return video, receipt


class X2LocomotionQualificationV2Tests(unittest.TestCase):
    def test_five_media_free_episodes_and_one_canonical_pair_qualify(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = make_v2_episodes(root)
            video, receipt = make_canonical(root, "suite-v2-pass", episodes[0])
            report = build_suite(
                episodes,
                suite_id="suite-v2-pass",
                suite_schema=SUITE_SCHEMA_V2,
                canonical_episode=episodes[0],
                canonical_video=video,
                canonical_video_receipt=receipt,
            )
        self.assertEqual("PASSED", report["gate_status"])
        self.assertTrue(report["qualification"])
        self.assertTrue(report["checks"]["canonical_media_passed"])
        self.assertTrue(all(item["gate_status"] == "PASSED" for item in report["attempts"]))

    def test_missing_canonical_is_incomplete_but_attempts_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            episodes = make_v2_episodes(Path(directory))
            report = build_suite(
                episodes, suite_id="suite-v2-missing", suite_schema=SUITE_SCHEMA_V2
            )
        self.assertEqual("INCOMPLETE", report["gate_status"])
        self.assertTrue(all(item["gate_status"] == "PASSED" for item in report["attempts"]))
        self.assertIn("CANONICAL_MEDIA_MISSING", {item["code"] for item in report["findings"]})

    def test_pending_review_is_incomplete_and_rejected_review_fails(self):
        for status, expected in (("PENDING", "INCOMPLETE"), ("REJECTED", "FAILED")):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                episodes = make_v2_episodes(root)
                video, receipt = make_canonical(root, f"suite-{status}", episodes[0], review_status=status)
                payload = read(receipt)
                payload["camera"]["visual_full_body_verification"] = (
                    "pending_user_review" if status == "PENDING" else "FAILED"
                )
                rewrite(receipt, payload)
                report = build_suite(
                    episodes,
                    suite_id=f"suite-{status}",
                    suite_schema=SUITE_SCHEMA_V2,
                    canonical_episode=episodes[0],
                    canonical_video=video,
                    canonical_video_receipt=receipt,
                )
            self.assertEqual(expected, report["gate_status"])

    def test_wrong_state_hash_video_tamper_and_unsafe_receipt_fail(self):
        mutations = (
            lambda payload, video: payload["canonical_episode"].update(state_command_trace_sha256=digest("other")),
            lambda payload, video: video.write_bytes(b"tampered"),
            lambda payload, video: payload.update(real_robot_commands=True),
        )
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                episodes = make_v2_episodes(root)
                suite_id = f"suite-fail-{index}"
                video, receipt = make_canonical(root, suite_id, episodes[0])
                payload = read(receipt)
                mutate(payload, video)
                rewrite(receipt, payload)
                report = build_suite(
                    episodes, suite_id=suite_id, suite_schema=SUITE_SCHEMA_V2,
                    canonical_episode=episodes[0], canonical_video=video,
                    canonical_video_receipt=receipt,
                )
            self.assertEqual("FAILED", report["gate_status"])
            if index == 2:
                self.assertFalse(report["checks"]["real_robot_commands_false"])

    def test_bad_media_hud_and_camera_fail_without_aggregator_exception(self):
        mutations = (
            lambda payload: payload["video"].update(fps=29.97),
            lambda payload: payload["video"].update(codec="vp9"),
            lambda payload: payload["overlay"].update(fields=[{}]),
            lambda payload: payload["camera"].update(offset_xyz_m=["1", "2", "3"]),
        )
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                episodes = make_v2_episodes(root)
                suite_id = f"suite-metadata-{index}"
                video, receipt = make_canonical(root, suite_id, episodes[0])
                payload = read(receipt)
                mutate(payload)
                rewrite(receipt, payload)
                report = build_suite(
                    episodes, suite_id=suite_id, suite_schema=SUITE_SCHEMA_V2,
                    canonical_episode=episodes[0], canonical_video=video,
                    canonical_video_receipt=receipt,
                )
            self.assertEqual("FAILED", report["gate_status"])

    def test_missing_canonical_real_robot_flag_is_incomplete_and_check_is_false(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = make_v2_episodes(root)
            video, receipt = make_canonical(root, "suite-missing-real", episodes[0])
            payload = read(receipt)
            del payload["real_robot_commands"]
            rewrite(receipt, payload)
            report = build_suite(
                episodes,
                suite_id="suite-missing-real",
                suite_schema=SUITE_SCHEMA_V2,
                canonical_episode=episodes[0],
                canonical_video=video,
                canonical_video_receipt=receipt,
            )
        self.assertEqual("INCOMPLETE", report["gate_status"])
        self.assertFalse(report["checks"]["real_robot_commands_false"])

    def test_v2_rejects_mixed_v1_episode_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = make_v2_episodes(root)
            payload = read(episodes[-1])
            payload["qualification_evidence"]["schema"] = "sceneforge/x2-locomotion-cold-start-episode-evidence/v1"
            rewrite(episodes[-1], payload)
            video, receipt = make_canonical(root, "suite-mixed", episodes[0])
            report = build_suite(
                episodes, suite_id="suite-mixed", suite_schema=SUITE_SCHEMA_V2,
                canonical_episode=episodes[0], canonical_video=video,
                canonical_video_receipt=receipt,
            )
        self.assertEqual("FAILED", report["gate_status"])
        self.assertIn("EVIDENCE_SCHEMA_UNSUPPORTED", {item["code"] for item in report["findings"]})


if __name__ == "__main__":
    unittest.main()
