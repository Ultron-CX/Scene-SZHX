import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from sceneforge.x2_canonical_media import (
    CanonicalMediaError,
    REQUIRED_REVIEW_CHECKS,
    REVIEW_ATTESTATION_SCHEMA,
    build_full_body_review_attestation,
    build_canonical_media_receipt,
    seal_canonical_media_receipt,
)
from sceneforge.x2_locomotion_qualification import SUITE_SCHEMA_V2, build_suite
from tests.test_x2_locomotion_qualification_v2 import make_v2_episodes


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def make_media(root: Path, episode: Path) -> tuple[Path, Path, Path]:
    payload = json.loads(episode.read_text(encoding="utf-8"))
    state_hash = next(
        row["sha256"]
        for row in payload["qualification_evidence"]["artifacts"]
        if row["role"] == "state_command_trace"
    )
    video = root / "combo.mp4"
    video.write_bytes(b"h264-fixture")
    qa = []
    for label, frame in (("first", 0), ("middle", 450), ("last", 899)):
        path = root / f"qa-{label}.png"
        path.write_bytes(b"\x89PNG\r\n\x1a\n" + label.encode("ascii"))
        qa.append({
            "label": label,
            "frame_index": frame,
            "path": str(path),
            "size": path.stat().st_size,
            "sha256": sha(path),
        })
    source = root / "video-receipt.json"
    write_json(source, {
        "schema": "sceneforge/x2-combination-video-receipt/v1",
        "status": "PASSED",
        "source_state_command_trace_sha256": state_hash,
        "source_physics_authority": "genie_sim_isaac_physx",
        "replay_physics_authority": "none_state_only_renderer",
        "renderer_advances_physics": False,
        "video": {
            "size": video.stat().st_size,
            "sha256": sha(video),
            "codec": "h264",
            "fps": 30.0,
            "width": 1280,
            "height": 720,
        },
        "overlay": {
            "schema": "sceneforge/x2-combination-video-overlay/v1",
            "position": "top_right",
            "frame_count": 900,
            "fields": [
                "scenario_phase_gait",
                "requested_and_effective_velocity",
                "measured_velocity",
                "left_and_right_foot_contact",
                "cpg_phase",
                "suspension",
                "sim_time",
                "global_lifecycle_rtf_estimate",
            ],
        },
        "camera": {
            "mode": "robot_follow_full_body_profile_v1",
            "offset_xyz_m": [-2.2, -2.2, 1.45],
            "target_xyz_m": [0.15, 0.0, 0.68],
            "left_menu_visible": False,
            "visual_full_body_verification": "pending_user_review",
        },
        "qa_frames": qa,
        "qualification": False,
        "real_robot_commands": False,
    })
    attestation = root / "full-body-review.json"
    write_json(attestation, {
        "schema": REVIEW_ATTESTATION_SCHEMA,
        "status": "PASSED",
        "video_sha256": sha(video),
        "source_video_receipt_sha256": sha(source),
        "qa_frame_sha256": {item["label"]: item["sha256"] for item in qa},
        "reviewer": "reviewer@example.test",
        "reviewed_at": "2026-08-20T07:00:00Z",
        "checks": {
            "full_robot_visible": True,
            "head_visible": True,
            "hands_visible": True,
            "legs_and_feet_visible": True,
            "left_menu_hidden": True,
            "hud_readable": True,
        },
        "qualification": False,
        "real_robot_commands": False,
    })
    return video, source, attestation


class X2CanonicalMediaTests(unittest.TestCase):
    def test_operational_clis_expose_review_and_canonical_sealing(self):
        root = Path(__file__).parents[1]
        attester = (root / "admin/spark/attest-x2-full-body-review.py").read_text(encoding="utf-8")
        sealer = (root / "admin/spark/seal-x2-canonical-media.py").read_text(encoding="utf-8")
        qualifier = (root / "admin/spark/qualify-x2-locomotion-cold-starts.py").read_text(encoding="utf-8")
        self.assertIn("--confirm-", attester)
        self.assertIn("--review-attestation", sealer)
        self.assertIn("--suite-version", qualifier)
        self.assertIn("--canonical-episode", qualifier)
        self.assertIn("--canonical-video-receipt", qualifier)
        for source in (attester, sealer, qualifier):
            self.assertNotIn("203.0.113.", source)
            self.assertNotIn("/cmd_vel", source)

    def test_review_attestation_recomputes_video_and_qa_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = make_v2_episodes(root)
            video, source, _ = make_media(root, episodes[0])
            payload = build_full_body_review_attestation(
                video=video,
                source_video_receipt=source,
                reviewer="reviewer@example.test",
                reviewed_at="2026-08-20T07:00:00Z",
                checks={name: True for name in REQUIRED_REVIEW_CHECKS},
            )
            expected_video_hash = sha(video)
        self.assertEqual("PASSED", payload["status"])
        self.assertEqual(expected_video_hash, payload["video_sha256"])
        self.assertEqual({"first", "middle", "last"}, set(payload["qa_frame_sha256"]))

    def test_seals_reviewed_receipt_and_suite_accepts_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = make_v2_episodes(root)
            video, source, review = make_media(root, episodes[0])
            output = root / "canonical-receipt.json"
            payload = seal_canonical_media_receipt(
                suite_id="canonical-suite",
                episode_result=episodes[0],
                video=video,
                source_video_receipt=source,
                review_attestation=review,
                output=output,
            )
            self.assertEqual("PASSED", payload["status"])
            self.assertEqual(sha(source), payload["source_video_receipt_sha256"])
            self.assertEqual(sha(review), payload["human_review_attestation_sha256"])
            report = build_suite(
                episodes,
                suite_id="canonical-suite",
                suite_schema=SUITE_SCHEMA_V2,
                canonical_episode=episodes[0],
                canonical_video=video,
                canonical_video_receipt=output,
            )
        self.assertEqual("PASSED", report["gate_status"])

    def test_source_state_qa_and_review_must_match(self):
        mutations = (
            lambda source, review, qa: source["qa_frames"][0].update(sha256="0" * 64),
            lambda source, review, qa: source.update(source_state_command_trace_sha256="0" * 64),
            lambda source, review, qa: review.update(status="PENDING"),
        )
        for index, mutation in enumerate(mutations):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                episodes = make_v2_episodes(root)
                video, source_path, review_path = make_media(root, episodes[0])
                source = json.loads(source_path.read_text())
                review = json.loads(review_path.read_text())
                mutation(source, review, None)
                write_json(source_path, source)
                if index != 2:
                    review["source_video_receipt_sha256"] = sha(source_path)
                write_json(review_path, review)
                with self.assertRaises(CanonicalMediaError):
                    build_canonical_media_receipt(
                        suite_id="invalid-media",
                        episode_result=episodes[0],
                        video=video,
                        source_video_receipt=source_path,
                        review_attestation=review_path,
                    )

    def test_output_is_no_clobber(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = make_v2_episodes(root)
            video, source, review = make_media(root, episodes[0])
            output = root / "canonical-receipt.json"
            seal_canonical_media_receipt(
                suite_id="no-clobber", episode_result=episodes[0], video=video,
                source_video_receipt=source, review_attestation=review, output=output,
            )
            with self.assertRaises(FileExistsError):
                seal_canonical_media_receipt(
                    suite_id="no-clobber", episode_result=episodes[0], video=video,
                    source_video_receipt=source, review_attestation=review, output=output,
                )


if __name__ == "__main__":
    unittest.main()
