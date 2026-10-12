#!/usr/bin/env python3
"""Create post-runtime A/B and pending/sealable X2 episode evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from sceneforge.x2_cold_start_finalization import finalize_combination  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--mujoco-straight", type=Path, required=True)
    parser.add_argument("--mujoco-turn-left", type=Path, required=True)
    parser.add_argument("--mujoco-turn-right", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--episode-id")
    parser.add_argument("--video", type=Path)
    parser.add_argument("--video-receipt", type=Path)
    parser.add_argument(
        "--episode-evidence-version",
        type=int,
        choices=(1, 2),
        default=1,
        help=(
            "v1 requires media on every episode; v2 seals motion evidence "
            "without per-episode media and defers one canonical pair to suite/v2"
        ),
    )
    parser.add_argument(
        "--output-dir-name",
        default="qualification-finalization",
        help=(
            "new finalization directory name; written below run/derived when "
            "the runner-provisioned derivation area exists"
        ),
    )
    args = parser.parse_args()
    receipt = finalize_combination(
        run_dir=args.run_dir,
        mujoco_results={
            "straight": args.mujoco_straight,
            "turn_left": args.mujoco_turn_left,
            "turn_right": args.mujoco_turn_right,
        },
        repo_root=args.repo_root,
        episode_id=args.episode_id,
        video=args.video,
        video_receipt=args.video_receipt,
        episode_evidence_version=args.episode_evidence_version,
        output_dir_name=args.output_dir_name,
    )
    print(json.dumps(receipt, indent=2, sort_keys=True, ensure_ascii=False))
    return 0 if receipt["status"] == "PASSED" else 3 if receipt["status"] == "INCOMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
