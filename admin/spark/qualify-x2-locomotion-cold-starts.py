#!/usr/bin/env python3
"""Aggregate five immutable X2 PhysX cold-start episodes."""

from __future__ import annotations

import argparse
from pathlib import Path

from sceneforge.x2_locomotion_qualification import atomic_write, build_suite
from sceneforge.x2_locomotion_qualification import SUITE_SCHEMA_V1, SUITE_SCHEMA_V2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite-id", required=True)
    parser.add_argument(
        "--episode",
        action="append",
        type=Path,
        default=[],
        help="immutable episode result; pass exactly five times",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--suite-version",
        type=int,
        choices=(1, 2),
        default=1,
        help="v2 requires one suite-level reviewed canonical media pair",
    )
    parser.add_argument(
        "--canonical-episode",
        type=Path,
        help="one of the exact --episode result paths; v2 only",
    )
    parser.add_argument("--canonical-video", type=Path, help="v2 only")
    parser.add_argument("--canonical-video-receipt", type=Path, help="v2 only")
    args = parser.parse_args()
    report = build_suite(
        args.episode,
        suite_id=args.suite_id,
        suite_schema=SUITE_SCHEMA_V2 if args.suite_version == 2 else SUITE_SCHEMA_V1,
        canonical_episode=args.canonical_episode,
        canonical_video=args.canonical_video,
        canonical_video_receipt=args.canonical_video_receipt,
    )
    atomic_write(args.output, report)
    print(args.output.resolve())
    return 0 if report["gate_status"] == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
