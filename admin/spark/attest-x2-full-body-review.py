#!/usr/bin/env python3
"""Create an immutable human review attestation for canonical X2 media."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from sceneforge.x2_canonical_media import (  # noqa: E402
    CanonicalMediaError,
    REQUIRED_REVIEW_CHECKS,
    seal_full_body_review_attestation,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--source-video-receipt", type=Path, required=True)
    parser.add_argument("--reviewer", required=True)
    parser.add_argument("--reviewed-at", required=True, help="ISO-8601 UTC")
    parser.add_argument("--output", type=Path, required=True)
    for name in REQUIRED_REVIEW_CHECKS:
        parser.add_argument(
            "--confirm-" + name.replace("_", "-"),
            action="store_true",
            help=f"explicitly attest {name}=true",
        )
    args = parser.parse_args()
    checks = {
        name: bool(getattr(args, "confirm_" + name))
        for name in REQUIRED_REVIEW_CHECKS
    }
    try:
        seal_full_body_review_attestation(
            video=args.video,
            source_video_receipt=args.source_video_receipt,
            reviewer=args.reviewer,
            reviewed_at=args.reviewed_at,
            checks=checks,
            output=args.output,
        )
    except (CanonicalMediaError, OSError, ValueError) as exc:
        print(f"full-body review attestation failed: {exc}", file=sys.stderr)
        return 2
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
