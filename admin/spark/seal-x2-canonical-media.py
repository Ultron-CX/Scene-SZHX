#!/usr/bin/env python3
"""Seal a reviewed offline replay for X2 qualification-suite/v2."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from sceneforge.x2_canonical_media import (  # noqa: E402
    CanonicalMediaError,
    seal_canonical_media_receipt,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite-id", required=True)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--source-video-receipt", type=Path, required=True)
    parser.add_argument("--review-attestation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        payload = seal_canonical_media_receipt(
            suite_id=args.suite_id,
            episode_result=args.episode,
            video=args.video,
            source_video_receipt=args.source_video_receipt,
            review_attestation=args.review_attestation,
            output=args.output,
        )
    except (CanonicalMediaError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"canonical media sealing failed: {exc}", file=sys.stderr)
        return 2
    print(args.output.resolve())
    return 0 if payload["status"] == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
