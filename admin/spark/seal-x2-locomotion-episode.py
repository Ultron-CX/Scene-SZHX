#!/usr/bin/env python3
"""Seal one runner-produced X2 PhysX cold-start episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from sceneforge.x2_locomotion_qualification import load_json
from sceneforge.x2_qualification_sealing import (
    EpisodeSealingError,
    SEALING_RECEIPT_SCHEMA,
    seal_episode,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-root", type=Path, required=True)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument(
        "--output",
        type=Path,
        help=(
            "direct child of episode-root or its reserved derived directory; "
            "defaults to derived/result.json when derived exists, otherwise result.json"
        ),
    )
    args = parser.parse_args()

    try:
        sealing_input = load_json(args.spec)
        receipt = seal_episode(
            episode_root=args.episode_root,
            sealing_input=sealing_input,
            output_path=args.output,
        )
    except EpisodeSealingError as exc:
        print(json.dumps(exc.to_dict(), indent=2, sort_keys=True), file=sys.stderr)
        return 2 if exc.gate_status == "INCOMPLETE" else 3
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        receipt = {
            "schema": SEALING_RECEIPT_SCHEMA,
            "status": "FAILED",
            "sealed": False,
            "qualification": False,
            "real_robot_commands": False,
            "findings": [
                {
                    "severity": "FAILED",
                    "code": "SEALING_INPUT_UNREADABLE",
                    "location": "sealing_input",
                    "message": str(exc),
                }
            ],
        }
        print(json.dumps(receipt, indent=2, sort_keys=True), file=sys.stderr)
        return 3

    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
