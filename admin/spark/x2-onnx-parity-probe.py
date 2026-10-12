#!/usr/bin/env python3
"""Replay vendor-recorded X2 policy inputs through ONNX Runtime.

This is an offline, read-only gate.  It does not create ROS publishers and it
must pass before an independently scheduled policy is allowed to command the
Genie Sim / PhysX adapter.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
import re
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Sequence


SCHEMA = "sceneforge/x2-onnx-parity-probe/v1"


@dataclass(frozen=True)
class ParityRow:
    frame: int
    profile: str
    inputs: tuple[float, ...]
    expected_outputs: tuple[float, ...]


@dataclass
class DebugSourceStats:
    source_row_count: int = 0
    trailing_partial_rows_skipped: int = 0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def parse_vector(value: str, *, label: str) -> tuple[float, ...]:
    try:
        result = tuple(float(part) for part in value.split())
    except ValueError as exc:
        raise ValueError(f"{label} contains a non-numeric value") from exc
    if not result:
        raise ValueError(f"{label} is empty")
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


def iter_parity_rows(
    frames_path: Path,
    outputs_path: Path,
    *,
    maximum_rows: int | None = None,
) -> Iterator[ParityRow]:
    with frames_path.open("r", encoding="utf-8", newline="") as frame_stream, outputs_path.open(
        "r", encoding="utf-8", newline=""
    ) as output_stream:
        frame_reader = csv.DictReader(frame_stream)
        output_reader = csv.DictReader(output_stream)
        if frame_reader.fieldnames != ["frame", "profile", "frame_values"]:
            raise ValueError(f"unexpected frame CSV columns: {frame_reader.fieldnames}")
        if output_reader.fieldnames != ["frame", "profile", "values"]:
            raise ValueError(f"unexpected output CSV columns: {output_reader.fieldnames}")
        for index, (frame_row, output_row) in enumerate(
            itertools.zip_longest(frame_reader, output_reader)
        ):
            if maximum_rows is not None and index >= maximum_rows:
                break
            if frame_row is None or output_row is None:
                raise ValueError("frame and output CSV row counts differ")
            frame_number = int(frame_row["frame"])
            output_number = int(output_row["frame"])
            if frame_number != output_number:
                raise ValueError(
                    f"frame index mismatch at row {index}: {frame_number} != {output_number}"
                )
            profile = frame_row["profile"]
            if profile != output_row["profile"]:
                raise ValueError(
                    f"profile mismatch at frame {frame_number}: "
                    f"{profile!r} != {output_row['profile']!r}"
                )
            yield ParityRow(
                frame=frame_number,
                profile=profile,
                inputs=parse_vector(
                    frame_row["frame_values"], label=f"frame {frame_number} inputs"
                ),
                expected_outputs=parse_vector(
                    output_row["values"], label=f"frame {frame_number} outputs"
                ),
            )


def indexed_columns(fieldnames: Sequence[str], prefix: str) -> list[str]:
    pattern = re.compile(rf"^{re.escape(prefix)}_(\d+)$")
    indexed = sorted(
        ((int(match.group(1)), name) for name in fieldnames if (match := pattern.match(name))),
        key=lambda item: item[0],
    )
    if not indexed or [item[0] for item in indexed] != list(range(len(indexed))):
        raise ValueError(f"{prefix} columns are missing or non-contiguous")
    return [item[1] for item in indexed]


def iter_debug_parity_rows(
    debug_path: Path,
    *,
    frame_stack: int,
    maximum_rows: int | None = None,
    profile: str = "cpgwalk",
    stats: DebugSourceStats | None = None,
) -> Iterator[ParityRow]:
    """Rebuild oldest-to-newest history from the vendor CPG debug CSV.

    The first observed frame seeds every history slot.  Subsequent frames drop
    the oldest 65-value observation and append the current observation.  This
    matches the vendor parity recorder's independently observed initialization
    and shift convention.
    """
    if frame_stack < 1:
        raise ValueError("frame_stack must be positive")
    with debug_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("debug CSV has no header")
        observation_columns = indexed_columns(reader.fieldnames, "obs")
        action_columns = indexed_columns(reader.fieldnames, "action")
        source_rows = list(reader)
        if stats is not None:
            stats.source_row_count = len(source_rows)
        selected_rows = (
            source_rows if maximum_rows is None else source_rows[:maximum_rows]
        )
        history: list[tuple[float, ...]] = []
        for index, row in enumerate(selected_rows):
            try:
                observation = tuple(float(row[name]) for name in observation_columns)
                action = tuple(float(row[name]) for name in action_columns)
            except (KeyError, TypeError, ValueError) as exc:
                # The vendor process can be terminated while it is writing its
                # final CSV row.  Only that physical last row may be ignored;
                # a malformed middle row remains a fail-closed error.
                if index == len(source_rows) - 1:
                    if stats is not None:
                        stats.trailing_partial_rows_skipped += 1
                    break
                raise ValueError(f"debug frame {index} is incomplete or non-numeric") from exc
            if not all(math.isfinite(item) for item in observation + action):
                raise ValueError(f"debug frame {index} contains NaN or Inf")
            if not history:
                history = [observation] * frame_stack
            else:
                history = history[1:] + [observation]
            flattened = tuple(item for frame in history for item in frame)
            yield ParityRow(
                frame=index,
                profile=profile,
                inputs=flattened,
                expected_outputs=action,
            )


class ErrorAccumulator:
    def __init__(self) -> None:
        self.values: list[float] = []
        self.sum_abs = 0.0
        self.sum_squared = 0.0
        self.maximum = 0.0

    def add(self, actual: Sequence[float], expected: Sequence[float]) -> None:
        if len(actual) != len(expected):
            raise ValueError(f"output width mismatch: {len(actual)} != {len(expected)}")
        for got, want in zip(actual, expected):
            error = abs(float(got) - float(want))
            if not math.isfinite(error):
                raise ValueError("inference produced NaN or Inf")
            self.values.append(error)
            self.sum_abs += error
            self.sum_squared += error * error
            self.maximum = max(self.maximum, error)

    def summary(self) -> dict[str, float | int]:
        if not self.values:
            raise ValueError("no output values were compared")
        ordered = sorted(self.values)
        p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
        return {
            "value_count": len(ordered),
            "maximum_absolute_error": self.maximum,
            "mean_absolute_error": self.sum_abs / len(ordered),
            "root_mean_squared_error": math.sqrt(self.sum_squared / len(ordered)),
            "p95_absolute_error": ordered[p95_index],
        }


def parity_passes(
    *,
    frame_count: int,
    metrics: dict[str, float | int],
    minimum_frames: int,
    maximum_absolute_error: float,
    maximum_rmse: float,
) -> bool:
    return (
        frame_count >= minimum_frames
        and float(metrics["maximum_absolute_error"]) <= maximum_absolute_error
        and float(metrics["root_mean_squared_error"]) <= maximum_rmse
    )


def static_width(shape: Sequence[object]) -> int | None:
    dimensions = [item for item in shape if isinstance(item, int) and item > 0]
    if not dimensions:
        return None
    product = 1
    for item in dimensions:
        product *= item
    return product


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--frames")
    source.add_argument("--debug-csv")
    parser.add_argument("--outputs")
    parser.add_argument("--frame-stack", type=int, default=10)
    parser.add_argument("--profile", default="cpgwalk")
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--minimum-frames", type=int, default=100)
    parser.add_argument("--maximum-frames", type=int, default=2000)
    parser.add_argument("--maximum-absolute-error", type=float, default=5e-6)
    parser.add_argument("--maximum-rmse", type=float, default=1e-6)
    parser.add_argument("--recorded-decimal-places", type=int)
    args = parser.parse_args()

    model = Path(args.model).resolve()
    frames = Path(args.frames).resolve() if args.frames else None
    outputs = Path(args.outputs).resolve() if args.outputs else None
    debug_csv = Path(args.debug_csv).resolve() if args.debug_csv else None
    receipt = Path(args.receipt).resolve()
    if frames is not None and outputs is None:
        raise ValueError("--outputs is required with --frames")
    if frames is None and outputs is not None:
        raise ValueError("--outputs is only valid with --frames")
    for path in (model, frames, outputs, debug_csv):
        if path is None:
            continue
        if not path.is_file():
            raise FileNotFoundError(path)
    if receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt}")
    if args.minimum_frames < 1 or args.maximum_frames < args.minimum_frames:
        raise ValueError("invalid frame limits")
    if args.recorded_decimal_places is not None:
        if debug_csv is None:
            raise ValueError("--recorded-decimal-places is only valid with --debug-csv")
        if not 0 <= args.recorded_decimal_places <= 17:
            raise ValueError("--recorded-decimal-places must be between 0 and 17")

    import numpy as np
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(
        str(model), sess_options=options, providers=["CPUExecutionProvider"]
    )
    model_inputs = session.get_inputs()
    model_outputs = session.get_outputs()
    if len(model_inputs) != 1 or not model_outputs:
        raise ValueError(
            f"expected one model input and at least one output, got "
            f"{len(model_inputs)} and {len(model_outputs)}"
        )
    input_meta = model_inputs[0]
    output_meta = model_outputs[0]
    expected_input_width = static_width(input_meta.shape)
    expected_output_width = static_width(output_meta.shape)

    accumulator = ErrorAccumulator()
    frame_count = 0
    profile: str | None = None
    first_frame: int | None = None
    last_frame: int | None = None
    input_width: int | None = None
    output_width: int | None = None
    started = time.monotonic()
    debug_stats = DebugSourceStats()
    rows = (
        iter_parity_rows(frames, outputs, maximum_rows=args.maximum_frames)
        if frames is not None and outputs is not None
        else iter_debug_parity_rows(
            debug_csv,
            frame_stack=args.frame_stack,
            maximum_rows=args.maximum_frames,
            profile=args.profile,
            stats=debug_stats,
        )
    )
    for row in rows:
        profile = profile or row.profile
        if profile != row.profile:
            raise ValueError(f"multiple profiles in parity input: {profile!r}, {row.profile!r}")
        input_width = input_width or len(row.inputs)
        output_width = output_width or len(row.expected_outputs)
        if len(row.inputs) != input_width or len(row.expected_outputs) != output_width:
            raise ValueError("parity CSV vector width changed between frames")
        if expected_input_width is not None and input_width != expected_input_width:
            raise ValueError(
                f"recorded input width {input_width} does not match model {expected_input_width}"
            )
        if expected_output_width is not None and output_width != expected_output_width:
            raise ValueError(
                f"recorded output width {output_width} does not match model {expected_output_width}"
            )
        tensor = np.asarray(row.inputs, dtype=np.float32)
        if len(input_meta.shape) == 2:
            tensor = tensor.reshape(1, -1)
        elif len(input_meta.shape) != 1:
            raise ValueError(f"unsupported model input rank: {input_meta.shape}")
        actual = session.run([output_meta.name], {input_meta.name: tensor})[0]
        accumulator.add(actual.reshape(-1).tolist(), row.expected_outputs)
        frame_count += 1
        first_frame = row.frame if first_frame is None else first_frame
        last_frame = row.frame

    metrics = accumulator.summary()
    passed = parity_passes(
        frame_count=frame_count,
        metrics=metrics,
        minimum_frames=args.minimum_frames,
        maximum_absolute_error=args.maximum_absolute_error,
        maximum_rmse=args.maximum_rmse,
    )
    payload = {
        "schema": SCHEMA,
        "status": "PASSED" if passed else "FAILED",
        "decision": (
            (
                "quantized_vendor_policy_inputs_reproduced_within_declared_tolerance"
                if debug_csv is not None
                else "recorded_vendor_policy_inputs_reproduced"
            )
            if passed
            else "independent_policy_execution_not_equivalent"
        ),
        "offline_only": True,
        "read_only_inputs": True,
        "published_topics": [],
        "model": {
            "path": str(model),
            "size": model.stat().st_size,
            "sha256": sha256(model),
            "input": {"name": input_meta.name, "shape": input_meta.shape, "type": input_meta.type},
            "output": {
                "name": output_meta.name,
                "shape": output_meta.shape,
                "type": output_meta.type,
            },
        },
        "vendor_evidence": {
            "source_mode": "recorded_frames" if frames is not None else "debug_history_rebuild",
            "evidence_classification": (
                "full_precision_recorded_frame_parity"
                if frames is not None
                else "quantized_vendor_debug_csv_parity"
            ),
            "frames": (
                {"path": str(frames), "size": frames.stat().st_size, "sha256": sha256(frames)}
                if frames is not None
                else None
            ),
            "outputs": (
                {"path": str(outputs), "size": outputs.stat().st_size, "sha256": sha256(outputs)}
                if outputs is not None
                else None
            ),
            "debug_csv": (
                {
                    "path": str(debug_csv),
                    "size": debug_csv.stat().st_size,
                    "sha256": sha256(debug_csv),
                    "frame_stack": args.frame_stack,
                    "history_order": "oldest_to_newest",
                    "initialization": "repeat_first_observation",
                    "source_row_count": debug_stats.source_row_count,
                    "trailing_partial_rows_skipped": debug_stats.trailing_partial_rows_skipped,
                    "recorded_decimal_places": args.recorded_decimal_places,
                    "maximum_scalar_rounding_error": (
                        0.5 * (10.0 ** -args.recorded_decimal_places)
                        if args.recorded_decimal_places is not None
                        else None
                    ),
                }
                if debug_csv is not None
                else None
            ),
            "profile": profile,
            "first_frame": first_frame,
            "last_frame": last_frame,
            "frame_count": frame_count,
            "input_width": input_width,
            "output_width": output_width,
        },
        "metrics": metrics,
        "thresholds": {
            "minimum_frames": args.minimum_frames,
            "maximum_frames": args.maximum_frames,
            "maximum_absolute_error": args.maximum_absolute_error,
            "maximum_root_mean_squared_error": args.maximum_rmse,
        },
        "runtime": {
            "onnxruntime_version": ort.__version__,
            "numpy_version": np.__version__,
            "provider": "CPUExecutionProvider",
            "elapsed_wall_seconds": time.monotonic() - started,
        },
        "qualification": False,
        "limitations": (
            [
                "Vendor debug observations and actions are decimal-quantized; "
                "this result is not bit-exact full-precision parity."
            ]
            if debug_csv is not None
            else []
        ),
        "real_robot_commands": False,
        "finished_at_unix": time.time(),
    }
    atomic_json(receipt, payload)
    print(receipt)
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        receipt_arg = None
        if "--receipt" in sys.argv:
            index = sys.argv.index("--receipt") + 1
            if index < len(sys.argv):
                receipt_arg = Path(sys.argv[index]).resolve()
        if receipt_arg is not None and not receipt_arg.exists():
            atomic_json(
                receipt_arg,
                {
                    "schema": SCHEMA,
                    "status": "FAILED",
                    "decision": "offline_parity_probe_failed_closed",
                    "error": {"type": type(exc).__name__, "message": str(exc)},
                    "offline_only": True,
                    "published_topics": [],
                    "qualification": False,
                    "real_robot_commands": False,
                    "finished_at_unix": time.time(),
                },
            )
        raise
