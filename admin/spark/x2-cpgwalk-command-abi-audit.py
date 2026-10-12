#!/usr/bin/env python3
"""Freeze and audit the vendor CPGWalk JointCommand construction ABI."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
import time
from pathlib import Path


SCHEMA = "sceneforge/x2-cpgwalk-command-abi-audit/v1"
SYMBOL = (
    "xyber_x1_infer::rl_control_module::RLCpgWalkController::GetJointCmdData()"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write(path: Path, data: str, mode: int = 0o444) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, mode)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def atomic_json(path: Path, payload: dict) -> None:
    atomic_write(path, json.dumps(payload, indent=2, sort_keys=True) + "\n")


def symbol_bounds(nm_output: str) -> tuple[int, int]:
    entries: list[tuple[int, str]] = []
    for line in nm_output.splitlines():
        match = re.match(r"^([0-9a-fA-F]+)\s+\S\s+(.+)$", line)
        if match:
            entries.append((int(match.group(1), 16), match.group(2)))
    for index, (address, name) in enumerate(entries):
        if name == SYMBOL:
            if index + 1 >= len(entries):
                raise ValueError("GetJointCmdData is the final nm symbol")
            return address, entries[index + 1][0]
    raise ValueError("GetJointCmdData symbol was not found")


def instruction_contract(disassembly: str) -> dict:
    patterns = {
        "position_target_store": r"\bstr\s+d2,\s*\[x1,\s*x0\]",
        "zero_vector_stores": r"\bstr\s+xzr,\s*\[x1,\s*x0\]",
        "stiffness_store": r"\bstr\s+d1,\s*\[x1,\s*x0\]",
        "damping_store": r"\bstr\s+d0,\s*\[x1,\s*x0\]",
    }
    counts = {name: len(re.findall(pattern, disassembly)) for name, pattern in patterns.items()}
    return {
        "counts": counts,
        "passed": (
            counts["position_target_store"] == 1
            and counts["zero_vector_stores"] == 2
            and counts["stiffness_store"] == 1
            and counts["damping_store"] == 1
        ),
        "interpretation": {
            "position": "q_cmd_filtered element loaded in d2",
            "velocity": "zero vector",
            "feedforward_effort": "zero vector",
            "stiffness": "joint_conf stiffness with configured elbow override",
            "damping": "joint_conf damping with configured elbow override",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    parser.add_argument("--disassembly", required=True)
    parser.add_argument("--receipt", required=True)
    args = parser.parse_args()

    binary = Path(args.binary).resolve()
    disassembly_path = Path(args.disassembly).resolve()
    receipt = Path(args.receipt).resolve()
    if not binary.is_file():
        raise FileNotFoundError(binary)
    if disassembly_path.exists() or receipt.exists():
        raise FileExistsError("refusing to overwrite command ABI evidence")

    nm = subprocess.run(
        ["nm", "-n", "-C", str(binary)], check=True, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    start, stop = symbol_bounds(nm.stdout)
    objdump = subprocess.run(
        [
            "objdump", "-d", "--demangle",
            f"--start-address=0x{start:x}", f"--stop-address=0x{stop:x}", str(binary),
        ],
        check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    disassembly = objdump.stdout
    contract = instruction_contract(disassembly)
    atomic_write(disassembly_path, disassembly)
    payload = {
        "schema": SCHEMA,
        "status": "PASSED" if contract["passed"] else "FAILED",
        "decision": (
            "vendor_position_pd_joint_command_abi_verified"
            if contract["passed"]
            else "vendor_joint_command_abi_not_verified"
        ),
        "offline_only": True,
        "published_topics": [],
        "binary": {
            "path": str(binary), "size": binary.stat().st_size, "sha256": sha256(binary),
        },
        "symbol": {
            "demangled_name": SYMBOL,
            "start_address_hex": f"0x{start:x}",
            "stop_address_hex": f"0x{stop:x}",
            "size_bytes": stop - start,
        },
        "disassembly": {
            "path": str(disassembly_path),
            "size": disassembly_path.stat().st_size,
            "sha256": sha256(disassembly_path),
        },
        "contract": contract,
        "limitations": [
            "This freezes the command-construction ABI of one ARM64 release binary; it is "
            "not a source-level vendor guarantee.",
            "The gate verifies the position/zero-velocity/zero-effort/gain lane construction "
            "but does not exercise ROS middleware serialization.",
            "Numeric position-target parity is established separately from the vendor CSV.",
        ],
        "qualification": False,
        "real_robot_commands": False,
        "finished_at_unix": time.time(),
    }
    atomic_json(receipt, payload)
    print(receipt)
    return 0 if contract["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
