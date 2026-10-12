"""Build and seal one immutable X2 PhysX cold-start episode result.

The runner owns runtime facts.  This module deliberately does not derive a
passing gate from legacy result fields, logs, or filenames.  It only:

* copies the explicit facts from a versioned sealing input;
* hashes an explicit artifact-role-to-path map;
* builds a new ``qualification_evidence`` block; and
* preflights that block with the same single-episode validator used by the
  five-attempt qualification aggregator.

An invalid or incomplete input never becomes the requested ``result.json``.
The final result always remains ``qualification=false``; only the five-attempt
suite aggregator may claim qualification.
"""

from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

from .x2_locomotion_qualification import (
    EPISODE_EVIDENCE_SCHEMA,
    EPISODE_EVIDENCE_SCHEMA_V1,
    EPISODE_EVIDENCE_SCHEMA_V2,
    REQUIRED_ARTIFACT_ROLES,
    SUITE_SCHEMA_V1,
    SUITE_SCHEMA_V2,
    build_suite,
    required_artifact_roles,
    sha256,
)


SEALING_INPUT_SCHEMA = "sceneforge/x2-locomotion-episode-sealing-input/v1"
SEALING_INPUT_SCHEMA_V1 = SEALING_INPUT_SCHEMA
SEALING_INPUT_SCHEMA_V2 = "sceneforge/x2-locomotion-episode-sealing-input/v2"
EPISODE_RESULT_SCHEMA = "sceneforge/x2-geniesim-cold-start-combination/v1"
SEALING_RECEIPT_SCHEMA = "sceneforge/x2-locomotion-episode-sealing-receipt/v1"
DERIVED_DIRECTORY_NAME = "derived"

_EVIDENCE_FACT_BLOCKS = (
    "frozen_inputs",
    "safety",
    "lifecycle",
    "reset",
    "sim_time_freeze",
    "cases",
    "stable_stop",
)


class EpisodeSealingError(RuntimeError):
    """A fail-closed sealing outcome with machine-readable findings."""

    def __init__(
        self,
        gate_status: str,
        findings: list[dict[str, str]],
    ) -> None:
        if gate_status not in {"INCOMPLETE", "FAILED"}:
            raise ValueError(f"unsupported sealing gate status: {gate_status}")
        self.gate_status = gate_status
        self.findings = findings
        summary = "; ".join(
            f"{item.get('code', 'UNKNOWN')}: {item.get('message', '')}"
            for item in findings
        )
        super().__init__(summary or gate_status)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": SEALING_RECEIPT_SCHEMA,
            "status": self.gate_status,
            "sealed": False,
            "qualification": False,
            "real_robot_commands": False,
            "findings": deepcopy(self.findings),
        }


def _finding(
    severity: str,
    code: str,
    location: str,
    message: str,
) -> dict[str, str]:
    return {
        "severity": severity,
        "code": code,
        "location": location,
        "message": message,
    }


def _raise_one(
    severity: str,
    code: str,
    location: str,
    message: str,
) -> None:
    raise EpisodeSealingError(
        severity,
        [_finding(severity, code, location, message)],
    )


def _normalize_episode_paths(
    episode_root: Path,
    output_path: Path | None,
) -> tuple[Path, Path]:
    root = episode_root.resolve()
    if not root.is_dir():
        _raise_one(
            "INCOMPLETE",
            "EPISODE_ROOT_MISSING",
            "episode_root",
            f"episode root does not exist: {root}",
        )
    derived = root / DERIVED_DIRECTORY_NAME
    if derived.exists() or derived.is_symlink():
        if derived.is_symlink() or not derived.is_dir():
            _raise_one(
                "FAILED",
                "DERIVED_DIRECTORY_INVALID",
                "episode_root.derived",
                "derived must be a non-symlink directory",
            )
    output = (
        derived / "result.json"
        if output_path is None and derived.is_dir()
        else root / "result.json"
        if output_path is None
        else output_path
    )
    if not output.is_absolute():
        output = root / output
    output = output.resolve()
    allowed_parents = {root}
    if derived.is_dir():
        allowed_parents.add(derived.resolve())
    if output.parent not in allowed_parents:
        _raise_one(
            "FAILED",
            "OUTPUT_PATH_ESCAPE",
            "output",
            "sealed result must be a direct child of episode_root or its reserved derived directory",
        )
    if output.exists():
        _raise_one(
            "FAILED",
            "OUTPUT_ALREADY_EXISTS",
            "output",
            f"immutable episode result already exists: {output}",
        )
    return root, output


def _artifact_descriptors(
    episode_root: Path,
    output: Path,
    artifact_paths: object,
    *,
    required_roles: tuple[str, ...],
) -> list[dict[str, Any]]:
    if not isinstance(artifact_paths, Mapping):
        _raise_one(
            "INCOMPLETE",
            "ARTIFACT_MAP_MISSING",
            "sealing_input.artifacts",
            "artifact role-to-path map is required",
        )

    invalid_roles = [
        role for role in artifact_paths if not isinstance(role, str) or not role.strip()
    ]
    if invalid_roles:
        _raise_one(
            "FAILED",
            "ARTIFACT_ROLE_INVALID",
            "sealing_input.artifacts",
            "every artifact role must be a non-empty string",
        )

    missing_roles = sorted(set(required_roles) - set(artifact_paths))
    if missing_roles:
        raise EpisodeSealingError(
            "INCOMPLETE",
            [
                _finding(
                    "INCOMPLETE",
                    "ARTIFACT_ROLE_MISSING",
                    "sealing_input.artifacts",
                    f"required role {role} is missing",
                )
                for role in missing_roles
            ],
        )

    descriptors: list[dict[str, Any]] = []
    seen_files: dict[Path, str] = {}
    for role in sorted(artifact_paths):
        raw_path = artifact_paths[role]
        if not isinstance(raw_path, str) or not raw_path.strip():
            _raise_one(
                "INCOMPLETE",
                "ARTIFACT_PATH_MISSING",
                f"sealing_input.artifacts.{role}",
                "artifact path must be a non-empty relative string",
            )
        relative = Path(raw_path)
        if relative.is_absolute() or ".." in relative.parts:
            _raise_one(
                "FAILED",
                "ARTIFACT_PATH_ESCAPE",
                f"sealing_input.artifacts.{role}",
                "artifact path must remain relative to episode_root",
            )
        candidate = episode_root / relative
        resolved = candidate.resolve()
        try:
            resolved.relative_to(episode_root)
        except ValueError:
            _raise_one(
                "FAILED",
                "ARTIFACT_PATH_ESCAPE",
                f"sealing_input.artifacts.{role}",
                "resolved artifact escapes episode_root",
            )
        if resolved == output:
            _raise_one(
                "FAILED",
                "SELF_REFERENTIAL_ARTIFACT",
                f"sealing_input.artifacts.{role}",
                "the episode result cannot hash itself",
            )
        if resolved in seen_files:
            _raise_one(
                "FAILED",
                "DUPLICATE_ARTIFACT_PATH",
                f"sealing_input.artifacts.{role}",
                f"artifact path is already used by role {seen_files[resolved]}",
            )
        seen_files[resolved] = role
        if not resolved.is_file():
            _raise_one(
                "INCOMPLETE",
                "ARTIFACT_FILE_MISSING",
                f"sealing_input.artifacts.{role}",
                f"artifact is absent: {relative.as_posix()}",
            )
        descriptors.append(
            {
                "role": role,
                "path": relative.as_posix(),
                "sha256": sha256(resolved),
                "size": resolved.stat().st_size,
            }
        )
    return descriptors


def build_episode_payload(
    *,
    episode_root: Path,
    sealing_input: Mapping[str, Any],
    output_path: Path | None = None,
) -> tuple[dict[str, Any], Path]:
    """Build, but do not write, a candidate immutable episode result.

    Runtime facts are copied exactly.  No missing boolean, count, hash, process
    identity, reset receipt, freeze measurement, case gate, or stop metric is
    inferred by this function.
    """

    root, output = _normalize_episode_paths(episode_root, output_path)
    if not isinstance(sealing_input, Mapping):
        _raise_one(
            "FAILED",
            "SEALING_INPUT_INVALID",
            "sealing_input",
            "sealing input must be a JSON object",
        )
    schema = sealing_input.get("schema")
    if schema is None:
        _raise_one(
            "INCOMPLETE",
            "SEALING_SCHEMA_MISSING",
            "sealing_input.schema",
            "versioned sealing input schema is required",
        )
    evidence_schema_by_input = {
        SEALING_INPUT_SCHEMA_V1: EPISODE_EVIDENCE_SCHEMA_V1,
        SEALING_INPUT_SCHEMA_V2: EPISODE_EVIDENCE_SCHEMA_V2,
    }
    if schema not in evidence_schema_by_input:
        _raise_one(
            "FAILED",
            "SEALING_SCHEMA_UNSUPPORTED",
            "sealing_input.schema",
            f"expected one of {tuple(evidence_schema_by_input)}, found {schema!r}",
        )
    if sealing_input.get("qualification") not in {None, False}:
        _raise_one(
            "FAILED",
            "EPISODE_RELABELLED",
            "sealing_input.qualification",
            "a single episode may not claim qualification",
        )

    evidence_schema = evidence_schema_by_input[str(schema)]
    descriptors = _artifact_descriptors(
        root,
        output,
        sealing_input.get("artifacts"),
        required_roles=required_artifact_roles(evidence_schema),
    )
    evidence: dict[str, Any] = {
        "schema": evidence_schema,
        "artifacts": descriptors,
    }
    if output.parent == episode_root.resolve() / DERIVED_DIRECTORY_NAME:
        # The result is physically published in the only writable child of a
        # frozen episode.  Validators accept exactly this one-level marker and
        # continue resolving every artifact inside the immutable episode root.
        evidence["artifact_root"] = ".."
    for name in (
        "episode_id",
        "physics_authority",
        "published_command_topics",
        "real_robot_commands",
        *_EVIDENCE_FACT_BLOCKS,
    ):
        if name in sealing_input:
            evidence[name] = deepcopy(sealing_input[name])

    payload = {
        "schema": EPISODE_RESULT_SCHEMA,
        "status": "PASSED",
        "qualification": False,
        "real_robot_commands": False,
        "qualification_evidence": evidence,
    }
    try:
        json.dumps(payload, allow_nan=False)
    except (TypeError, ValueError) as exc:
        _raise_one(
            "FAILED",
            "SEALING_INPUT_NOT_STRICT_JSON",
            "sealing_input",
            str(exc),
        )
    return payload, output


def seal_episode(
    *,
    episode_root: Path,
    sealing_input: Mapping[str, Any],
    output_path: Path | None = None,
) -> dict[str, Any]:
    """Validate and atomically expose a new immutable episode result.

    The preflight invokes the public suite builder and inspects its
    single-attempt verdict.  Suite-level attempt-count failure is intentionally
    ignored here; the final five-attempt decision remains the aggregator's job.
    """

    payload, output = build_episode_payload(
        episode_root=episode_root,
        sealing_input=sealing_input,
        output_path=output_path,
    )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.",
        suffix=".preflight",
        dir=output.parent,
    )
    temporary = Path(temporary_name)
    linked = False
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(
                json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
            )
            stream.flush()
            os.fsync(stream.fileno())

        evidence_schema = payload["qualification_evidence"]["schema"]
        suite_schema = (
            SUITE_SCHEMA_V2
            if evidence_schema == EPISODE_EVIDENCE_SCHEMA_V2
            else SUITE_SCHEMA_V1
        )
        report = build_suite(
            [temporary],
            suite_id="episode-sealing-preflight",
            suite_schema=suite_schema,
        )
        attempt = report["attempts"][0]
        if attempt["gate_status"] != "PASSED":
            raise EpisodeSealingError(
                attempt["gate_status"],
                deepcopy(attempt["findings"]),
            )

        # A hard link is an atomic no-clobber publication on the same volume.
        # Unlike os.replace, it cannot silently overwrite immutable evidence.
        try:
            os.link(temporary, output)
        except FileExistsError:
            _raise_one(
                "FAILED",
                "OUTPUT_ALREADY_EXISTS",
                "output",
                f"immutable episode result already exists: {output}",
            )
        linked = True
        temporary.unlink()
        os.chmod(output, 0o444)
        return {
            "schema": SEALING_RECEIPT_SCHEMA,
            "status": "PASSED",
            "sealed": True,
            "qualification": False,
            "real_robot_commands": False,
            "episode_id": payload["qualification_evidence"]["episode_id"],
            "output": str(output),
            "sha256": sha256(output),
            "size": output.stat().st_size,
            "attempt_gate_status": attempt["gate_status"],
            "findings": [],
        }
    except Exception:
        if linked and output.exists():
            try:
                os.chmod(output, 0o600)
                output.unlink()
            except OSError:
                pass
        raise
    finally:
        if temporary.exists():
            try:
                os.chmod(temporary, 0o600)
                temporary.unlink()
            except OSError:
                pass
