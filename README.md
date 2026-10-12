# SceneForge

Deterministic service-building scene generation and X2 simulation control/qualification tooling. This branch is a sanitized source snapshot, not a bundled robot or simulator deployment.

## Components

- Validated building/scene specifications, asset contracts and deterministic layout generation.
- Layered OpenUSD export, canonical JSON and build fingerprints.
- Engine-neutral X2 control sessions, simulation-time state handling and qualification metrics.
- Source-only runtime adapters and test harnesses under `admin`. Their environment paths are generic placeholders. Do not run these until the licensed runtime and robot assets have been independently provisioned and reviewed.

## Local development

Python 3.11+ is required. Dependencies and optional extras are declared in `pyproject.toml`.

```bash
python -m pip install -e '.[dev]'
python -m sceneforge --help
python -m pytest tests/test_canonical.py tests/test_contracts.py tests/test_layout.py tests/test_preview.py
```

`examples/hospital` contains synthetic building and scene specifications, not a scan of an actual hospital. Check CLI help for supported build commands and supply explicitly permitted assets for any downstream simulator integration.

## Simulation boundaries

No robot model, vendor controller binary, weights, Isaac/Genie runtime, captured telemetry, render media or third-party engine patch is included. Some integration tests and adapters therefore require separately supplied dependencies. Core unit tests are distinct from GPU, navigation, collision or manipulation acceptance. No physical robot connection or command is authorized by this code snapshot.

The original runtime is not included or certified here. Historic operating instructions and internal machine inventories were deliberately excluded. Runtime wrappers use `/opt/sceneforge/runtime` as an example root; configure an independently reviewed environment before execution.

## Publication scope

This private source snapshot starts new Git history, without company scene assets, evidence, internal runbooks, credentials or original deployment paths. The internal source project and its Git remotes remain unchanged. Authorization to upload this snapshot does not grant a public open-source license. See `LICENSE_STATUS.md`; all third-party licenses remain applicable.
