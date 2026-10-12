# Source snapshot validation

Validation performed on 2026-10-12 on the exported source copy.

**53 selected offline unit tests passed**, covering canonical JSON, contracts, deterministic layout, preview generation, X2 control, sessions and simulation-time differential logic.

Test files:

- `test_canonical.py`
- `test_contracts.py`
- `test_layout.py`
- `test_preview.py`
- `test_x2_control.py`
- `test_x2_session.py`
- `test_x2_sim_time_differential.py`

These were executed through Python's unittest runner with the existing local dependency environment. A clean-machine installation and the full integration test suite were not verified. No model weights, vendor runtime, GPU simulation, physical robot, navigation or manipulation was run.

Runtime adapters and integration tests need separately provisioned, licensed components and explicitly reviewed configuration.
