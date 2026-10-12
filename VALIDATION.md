# Source snapshot validation

Validation performed on 2026-10-12 on the exported source copy.

- Existing Node regression runner: **24 of 31 scripts passed**.
- The full runner does not pass in this source-only export. Its remaining checks rely on the original local Python environment, PlayCanvas source or scene fixtures, which are deliberately not distributed here.
- Model inference, browser/GPU rendering, scene semantic accuracy, navigation and manipulation were not tested for this export.

Provision licensed dependencies and your own permitted scene fixtures before running all environment-dependent tests. This branch is a code snapshot, not a qualified ready-to-run dataset or deployment.
