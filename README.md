# SceneSem

Local 3D scene semantic annotation workbench. This branch is a sanitized source snapshot, not a packaged scene dataset or a ready-to-run deployment.

## Features

- Inspect 3D Gaussian Splatting and point-cloud views with editable annotations.
- Use local Mask2Former instance segmentation, then map visible pixels into 3D using depth and camera parameters.
- Review, extend or erase surface annotations, maintain object identities, and save immutable project versions.
- Export independent semantic datasets and an offline robot-map draft. Navigation and manipulation readiness require separate validation.

## Requirements and startup

Node.js 20+ and a compatible Python environment are required. `requirements.txt` and `requirements-observed.lock.txt` record the original environment; installation on a new machine is not verified and CPU PyTorch wheels may need their official wheel source.

Obtain appropriately licensed PlayCanvas 2.21.4 under `vendor/playcanvas` and separately authorized model files under `models`. No dependencies, model weights, scanned scenes, annotation records or browser drafts are bundled.

Create `config/local.json` from `config/local.example.json` and supply your own permitted raw and derived scene inputs. The backend's `scripts/scenesem_config.py` defines the required cache files. Merely pointing to an arbitrary point cloud is not sufficient: preprocessing and the scene contract must match the application.

```powershell
node scripts/test.cjs
python -X utf8 scripts/serve_annotation_tool.py
```

The service binds to `127.0.0.1:18876`. Open `http://127.0.0.1:18876/` after provisioning the inputs. Port conflicts must be resolved explicitly; do not automatically switch ports or terminate unrelated processes.

## Snapshot boundaries

This export starts new Git history. It excludes company handoff/runbooks, historical evidence, original configuration, source scenes and derived data, runtime resource packages, presentations, credentials, third-party source copies and weights. Runtime preparation must use your own permitted assets. The source project and its internal Git remotes were not modified.

This private repository authorization does not grant an open-source license. See `LICENSE_STATUS.md`. Third-party software and models retain their own license and usage restrictions.
