import inspect

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim


for name in sorted(
    item
    for item in dir(SingleArticulation)
    if "force" in item.lower() or "pose" in item.lower() or "velocity" in item.lower()
):
    value = getattr(SingleArticulation, name)
    try:
        signature = inspect.signature(value)
    except (TypeError, ValueError):
        signature = "<no signature>"
    print(name, signature)

print("-- SingleRigidPrim --")
for name in sorted(
    item
    for item in dir(SingleRigidPrim)
    if "force" in item.lower() or "pose" in item.lower() or "velocity" in item.lower()
):
    value = getattr(SingleRigidPrim, name)
    try:
        signature = inspect.signature(value)
    except (TypeError, ValueError):
        signature = "<no signature>"
    print(name, signature)

app.close()
