#!/usr/bin/env python3
import importlib.util
import json
import platform
import sys

modules = ("omni", "pxr", "rclpy", "isaacsim")
print(json.dumps({
    "python": sys.version,
    "architecture": platform.machine(),
    "modules": {name: importlib.util.find_spec(name) is not None for name in modules},
    "real_robot_commands": False,
}, indent=2))
