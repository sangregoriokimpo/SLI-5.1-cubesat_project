import os
import sys
import yaml
from pathlib import Path
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import omni.kit.app
import omni.usd
from isaacsim.core.utils.stage import is_stage_loading
from ament_index_python.packages import get_package_share_directory

pkg = get_package_share_directory('sli')
usd_path = os.path.join(pkg, 'usd_files', 'earth', 'earthmodel.usd')

manager = omni.kit.app.get_app().get_extension_manager()

# Register Cesium ext paths
for p in [
    '~/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/exts/3',
    '~/.local/share/ov/data/exts/v2',
]:
    resolved = str(Path(p).expanduser())
    manager.add_path(resolved)
    print(f"[SLI] Added path: {resolved}")

simulation_app.update()

# Enable cesium.usd.plugins FIRST — this registers USD schema types
if not manager.is_extension_enabled('cesium.usd.plugins'):
    manager.set_extension_enabled_immediate('cesium.usd.plugins', True)
print(f"[SLI] cesium.usd.plugins enabled: {manager.is_extension_enabled('cesium.usd.plugins')}")

# Tick several times to let USD plugin registry propagate
for _ in range(10):
    simulation_app.update()

# Then enable the omniverse runtime
if not manager.is_extension_enabled('cesium.omniverse'):
    manager.set_extension_enabled_immediate('cesium.omniverse', True)
print(f"[SLI] cesium.omniverse enabled: {manager.is_extension_enabled('cesium.omniverse')}")

# Wait for Cesium runtime and ion session
print("[SLI] Waiting for Cesium ion session...")
for _ in range(90):
    simulation_app.update()

print(f"[SLI] Token already in USD, skipping carb set")
print(f"[SLI] Opening stage: {usd_path}")

# Use omni.usd context directly instead of open_stage helper
context = omni.usd.get_context()
context.open_stage(usd_path)

while is_stage_loading():
    simulation_app.update()

# Verify what actually loaded
stage = context.get_stage()
if stage:
    prims = [str(p.GetPath()) for p in stage.Traverse()]
    cesium_prims = [p for p in prims if 'Cesium' in p or 'cesium' in p.lower()]
    print(f"[SLI] Total prims: {len(prims)}")
    print(f"[SLI] Cesium prims found: {cesium_prims}")
else:
    print("[SLI] ERROR: Stage is None — failed to open")

print("[SLI] Stage loaded — watching for tile streaming...")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()