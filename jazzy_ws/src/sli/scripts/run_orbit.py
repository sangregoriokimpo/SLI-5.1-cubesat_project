import os
import yaml
from pathlib import Path
from isaacsim import SimulationApp
from ament_index_python.packages import get_package_share_directory

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.stage import open_stage, is_stage_loading
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.extensions import enable_extension
import omni.kit.app

pkg = get_package_share_directory('sli')
usd_path = os.path.join(pkg, 'usd_files', 'earth', 'earthmodel.usd')
config_path = os.path.join(pkg, 'config', 'orbit_scene.yaml')

with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

manager = omni.kit.app.get_app().get_extension_manager()
for ext_path in config['scene'].get('ext_paths', []):
    manager.add_path(str(Path(ext_path).expanduser()))

for ext in config['scene'].get('extensions', []):
    enable_extension(ext)

for _ in range(20):
    simulation_app.update()

open_stage(usd_path)
while is_stage_loading():
    simulation_app.update()

for body in config['bodies']:
    create_prim(
        prim_path=body['prim_path'],
        prim_type='Cube',
        scale=(0.1, 0.1, 0.1),
    )

print(f"[SLI] Loaded {len(config['bodies'])} bodies from config")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()