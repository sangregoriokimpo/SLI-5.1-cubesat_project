import os
from isaacsim import SimulationApp
from ament_index_python.packages import get_package_share_directory


simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.stage import open_stage, is_stage_loading

pkg = get_package_share_directory('sli')

usd_path = os.path.join(pkg, 'usd_files','earth', 'earthmodel.usd')
config_path = os.path.join(pkg, 'config', 'orbit_scene.yaml')

open_stage(usd_path)
while is_stage_loading():
    simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()