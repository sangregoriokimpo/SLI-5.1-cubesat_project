import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.stage import open_stage, is_stage_loading

usd_path = os.path.expanduser("~/Documents/SLI/orbitTest3/orbitTest3A.usd")
open_stage(usd_path)
while is_stage_loading():
    simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()