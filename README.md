# SLI 5.1 CubeSat Project

ROS2 Jazzy + Isaac Sim 5.1 orbital mechanics simulation for CubeSat proximity operations.

## Requirements

- Ubuntu 22.04 or Ubuntu 24.04
- Isaac Sim 5.1 (pip install into a Python 3.11 environment)
- Docker
- ROS2 Jazzy (for external nodes and tooling)

## Setup

### 1. Clone the repository
```bash
git clone https://github.com/sangregoriokimpo/SLI-5.1-cubesat_project.git
cd SLI-5.1-cubesat_project/IsaacSim-5.1-ROS2_workspace
```

### 2. Install Docker
```bash
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER
sudo ./build_ros.sh -d jazzy -v 24.04   # Ubuntu 24.04
# or
sudo ./build_ros.sh -d jazzy -v 22.04   # Ubuntu 22.04
```

### 3. Build the workspace

Ubuntu 24.04:
```bash
sudo ./build_ros.sh -d jazzy -v 24.04
```

Ubuntu 22.04:
```bash
sudo ./build_ros.sh -d jazzy -v 22.04
```

### 4. Source the workspace
```bash
source IsaacSim-5.1-ROS2_workspace/build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash
```

Add to `~/.bashrc` to make it permanent:
```bash
echo 'source ~/SLI-5.1-cubesat_project/IsaacSim-5.1-ROS2_workspace/build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 5. Install Isaac Sim 5.1

Isaac Sim must be installed separately into a Python 3.11 environment. Follow the [NVIDIA Isaac Sim installation guide](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html).

The project assumes Isaac Sim is installed at:
```
~/miniforge3/envs/isaaclab311/
```

If installed elsewhere, update the Python executable path in:
```
jazzy_ws/src/sli/launch/orbit.launch.py
```

## Launch
```bash
source IsaacSim-5.1-ROS2_workspace/build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash
ros2 launch sli orbit.launch.py
```

Headless mode (for AWS or server deployment):
```bash
ros2 launch sli orbit.launch.py headless:=true
```

## Project Structure
```
IsaacSim-5.1-ROS2_workspace/
└── jazzy_ws/
    └── src/
        ├── orbit_interfaces/       # Custom ROS2 message definitions
        │   └── msg/
        │       ├── ThrustCmd.msg
        │       └── OrbitState.msg
        └── sli/                    # Main simulation package
            ├── config/
            │   └── orbit_scene.yaml    # Scene and body configuration
            ├── launch/
            │   └── orbit.launch.py     # Main launch file
            ├── scripts/
            │   └── run_orbit.py        # Isaac Sim standalone entry point
            └── usd_files/
                └── earth/
                    └── earthmodel.usd  # 1:1 scale Earth model (meters)
```

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/orbit/state` | `orbit_interfaces/OrbitState` | Publish |
| `/orbit/thrust_cmd` | `orbit_interfaces/ThrustCmd` | Subscribe |

## Scene Configuration

Edit `jazzy_ws/src/sli/config/orbit_scene.yaml` to configure the scene and orbiting bodies.

Example configuration for a CubeSat in Low Earth Orbit:
```yaml
scene:
  usd_path: usd_files/earth/earthmodel.usd
  attractor_prim: /World/Earth

bodies:
  - prim_path: /World/CubeSat
    orbit_type: circular
    radius: 6771000.0      # meters — Low Earth Orbit
    plane: xy
    mu: 398600441800000.0  # m^3/s^2
    dt_sim: 0.5
```

## Rebuild after changes

Any changes to `orbit_scene.yaml`, `orbit.launch.py`, or `run_orbit.py` require a rebuild:
```bash
cd ~/SLI-5.1-cubesat_project/IsaacSim-5.1-ROS2_workspace
sudo ./build_ros.sh -d jazzy -v 24.04
```

## Verify topics are live

In a sourced terminal after launching:
```bash
source /opt/ros/jazzy/setup.bash
source ~/SLI-5.1-cubesat_project/IsaacSim-5.1-ROS2_workspace/build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash
ros2 topic list
ros2 topic echo /orbit/state
```

## Notes

- The workspace must be built with Docker to ensure Python 3.11 compatibility with Isaac Sim 5.1
- Both Ubuntu 22.04 and 24.04 require the Docker build — the system Python (3.10 and 3.12 respectively) is incompatible with Isaac Sim
- The `orbit_interfaces` package contains custom message definitions and must be built before the `sli` package
- Isaac Sim extensions (`com.ov.core`, `com.ov.controls`, `com.ov.ros2`) must be registered separately in the Isaac Sim Extension Manager