# SLI CubeSat Orbital Mechanics Simulation
## Isaac Sim 5.1 + ROS2 Jazzy

# DOES NOT HAVE CLASSIFIED/ITAR FILES YET

Sierra Lobo Inc — CubeSat orbital mechanics simulation using NVIDIA Isaac Sim 5.1 and ROS2 Jazzy. Physics runs as pure ROS2 nodes outside Isaac Sim. Isaac Sim is used as a renderer only, controlled via the `simulation_interfaces` standard.

---

## Architecture

```
External ROS2 process (Python 3.12, system Jazzy)
┌─────────────────────────────────────────────────┐
│  orbit_vel_node  (one per body, role: orbit)     │
│    RK4 integrator → publishes /satN/orbit_state  │
│                                                  │
│  cw_node  (one per body, role: cw)               │
│    Hill's equations → publishes /satN/orbit_state│
│    subscribes to chief's orbit_state             │
│                                                  │
│  isaac_bridge_node  (one per body)               │
│    subscribes orbit_state → /set_entity_state    │
│                                                  │
│  scene_loader_node  (runs once at startup)       │
│    /load_world → /spawn_entity → /play → exits   │
└─────────────────────────────────────────────────┘
              │  ROS2 DDS (FastRTPS)
              ▼
Isaac Sim process (Python 3.11, internal Jazzy)
┌─────────────────────────────────────────────────┐
│  isaacsim.ros2.sim_control extension             │
│    /load_world  /spawn_entity  /set_entity_state │
│    /set_simulation_state  /get_entities  ...     │
└─────────────────────────────────────────────────┘
```

Key principle: **physics has zero Isaac Sim dependency**. Nodes are pure Python and can be unit tested without a GPU.

---

## Repository Structure

```
SLI-5.1-cubesat_project/
├── build_ros.sh                    # Docker build script (Python 3.11 binaries)
├── dockerfiles/
│   └── ubuntu_24_jazzy_python_311_minimal.dockerfile
├── jazzy_ws/                       # Host workspace (Python 3.12, for ROS2 nodes)
│   ├── fastdds.xml                 # FastDDS UDP config (required)
│   └── src/
│       ├── orbit_interfaces/       # Custom ROS2 messages
│       │   └── msg/
│       │       ├── OrbitState.msg
│       │       └── ThrustCmd.msg
│       └── sli/                    # Main simulation package
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── config/
│           │   └── orbit_scene.yaml
│           ├── launch/
│           │   └── orbit.launch.py
│           ├── scripts/
│           │   ├── scene_loader_node.py
│           │   ├── orbit_vel_node.py
│           │   ├── cw_node.py
│           │   └── isaac_bridge_node.py
│           ├── usd_files/
│           │   └── earth/
│           │       └── earthmodel.usd
│           └── urdf/
│               ├── bigsat/
│               │   ├── bigsat.urdf
│               │   └── bigsat.usd   (generated — see Setup)
│               └── smallsat/
│                   ├── smallsat.urdf
│                   └── smallsat.usd (generated — see Setup)
└── build_ws/                       # Docker build output (Python 3.11 binaries)
    └── jazzy/
        ├── jazzy_ws/install/       # Source before running Isaac Sim
        └── isaac_sim_ros_ws/install/
```

---

## Message Definitions

### `orbit_interfaces/msg/OrbitState`
```
std_msgs/Header header
string body_id               # USD prim path used as identifier
geometry_msgs/Vector3 position   # km, inertial frame, relative to attractor
geometry_msgs/Vector3 velocity   # km/s, inertial frame
geometry_msgs/Quaternion attitude
```

### `orbit_interfaces/msg/ThrustCmd`
```
std_msgs/Header header
string body_id
float64 throttle             # acceleration magnitude km/s²
float64 gimbal_pitch         # thrust direction angle (rad) from XY plane
float64 gimbal_yaw           # thrust direction angle (rad) in XY plane from +X
```

Thrust direction convention: `pitch=0, yaw=π/2` = prograde for xy-plane circular orbit.

---

## Prerequisites

### System
- Ubuntu 24.04 (Zorin 18 confirmed working)
- NVIDIA GPU with driver ≥ 535
- Docker (for building Python 3.11 custom message binaries)
- NVIDIA Isaac Sim 5.1 (workstation install at `/isaac-sim`)
- ROS2 Jazzy (system apt install, Python 3.12)
- Conda environment `env_isaaclab` with Isaac Sim dependencies

### ROS2 packages
```bash
sudo apt install ros-jazzy-simulation-interfaces
```

Note: Isaac Sim bundles `simulation_interfaces` version **1.1.0** internally. The apt package is 1.2.0 which causes a crash. The fix is to set `LD_LIBRARY_PATH` to use Isaac Sim's bundled version before launching (see Running section).

---

## Setup

### 1. Build custom messages (Docker, Python 3.11 — for Isaac Sim)

```bash
cd ~/SLI-5.1-cubesat_project
git submodule update --init --recursive
sudo ./build_ros.sh -d jazzy -v 24.04
```

Output at:
- `build_ws/jazzy/jazzy_ws/install/` — Python 3.11 Jazzy base
- `build_ws/jazzy/isaac_sim_ros_ws/install/` — Python 3.11 custom messages

### 2. Build host workspace (Python 3.12 — for ROS2 nodes)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/SLI-5.1-cubesat_project/jazzy_ws
colcon build
```

<!-- ### 3. Generate satellite USD files (one-time, in Isaac Sim Script Editor)

Open Isaac Sim, go to `Window → Script Editor`, run:

```python
from pxr import Usd, UsdGeom, Gf

# bigsat — 1m cube
stage = Usd.Stage.CreateNew(
    "/home/sgq/SLI-5.1-cubesat_project/jazzy_ws/src/sli/urdf/bigsat/bigsat.usd"
)
xform = UsdGeom.Xform.Define(stage, "/bigsat")
cube = UsdGeom.Cube.Define(stage, "/bigsat/geom")
cube.CreateSizeAttr(1.0)
stage.SetDefaultPrim(xform.GetPrim())
stage.GetRootLayer().Save()

# smallsat — 10x10x30 cm box
stage2 = Usd.Stage.CreateNew(
    "/home/sgq/SLI-5.1-cubesat_project/jazzy_ws/src/sli/urdf/smallsat/smallsat.usd"
)
xform2 = UsdGeom.Xform.Define(stage2, "/smallsat")
cube2 = UsdGeom.Cube.Define(stage2, "/smallsat/geom")
cube2.CreateSizeAttr(1.0)
UsdGeom.Xformable(cube2).AddScaleOp().Set(Gf.Vec3f(0.1, 0.1, 0.3))
stage2.SetDefaultPrim(xform2.GetPrim())
stage2.GetRootLayer().Save()
print("Done")
```

Then rebuild:
```bash
colcon build --packages-select sli
``` -->

---

## 3. Running

### Terminal 1 — Isaac Sim

```bash
# Set isaac_sim path
export isaac_sim_package_path=/isaac-sim
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/jazzy/lib

# Source Python 3.11 build (for orbit_interfaces custom messages)
source ~/SLI-5.1-cubesat_project/build_ws/jazzy/jazzy_ws/install/local_setup.bash
source ~/SLI-5.1-cubesat_project/build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash

# Launch
/isaac-sim/isaac-sim.sh \
  --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge \
  --/isaac/startup/ros_sim_control_extension=True
```

Wait for Isaac Sim viewport to fully load before proceeding.

### Terminal 2 — ROS2 nodes

Open a **fresh terminal** (not the same session as Terminal 1):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/SLI-5.1-cubesat_project/jazzy_ws
source install/local_setup.bash
ros2 launch sli orbit.launch.py
```

### What happens at launch

1. `scene_loader_node` waits for Isaac Sim services, calls `/load_world` with `earthmodel.usd`, calls `/spawn_entity` for each body with `spawn: true`, calls `/set_simulation_state → PLAYING`, then exits cleanly.
2. `orbit_vel_node` instances start integrating (RK4) and publish `OrbitState` at 30 Hz.
3. `cw_node` instance subscribes to chief's `OrbitState`, integrates Hill's equations, publishes deputy `OrbitState`.
4. `isaac_bridge_node` instances subscribe to orbit state and call `/set_entity_state` to move prims each frame.

---

## Configuration — `orbit_scene.yaml`

```yaml
workspace_root: ${ORBIT_WS}   # auto-resolved from install path

world:
  base_scene: usd_files/earth/earthmodel.usd

simulation:
  mu: 398600.4418    # km³/s²
  dt_sim: 0.00833   # ~1/120 s

bodies:
  - name: earth
    prim_path: /World/Earth
    role: attractor
    spawn: false

  - name: sat1
    prim_path: /World/Sat1
    role: orbit
    spawn: true
    usd: urdf/bigsat/bigsat.usd
    attractor: /World/Earth
    orbit:
      type: circular
      radius: 6778000   # m (Isaac Sim uses meters)
      plane: xy
    scale: 1000.0       # km→m: orbit math in km, Isaac Sim in m

  - name: sat2
    prim_path: /World/Sat2
    role: orbit
    spawn: true
    usd: urdf/smallsat/smallsat.usd
    attractor: /World/Earth
    orbit:
      type: elements
      a: 6778000        # m
      e: 0.01
      inc: 28.5
      raan: 0.0
      argp: 0.0
      nu: 45.0
    scale: 1000.0

  - name: sat3
    prim_path: /World/Sat3
    role: cw            # Clohessy-Wiltshire deputy
    spawn: true
    usd: urdf/smallsat/smallsat.usd
    attractor: /World/Earth
    chief: /World/Sat1
    orbit:
      type: cw_relative
      dr: [1.0, 0.0, 0.5]    # initial relative pos LVLH frame (km)
      dv: [0.0, 0.001, 0.0]  # initial relative vel LVLH frame (km/s)
    scale: 1000.0
```

**Body roles:**
- `attractor` — anchor only, not simulated
- `orbit` — RK4 two-body integrator
- `cw` — Clohessy-Wiltshire Hill's equations (proximity ops)

**Scale note:** Orbit math runs in km throughout. `scale: 1000.0` converts km → m for Isaac Sim's `/set_entity_state`. The `radius`/`a` fields in the config are passed to Isaac Sim for display scaling only — the integrator uses the `mu` and orbital element values directly in km units.

---

## Sending Thrust Commands

Apply a prograde impulse to sat1:

```bash
ros2 topic pub --once /sat1/cmd_thrust orbit_interfaces/msg/ThrustCmd \
  "{header: {frame_id: 'world'}, body_id: '/World/Sat1', \
    throttle: 0.1, gimbal_pitch: 0.0, gimbal_yaw: 1.5708}"
```

Stop thrust:

```bash
ros2 topic pub --once /sat1/cmd_thrust orbit_interfaces/msg/ThrustCmd \
  "{header: {frame_id: 'world'}, body_id: '/World/Sat1', \
    throttle: 0.0, gimbal_pitch: 0.0, gimbal_yaw: 0.0}"
```

Monitor orbit state:

```bash
ros2 topic echo /sat1/orbit_state
ros2 topic hz /sat1/orbit_state    # should be ~30 Hz
```

---

## Isaac Sim Services Reference

Available once `isaacsim.ros2.sim_control` is enabled:

| Service | Purpose |
|---|---|
| `/load_world` | Load a USD file as the scene |
| `/spawn_entity` | Spawn a USD asset into the stage |
| `/set_entity_state` | Set pose + twist of any prim |
| `/get_entity_state` | Query pose + twist of any prim |
| `/get_entities` | List all prims in stage |
| `/set_simulation_state` | Play / Pause / Stop |
| `/step_simulation` | Step N frames (requires paused) |
| `/reset_simulation` | Remove spawned entities, reset |

---

## Known Issues & Limitations

**Orbital motion appears slow** — orbit math is in km, Isaac Sim is in meters. `scale: 1000.0` converts positions correctly, but the visual motion speed depends on how the earthmodel.usd scene is scaled. At true LEO scale (6778 km radius) the orbital period is ~92 minutes.

**`simulation_interfaces` version mismatch** — Isaac Sim bundles version 1.1.0, apt installs 1.2.0. The `LD_LIBRARY_PATH` fix in Terminal 1 forces Isaac Sim to use its bundled version. Do not source system ROS2 in the same terminal as Isaac Sim.

**Terminal isolation required** — Isaac Sim terminal must have `LD_LIBRARY_PATH` set to Isaac Sim's internal libs. ROS2 nodes terminal must be a fresh terminal without those vars. Mixing them causes either Isaac Sim crash or `ros2` CLI failure.

**Cesium crash without LD_LIBRARY_PATH fix** — if Isaac Sim is launched without the internal lib path set, it may crash on `/load_world` with a Cesium segfault. Always use the full Terminal 1 launch sequence above.

**USD default prim** — satellite USDs must have a default prim set. The Script Editor generation step above handles this correctly. URDF-imported USDs do not set a default prim automatically.

---

## Development Notes

### Two Python environments

| Environment | Python | Used for |
|---|---|---|
| Docker build output | 3.11 | Isaac Sim internal ROS2 bridge, custom message `.so` files |
| System ROS2 Jazzy | 3.12 | `orbit_vel_node`, `cw_node`, `isaac_bridge_node`, `scene_loader_node` |

DDS handles transport between them — Python version mismatch only affects direct `.so` imports, not over-the-wire communication.

### Rebuilding after node changes

```bash
cd ~/SLI-5.1-cubesat_project/jazzy_ws
colcon build --packages-select sli
source install/local_setup.bash
```

### Rebuilding after message changes (requires Docker)

```bash
cd ~/SLI-5.1-cubesat_project
sudo ./build_ros.sh -d jazzy -v 24.04
cd jazzy_ws
colcon build --packages-select orbit_interfaces
source install/local_setup.bash
```

### Adding a new body

1. Add entry to `orbit_scene.yaml` with appropriate `role`, `usd`, `orbit` fields
2. No code changes needed — launch file generates nodes dynamically from config
3. Rebuild `sli` package if config was installed (or use `ORBIT_WS` pointing to source)