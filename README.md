# SLI CubeSat Orbital Mechanics Simulation
## Isaac Sim 5.1 + ROS2 Jazzy

# DOES NOT HAVE CLASSIFIED/ITAR FILES YET

Sierra Lobo Inc CubeSat orbital mechanics simulation using NVIDIA Isaac Sim 5.1 and ROS2 Jazzy. Physics runs as pure ROS2 nodes outside Isaac Sim. Isaac Sim is used as a renderer only, controlled via the `simulation_interfaces` standard.

---

## Architecture

```
External ROS2 process (Python 3.12, system Jazzy)
┌─────────────────────────────────────────────────┐
│  orbit_vel_node  (one per body, role: orbit)     │
│    RK4 integrator → publishes /sat1/orbit_state  │
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
IsaacSim-5.1-ROS2_workspace/
├── build_ros.sh                    # Docker build script (Python 3.11 binaries)
├── dockerfiles/
│   └── ubuntu_24_jazzy_python_311_minimal.dockerfile
├── jazzy_ws/                       # Host workspace (Python 3.12, for ROS2 nodes)
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
│           ├── sli/
│           │   ├── scene_loader_node.py
│           │   ├── orbit_vel_node.py
│           │   └── isaac_bridge_node.py
│           └── usd_files/
│               └── earth/
│                   └── earthmodel.usd
└── build_ws/                       # Docker build output (Python 3.11 binaries)
    └── jazzy/
        └── isaac_sim_ros_ws/
            └── install/            # Source before running Isaac Sim
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
- Ubuntu 24.04
- NVIDIA GPU with driver ≥ 535
- Docker (for building Python 3.11 custom message binaries)
- NVIDIA Isaac Sim 5.1 (pip-installed in conda env)
- ROS2 Jazzy (system install, Python 3.12)

### ROS2 packages
```bash
sudo apt install ros-jazzy-simulation-interfaces
```

---

## Setup

### 1. Build custom messages (Docker, Python 3.11 — for Isaac Sim)

```bash
cd ~/SLI-5.1-cubesat_ws/IsaacSim-5.1-ROS2_workspace
git submodule update --init --recursive
sudo ./build_ros.sh -d jazzy -v 24.04
```

Output at: `build_ws/jazzy/isaac_sim_ros_ws/install/`

### 2. Build host workspace (Python 3.12 — for ROS2 nodes)

```bash
source /opt/ros/jazzy/setup.bash
cd jazzy_ws
colcon build
```

### 3. Set workspace root (for USD file resolution)

```bash
export ORBIT_WS=/path/to/jazzy_ws/install/sli/share/sli
```

Or add to `~/.bashrc` for persistence.

---

## Running

### Terminal 1 — Isaac Sim

Source the Python 3.11 build before launching Isaac Sim so the ROS2 bridge can find custom messages:

```bash
source ~/SLI-5.1-cubesat_ws/IsaacSim-5.1-ROS2_workspace/build_ws/jazzy/jazzy_ws/install/local_setup.bash
source ~/SLI-5.1-cubesat_ws/IsaacSim-5.1-ROS2_workspace/build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash
conda activate isaaclab311
python -m isaacsim --/isaac/startup/ros_sim_control_extension=True
```

Wait for Isaac Sim to fully load before proceeding.

### Terminal 2 — ROS2 nodes

```bash
source /opt/ros/jazzy/setup.bash
cd ~/SLI-5.1-cubesat_ws/IsaacSim-5.1-ROS2_workspace/jazzy_ws
source install/setup.bash
export ORBIT_WS=$(pwd)/install/sli/share/sli
ros2 launch sli orbit.launch.py
```

### What happens at launch

1. `scene_loader_node` waits for Isaac Sim services, then calls `/load_world` with `earthmodel.usd`, then `/spawn_entity` for each body with `spawn: true`, then `/set_simulation_state → PLAYING`, then exits.
2. `orbit_vel_node` instances start integrating immediately and publish `OrbitState` at 30 Hz.
3. `isaac_bridge_node` instances subscribe to orbit state and call `/set_entity_state` to move prims each frame.

---

## Configuration — `orbit_scene.yaml`

```yaml
workspace_root: ${ORBIT_WS}   # set via env var before launch

world:
  base_scene: usd_files/earth/earthmodel.usd

simulation:
  mu: 398600.4418    # km³/s² (real Earth)
  dt_sim: 0.00833   # ~1/120 s

bodies:
  - name: earth
    prim_path: /World/Earth
    role: attractor
    spawn: false          # already in base_scene

  - name: sat1
    prim_path: /World/Sat1
    role: orbit
    spawn: true
    usd: NASA/Ingenuity/ingenuity.usd   # resolved by Isaac Sim asset root
    attractor: /World/Earth
    orbit:
      type: circular
      radius: 6778.0      # km
      plane: xy

  - name: sat2
    prim_path: /World/Sat2
    role: orbit
    spawn: true
    usd: NASA/Ingenuity/ingenuity.usd
    attractor: /World/Earth
    orbit:
      type: elements      # classical orbital elements
      a: 6778.0
      e: 0.01
      inc: 28.5
      raan: 0.0
      argp: 0.0
      nu: 45.0

  - name: sat3
    prim_path: /World/Sat3
    role: cw              # Clohessy-Wiltshire deputy (not yet implemented)
    spawn: true
    usd: NASA/Ingenuity/ingenuity.usd
    attractor: /World/Earth
    chief: /World/Sat1
    orbit:
      type: cw_relative
      dr: [1.0, 0.0, 0.5]
      dv: [0.0, 0.001, 0.0]
```

Body roles: `attractor` (anchor, not simulated), `orbit` (RK4 two-body), `cw` (Clohessy-Wiltshire, not yet implemented).

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

These are available once `isaacsim.ros2.sim_control` is enabled:

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

Install: `sudo apt install ros-jazzy-simulation-interfaces`

---

## Known Issues & Limitations

**Satellite spawning** — `/spawn_entity` requires a valid USD file URI resolvable by Isaac Sim's asset root. The Ingenuity asset path (`NASA/Ingenuity/ingenuity.usd`) is resolved against Isaac Sim's cloud asset root. If Isaac Sim cannot reach the cloud, spawning will fail with code 103. Custom local USD files are the long-term solution.

**`ORBIT_WS` must be set** — if not set, the workspace root fallback resolves to the `config/` subdirectory, making `usd_files/` unreachable. Always export `ORBIT_WS` before launching.

**CW node not implemented** — bodies with `role: cw` are skipped at launch with a warning. Clohessy-Wiltshire proximity operations are planned.

**Scale** — orbit state is in km, Isaac Sim scene units depend on the loaded USD. The `scale` parameter in `isaac_bridge_node` defaults to `1.0` (km = scene units). If your scene is in meters, set `scale: 1000.0` in `orbit.launch.py`.

**Simulation must be paused before `/load_world`** — if Isaac Sim is already playing when the launch file runs, the scene loader will stop the simulation first before loading.

---

## Development Notes

**Two Python environments are required:**

| Environment | Python | Used for |
|---|---|---|
| Docker build output | 3.11 | Isaac Sim internal ROS2 bridge, custom message `.so` files |
| System ROS2 Jazzy | 3.12 | `orbit_vel_node`, `isaac_bridge_node`, `scene_loader_node` |

DDS handles transport between them — Python version mismatch only affects direct import of message type support libraries, not over-the-wire communication.

**Rebuilding after node changes:**

```bash
cd jazzy_ws
colcon build --packages-select sli
source install/setup.bash
```

**Rebuilding after message changes** (requires Docker):

```bash
sudo ./build_ros.sh -d jazzy -v 24.04
colcon build --packages-select orbit_interfaces  # also rebuild host-side
```