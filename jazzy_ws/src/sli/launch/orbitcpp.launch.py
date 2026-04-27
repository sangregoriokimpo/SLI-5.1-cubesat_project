import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def _resolve_config_path(scene_arg: str) -> str:
    if os.path.isabs(scene_arg):
        return scene_arg
    pkg_dir = os.path.dirname(__file__)
    candidate = os.path.join(pkg_dir, "..", "config", scene_arg)
    if os.path.exists(candidate):
        return os.path.abspath(candidate)
    if os.path.exists(scene_arg):
        return os.path.abspath(scene_arg)
    raise FileNotFoundError(f"Cannot find config file: {scene_arg}")


def _load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _orbit_params(body: dict, sim_cfg: dict) -> dict:
    orbit = body.get("orbit", {})
    return {
        "mu":           sim_cfg.get("mu",           398600.4418),
        "dt_sim":       sim_cfg.get("dt_sim",        0.00833),
        "publish_rate": sim_cfg.get("publish_rate",  30.0),
        "prim_path":    body["prim_path"],
        "attractor":    body.get("attractor", "/World/Earth"),
        "orbit_type":   orbit.get("type", "circular"),
        "radius":       float(orbit.get("radius", 6778.0)),
        "plane":        orbit.get("plane", "xy"),
        "a":            float(orbit.get("a",    6778.0)),
        "e":            float(orbit.get("e",    0.0)),
        "inc":          float(orbit.get("inc",  0.0)),
        "raan":         float(orbit.get("raan", 0.0)),
        "argp":         float(orbit.get("argp", 0.0)),
        "nu":           float(orbit.get("nu",   0.0)),
    }



def generate_nodes(context, *args, **kwargs):
    scene_arg   = LaunchConfiguration("scene").perform(context)
    config_path = _resolve_config_path(scene_arg)
    cfg         = _load_config(config_path)

    sim_cfg = cfg.get("simulation", {})
    bodies  = cfg.get("bodies", [])

    nodes = []

    nodes.append(
        Node(
            package="sli",
            executable="scene_loader_node",
            name="scene_loader",
            output="screen",
            arguments=[config_path],
        )
    )

    for body in bodies:
        role = body.get("role", "")
        name = body["name"]

        if role == "attractor":
            continue

        if role == "orbit":
            nodes.append(
                Node(
                    package="sli_gnc",
                    executable="orbit_vel_node",
                    name=f"orbit_vel_{name}",
                    namespace=name,
                    output="screen",
                    parameters=[_orbit_params(body, sim_cfg)],
                )
            )

        elif role == "cw":
            chief_prim  = body.get("chief", "")
            chief_ns    = chief_prim.split("/")[-1].lower()
            chief_topic = f"/{chief_ns}/orbit_state"
            orbit       = body.get("orbit", {})
            dr          = orbit.get("dr", [0.0, 0.0, 0.0])
            dv          = orbit.get("dv", [0.0, 0.0, 0.0])
            nodes.append(
                Node(
                    package="sli_gnc",
                    executable="cw_node",
                    name=f"cw_{name}",
                    namespace=name,
                    output="screen",
                    parameters=[{
                        "mu":           sim_cfg.get("mu",           398600.4418),
                        "dt_sim":       sim_cfg.get("dt_sim",        0.00833),
                        "publish_rate": sim_cfg.get("publish_rate",  500.0),
                        "prim_path":    body["prim_path"],
                        "chief_topic":  chief_topic,
                        "dr_x":         float(dr[0]),
                        "dr_y":         float(dr[1]),
                        "dr_z":         float(dr[2]),
                        "dv_x":         float(dv[0]),
                        "dv_y":         float(dv[1]),
                        "dv_z":         float(dv[2]),
                    }],
                )
            )

        if role in ("orbit", "cw"):
            nodes.append(
                Node(
                    package="sli",
                    executable="isaac_bridge_node",
                    name=f"isaac_bridge_{name}",
                    namespace=name,
                    output="screen",
                    parameters=[{
                        "prim_path":      body["prim_path"],
                        "state_topic":    "orbit_state",
                        "attractor_path": body.get("attractor", "/World/Earth"),
                        "scale":          float(body.get("scale", 1.0)),
                    }],
                )
            )

    return nodes




def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value="orbit_scene.yaml",
            description="Path to orbit_scene.yaml config file",
        ),
        OpaqueFunction(function=generate_nodes),
    ])