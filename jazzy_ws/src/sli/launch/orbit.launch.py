from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

'''
EXAMPLE LAUNCH FILE

SEE scripts/run_orbit.py

'''
def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                '/home/sgk/miniforge3/envs/isaaclab311/bin/python3.11',
                os.path.expanduser('~/SLI-5.1-cubesat_ws/IsaacSim-5.1-ROS2_workspace/jazzy_ws/src/sli/scripts/run_orbit.py'),
            ],
            output='screen',
            additional_env={
                'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),
                'ROS_DISTRO': 'jazzy',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            }
        )
    ])