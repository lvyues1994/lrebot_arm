"""The reBot B601-RS in MuJoCo, served by the larm runtime, with robot_state_publisher and RViz.

    ros2 launch rebot_b601 sim.launch.py [rviz:=false] [real_time_factor:=1.0]
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("rebot_b601"))
    generated = share / "generated"
    # RViz resolves meshes by URI, not relative to the URDF file.
    urdf = (generated / "urdf" / "rebot_b601_rs.urdf").read_text()
    robot_description = urdf.replace('filename="../meshes/', f'filename="file://{generated}/meshes/')

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0"),
        Node(
            package="larm",
            executable="larm_runtime_node",
            name="larm_runtime",
            output="screen",
            parameters=[{
                "profile": str(share / "config" / "rebot_b601_rs.yaml"),
                "backend": "mujoco",
                "real_time_factor": ParameterValue(LaunchConfiguration("real_time_factor"), value_type=float),
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "rebot.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
