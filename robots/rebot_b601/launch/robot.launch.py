"""The physical reBot B601-RS on RobStride motors over SocketCAN, served by the larm runtime.

    ros2 launch rebot_b601 robot.launch.py [backend:=robstride_simulated] [rt_priority:=80] [rviz:=true]

Bring the CAN interface up first: sudo ip link set can0 up type can bitrate 1000000
backend:=robstride_simulated runs the same driver against simulated motors, without hardware.
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
        DeclareLaunchArgument("backend", default_value="robstride"),
        DeclareLaunchArgument("rt_priority", default_value="0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="larm",
            executable="larm_runtime_node",
            name="larm_runtime",
            output="screen",
            parameters=[{
                "profile": str(share / "config" / "rebot_b601_rs.yaml"),
                "backend": LaunchConfiguration("backend"),
                "rt_priority": ParameterValue(LaunchConfiguration("rt_priority"), value_type=int),
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "rebot.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
