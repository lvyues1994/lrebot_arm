"""The reBot B601-RS in MuJoCo with larm studio; closing the studio ends the launch.

    ros2 launch rebot_b601 studio.launch.py [rviz:=false] [real_time_factor:=1.0]
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rebot_b601"))
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "sim.launch.py")),
            launch_arguments={
                "rviz": LaunchConfiguration("rviz"),
                "real_time_factor": LaunchConfiguration("real_time_factor"),
            }.items(),
        ),
        Node(
            package="larm",
            executable="larm_studio",
            arguments=["--profile", str(share / "config" / "rebot_b601_rs.yaml")],
            output="screen",
            on_exit=Shutdown(reason="larm studio closed"),
        ),
    ])
