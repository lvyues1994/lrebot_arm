"""The reBot B601-RS with larm studio; closing the studio ends the launch.

    ros2 launch rebot_b601 studio.launch.py [backend:=mujoco|robstride|robstride_simulated] [rviz:=false]
                                            [real_time_factor:=1.0] [rt_priority:=0]

backend:=mujoco (the default) simulates the arm in MuJoCo; backend:=robstride drives the physical arm (bring
can0 up first, see robot.launch.py); backend:=robstride_simulated runs the RobStride driver against
simulated motors. real_time_factor applies to MuJoCo, rt_priority to the RobStride backends.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rebot_b601"))
    backend = LaunchConfiguration("backend")
    simulated = PythonExpression(["'", backend, "' == 'mujoco'"])
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mujoco",
                              choices=["mujoco", "robstride", "robstride_simulated"]),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("rt_priority", default_value="0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "sim.launch.py")),
            launch_arguments={
                "rviz": LaunchConfiguration("rviz"),
                "real_time_factor": LaunchConfiguration("real_time_factor"),
            }.items(),
            condition=IfCondition(simulated),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "robot.launch.py")),
            launch_arguments={
                "backend": backend,
                "rviz": LaunchConfiguration("rviz"),
                "rt_priority": LaunchConfiguration("rt_priority"),
            }.items(),
            condition=UnlessCondition(simulated),
        ),
        Node(
            package="larm",
            executable="larm_studio",
            arguments=["--profile", str(share / "config" / "rebot_b601_rs.yaml")],
            output="screen",
            on_exit=Shutdown(reason="larm studio closed"),
        ),
    ])
