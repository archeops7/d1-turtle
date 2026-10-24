"""Launch Gazebo with a selected room layout, TurtleBot3 Waffle Pi, and the d1 driver node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3 = get_package_share_directory("turtlebot3_gazebo")
    gazebo = get_package_share_directory("gazebo_ros")
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    arguments = [
        DeclareLaunchArgument("world", description="World SDF to load"),
        DeclareLaunchArgument("spawn_x", description="Robot spawn x in meters"),
        DeclareLaunchArgument("spawn_y", description="Robot spawn y in meters"),
        DeclareLaunchArgument("spawn_yaw", description="Robot spawn yaw in radians"),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world, "use_sim_time": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo, "launch", "gzclient.launch.py")),
    )
    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3, "launch", "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "waffle_pi",
            "-file", os.path.join(tb3, "models", "turtlebot3_waffle_pi", "model.sdf"),
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.01",
            "-Y", spawn_yaw,
        ],
        output="screen",
    )

    driver_env = {}
    libcuda = os.environ.get("D1_LIBCUDA")
    if libcuda:
        driver_env["LD_PRELOAD"] = libcuda

    driver = ExecuteProcess(
        cmd=[
            "python3",
            "/opt/d1_tb3/d1_driver.py",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        additional_env=driver_env,
        output="screen",
    )
    return LaunchDescription([*arguments, gzserver, gzclient, robot_state, spawn, driver])
