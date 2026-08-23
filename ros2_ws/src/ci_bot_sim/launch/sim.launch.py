import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    description_share = get_package_share_directory(
        "ci_bot_description"
    )

    sim_share = get_package_share_directory(
        "ci_bot_sim"
    )

    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )

    xacro_file = os.path.join(
        description_share,
        "urdf",
        "ci_bot.urdf.xacro",
    )

    world_file = os.path.join(
        sim_share,
        "worlds",
        "ci_test_world.sdf",
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r -s -v 2 {world_file}",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    velocity_guard = Node(
        package="ci_bot_control",
        executable="velocity_guard_node",
        name="velocity_guard",
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "ci_bot",
                    "-z",
                    "0.15",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            velocity_guard,
            bridge,
            spawn_robot,
        ]
    )
