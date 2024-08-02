import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

import xacro


def generate_launch_description():

    # Path Variables
    crater_bot_path = os.path.join(get_package_share_directory("crater_bot"))

    # Launch Robot State Publisher
    publisher_robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(crater_bot_path, "launch"), "/rsp.launch.py"]
        )
    )

    robot_controllers = os.path.join(crater_bot_path, "config", "controllers.yaml")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )


    return LaunchDescription([
        publisher_robot_state,
        control_node,
        robot_controller_spawner,
        # joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ])


    # Check if we're told to use sim time
    # use_sim_time = LaunchConfiguration("use_sim_time")

    # Process the URDF file
    # pkg_path = os.path.join(get_package_share_directory("crater_bot"))
    # xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    # robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    # params = {
    #     "robot_description": robot_description_config.toxml(),
    #     "use_sim_time": use_sim_time,
    # }
    # node_robot_state_publisher = Node(
    # package="robot_state_publisher",
    # executable="robot_state_publisher",
    # output="screen",
    # parameters=[params],
    # )

    #  # Launch!
    # return LaunchDescription(
    #     [
    #         DeclareLaunchArgument(
    #             "use_sim_time",
    # default_value="false",
    # description="Use sim time if true",
    #         ),
    #         node_robot_state_publisher,
    #     ]
    # )
