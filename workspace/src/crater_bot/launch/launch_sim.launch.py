import os
import xacro
from ament_index_python.packages import get_package_share_directory

from pathlib import Path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch.actions import (
    RegisterEventHandler,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    AppendEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    # Path Variables
    crater_bot_path = os.path.join(get_package_share_directory("crater_bot"))
    gazebo_sim_path = get_package_share_directory("ros_gz_sim")

    # Launch Descriptions
    publisher_robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(crater_bot_path, "launch"), "/rsp.launch.py"]
        )
    )

    # Gazebo
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(crater_bot_path, "worlds")
    )
    gazebo_arguments = LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_world", default_value="empty.world", description="Gz sim World"
            ),
        ]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_sim_path, "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", [LaunchConfiguration("gazebo_world"), " -v4", " -r"])
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description", "-entity", "crater_bot"],
        output="screen",
    )

    # ROS2 Control
    robot_controllers = os.path.join(crater_bot_path, "config", "controllers.yaml")
    
    control_node = Node( package="controller_manager", executable="ros2_control_node", parameters=[
            robot_controllers,
        ], output="both",
    )


    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["botwheel_explorer", "--controller-manager", "/controller_manager"],
    # )

    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    # ROS Visualizer
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )



    res = LaunchDescription()
    # res.add_entity(control_node)
    # res.add_entity(joint_state_broadcaster_spawner)
    # res.add_entity(robot_controller_spawner)
    res.add_entity(publisher_robot_state)
    res.add_entity(gazebo_resource_path)
    res.add_entity(gazebo_arguments)
    res.add_entity(gazebo)
    res.add_entity(gz_spawn_entity)

    # res.add_entity(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)
    # res.add_entity(publisher_joint_state)
    # res.add_entity(gz_spawn_entity)
    # res.add_entity(rviz)
    return res
