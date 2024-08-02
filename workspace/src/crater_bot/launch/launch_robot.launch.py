import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

import xacro

# Constants
CRATER_BOT_PATH = os.path.join(get_package_share_directory("crater_bot"))


# Util Functions
def load_launchfile_description(filename: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(CRATER_BOT_PATH, "launch"), filename]
        )
    )


def create_controller_node(controller_name: str):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
        ],
    )


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Publishers
    publisher_robot_state = load_launchfile_description("/rsp.launch.py")

    # Configure ros2_control (controller, spawners)
    yaml_controllers = os.path.join(CRATER_BOT_PATH, "config", "controllers.yaml")
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[yaml_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_controller_spawner = create_controller_node("diff_cont")
    joint_state_broadcaster_spawner = create_controller_node("joint_state_broadcaster")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            publisher_robot_state,
            control_node,
            robot_controller_spawner,
            joint_state_broadcaster_spawner,
        ]
    )