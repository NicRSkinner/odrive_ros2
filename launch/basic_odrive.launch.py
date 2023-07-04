from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    odrive_node = Node(
        package="odrive_ros2",
        executable="odrive"
    )

    ld.add_action(odrive_node)

    return ld
