from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    drone = Node(
        package='cf_drone',
        executable='drone',
        output='screen',
        parameters=[{

        }]
    )

    return LaunchDescription(
        [
            drone
        ]
    )