from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pathplanning = Node(
        package='cf_pathplanning',
        executable='pathplanning',
        output='screen',
        parameters=[{

        }]
    )

    return LaunchDescription(
        [
            pathplanning
        ]
    )