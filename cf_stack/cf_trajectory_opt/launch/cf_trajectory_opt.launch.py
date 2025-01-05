from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    trajectory_opt = Node(
        package='cf_trajectory_opt',
        executable='trajectory_opt',
        output='screen',
        parameters=[{

        }]
    )

    return LaunchDescription(
        [
            trajectory_opt
        ]
    )