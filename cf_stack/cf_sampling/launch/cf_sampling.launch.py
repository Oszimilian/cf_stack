from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    sampling = Node(
        package='cf_sampling',
        executable='sampling',
        output='screen',
        parameters=[{

        }]
    )

    return LaunchDescription(
        [
            sampling
        ]
    )