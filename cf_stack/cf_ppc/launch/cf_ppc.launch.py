from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ppc = Node(
        package='cf_ppc',
        executable='ppc',
        output='screen',
        parameters=[{

        }]
    )

    return LaunchDescription(
        [
            ppc
        ]
    )