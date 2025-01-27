from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    velocity_planner = Node(
        package='cf_velocity_planner',
        executable='velocity_planner',
        output='screen',
        parameters=[{

        }]
    )

    return LaunchDescription(
        [
            velocity_planner
        ]
    )