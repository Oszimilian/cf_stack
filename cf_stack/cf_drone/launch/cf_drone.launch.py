from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    id_arg = DeclareLaunchArgument(
        'id',
        default_value='0',
        description='id'
    )

    id = LaunchConfiguration('id')

    drone = Node(
        package='cf_drone',
        executable='drone',
        output='screen',
        parameters=[{
            'id': id
        }]
    )

    return LaunchDescription(
        [
            drone,
            id_arg
        ]
    )