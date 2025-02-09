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

    sampling = Node(
        package='cf_sampling',
        executable='sampling',
        output='screen',
        parameters=[{
            'id': id
        }]
    )

    return LaunchDescription(
        [
            sampling,
            id_arg
        ]
    )