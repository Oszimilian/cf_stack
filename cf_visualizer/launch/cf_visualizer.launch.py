from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    x_segment_size_arg = DeclareLaunchArgument(
        'x_segment_size',
        default_value='0.2',
        description='x size of a segment'
    )

    y_segment_size_arg = DeclareLaunchArgument(
        'y_segment_size',
        default_value='0.2',
        description='y size of a segment'
    )

    x_segment_size = LaunchConfiguration('x_segment_size')
    y_segment_size = LaunchConfiguration('y_segment_size')


    visualizer = Node(
        package='cf_visualizer',
        executable='visualizer',
        output='screen',
        parameters=[{
            'x_segment_size' : x_segment_size,
            'y_segment_size' : y_segment_size
        }]
    )


    return LaunchDescription(
        [
            x_segment_size_arg,
            y_segment_size_arg,
            visualizer
        ]
    )