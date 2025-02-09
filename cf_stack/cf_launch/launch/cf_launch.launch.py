from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Erstelle Launch-Argumente
    input_grid_path_arg = DeclareLaunchArgument(
        'input_grid_path',
        default_value='default.json',
        description='Path to the json-file which describes the grid of the arena'
    )

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

    x_size_offset_arg = DeclareLaunchArgument(
        'x_size_offset',
        default_value='-0.9',
        description='Grid begins after x size offset'
    )

    y_size_offset_arg = DeclareLaunchArgument(
        'y_size_offset',
        default_value='-0.9',
        description='Grid begins after y size offset'
    )

    id_arg = DeclareLaunchArgument(
        'id',
        default_value='0',
        description='id of the drone and the safeflie'
    )


    grid_launch_file = os.path.join(
        get_package_share_directory('cf_grid'),
        'launch',
        'cf_grid.launch.py'
    )

    pathplanner_launch_file = os.path.join(
        get_package_share_directory('cf_pathplanning'),
        'launch',
        'cf_pathplanning.launch.py'
    )

    ppc_launch_file = os.path.join(
        get_package_share_directory('cf_ppc'),
        'launch',
        'cf_ppc.launch.py'
    )

    trajectory_opt_launch_file = os.path.join(
        get_package_share_directory('cf_trajectory_opt'),
        "launch", 
        "cf_trajectory_opt.launch.py"
    )

    drone_launch_file = os.path.join(
        get_package_share_directory('cf_drone'),
        "launch", 
        "cf_drone.launch.py"
    )

    visualizer_launch_file = os.path.join(
        get_package_share_directory('cf_visualizer'),
        'launch',
        'cf_visualizer.launch.py'
    )

    velocity_planner_launch_file = os.path.join(
        get_package_share_directory('cf_velocity_planner'),
        'launch',
        'cf_velocity_planner.launch.py'
    )

    sampling_launch_file = os.path.join(
        get_package_share_directory('cf_sampling'),
        'launch',
        'cf_sampling.launch.py'
    )

    grid_launch = IncludeLaunchDescription(grid_launch_file, launch_arguments={
        'input_grid_path': LaunchConfiguration('input_grid_path'),
        'x_segment_size': LaunchConfiguration('x_segment_size'),
        'y_segment_size': LaunchConfiguration('y_segment_size'),
        'x_size_offset': LaunchConfiguration('x_size_offset'),
        'y_size_offset': LaunchConfiguration('y_size_offset'),
        'id_drone': LaunchConfiguration('id')
    }.items())

    pathplanner_launch = IncludeLaunchDescription(pathplanner_launch_file)

    trayjectory_launch = IncludeLaunchDescription(trajectory_opt_launch_file)

    ppc_launch = IncludeLaunchDescription(ppc_launch_file)

    drone_launch = IncludeLaunchDescription(drone_launch_file, launch_arguments={
        'id': LaunchConfiguration('id')
    }.items())


    velocity_planner_launch = IncludeLaunchDescription(velocity_planner_launch_file)

    visualizer_launch = IncludeLaunchDescription(visualizer_launch_file, launch_arguments={
        'x_segment_size': LaunchConfiguration('x_segment_size'),
        'y_segment_size': LaunchConfiguration('y_segment_size')
    }.items())

    sampling_launch = IncludeLaunchDescription(sampling_launch_file, launch_arguments={
        'id': LaunchConfiguration('id')
    }.items())

    return LaunchDescription([
        id_arg,
        input_grid_path_arg,
        x_segment_size_arg,
        y_segment_size_arg,
        x_size_offset_arg,
        y_size_offset_arg,
        grid_launch,
        visualizer_launch,
        pathplanner_launch,
        trayjectory_launch,
        ppc_launch,
        drone_launch,
        velocity_planner_launch,
        sampling_launch
    ])