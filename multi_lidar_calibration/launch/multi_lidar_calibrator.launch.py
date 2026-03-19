from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('multi_lidar_calibrator')

    # Declare launch arguments
    points_parent_src_arg = DeclareLaunchArgument(
        'points_parent_src',
        default_value='/livox/lidar_192_168_1_139',
        description='Parent LiDAR point cloud topic'
    )

    points_child_src_arg = DeclareLaunchArgument(
        'points_child_src',
        default_value='/livox/lidar_192_168_1_118',
        description='Child LiDAR point cloud topic'
    )

    init_params_file_path_arg = DeclareLaunchArgument(
        'init_params_file_path',
        default_value=os.path.join(pkg_share, 'cfg', 'child_topic_list'),
        description='Path to initial parameters file'
    )

    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.1',
        description='Voxel size for downsampling'
    )

    ndt_epsilon_arg = DeclareLaunchArgument(
        'ndt_epsilon',
        default_value='0.1',
        description='NDT transformation epsilon'
    )

    ndt_step_size_arg = DeclareLaunchArgument(
        'ndt_step_size',
        default_value='0.1',
        description='NDT step size'
    )

    ndt_resolution_arg = DeclareLaunchArgument(
        'ndt_resolution',
        default_value='0.5',
        description='NDT resolution'
    )

    ndt_iterations_arg = DeclareLaunchArgument(
        'ndt_iterations',
        default_value='100',
        description='NDT maximum iterations'
    )

    # Node
    multi_lidar_calibrator_node = Node(
        package='multi_lidar_calibrator',
        executable='multi_lidar_calibrator',
        name='lidar_calibrator',
        output='screen',
        parameters=[{
            'points_parent_src': LaunchConfiguration('points_parent_src'),
            'points_child_src': LaunchConfiguration('points_child_src'),
            'init_params_file_path': LaunchConfiguration('init_params_file_path'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'ndt_epsilon': LaunchConfiguration('ndt_epsilon'),
            'ndt_step_size': LaunchConfiguration('ndt_step_size'),
            'ndt_resolution': LaunchConfiguration('ndt_resolution'),
            'ndt_iterations': LaunchConfiguration('ndt_iterations'),
        }]
    )

    return LaunchDescription([
        points_parent_src_arg,
        points_child_src_arg,
        init_params_file_path_arg,
        voxel_size_arg,
        ndt_epsilon_arg,
        ndt_step_size_arg,
        ndt_resolution_arg,
        ndt_iterations_arg,
        multi_lidar_calibrator_node,
    ])
