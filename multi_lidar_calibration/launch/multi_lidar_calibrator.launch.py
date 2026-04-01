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
        default_value='0.05',
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
        default_value='0.4',
        description='NDT resolution'
    )

    use_multi_resolution_arg = DeclareLaunchArgument(
        'use_multi_resolution',
        default_value='true',
        description='Enable coarse-to-fine NDT pyramid'
    )

    ndt_iterations_arg = DeclareLaunchArgument(
        'ndt_iterations',
        default_value='95',
        description='NDT maximum iterations'
    )

    max_fitness_score_arg = DeclareLaunchArgument(
        'max_fitness_score',
        default_value='1.0',
        description='Reject if NDT fitness exceeds this value'
    )

    max_correspondence_distance_arg = DeclareLaunchArgument(
        'max_correspondence_distance',
        default_value='1.0',
        description='Max correspondence distance used by robust scoring'
    )

    min_inlier_ratio_arg = DeclareLaunchArgument(
        'min_inlier_ratio',
        default_value='0.25',
        description='Reject if inlier ratio is lower than this threshold'
    )

    min_transform_probability_arg = DeclareLaunchArgument(
        'min_transform_probability',
        default_value='0.02',
        description='Reject if NDT transform probability is too low'
    )

    max_translation_jump_arg = DeclareLaunchArgument(
        'max_translation_jump',
        default_value='0.8',
        description='Reject if one-step translation jump is too large (meters)'
    )

    max_rotation_jump_deg_arg = DeclareLaunchArgument(
        'max_rotation_jump_deg',
        default_value='12.0',
        description='Reject if one-step rotation jump is too large (degrees)'
    )

    max_rejections_before_reset_arg = DeclareLaunchArgument(
        'max_rejections_before_reset',
        default_value='20',
        description='Reset guess to configured initial transform after this many rejections'
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
            'use_multi_resolution': LaunchConfiguration('use_multi_resolution'),
            'ndt_iterations': LaunchConfiguration('ndt_iterations'),
            'max_fitness_score': LaunchConfiguration('max_fitness_score'),
            'max_correspondence_distance': LaunchConfiguration('max_correspondence_distance'),
            'min_inlier_ratio': LaunchConfiguration('min_inlier_ratio'),
            'min_transform_probability': LaunchConfiguration('min_transform_probability'),
            'max_translation_jump': LaunchConfiguration('max_translation_jump'),
            'max_rotation_jump_deg': LaunchConfiguration('max_rotation_jump_deg'),
            'max_rejections_before_reset': LaunchConfiguration('max_rejections_before_reset'),
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
        use_multi_resolution_arg,
        ndt_iterations_arg,
        max_fitness_score_arg,
        max_correspondence_distance_arg,
        min_inlier_ratio_arg,
        min_transform_probability_arg,
        max_translation_jump_arg,
        max_rotation_jump_deg_arg,
        max_rejections_before_reset_arg,
        multi_lidar_calibrator_node,
    ])
