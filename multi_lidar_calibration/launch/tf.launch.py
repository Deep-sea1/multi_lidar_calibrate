from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch_ros.actions import Node


def generate_launch_description():
    # Set use_sim_time parameter
    use_sim_time = SetParameter(name='use_sim_time', value=True)

    # Static transform publishers
    fl_fh_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fl_fh',
        arguments=[
            '--x', '0.047514',
            '--y', '-1.61584',
            '--z', '-1.10422',
            '--yaw', '0.0154157',
            '--pitch', '0.00829276',
            '--roll', '0.00490281',
            '--frame-id', 'fh',
            '--child-frame-id', 'fl'
        ]
    )

    lf_fh_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lf_fh',
        arguments=[
            '--x', '1.00785',
            '--y', '-0.443069',
            '--z', '-0.471545',
            '--yaw', '-0.0816414',
            '--pitch', '0.0717998',
            '--roll', '1.36427',
            '--frame-id', 'fh',
            '--child-frame-id', 'lf'
        ]
    )

    rf_fh_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rf_fh',
        arguments=[
            '--x', '-0.993104',
            '--y', '-0.465174',
            '--z', '-0.470066',
            '--yaw', '3.06347',
            '--pitch', '3.1134',
            '--roll', '1.68516',
            '--frame-id', 'fh',
            '--child-frame-id', 'rf'
        ]
    )

    lr_fh_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lr_fh',
        arguments=[
            '--x', '0.814268',
            '--y', '3.60834',
            '--z', '-1.09568',
            '--yaw', '-0.00206728',
            '--pitch', '-0.0096379',
            '--roll', '1.53946',
            '--frame-id', 'fh',
            '--child-frame-id', 'lr'
        ]
    )

    rr_fh_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rr_fh',
        arguments=[
            '--x', '-0.993823',
            '--y', '3.63243',
            '--z', '-1.06587',
            '--yaw', '3.13014',
            '--pitch', '3.12154',
            '--roll', '1.61811',
            '--frame-id', 'fh',
            '--child-frame-id', 'rr'
        ]
    )

    return LaunchDescription([
        use_sim_time,
        fl_fh_tf,
        lf_fh_tf,
        rf_fh_tf,
        lr_fh_tf,
        rr_fh_tf,
    ])
