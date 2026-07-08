from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('gcs_planner')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'planner_params.yaml']),
        description='YAML file with planner_node parameters')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'gcs_planner.rviz']),
        description='RViz display config file')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz alongside the planner node')

    use_executor_arg = DeclareLaunchArgument(
        'use_executor', default_value='true',
        description='Launch the trajectory executor (moving velocity-vector viz) alongside the planner node')

    planner_node = Node(
        package='gcs_planner',
        executable='planner_node',
        name='planner',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    trajectory_executor_node = Node(
        package='gcs_planner',
        executable='trajectory_executor_node',
        name='trajectory_executor',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(LaunchConfiguration('use_executor')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        params_file_arg,
        rviz_config_arg,
        use_rviz_arg,
        use_executor_arg,
        planner_node,
        trajectory_executor_node,
        rviz_node,
    ])
