from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    package_path = get_package_share_path('frontier_simulation')
    param_file_path = str(package_path / 'config/frontier_params.yaml')

    # expose the parameter to the launch command line
    param_file_arg = DeclareLaunchArgument('config_file',
                                           default_value=param_file_path)
    launch_description.add_action(param_file_arg)

    node1 = Node(executable='hippo_visualization',
                 package='frontier_simulation',
                 parameters=[{
                     'vehicle_name': LaunchConfiguration('vehicle_name')
                 },
                             LaunchConfiguration('config_file')])

    # Single process hosting both the "map_updater" and "path_planning_manager" nodes,
    # so they can share the same Map instance without serialization.
    node2 = Node(executable='map_and_pathplanning',
                 package='frontier_simulation',
                 parameters=[{
                     'vehicle_name': LaunchConfiguration('vehicle_name')
                 },
                             LaunchConfiguration('config_file')])

    node3 = Node(executable='point_cloud_processor',
                 package='frontier_simulation',
                 parameters=[{
                     'vehicle_name': LaunchConfiguration('vehicle_name')
                 },
                             LaunchConfiguration('config_file')])

    node4 = Node(executable='path_follower',
                 package='frontier_simulation',
                 parameters=[{
                     'vehicle_name': LaunchConfiguration('vehicle_name')
                 },
                             LaunchConfiguration('config_file')])

    node5 = Node(executable='velocity_control',
                 package='frontier_simulation',
                 parameters=[{
                     'vehicle_name': LaunchConfiguration('vehicle_name')
                 },
                             LaunchConfiguration('config_file')])

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        node1,
        node2,
        node3,
        node4,
        node5,
    ])
    launch_description.add_action(group)

    return launch_description
