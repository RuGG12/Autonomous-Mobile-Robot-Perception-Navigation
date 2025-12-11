from ament_index_python.packages import get_package_share_directory
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_config.clearpath_config import ClearpathConfig
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic',
                          default_value='scan',
                          description='Topic for the LaserScan')
]

def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    scan_topic = LaunchConfiguration('scan_topic')

    config = read_yaml(setup_path.perform(context) + 'robot.yaml')
    clearpath_config = ClearpathConfig(config)
    namespace = ''
    platform_model = clearpath_config.platform.get_platform_model()

    file_parameters = PathJoinSubstitution([
        pkg_clearpath_nav2_demos,
        'config',
        platform_model,
        'slam.yaml'])

    rewritten_parameters = RewrittenYaml(
        source_file=file_parameters,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[
          rewritten_parameters,
          {'use_sim_time': use_sim_time},
          {'scan_topic': scan_topic}
        ],
        remappings=[
          ('/tf', 'tf'),
          ('/tf_static', 'tf_static'),
          ('/scan', scan_topic),
          ('/map', 'map'),
          ('/map_metadata', 'map_metadata'),
        ]
    )

    return [slam]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld