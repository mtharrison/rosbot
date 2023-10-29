from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
import lifecycle_msgs.msg
from launch.actions import LogInfo
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(nav2_bringup_dir + '/launch/navigation_launch.py'))
    
    # slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    # slam_toolbox_launch = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(slam_toolbox_dir + '/launch/online_async_launch.py'))
    
    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory(
                'wave_bot_description')
            + '/config/slam.yaml',
            {'use_sim_time': False}
        ],
        
    )
    
    foxbridge_dir = get_package_share_directory('foxglove_bridge')
    foxbridge_launch = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.FrontendLaunchDescriptionSource(foxbridge_dir + '/launch/foxglove_bridge_launch.xml'))
    
    pkg_share = launch_ros.substitutions.FindPackageShare(package='wave_bot_description').find('wave_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/wave_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': False}]
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
        slam_node,
        nav2_bringup_launch,
        foxbridge_launch
    ])
