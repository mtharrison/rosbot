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

# Launches:
# Lidar
# Photo
# Motor
# IMU
# Microros agent

def generate_launch_description():
    
    lidar = LifecycleNode(    
        namespace = '',
        package = 'ldlidar_node',
        executable = 'ldlidar_node',
        name = 'ldlidar_node',
        parameters=[{
            "general.debug_mode": False,
            "lidar.frame_id": 'laser_link',
        }]
    )
    
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    
    return LaunchDescription([
         Node(
             package='photo',
             executable='capture',
             name='photo'
         ),
        Node(
            package='motor',
            executable='motor',
            name='motor',
            output="screen",
            parameters=[
                {"pid_kp": 0.6},
                {"pid_ki": 0.8},
                {"pid_kd": 0.5}
            ]
        ),
        Node(
            package='imu',
            executable='publisher',
            name='imu'
        ),
        lidar,
        activate_event,
        configure_event,
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', '8888']
        ),
    ])
