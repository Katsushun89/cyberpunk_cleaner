import os

import launch
import launch.actions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import EmitEvent
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    #config_filepath = LaunchConfiguration('config_filepath')

    declare_joy_config = DeclareLaunchArgument('joy_config', default_value='gamepad')

    declare_joy_dev = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')

    declare_joy_config_filepath = DeclareLaunchArgument('joy_config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('cyberpunk_cleaner'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')])

#    joy_config_filepath = LaunchConfiguration(
#        'joy_config_filepath',
#        default=os.path.join(
#            get_package_share_directory('cyberpunk_cleaner'),
#            'config',
#            'gamepad.config.yaml'))

    joy_node = Node(
        package='joy',
        node_executable='joy_node',
        parameters=[{'dev': joy_dev,
                    'deadzone': 0.3,
                    'autorepeat_rate': 20.0,}])

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy', node_executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[LaunchConfiguration('joy_config_filepath')])

    ca_config_filepath = LaunchConfiguration(
        'ca_config_filepath',
        default=os.path.join(
            get_package_share_directory('cyberpunk_cleaner'),
            'config',
            'ca_driver.default.yaml'))

    ca_driver = LifecycleNode(
        node_name='ca_driver',
        package='ca_driver',
        node_executable='ca_driver',
        parameters=[ca_config_filepath],
        output='screen')

    to_inactive = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(ca_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    to_inactive = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(ca_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=ca_driver, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(ca_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=ca_driver,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(ca_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld.add_action(declare_joy_dev)
    ld.add_action(declare_joy_config)
    ld.add_action(declare_joy_config_filepath)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    
    ld.add_action(ca_driver)
    ld.add_action(to_inactive)
    return ld

