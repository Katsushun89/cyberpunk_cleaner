import os

import launch
import launch.actions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription()

    config_param_dir = launch.substitutions.LaunchConfiguration(
        'config_param_dir',
        default=os.path.join(
            get_package_share_directory('ca_driver'),
            'config',
            'default.yaml'))

    ca_driver = launch_ros.actions.LifecycleNode(
        node_name='ca_driver',
        package='ca_driver',
        node_executable='ca_driver',
        parameters=[config_param_dir],
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(ca_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    to_inactive = launch.actions.EmitEvent(
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

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    
    ld.add_action(ca_driver)
    ld.add_action(to_inactive)
    return ld

