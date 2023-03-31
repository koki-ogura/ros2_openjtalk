import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg


def generate_launch_description():
    ld = launch.LaunchDescription()

    device_name = launch.substitutions.LaunchConfiguration('device_name', default='default')

    node = launch_ros.actions.LifecycleNode(
        name='ros2_openjtalk_node', namespace='',
        package='ros2_openjtalk', executable='ros2_openjtalk_node', output='screen',
        parameters=[{'device_name':device_name}],
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    to_active = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )
    
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="<< Unconfigured >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node, 
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="<< Inactive >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    from_active_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node, 
            goal_state='active',
            entities=[
                launch.actions.LogInfo(msg="<< Active >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                )),
            ],
        )
    )

    from_inactive_to_finalized = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node, 
            start_state = 'deactivating',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="<< Inactive >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_INACTIVE_SHUTDOWN,
                )),
            ],
        )
    )
    
    from_finalized_to_exit = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node, goal_state='finalized',
            entities=[
                launch.actions.LogInfo(msg="<< Finalized >>"),
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        )
    )
    
    #ld.add_action(from_unconfigured_to_inactive)
    #ld.add_action(from_inactive_to_active)
    #ld.add_action(from_active_to_inactive)
    #ld.add_action(from_inactive_to_finalized)
    ld.add_action(from_finalized_to_exit)
    
    ld.add_action(node)
    ld.add_action(to_inactive)
    ld.add_action(to_active)

    return ld
