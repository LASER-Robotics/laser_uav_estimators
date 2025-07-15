from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import RegisterEventHandler, EmitEvent

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg

def generate_launch_description():
    #Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'quadrotor_model_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_state_estimator'),
                                                'params', 'quadrotor_model.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

    #Initialize arguments
    namespace = LaunchConfiguration('namespace')
    quadrotor_model_file = LaunchConfiguration('quadrotor_model_file')

    quadrotor_model_lifecycle_node = LifecycleNode(
        package='laser_state_estimator',
        executable='quadrotor_model',
        name='quadrotor_model',
        namespace=namespace,
        output='screen',
        parameters=[quadrotor_model_file],
    )

    event_handlers = []

    event_handlers.append(
        #Right after the node starts, make it take the 'configure' transition.
        RegisterEventHandler(
            OnProcessStart(
                target_action=quadrotor_model_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(quadrotor_model_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=quadrotor_model_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(quadrotor_model_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

    #Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

    #Add client node
    ld.add_action(quadrotor_model_lifecycle_node)

    #Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
