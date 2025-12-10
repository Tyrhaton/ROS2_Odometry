from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Serial IMU Lifecycle Node
    serial_imu_node = LifecycleNode(
        package='imu_sensor_pkg',
        executable='serial_imu_node',
        name='serial_imu_node',
        namespace='',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'publish_rate_ms': 100
        }],
        output='screen'
    )

    # Database Subscriber Node
    database_node = Node(
        package='imu_sensor_pkg',
        executable='imu_database_node',
        name='imu_database_node',
        parameters=[{
            'database_path': 'imu_data.db'
        }],
        output='screen'
    )

    # Configure transition
    configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # When configured, activate
    activate_trans_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=serial_imu_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: True,
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        serial_imu_node,
        database_node,
        # Delay configure to give node time to start
        TimerAction(
            period=2.0,
            actions=[configure_trans_event]
        ),
        activate_trans_event
    ])
