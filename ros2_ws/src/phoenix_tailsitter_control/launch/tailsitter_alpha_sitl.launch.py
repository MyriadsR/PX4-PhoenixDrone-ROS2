"""Launch the parallel full-alpha PhoenixDrone SITL/controller path."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PX4_ROOT = Path('/home/zr/PX4-PhoenixDrone-ROS2')
AGENT = Path(
    '/home/zr/px4_ros_uxrce_dds_ws/src/install/'
    'microxrcedds_agent/bin/MicroXRCEAgent')
AGENT_LIB = Path(
    '/home/zr/px4_ros_uxrce_dds_ws/src/install/'
    'microxrcedds_agent/lib')
MODEL_NAME = 'phoenixdrone_alpha_0'
FEEDBACK_TOPIC = f'/world/stars_ts/model/{MODEL_NAME}/joint_state'


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    controller_enabled = LaunchConfiguration('controller_enabled')
    output_enabled = LaunchConfiguration('output_enabled')
    setpoint_source = LaunchConfiguration('setpoint_source')
    setpoint_frame = LaunchConfiguration('setpoint_frame')
    feedback_enabled = LaunchConfiguration('feedback_enabled')
    actuator_feedback_mode = LaunchConfiguration('actuator_feedback_mode')
    use_measured_control_dt = LaunchConfiguration('use_measured_control_dt')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('controller_enabled', default_value='true'),
        DeclareLaunchArgument('output_enabled', default_value='false'),
        DeclareLaunchArgument('setpoint_source', default_value='latch'),
        DeclareLaunchArgument('setpoint_frame', default_value='px4'),
        DeclareLaunchArgument('feedback_enabled', default_value='true'),
        DeclareLaunchArgument('actuator_feedback_mode', default_value='auto'),
        DeclareLaunchArgument('use_measured_control_dt', default_value='false'),
        ExecuteProcess(
            cmd=[str(AGENT), 'udp4', '-p', '8888'],
            additional_env={
                'LD_LIBRARY_PATH': f'{AGENT_LIB}:/opt/ros/jazzy/lib'
            },
            output='screen',
        ),
        ExecuteProcess(
            cmd=['make', 'px4_sitl_default', 'gz_phoenixdrone_alpha'],
            cwd=str(PX4_ROOT),
            additional_env={
                'CCACHE_DISABLE': '1',
                'HEADLESS': PythonExpression([
                    "'1' if '", headless, "'.lower() == 'true' else ''"
                ]),
            },
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                FEEDBACK_TOPIC
                + '@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            condition=IfCondition(feedback_enabled),
            output='screen',
        ),
        TimerAction(
            period=12.0,
            actions=[ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/stars_ts/entity/system/add',
                    '--reqtype', 'gz.msgs.EntityPlugin_V',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', (
                        'entity: {id: 10, type: MODEL}, plugins: {'
                        'filename: '
                        '"gz-sim-joint-state-publisher-system", '
                        'name: '
                        '"gz::sim::systems::JointStatePublisher", '
                        'innerxml: "<update_rate>100</update_rate>"}'
                    ),
                ],
                output='screen',
            )],
            condition=IfCondition(feedback_enabled),
        ),
        Node(
            package='phoenix_tailsitter_control',
            executable='alpha_tailsitter_controller',
            parameters=[{
                'output_enabled': ParameterValue(
                    output_enabled, value_type=bool),
                'setpoint_source': setpoint_source,
                'setpoint_frame': setpoint_frame,
                'actuator_feedback_mode': actuator_feedback_mode,
                'actuator_feedback_topic': FEEDBACK_TOPIC,
                'use_measured_control_dt': ParameterValue(
                    use_measured_control_dt, value_type=bool),
            }],
            condition=IfCondition(controller_enabled),
            output='screen',
        ),
    ])
