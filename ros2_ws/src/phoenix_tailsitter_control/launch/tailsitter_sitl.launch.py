from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PX4_ROOT = Path('/home/zr/PX4-PhoenixDrone-ROS2')
AGENT = Path('/home/zr/px4_ros_uxrce_dds_ws/src/install/microxrcedds_agent/bin/MicroXRCEAgent')
AGENT_LIB = Path('/home/zr/px4_ros_uxrce_dds_ws/src/install/microxrcedds_agent/lib')


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    controller_enabled = LaunchConfiguration('controller_enabled')
    output_enabled = LaunchConfiguration('output_enabled')
    setpoint_source = LaunchConfiguration('setpoint_source')
    setpoint_frame = LaunchConfiguration('setpoint_frame')
    feedback_enabled = LaunchConfiguration('feedback_enabled')
    actuator_feedback_mode = LaunchConfiguration('actuator_feedback_mode')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('controller_enabled', default_value='true'),
        DeclareLaunchArgument('output_enabled', default_value='false'),
        DeclareLaunchArgument('setpoint_source', default_value='latch'),
        DeclareLaunchArgument('setpoint_frame', default_value='px4'),
        DeclareLaunchArgument('feedback_enabled', default_value='true'),
        DeclareLaunchArgument('actuator_feedback_mode', default_value='auto'),
        ExecuteProcess(
            cmd=[str(AGENT), 'udp4', '-p', '8888'],
            additional_env={'LD_LIBRARY_PATH': f'{AGENT_LIB}:/opt/ros/jazzy/lib'},
            output='screen',
        ),
        ExecuteProcess(
            cmd=['make', 'px4_sitl_default', 'gz_phoenixdrone'],
            cwd=str(PX4_ROOT),
            additional_env={
                'HEADLESS': PythonExpression([
                    "'1' if '", headless, "'.lower() == 'true' else ''"
                ])
            },
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/stars_ts/model/phoenixdrone_0/joint_state'
                '@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            condition=IfCondition(feedback_enabled),
            output='screen',
        ),
        # Attach feedback publishing at runtime: this deliberately leaves the
        # existing PhoenixDrone model and PX4 source tree unchanged.  Entity 10
        # is the deterministic model ID in the stars_ts single-model world.
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
                        'filename: "gz-sim-joint-state-publisher-system", '
                        'name: "gz::sim::systems::JointStatePublisher"}'
                    ),
                ],
                output='screen',
            )],
            condition=IfCondition(feedback_enabled),
        ),
        Node(
            package='phoenix_tailsitter_control',
            executable='tailsitter_controller',
            parameters=[{
                'output_enabled': ParameterValue(output_enabled, value_type=bool),
                'setpoint_source': setpoint_source,
                'setpoint_frame': setpoint_frame,
                'actuator_feedback_mode': actuator_feedback_mode,
            }],
            condition=IfCondition(controller_enabled),
            output='screen',
        ),
    ])
