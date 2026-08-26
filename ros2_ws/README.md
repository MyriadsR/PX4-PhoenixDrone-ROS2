# PhoenixDrone ROS 2 Jazzy workspace

This workspace pins px4_msgs to PX4 v1.16.2 commit 392e831 and ports the two
ROS 1 offboard demos from PX4-PhoenixDrone commit 70af6cbc without adding
automatic mode switching or arming.

Build:

    source /opt/ros/jazzy/setup.bash
    cd /home/zr/PX4-PhoenixDrone-ROS2/ros2_ws
    colcon build --symlink-install

Run position, rectangle, or attitude demo:

    source install/setup.bash
    ros2 launch phoenix_offboard phoenix_sitl.launch.py demo:=position
    ros2 launch phoenix_offboard phoenix_sitl.launch.py demo:=rectangle
    ros2 launch phoenix_offboard phoenix_sitl.launch.py demo:=attitude

After PX4 prints `Ready for takeoff!` and `time sync converged`, keep the
launch terminal running. In another terminal, enter Offboard and arm through
DDS (QGC does not need to show an Offboard item):

    source /opt/ros/jazzy/setup.bash
    cd /home/zr/PX4-PhoenixDrone-ROS2/ros2_ws
    source install/setup.bash

    ros2 topic pub --once /fmu/in/vehicle_command \
      px4_msgs/msg/VehicleCommand \
      "{timestamp: 1, param1: 1.0, param2: 6.0, command: 176, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"

    ros2 topic pub --once /fmu/in/vehicle_command \
      px4_msgs/msg/VehicleCommand \
      "{timestamp: 1, param1: 1.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"

The first command selects PX4 custom main mode 6 (Offboard); the second arms.
The position demo then holds NED `(0, 0, -1)`, which is one metre above the
takeoff point. The rectangle demo flies four NED vertices `(0, 0, -1)`,
`(0.5, 0, -1)`, `(0.5, 0.3, -1)`, and `(0, 0.3, -1)`, then descends and
disarms. Stop the launch with Ctrl-C.

For a visible Gazebo window, add `headless:=false`:

    ros2 launch phoenix_offboard phoenix_sitl.launch.py \
      demo:=position headless:=false

Do not use QGC's one-click Takeoff for these demos. They use the original
Phoenix direct-actuator controller and therefore require Offboard mode.

The demos publish at 10 Hz, matching the original code. They intentionally do
not arm the vehicle or switch it into Offboard mode.

For the position demo, the launch file runs phoenix_position_controller. It
ports the original TS_POS time-constant/damping law, 0.63 kg gravity
compensation, force limits, fixed-heading behavior, and force-to-attitude
conversion. The trajectory command remains NED (0, 0, -1), equivalent to the
original MAVROS ENU (0, 0, +1) command.

The launch file also runs phoenix_controller. It ports the original reduced
attitude controller, rigid-body rate law, and nonlinear dual-motor/elevon
allocation constants. It publishes direct actuator commands only after PX4
reports the vehicle armed; the operator remains responsible for switching to
Offboard and arming, as in the original demos.
