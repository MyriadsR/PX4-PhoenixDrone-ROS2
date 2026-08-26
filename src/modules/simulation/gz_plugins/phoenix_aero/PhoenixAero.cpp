/****************************************************************************
 *
 * Port of PX4-PhoenixDrone's Gazebo Classic tailsitter aerodynamic model to
 * Gazebo Harmonic.
 *
 ****************************************************************************/

#include "PhoenixAero.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

#include <cmath>
#include <vector>
#include <string>

using namespace gz;
using namespace sim;

namespace phoenix
{

void PhoenixAero::Configure(const Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		EntityComponentManager &_ecm,
		EventManager &)
{
	_model = Model(_entity);

	if (!_model.Valid(_ecm)) {
		gzerr << "PhoenixAero must be attached to a model entity\n";
		return;
	}

	const std::string link_name = _sdf->Get<std::string>("link_name");
	const std::string motor_joint_name = _sdf->Get<std::string>("motor_joint_name");
	const std::string control_joint_name = _sdf->Get<std::string>("control_joint_name");

	_link = Link(_model.LinkByName(_ecm, link_name));
	_motor_joint = Joint(_model.JointByName(_ecm, motor_joint_name));
	_control_joint = Joint(_model.JointByName(_ecm, control_joint_name));

	if (!_link.Valid(_ecm) || !_motor_joint.Valid(_ecm) || !_control_joint.Valid(_ecm)) {
		gzerr << "PhoenixAero could not find link/joints: " << link_name << ", "
		      << motor_joint_name << ", " << control_joint_name << "\n";
		return;
	}

	if (_sdf->HasElement("k_lift")) { _k_lift = _sdf->Get<double>("k_lift"); }
	if (_sdf->HasElement("k_drag")) { _k_drag = _sdf->Get<double>("k_drag"); }
	if (_sdf->HasElement("k_pitch")) { _k_pitch = _sdf->Get<double>("k_pitch"); }
	if (_sdf->HasElement("rotor_velocity_slowdown")) {
		_rotor_velocity_slowdown = _sdf->Get<double>("rotor_velocity_slowdown");
	}
	if (_sdf->HasElement("cp")) { _cp = _sdf->Get<math::Vector3d>("cp"); }
	if (_sdf->HasElement("forward")) { _forward = _sdf->Get<math::Vector3d>("forward"); }
	if (_sdf->HasElement("upward")) { _upward = _sdf->Get<math::Vector3d>("upward"); }

	const std::string control_sub_topic = _sdf->Get<std::string>("control_sub_topic");
	_control_topic = "/model/" + _model.Name(_ecm) + "/" + control_sub_topic;
	if (!_node.Subscribe(_control_topic, &PhoenixAero::OnControlCommand, this)) {
		gzerr << "PhoenixAero failed to subscribe to " << _control_topic << "\n";
		return;
	}

	_motor_joint.EnableVelocityCheck(_ecm, true);
	_configured = true;
}

void PhoenixAero::OnControlCommand(const gz::msgs::Double &_msg)
{
	_control_command.store(_msg.data(), std::memory_order_relaxed);
}

void PhoenixAero::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
	if (!_configured || _info.paused) {
		return;
	}

	const auto motor_velocity = _motor_joint.Velocity(_ecm);

	if (!motor_velocity || motor_velocity->empty()) {
		return;
	}

	const double omega = std::abs((*motor_velocity)[0]) * _rotor_velocity_slowdown;
	const double delta = _control_command.load(std::memory_order_relaxed);

	if (!std::isfinite(omega) || !std::isfinite(delta)) {
		return;
	}

	// Classic used position_kinematic: no additional servo PID dynamics.
	// Keep both the aerodynamic angle and displayed joint at the command.
	const std::vector<double> joint_position{delta};
	auto reset = _ecm.Component<components::JointPositionReset>(_control_joint.Entity());
	if (reset) {
		reset->SetData(joint_position,
			[](const auto &a, const auto &b) { return a == b; });
	} else {
		_ecm.CreateComponent(_control_joint.Entity(),
			components::JointPositionReset(joint_position));
	}
	const auto world_pose_opt = _link.WorldPose(_ecm);
	if (!world_pose_opt) { return; }
	const math::Pose3d world_pose = *world_pose_opt;
	const math::Vector3d forward_world = world_pose.Rot().RotateVector(_forward.Normalized());
	const math::Vector3d upward_world = world_pose.Rot().RotateVector(_upward.Normalized());
	const math::Vector3d spanwise_world = forward_world.Cross(upward_world).Normalized();
	const double omega_squared = omega * omega;

	const math::Vector3d force = upward_world * (_k_lift * omega_squared * delta)
		-forward_world * (_k_drag * omega_squared * delta * delta);
	const math::Vector3d torque = spanwise_world * (_k_pitch * omega_squared * delta);

	_link.AddWorldWrench(_ecm, force, torque, _cp);
}

} // namespace phoenix

GZ_ADD_PLUGIN(phoenix::PhoenixAero,
	gz::sim::System,
	phoenix::PhoenixAero::ISystemConfigure,
	phoenix::PhoenixAero::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(phoenix::PhoenixAero, "phoenix::PhoenixAero")
