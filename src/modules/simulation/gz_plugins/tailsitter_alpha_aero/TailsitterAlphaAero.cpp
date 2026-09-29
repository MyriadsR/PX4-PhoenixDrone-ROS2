/****************************************************************************
 * Complete Tailsitter-control equations 5--14 for Gazebo Harmonic.
 ****************************************************************************/

#include "TailsitterAlphaAero.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <sdf/Element.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

using namespace gz;
using namespace sim;

namespace phoenix
{

double TailsitterAlphaAero::ReadDouble(
		const std::shared_ptr<const sdf::Element> &_sdf,
		const std::string &_name, double _default) const
{
	return _sdf->HasElement(_name) ? _sdf->Get<double>(_name) : _default;
}

int TailsitterAlphaAero::ReadInt(
		const std::shared_ptr<const sdf::Element> &_sdf,
		const std::string &_name, int _default) const
{
	return _sdf->HasElement(_name) ? _sdf->Get<int>(_name) : _default;
}

void TailsitterAlphaAero::Configure(const Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		EntityComponentManager &_ecm, EventManager &)
{
	_model = Model(_entity);
	if (!_model.Valid(_ecm)) {
		gzerr << "TailsitterAlphaAero must be attached to a model entity\n";
		return;
	}

	_link = Link(_model.LinkByName(_ecm, _sdf->Get<std::string>("link_name")));
	const std::array<std::string, 2> motor_names{{
		_sdf->Get<std::string>("left_motor_joint_name"),
		_sdf->Get<std::string>("right_motor_joint_name")}};
	const std::array<std::string, 2> flap_names{{
		_sdf->Get<std::string>("left_flap_joint_name"),
		_sdf->Get<std::string>("right_flap_joint_name")}};
	for (int i = 0; i < 2; ++i) {
		_motor_joints[i] = Joint(_model.JointByName(_ecm, motor_names[i]));
		_flap_joints[i] = Joint(_model.JointByName(_ecm, flap_names[i]));
		if (!_motor_joints[i].Valid(_ecm) || !_flap_joints[i].Valid(_ecm)) {
			gzerr << "TailsitterAlphaAero could not find actuator joints\n";
			return;
		}
		_motor_joints[i].EnableVelocityCheck(_ecm, true);
	}
	if (!_link.Valid(_ecm)) {
		gzerr << "TailsitterAlphaAero could not find base link\n";
		return;
	}
	// Gazebo does not create world-velocity components for links by default.
	// Without this, WorldLinearVelocity() returns nullopt and PreUpdate exits
	// before applying the alpha-theory wrench.
	_link.EnableVelocityChecks(_ecm, true);

	_c_t = ReadDouble(_sdf, "c_T", _c_t);
	_c_mu = ReadDouble(_sdf, "c_mu", _c_mu);
	_c_mu_t = ReadDouble(_sdf, "c_mu_T", _c_mu_t);
	_c_lv = ReadDouble(_sdf, "c_LV", _c_lv);
	_c_dv = ReadDouble(_sdf, "c_DV", _c_dv);
	_c_lt = ReadDouble(_sdf, "c_LT", _c_lt);
	_c_dt = ReadDouble(_sdf, "c_DT", _c_dt);
	_c_lv_delta = ReadDouble(_sdf, "c_LV_delta", _c_lv_delta);
	_c_lt_delta = ReadDouble(_sdf, "c_LT_delta", _c_lt_delta);
	_alpha_zero = ReadDouble(_sdf, "alpha_0", _alpha_zero);
	_alpha_thrust = ReadDouble(_sdf, "alpha_T", _alpha_thrust);
		_motor_arm_y = ReadDouble(_sdf, "l_Ty", _motor_arm_y);
		_flap_arm_y = ReadDouble(_sdf, "l_dy", _flap_arm_y);
		_flap_arm_x = ReadDouble(_sdf, "l_dx", _flap_arm_x);
		_motor_numbers[0] = ReadInt(_sdf, "left_motor_number", _motor_numbers[0]);
		_motor_numbers[1] = ReadInt(_sdf, "right_motor_number", _motor_numbers[1]);
		_max_motor_speed = ReadDouble(_sdf, "max_motor_speed", _max_motor_speed);
		_motor_time_constant = ReadDouble(
			_sdf, "motor_time_constant", _motor_time_constant);
		_motor_input_scaling = ReadDouble(
			_sdf, "motor_input_scaling", _motor_input_scaling);
		_rotor_velocity_slowdown = ReadDouble(
			_sdf, "rotor_velocity_slowdown", _rotor_velocity_slowdown);
	_servo_time_constant = ReadDouble(
		_sdf, "servo_time_constant", _servo_time_constant);
	_servo_rate_limit = ReadDouble(
		_sdf, "servo_rate_limit", _servo_rate_limit);
	_flap_limit[0] = ReadDouble(_sdf, "left_flap_limit", _flap_limit[0]);
	_flap_limit[1] = ReadDouble(_sdf, "right_flap_limit", _flap_limit[1]);

	const std::array<std::string, 2> control_topics{{
		_sdf->Get<std::string>("left_control_sub_topic"),
		_sdf->Get<std::string>("right_control_sub_topic")}};
	const std::string left_topic = "/model/" + _model.Name(_ecm) + "/" + control_topics[0];
	const std::string right_topic = "/model/" + _model.Name(_ecm) + "/" + control_topics[1];
		if (!_node.Subscribe(left_topic,
			&TailsitterAlphaAero::OnLeftFlapCommand, this)
			|| !_node.Subscribe(right_topic,
			&TailsitterAlphaAero::OnRightFlapCommand, this)) {
			gzerr << "TailsitterAlphaAero failed to subscribe to flap commands\n";
			return;
		}
		const std::string motor_sub_topic = _sdf->HasElement("motor_command_sub_topic")
			? _sdf->Get<std::string>("motor_command_sub_topic")
			: "command/motor_speed";
		const std::string motor_topic = motor_sub_topic.empty()
			? ""
			: (motor_sub_topic[0] == '/'
				? motor_sub_topic
				: "/" + _model.Name(_ecm) + "/" + motor_sub_topic);
		if (motor_topic.empty()
			|| !_node.Subscribe(
				motor_topic, &TailsitterAlphaAero::OnMotorCommand, this)) {
			gzerr << "TailsitterAlphaAero failed to subscribe to motor commands\n";
			return;
		}
		_configured = true;
}

void TailsitterAlphaAero::OnLeftFlapCommand(const msgs::Double &_msg)
{
	_flap_commands[0].store(_msg.data(), std::memory_order_relaxed);
}

void TailsitterAlphaAero::OnRightFlapCommand(const msgs::Double &_msg)
{
	_flap_commands[1].store(_msg.data(), std::memory_order_relaxed);
}

void TailsitterAlphaAero::OnMotorCommand(const msgs::Actuators &_msg)
{
	for (int i = 0; i < 2; ++i) {
		const int motor_number = _motor_numbers[i];
		if (motor_number >= 0 && motor_number < _msg.velocity_size()) {
			const double command = std::max(0.0, _msg.velocity(motor_number));
			_motor_speed_commands[i].store(
				command * _motor_input_scaling, std::memory_order_relaxed);
		}
	}
	_motor_command_received.store(true, std::memory_order_relaxed);
}

math::Vector3d TailsitterAlphaAero::GzFluToTs(const math::Vector3d &_value)
{
	return {_value.Z(), -_value.Y(), _value.X()};
}

math::Vector3d TailsitterAlphaAero::TsToGzFlu(const math::Vector3d &_value)
{
	return {_value.Z(), -_value.Y(), _value.X()};
}

TailsitterAlphaAero::Result TailsitterAlphaAero::Compute(
	const math::Vector3d &_velocity_body_ts,
	const std::array<double, 2> &_motor_speed,
	const std::array<double, 2> &_flap_angle) const
{
	const double speed = std::max(_velocity_body_ts.Length(), 1e-3);
	const double ca0 = std::cos(_alpha_zero);
	const double sa0 = std::sin(_alpha_zero);
	const double alpha_tilde = _alpha_zero + _alpha_thrust;
	const double cat = std::cos(alpha_tilde);
	const double sat = std::sin(alpha_tilde);
	const math::Vector3d velocity_alpha{
		ca0 * _velocity_body_ts.X() + sa0 * _velocity_body_ts.Z(),
		_velocity_body_ts.Y(),
		-sa0 * _velocity_body_ts.X() + ca0 * _velocity_body_ts.Z()};

	std::array<double, 2> thrust{};
	std::array<double, 2> flap_force_z{};
	std::array<math::Vector3d, 2> thrust_force_alpha{};
	const math::Vector3d thrust_direction{
		cat * (1.0 - _c_dt), 0.0, sat * (_c_lt - 1.0)};
	for (int i = 0; i < 2; ++i) {
		thrust[i] = _c_t * _motor_speed[i] * _motor_speed[i];
		thrust_force_alpha[i] = thrust_direction * thrust[i];
		flap_force_z[i] = -(_c_lt_delta * cat * thrust[i]
			+ _c_lv_delta * speed * velocity_alpha.X()) * _flap_angle[i];
	}

	const math::Vector3d flap_force{0.0, 0.0,
		flap_force_z[0] + flap_force_z[1]};
	const math::Vector3d wing_force{
		-_c_dv * velocity_alpha.X() * speed,
		0.0,
		-_c_lv * velocity_alpha.Z() * speed};
	const math::Vector3d force_alpha = thrust_force_alpha[0]
		+ thrust_force_alpha[1] + flap_force + wing_force;

	const math::Vector3d difference_alpha =
		thrust_force_alpha[1] - thrust_force_alpha[0];
	const math::Vector3d difference_body{
		ca0 * difference_alpha.X() - sa0 * difference_alpha.Z(),
		difference_alpha.Y(),
		sa0 * difference_alpha.X() + ca0 * difference_alpha.Z()};
	const math::Vector3d thrust_moment{
		_motor_arm_y * difference_body.Z(),
		_c_mu_t * (thrust[0] + thrust[1]),
		-_motor_arm_y * difference_body.X()};

	const double reaction = _c_mu
		* (_motor_speed[0] * _motor_speed[0]
		   - _motor_speed[1] * _motor_speed[1]);
	const math::Vector3d reaction_moment{
		std::cos(_alpha_thrust) * reaction,
		0.0,
		-std::sin(_alpha_thrust) * reaction};
	const double flap_difference = flap_force_z[1] - flap_force_z[0];
	const math::Vector3d flap_moment{
		_flap_arm_y * ca0 * flap_difference,
		_flap_arm_x * (flap_force_z[0] + flap_force_z[1]),
		_flap_arm_y * sa0 * flap_difference};
	return {force_alpha, thrust_moment + reaction_moment + flap_moment};
}

void TailsitterAlphaAero::PreUpdate(const UpdateInfo &_info,
	EntityComponentManager &_ecm)
{
	if (!_configured || _info.paused) {
		return;
	}
	const double dt = std::chrono::duration<double>(_info.dt).count();
	if (!std::isfinite(dt) || dt <= 0.0) {
		return;
	}

	std::array<double, 2> motor_speed{};
	const bool use_motor_commands = _motor_command_received.load(
		std::memory_order_relaxed);
	for (int i = 0; i < 2; ++i) {
		if (use_motor_commands) {
			const double target = std::clamp(
				_motor_speed_commands[i].load(std::memory_order_relaxed),
				0.0, _max_motor_speed);
			const double alpha = std::min(
				1.0, dt / std::max(_motor_time_constant, 1e-4));
			_motor_speed_state[i] += alpha * (target - _motor_speed_state[i]);
			motor_speed[i] = _motor_speed_state[i];
		} else {
			const auto velocity = _motor_joints[i].Velocity(_ecm);
			if (!velocity || velocity->empty()) {
				return;
			}
			motor_speed[i] = std::abs((*velocity)[0]) * _rotor_velocity_slowdown;
		}
		const double command = std::clamp(
			_flap_commands[i].load(std::memory_order_relaxed),
			-_flap_limit[i], _flap_limit[i]);
		const double desired_rate = (command - _flap_state[i])
			/ std::max(_servo_time_constant, 1e-4);
		const double rate = std::clamp(
			desired_rate, -_servo_rate_limit, _servo_rate_limit);
		_flap_state[i] = std::clamp(
			_flap_state[i] + dt * rate, -_flap_limit[i], _flap_limit[i]);
		const std::vector<double> position{_flap_state[i]};
		auto reset = _ecm.Component<components::JointPositionReset>(
			_flap_joints[i].Entity());
		if (reset) {
			reset->SetData(position,
				[](const auto &a, const auto &b) { return a == b; });
		} else {
			_ecm.CreateComponent(_flap_joints[i].Entity(),
				components::JointPositionReset(position));
		}
	}

	const auto pose = _link.WorldPose(_ecm);
	const auto inertial_pose = _link.WorldInertialPose(_ecm);
	if (!pose || !inertial_pose) {
		return;
	}
	// Equations 5--14 define the translational velocity and wrench at the
	// vehicle centre of mass.  The Phoenix base_link origin is 0.157 m away
	// from its centre of mass, so using the link-origin overload here creates
	// a large, artificial pitch moment from the small alpha-axis force.
	const math::Vector3d com_offset_body_gz = pose->Rot().Inverse().RotateVector(
		inertial_pose->Pos() - pose->Pos());
	const auto velocity_world = _link.WorldLinearVelocity(
		_ecm, com_offset_body_gz);
	if (!velocity_world) {
		return;
	}
	const math::Vector3d velocity_body_gz =
		pose->Rot().Inverse().RotateVector(*velocity_world);
	const math::Vector3d velocity_body_ts = GzFluToTs(velocity_body_gz);
	const Result result = Compute(velocity_body_ts, motor_speed, _flap_state);

	const double ca0 = std::cos(_alpha_zero);
	const double sa0 = std::sin(_alpha_zero);
	const math::Vector3d force_body_ts{
		ca0 * result.force_alpha.X() - sa0 * result.force_alpha.Z(),
		result.force_alpha.Y(),
		sa0 * result.force_alpha.X() + ca0 * result.force_alpha.Z()};
	const math::Vector3d force_world = pose->Rot().RotateVector(
		TsToGzFlu(force_body_ts));
	const math::Vector3d moment_world = pose->Rot().RotateVector(
		TsToGzFlu(result.moment_body_ts));
	if (force_world.IsFinite() && moment_world.IsFinite()) {
		_link.AddWorldWrench(
			_ecm, force_world, moment_world, com_offset_body_gz);
	}
}

} // namespace phoenix

GZ_ADD_PLUGIN(phoenix::TailsitterAlphaAero,
	gz::sim::System,
	phoenix::TailsitterAlphaAero::ISystemConfigure,
	phoenix::TailsitterAlphaAero::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
	phoenix::TailsitterAlphaAero, "phoenix::TailsitterAlphaAero")
