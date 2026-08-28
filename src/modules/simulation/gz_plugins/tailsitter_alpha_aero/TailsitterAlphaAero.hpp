/****************************************************************************
 * Complete Tailsitter-control alpha-theory model for PhoenixDrone.
 ****************************************************************************/

#pragma once

#include <gz/msgs/double.pb.h>
#include <gz/math/Vector3.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <array>
#include <atomic>
#include <memory>
#include <string>

namespace phoenix
{

class TailsitterAlphaAero final : public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		gz::sim::EntityComponentManager &_ecm,
		gz::sim::EventManager &_eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm) override;

private:
	struct Result {
		gz::math::Vector3d force_alpha;
		gz::math::Vector3d moment_body_ts;
	};

	Result Compute(const gz::math::Vector3d &_velocity_body_ts,
		const std::array<double, 2> &_motor_speed,
		const std::array<double, 2> &_flap_angle) const;

	static gz::math::Vector3d GzFluToTs(const gz::math::Vector3d &_value);
	static gz::math::Vector3d TsToGzFlu(const gz::math::Vector3d &_value);
	double ReadDouble(const std::shared_ptr<const sdf::Element> &_sdf,
		const std::string &_name, double _default) const;
	void OnLeftFlapCommand(const gz::msgs::Double &_msg);
	void OnRightFlapCommand(const gz::msgs::Double &_msg);

	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Link _link{gz::sim::kNullEntity};
	std::array<gz::sim::Joint, 2> _motor_joints;
	std::array<gz::sim::Joint, 2> _flap_joints;
	gz::transport::Node _node;
	std::array<std::atomic<double>, 2> _flap_commands{{0.0, 0.0}};
	std::array<double, 2> _flap_state{{0.0, 0.0}};

	double _c_t{7.864e-6};
	double _c_mu{1.80872e-7};
	double _c_mu_t{0.0};
	double _c_lv{0.29};
	double _c_dv{0.0};
	double _c_lt{2.23};
	double _c_dt{0.0};
	double _c_lv_delta{0.18};
	double _c_lt_delta{1.25};
	double _alpha_zero{-0.03490658503988659};
	double _alpha_thrust{0.0};
	double _motor_arm_y{0.195};
	double _flap_arm_y{0.195};
	double _flap_arm_x{0.036};
	double _rotor_velocity_slowdown{10.0};
	double _servo_time_constant{0.03};
	double _servo_rate_limit{25.0};
	std::array<double, 2> _flap_limit{{1.0471975512, 0.5235987756}};
	bool _configured{false};
};

} // namespace phoenix
