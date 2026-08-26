/****************************************************************************
 *
 * Port of PX4-PhoenixDrone's Gazebo Classic tailsitter aerodynamic model to
 * Gazebo Harmonic. The model captures elevon forces inside rotor propwash.
 *
 ****************************************************************************/

#pragma once

#include <gz/msgs/double.pb.h>
#include <gz/math/Vector3.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <atomic>
#include <memory>
#include <string>

namespace phoenix
{

class PhoenixAero final : public gz::sim::System,
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

	void OnControlCommand(const gz::msgs::Double &_msg);

private:
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Link _link{gz::sim::kNullEntity};
	gz::sim::Joint _motor_joint{gz::sim::kNullEntity};
	gz::sim::Joint _control_joint{gz::sim::kNullEntity};

	gz::transport::Node _node;
	std::string _control_topic;
	std::atomic<double> _control_command{0.0};

	gz::math::Vector3d _cp{0.0, 0.0, 0.0};
	gz::math::Vector3d _forward{0.0, 0.0, 1.0};
	gz::math::Vector3d _upward{1.0, 0.0, 0.0};

	double _k_lift{3.48e-6};
	double _k_drag{1.75e-6};
	double _k_pitch{-3.44e-7};
	double _rotor_velocity_slowdown{10.0};
	bool _configured{false};
};

} // namespace phoenix
