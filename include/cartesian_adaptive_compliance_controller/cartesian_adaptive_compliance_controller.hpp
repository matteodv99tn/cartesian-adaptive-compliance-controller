#ifndef CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_HPP__
#define CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_HPP__


#include <functional>
#include <vector>

#include "cartesian_compliance_controller/cartesian_compliance_controller.h"
#include "hardware_interface/loaned_state_interface.hpp"
#include "qpOASES.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

USING_NAMESPACE_QPOASES

namespace cartesian_adaptive_compliance_controller {

class CartesianAdaptiveComplianceController
        : public cartesian_compliance_controller::CartesianComplianceController {
public:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;


    // clang-format off
#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE ||  \
        defined                                       CARTESIAN_CONTROLLERS_IRON
    controller_interface::return_type
    update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
    controller_interface::return_type update() override;
#endif
    // clang-format on
private:
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
                   _joint_state_vel_handles;
    ctrl::VectorND _joint_velocities;

    void _updateStiffness();
    void _synchroniseJointVelocities();
};

}  // namespace cartesian_adaptive_compliance_controller

#endif  // CARTESIAN_ADAPTIVE_COMPLIVECTOR_HPP__
