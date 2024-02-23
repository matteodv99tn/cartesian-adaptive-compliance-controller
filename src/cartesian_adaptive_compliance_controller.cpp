// Empty

/**
 * Changes are in the functions:
 * - on_configure(): this changes might not be relevant to me
 * - on_activate(): there are changes
 *      mainly initialisation of publisher/subscribers (for both logging and control
 * purposes)
 * - update(): this must be changed for sure, since it implements the main control
 * algorithm see where we can call the parent class!
 *
 */

#include "cartesian_adaptive_compliance_controller/cartesian_adaptive_compliance_controller.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_configure(
        const rclcpp_lifecycle::State& previous_state
) {
    CallbackReturn parent_ret =
            CartesianComplianceController::on_configure(previous_state);
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to configure");
        return parent_ret;
    }

    // TODO: add custom configuration
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_activate(
        const rclcpp_lifecycle::State& previous_state
) {
    CallbackReturn parent_ret =
            CartesianComplianceController::on_activate(previous_state);
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to activate");
        return parent_ret;
    }

    // TODO: add things that needs to be activated
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_deactivate(
        const rclcpp_lifecycle::State& previous_state
) {
    CallbackReturn parent_ret =
            CartesianComplianceController::on_deactivate(previous_state);
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to deactivate");
        return parent_ret;
    }

    // TODO: add things that needs to be deactivated (dual to activation)
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianAdaptiveComplianceController::update() {
    // TODO: proper control loop
    // Most probably we can't directly call CartesianComplianceController::update()
    // since we need to add stuff in the middle of the code
    return parent_ret;
}