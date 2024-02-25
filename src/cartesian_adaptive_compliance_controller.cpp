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

#include <algorithm>
#include <cstdio>

#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
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

    // Configure velocity state interface
    std::vector<std::string> joints;
    joints.reserve(Base::m_joint_state_pos_handles.size());
    std::transform(
            Base::m_joint_state_pos_handles.begin(),
            Base::m_joint_state_pos_handles.end(),
            std::back_inserter(joints),
            [](auto& handle) { return handle.get().get_name(); }
    );
    const bool got_vel_interfaces = controller_interface::get_ordered_interfaces(
            state_interfaces_,
            joints,
            hardware_interface::HW_IF_VELOCITY,
            _joint_state_vel_handles
    );
    if (!got_vel_interfaces) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get velocity interfaces");
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    _joint_velocities = ctrl::VectorND::Zero(joints.size());

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

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE ||  \
        defined                                       CARTESIAN_CONTROLLERS_IRON
controller_interface::return_type CartesianAdaptiveComplianceController::update(
        const rclcpp::Time& time, const rclcpp::Duration& period
) {
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianAdaptiveComplianceController::update() {
#endif
    // TODO: proper control loop
    // Most probably we can't directly call CartesianComplianceController::update()
    // since we need to add stuff in the middle of the code

    // Synchronize the internal model and the real robot
    Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);
    _synchroniseJointVelocities();
    _updateStiffness();

    // --- Same control loop of the cartesian compliance controller
    for (int i = 0; i < Base::m_iterations; ++i) {
        auto           internal_period = rclcpp::Duration::from_seconds(0.02);
        ctrl::Vector6D error           = computeComplianceError();
        Base::computeJointControlCmds(error, internal_period);
    }
    Base::writeJointControlCmds();

    return controller_interface::return_type::OK;
}


void CartesianAdaptiveComplianceController::_synchroniseJointVelocities(){
    for(std::size_t i = 0; i < _joint_state_vel_handles.size(); ++i){
        _joint_velocities(i) = _joint_state_vel_handles[i].get().get_value();
    }
}


void CartesianAdaptiveComplianceController::_initializeQpProblem(){
    // TODO
}


void CartesianAdaptiveComplianceController::_updateStiffness(){
    // TODO
}
