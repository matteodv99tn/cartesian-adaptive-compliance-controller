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
#include <memory>
#include <vector>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jntarrayvel.hpp"
#include "qpOASES.hpp"
#include "qpOASES/MessageHandling.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace defaults {
const double              x0_tank   = 1.0;
const std::vector<double> Q_weights = {3200.0, 3200.0, 3200.0};
const std::vector<double> R_weights = {0.01, 0.01, 0.01};
const std::vector<double> F_min     = {-15.0, -15.0, -15.0};
const std::vector<double> F_max     = {15.0, 15.0, 15.0};
}  // namespace defaults

LifecycleNodeInterface::CallbackReturn CartesianAdaptiveComplianceController::on_init(
) {
    LifecycleNodeInterface::CallbackReturn parent_ret =
            CartesianComplianceController::on_init();
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to initialize");
        return parent_ret;
    }

    auto_declare<double>("tank.initial_state", defaults::x0_tank);
    auto_declare<std::vector<double>>("Q_weights", defaults::Q_weights);
    auto_declare<std::vector<double>>("R_weights", defaults::R_weights);
    auto_declare<std::vector<double>>("F_min", defaults::F_min);
    auto_declare<std::vector<double>>("F_max", defaults::F_max);
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

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

    // Initialisation of the variables
    _initializeVariables();
    RCLCPP_INFO(get_node()->get_logger(), "Adaptive controller initialised");

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_activate(
        const rclcpp_lifecycle::State& previous_state
) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating adaptive controller");
    CallbackReturn parent_ret =
            CartesianComplianceController::on_activate(previous_state);
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to activate");
        return parent_ret;
    }

    // TODO: add things that needs to be activated
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
    for (auto& handle : _joint_state_vel_handles) {
        RCLCPP_INFO(
                get_node()->get_logger(),
                "Joint %s velocity interface activated",
                handle.get().get_name()
        );
    }

    if (!got_vel_interfaces) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get velocity interfaces");
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
            get_node()->get_logger(),
            "Got %d state interfaces",
            state_interfaces_.size()
    );
    _joint_velocities = ctrl::VectorND::Zero(joints.size());
    _kin_solver =
            std::make_unique<KDL::ChainFkSolverVel_recursive>(Base::m_robot_chain);
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
    RCLCPP_INFO(get_node()->get_logger(), "Synchronis");
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

void CartesianAdaptiveComplianceController::_synchroniseJointVelocities() {
    for (std::size_t i = 0; i < _joint_state_vel_handles.size(); ++i) {
        _joint_velocities(i) = _joint_state_vel_handles[i].get().get_value();
        RCLCPP_INFO(get_node()->get_logger(), "Joint #%d velocity: %f", i, _joint_velocities(i));
    }
}

void CartesianAdaptiveComplianceController::_initializeVariables() {
    // Tank initialisation
    _x_tank = get_node()->get_parameter("tank.initial_state").as_double();

    // Minimum and maximum stiffness
    _Kmin.diagonal() = ctrl::Vector6D(
            {get_node()->get_parameter("stiffness.trans_x").as_double(),
             get_node()->get_parameter("stiffness.trans_y").as_double(),
             get_node()->get_parameter("stiffness.trans_z").as_double(),
             get_node()->get_parameter("stiffness.rot_x").as_double(),
             get_node()->get_parameter("stiffness.rot_y").as_double(),
             get_node()->get_parameter("stiffness.rot_z").as_double()}
    );
    _Kmax.diagonal() = ctrl::Vector6D({1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0});
    m_stiffness      = _Kmin;

    // Weight matrices for the QP problem
    auto Q_weights = get_node()->get_parameter("Q_weights").as_double_array();
    auto R_weights = get_node()->get_parameter("R_weights").as_double_array();
    if (Q_weights.size() != 3) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "Q_weights must have 3 elements, switching to default values"
        );
        Q_weights = defaults::Q_weights;
    }
    if (R_weights.size() != 3) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "R_weights must have 3 elements, switching to default values"
        );
        R_weights = defaults::R_weights;
    }
    _Q.diagonal() = ctrl::Vector3D({Q_weights[0], Q_weights[1], Q_weights[2]});
    _R.diagonal() = ctrl::Vector3D({R_weights[0], R_weights[1], R_weights[2]});

    // Force limits
    auto F_min = get_node()->get_parameter("F_min").as_double_array();
    auto F_max = get_node()->get_parameter("F_max").as_double_array();
    if (F_min.size() != 3) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "F_min must have 3 elements, switching to default values"
        );
        F_min = defaults::F_min;
    }
    if (F_max.size() != 3) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "F_max must have 3 elements, switching to default values"
        );
        F_max = defaults::F_max;
    }
    _F_min = ctrl::Vector3D({F_min[0], F_min[1], F_min[2]});
    _F_max = ctrl::Vector3D({F_max[0], F_max[1], F_max[2]});
}

void CartesianAdaptiveComplianceController::_initializeQpProblem() {
    // TODO

    // Initialise solver
    const int        nv = 3;  // Numbers of variables
    const int        nc = 5;  // Number of constraints
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;

    _qp_prob = qpOASES::QProblem(nv, nc);
    _qp_prob.setOptions(options);

    _qp_H     = QpMatrix::Zero(nv, nv);
    _qp_A     = QpMatrix::Zero(nc, nv);
    _qp_x_lb  = QpVector::Zero(nv);
    _qp_x_ub  = QpVector::Zero(nv);
    _qp_A_lb  = QpVector::Zero(nc);
    _qp_A_ub  = QpVector::Zero(nc);
    _qp_x_sol = QpVector::Zero(nv);
}

void CartesianAdaptiveComplianceController::_updateStiffness() {
    // TODO: fill values of the qp variables

    RCLCPP_INFO(get_node()->get_logger(), "_updateStiffness() call");
    KDL::FrameVel  frame_vel = _getEndEffectorFrameVel();
    RCLCPP_INFO(get_node()->get_logger(), "frame velocity");
    ctrl::Vector3D x{// Current position
                     frame_vel.p.p.x(),
                     frame_vel.p.p.y(),
                     frame_vel.p.p.z()};

    ctrl::Vector3D xd{// Current velocity
                      frame_vel.p.v.x(),
                      frame_vel.p.v.y(),
                      frame_vel.p.v.z()};

    ctrl::Vector3D x_des{// Desired position
                         MotionBase::m_target_frame.p.x(),
                         MotionBase::m_target_frame.p.y(),
                         MotionBase::m_target_frame.p.z()};

    ctrl::Vector3D x_tilde  = x - x_des;
    ctrl::Vector3D xd_tilde = -xd;

    // Solve the QP problem:
    qpOASES::int_t nWSR            = 10;
    RCLCPP_INFO(get_node()->get_logger(), "Solving QP problem");
    qpOASES::int_t qp_solve_status = qpOASES::getSimpleStatus(_qp_prob.init(
            _qp_H.data(),
            _qp_g.data(),
            _qp_A.data(),
            _qp_x_lb.data(),
            _qp_x_ub.data(),
            _qp_A_lb.data(),
            _qp_A_ub.data(),
            nWSR
    ));
    RCLCPP_INFO(get_node()->get_logger(), "Retrieving solution from QP problem");
    _qp_prob.getPrimalSolution(_qp_x_sol.data());


    if (qp_solve_status != qpOASES::SUCCESSFUL_RETURN) {
        RCLCPP_ERROR(get_node()->get_logger(), "QP solver failed");
        // Maybe set default values
        return;
    }

    // TODO: write back results to update stiffness
}

KDL::FrameVel CartesianAdaptiveComplianceController::_getEndEffectorFrameVel() const {
    KDL::JntArray q(Base::m_joint_state_pos_handles.size());
    KDL::JntArray q_dot(Base::m_joint_state_pos_handles.size());
    for (std::size_t i = 0; i < Base::m_joint_state_pos_handles.size(); ++i) {
        q(i)     = Base::m_joint_state_pos_handles[i].get().get_value();
        q_dot(i) = _joint_velocities(i);
    }

    KDL::FrameVel    frame_vel;
    KDL::JntArrayVel joint_data(q, q_dot);
    RCLCPP_INFO(this->get_node()->get_logger(), "Updating kinematics");
    _kin_solver->JntToCart(joint_data, frame_vel);
    return frame_vel;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
        cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController,
        controller_interface::ControllerInterface
)
