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
#include <iostream>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
const std::string         target_velocity_topic = "/target_velocity";
const std::string         target_wrench_topic   = "/target_wrench";
const double              sampling_period       = 0.002;
const double              x0_tank               = 1.0;
const double              xmin_tank             = 0.4;
const double              eta_tank              = -0.1;
const std::vector<double> Q_weights             = {3200.0, 3200.0, 3200.0};
const std::vector<double> R_weights             = {0.01, 0.01, 0.01};
const std::vector<double> F_min                 = {-15.0, -15.0, -15.0};
const std::vector<double> F_max                 = {15.0, 15.0, 15.0};
const std::vector<double> K_min                 = {300.0, 300.0, 100.0};
const std::vector<double> K_max                 = {1000.0, 1000.0, 1000.0};
}  // namespace defaults

LifecycleNodeInterface::CallbackReturn CartesianAdaptiveComplianceController::on_init(
) {
    LifecycleNodeInterface::CallbackReturn parent_ret
            = CartesianComplianceController::on_init();
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to initialize");
        return parent_ret;
    }

    auto_declare<std::string>("target_velocity_topic", defaults::target_velocity_topic);
    auto_declare<std::string>("target_wrench_topic", defaults::target_wrench_topic);
    auto_declare<double>("sampling_period", defaults::sampling_period);
    auto_declare<double>("tank.initial_state", defaults::x0_tank);
    auto_declare<double>("tank.minimum_energy", defaults::xmin_tank);
    auto_declare<double>("tank.eta", defaults::eta_tank);
    auto_declare<std::vector<double>>("Qp.Q_weights", defaults::Q_weights);
    auto_declare<std::vector<double>>("Qp.R_weights", defaults::R_weights);
    auto_declare<std::vector<double>>("Qp.F_min", defaults::F_min);
    auto_declare<std::vector<double>>("Qp.F_max", defaults::F_max);
    auto_declare<std::vector<double>>("Qp.K_min", defaults::K_min);
    auto_declare<std::vector<double>>("Qp.K_max", defaults::K_max);
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_configure(
        const rclcpp_lifecycle::State& previous_state
) {
    CallbackReturn parent_ret
            = CartesianComplianceController::on_configure(previous_state);
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to configure");
        return parent_ret;
    }

    // TODO: add custom configuration

    // Configure velocity state interface

    // Initialisation of the variables
    _initializeVariables();
    _initializeQpProblem();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_activate(
        const rclcpp_lifecycle::State& previous_state
) {
    CallbackReturn parent_ret
            = CartesianComplianceController::on_activate(previous_state);
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to activate");
        return parent_ret;
    }

    // Create subscription for desired velocity
    _des_vel = ctrl::Vector3D::Zero();
    auto on_target_velocity_received =
            [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                this->_des_vel = ctrl::Vector3D(
                        {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z}
                );
            };
    _twist_sub = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            get_node()->get_parameter("target_velocity_topic").as_string(),
            10,
            on_target_velocity_received
    );

    // Create subscription for desired wrench
    _des_wrench = ctrl::Vector3D::Zero();
    auto on_target_wrench_received =
            [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                this->_des_wrench = ctrl::Vector3D(
                        {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z}
                );
            };
    _wrench_sub = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
            get_node()->get_parameter("target_wrench_topic").as_string(),
            10,
            on_target_wrench_received
    );

    // Configure velocity state interface
    std::vector<std::string> joints;
    joints.reserve(Base::m_joint_state_pos_handles.size());
    std::transform(
            Base::m_joint_state_pos_handles.begin(),
            Base::m_joint_state_pos_handles.end(),
            std::back_inserter(joints),
            [](auto& handle) { return handle.get().get_prefix_name(); }
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
    _kin_solver
            = std::make_unique<KDL::ChainFkSolverVel_recursive>(Base::m_robot_chain);

    _dt = get_node()->get_parameter("sampling_period").as_double();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_deactivate(
        const rclcpp_lifecycle::State& previous_state
) {
    CallbackReturn parent_ret
            = CartesianComplianceController::on_deactivate(previous_state);
    if (parent_ret != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(get_node()->get_logger(), "Parent class failed to deactivate");
        return parent_ret;
    }

    // TODO: add things that needs to be deactivated (dual to activation)
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE     \
        || defined                                    CARTESIAN_CONTROLLERS_IRON
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
        ctrl::Vector6D error           = cartesian_compliance_controller::
                CartesianComplianceController::computeComplianceError();
        Base::computeJointControlCmds(error, internal_period);
    }
    Base::writeJointControlCmds();

    return controller_interface::return_type::OK;
}

void CartesianAdaptiveComplianceController::_synchroniseJointVelocities() {
    for (std::size_t i = 0; i < _joint_state_vel_handles.size(); ++i) {
        _joint_velocities(i) = _joint_state_vel_handles[i].get().get_value();
    }
}

void CartesianAdaptiveComplianceController::_initializeVariables() {
    // Tank initialisation
    _x_tank = get_node()->get_parameter("tank.initial_state").as_double();

    // Minimum and maximum stiffness
    const std::vector<double> Kmin_vals
            = get_node()->get_parameter("Qp.K_min").as_double_array();
    const std::vector<double> Kmax_vals
            = get_node()->get_parameter("Qp.K_max").as_double_array();
    _Kmin = ctrl::Vector3D(Kmin_vals.data());
    _Kmax = ctrl::Vector3D(Kmax_vals.data());
    _K    = _Kmin.asDiagonal();
    _updateDamping();

    m_stiffness = ctrl::Vector6D(
                          {get_node()->get_parameter("stiffness.trans_x").as_double(),
                           get_node()->get_parameter("stiffness.trans_y").as_double(),
                           get_node()->get_parameter("stiffness.trans_z").as_double(),
                           get_node()->get_parameter("stiffness.rot_x").as_double(),
                           get_node()->get_parameter("stiffness.rot_y").as_double(),
                           get_node()->get_parameter("stiffness.rot_z").as_double()}
    )
                          .asDiagonal();

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
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    _qp_prob           = qpOASES::QProblem(nv, nc);
    _qp_prob.setOptions(options);

    _qp_H     = decltype(_qp_H)::Zero();
    _qp_g     = decltype(_qp_g)::Zero();
    _qp_A     = decltype(_qp_A)::Zero();
    _qp_A_lb  = decltype(_qp_A_lb)::Zero();
    _qp_A_ub  = decltype(_qp_A_ub)::Zero();
    _qp_x_lb  = decltype(_qp_x_lb)::Zero();
    _qp_x_ub  = decltype(_qp_x_ub)::Zero();
    _qp_x_sol = decltype(_qp_x_sol)::Zero();
}

void CartesianAdaptiveComplianceController::_updateStiffness() {
    // TODO: fill values of the qp variables

    // Variable preparation
    KDL::FrameVel        frame_vel = _getEndEffectorFrameVel();
    const ctrl::Vector3D x{// Current position
                           frame_vel.p.p.x(),
                           frame_vel.p.p.y(),
                           frame_vel.p.p.z()};

    const ctrl::Vector3D xd{// Current velocity
                            frame_vel.p.v.x(),
                            frame_vel.p.v.y(),
                            frame_vel.p.v.z()};

    const ctrl::Vector3D x_des{// Desired position
                               MotionBase::m_target_frame.p.x(),
                               MotionBase::m_target_frame.p.y(),
                               MotionBase::m_target_frame.p.z()};

    const ctrl::Vector3D x_tilde  = x_des - x;      // pos tracking error
    const ctrl::Vector3D xd_tilde = _des_vel - xd;  // vel tracking error

    const ctrl::Matrix3D X_tilde
            = x_tilde.asDiagonal();  // pos tracking error diag matrix
    const ctrl::Matrix3D Xd_tilde = xd_tilde.asDiagonal();

    const ctrl::Vector3D d = _D * xd_tilde;
    const ctrl::Vector3D f = d - _des_wrench;

    const ctrl::Vector3D A_bot_row = -Xd_tilde * x_tilde;

    const double T     = _tankEnergy();
    const double Tmin  = get_node()->get_parameter("tank.minimum_energy").as_double();
    const double sigma = T >= 1.0 ? 1 : 0;
    const double eta   = get_node()->get_parameter("tank.eta").as_double();

    // const double k1 = _sigma * xd_tilde.transpose() * _D * xd_tilde -
    // x_tilde.transpose() * _Kmin.asDiagonal() * xd_tilde;
    const double k_tmp1 = sigma * xd_tilde.transpose() * _D * xd_tilde;
    const double k_tmp2 = x_tilde.transpose() * _Kmin.asDiagonal() * xd_tilde;
    const double k_tmp3 = k_tmp1 - k_tmp2;

    const double k1 = k_tmp3 + (T - Tmin) / _dt;
    const double k2 = k_tmp3 - eta;


    // Fill the QP problem
    _qp_H                       = X_tilde.transpose() * _Q * X_tilde + _R;
    _qp_g                       = X_tilde.transpose() * _Q * f - _R * _Kmin;
    _qp_A.topLeftCorner<3, 3>() = X_tilde;
    _qp_A.row(3)                = A_bot_row;
    _qp_A.row(4)                = A_bot_row;
    _qp_x_lb                    = _Kmin;
    _qp_x_ub                    = _Kmax;
    _qp_A_lb.topRows<3>()       = _F_min - d;
    _qp_A_ub.topRows<3>()       = _F_max - d;
    _qp_A_lb.bottomRows<2>()    = -1e9 * Eigen::Vector2d::Ones();
    _qp_A_ub.bottomRows<2>()    = Eigen::Vector2d({k1, k2});


    // Solve the QP problem:
    qpOASES::int_t       nWSR = 10;
    qpOASES::returnValue ret  = _qp_prob.init(
            _qp_H.data(),
            _qp_g.data(),
            _qp_A.data(),
            _qp_x_lb.data(),
            _qp_x_ub.data(),
            _qp_A_lb.data(),
            _qp_A_ub.data(),
            nWSR
    );
    qpOASES::int_t qp_solve_status = qpOASES::getSimpleStatus(ret);
    _qp_prob.getPrimalSolution(_qp_x_sol.data());


    if (qp_solve_status != qpOASES::SUCCESSFUL_RETURN) {
        if (qp_solve_status == -2) {
            RCLCPP_ERROR(get_node()->get_logger(), "Unfeasible QP (err. code %d)", ret);
        } else {
            RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "QP solver failed with code %d, init code: %d",
                    qp_solve_status,
                    ret
            );
        }
        // Maybe set default values
        return;
    }

    // Update the stiffness matrix
    _K = _qp_x_sol.asDiagonal();
    _updateDamping();
    m_stiffness.topLeftCorner<3, 3>() = _K;

    // Integrate energy tank
    const ctrl::Matrix3D Kmin    = _Kmin.asDiagonal();
    const ctrl::Vector3D w       = -(_K - Kmin) * x_tilde;
    double               dx_tank = sigma / T * xd_tilde.transpose() * _D * xd_tilde;
    dx_tank += -1 / T * w.transpose() * xd_tilde;
    _x_tank += dx_tank * _dt;
}

void CartesianAdaptiveComplianceController::_updateDamping() {
    _D = 2.0 * 0.707 * _K.diagonal().cwiseSqrt().asDiagonal();
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
    _kin_solver->JntToCart(joint_data, frame_vel);
    return frame_vel;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
        cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController,
        controller_interface::ControllerInterface
)
