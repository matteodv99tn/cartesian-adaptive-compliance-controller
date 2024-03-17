/**
 * Changes are in the functions:
 * - on_configure(): this changes might not be relevant to me
 * - on_activate(): there are changes
 *      mainly initialisation of publisher/subscribers (for both logging and control
 * purposes)
 * - update(): this must be changed for sure, since it implements the main control
 * algorithm sbase where we can call the parent class!
 *
 */

#include "cartesian_adaptive_compliance_controller/cartesian_adaptive_compliance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "qpOASES.hpp"
#include "qpOASES/MessageHandling.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController;
using geometry_msgs::msg::TwistStamped;
using geometry_msgs::msg::WrenchStamped;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using std_msgs::msg::Float64MultiArray;

using Quaternion  = Eigen::Quaterniond;
using Transform3d = Eigen::Transform<double, 3, Eigen::Affine>;
using Vector3d    = Eigen::Matrix<double, 3, 1>;
using Vector6d    = Eigen::Matrix<double, 6, 1>;
using Matrix3d    = Eigen::Matrix<double, 3, 3>;
using Matrix6d    = Eigen::Matrix<double, 6, 6>;

namespace defaults {
const std::string         target_velocity_topic = "/target_velocity";
const std::string         target_wrench_topic   = "/target_wrench";
const double              sampling_period       = 0.002;
const double              x0_tank               = 1.0;
const double              xmin_tank             = 0.4;
const double              eta_tank              = -0.1;
const std::vector<double> Q_weights = {3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0};
const std::vector<double> R_weights = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
const std::vector<double> F_min     = {-15.0, -15.0, -15.0, -1.5, -1.5, -1.5};
const std::vector<double> F_max     = {15.0, 15.0, 15.0, 1.5, 1.5, 1.5};
const std::vector<double> K_min     = {300.0, 300.0, 100.0, 30.0, 30.0, 30.0};
const std::vector<double> K_max     = {1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0};
}  // namespace defaults

//  _   _      _                   _____
// | | | | ___| |_ __   ___ _ __  |  ___|   _ _ __   ___ ___
// | |_| |/ _ \ | '_ \ / _ \ '__| | |_ | | | | '_ \ / __/ __|
// |  _  |  __/ | |_) |  __/ |    |  _|| |_| | | | | (__\__ \_
// |_| |_|\___|_| .__/ \___|_|    |_|   \__,_|_| |_|\___|___(_)
//              |_|
// Helper functions

std::optional<pinocchio::FrameIndex> get_frame_id(
        const pinocchio::Model& model, const std::string& frame_name
) {
    const auto frame_id = model.getFrameId(frame_name);
    if (frame_id == model.frames.size()) return std::nullopt;
    return frame_id;
}

std::optional<pinocchio::SE3> get_frame(
        const pinocchio::Model& model,
        const pinocchio::Data&  data,
        const std::string&      frame_name
) {
    auto frame_id = get_frame_id(model, frame_name);
    if (!frame_id) return std::nullopt;
    return data.oMf[frame_id.value()];
}

Vector3d get_position(const KDL::Frame& frame) {
    return {frame.p.x(), frame.p.y(), frame.p.z()};
}

Quaternion get_quaternion(const KDL::Frame& frame) {
    Quaternion quat;
    frame.M.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    return quat;
}

Vector6d stack_vectors(const Vector3d& v1, const Vector3d& v2) {
    return Vector6d({v1(0), v1(1), v1(2), v2(0), v2(1), v2(2)});
}

Vector6d rotate_6d_vec(const Matrix3d& R, const Vector6d& vec) {
    return stack_vectors(R * vec.head<3>(), R * vec.tail<3>());
}

Vector6d rotate_6d_vec(const pinocchio::SE3& RF, const Vector6d& vec) {
    return rotate_6d_vec(RF.rotation(), vec);
}

Vector6d rotate_6d_vec(const std::optional<pinocchio::SE3> RF, const Vector6d& vec) {
    return rotate_6d_vec(RF.value_or(pinocchio::SE3::Identity()).rotation(), vec);
}

const Vector3d quat_logarithmic_map(const Quaternion& q) {
    const Quaternion q_normalized = q.normalized();
    const Vector3d   u            = q_normalized.vec();
    const double     nu           = q_normalized.w();

    if (u.norm() < 1e-9) return Vector3d::Zero();
    else return std::acos(nu) * u.normalized();
}

//  _   _           _        _     _  __                      _
// | \ | | ___   __| | ___  | |   (_)/ _| ___  ___ _   _  ___| | ___
// |  \| |/ _ \ / _` |/ _ \ | |   | | |_ / _ \/ __| | | |/ __| |/ _ \
// | |\  | (_) | (_| |  __/ | |___| |  _|  __/ (__| |_| | (__| |  __/
// |_| \_|\___/ \__,_|\___| |_____|_|_|  \___|\___|\__, |\___|_|\___|
//                                                 |___/

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

    // Initialisation of the variables
    _initialize_variables();
    _initialize_qp_problem();

    _tank_state_pub = get_node()->create_publisher<Float64MultiArray>(
            "/log/tank_state", rclcpp::QoS(10).transient_local()
    );
    _stiffness_pub = get_node()->create_publisher<Float64MultiArray>(
            "/log/stiffness", rclcpp::QoS(10).transient_local()
    );
    _damping_pub = get_node()->create_publisher<Float64MultiArray>(
            "/log/damping", rclcpp::QoS(10).transient_local()
    );
    _xtilde_pub = get_node()->create_publisher<Float64MultiArray>(
            "/log/xtilde", rclcpp::QoS(10).transient_local()
    );
    _dxtilde_pub = get_node()->create_publisher<Float64MultiArray>(
            "/log/dxtilde", rclcpp::QoS(10).transient_local()
    );
    _q.resize(Base::m_joint_state_pos_handles.size());
    _qd.resize(Base::m_joint_state_pos_handles.size());
    _des_vel_link    = "world";
    _des_wrench_link = "world";

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
    _des_vel   = decltype(_des_vel)::Zero();
    _twist_sub = get_node()->create_subscription<TwistStamped>(
            get_node()->get_parameter("target_velocity_topic").as_string(),
            10,
            [this](const TwistStamped::SharedPtr msg) {
                this->_onDesiredTwistReceived(msg);
            }
    );

    // Create subscription for desired wrench
    _des_wrench = decltype(_des_wrench)::Zero();
    _wrench_sub = get_node()->create_subscription<WrenchStamped>(
            get_node()->get_parameter("target_wrench_topic").as_string(),
            10,
            [this](const WrenchStamped::SharedPtr msg) {
                this->_onDesiredWrenchReceived(msg);
            }
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

    _q  = Eigen::VectorXd::Zero(joints.size());
    _qd = Eigen::VectorXd::Zero(joints.size());
    pinocchio::urdf::buildModelFromXML(
            get_node()->get_parameter("robot_description").as_string(), _pin_model
    );
    _pin_data = pinocchio::Data(_pin_model);

    _base_link_name = get_node()->get_parameter("robot_base_link").as_string();
    _compliance_link_name
            = get_node()->get_parameter("compliance_ref_link").as_string();
    _ee_link_name       = get_node()->get_parameter("end_effector_link").as_string();
    _base_link_id       = _pin_model.getFrameId(_base_link_name);
    _ee_link_id         = _pin_model.getFrameId(_ee_link_name);
    _compliance_link_id = _pin_model.getFrameId(_compliance_link_name);

    _dt = get_node()->get_parameter("sampling_period").as_double();
    _t  = 0;
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

controller_interface::return_type CartesianAdaptiveComplianceController::update(
        const rclcpp::Time& time, const rclcpp::Duration& period
) {
    // TODO: proper control loop
    // Most probably we can't directly call CartesianComplianceController::update()
    // since we need to add stuff in the middle of the code

    // Synchronize the internal model and the real robot
    Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);
    _synchronise_pinocchio_model();
    if (!_update_stiffness()) return controller_interface::return_type::ERROR;

    // --- Same control loop of the cartesian compliance controller
    for (int i = 0; i < Base::m_iterations; ++i) {
        auto     internal_period = rclcpp::Duration::from_seconds(0.02);
        Vector6d error
                = CartesianAdaptiveComplianceController::_compute_compliance_error();
        Base::computeJointControlCmds(error, internal_period);
    }
    Base::writeJointControlCmds();

    return controller_interface::return_type::OK;
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__|
// | |  | |  __/ | | | | | |_) |  __/ |
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|
//
//   __                  _   _
//  / _|_   _ _ __   ___| |_(_) ___  _ __  ___
// | |_| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
// |  _| |_| | | | | (__| |_| | (_) | | | \__ \
// |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
//
void CartesianAdaptiveComplianceController::_update_damping() {
    _D = 2.0 * _K.diagonal().cwiseSqrt().asDiagonal();
    // _D = 0.02 * _K.diagonal().cwiseSqrt().asDiagonal();
}

void CartesianAdaptiveComplianceController::_onDesiredTwistReceived(
        const TwistStamped::SharedPtr msg
) {
    _des_vel = Vector6d(
            msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z,
            msg->twist.angular.x,
            msg->twist.angular.y,
            msg->twist.angular.z
    );
    _des_vel_link = msg->header.frame_id;
}

void CartesianAdaptiveComplianceController::_onDesiredWrenchReceived(
        const WrenchStamped::SharedPtr msg
) {
    _des_wrench = Vector6d(
            msg->wrench.force.x,
            msg->wrench.force.y,
            msg->wrench.force.z,
            msg->wrench.torque.x,
            msg->wrench.torque.y,
            msg->wrench.torque.z
    );
    _des_wrench_link = msg->header.frame_id;
}

Vector6d CartesianAdaptiveComplianceController::_compute_compliance_error() {
    Vector6d net_force = Base::displayInBaseLink(m_stiffness, m_compliance_ref_link)
                                 * MotionBase::computeMotionError()
                         - Base::displayInBaseLink(_D, m_compliance_ref_link) * _ee_vel
                         + ForceBase::computeForceError();
    return net_force;
}

void CartesianAdaptiveComplianceController::_synchronise_pinocchio_model() {
    for (std::size_t i = 0; i < _joint_state_vel_handles.size(); ++i) {
        _q(i)  = Base::m_joint_state_pos_handles[i].get().get_value();
        _qd(i) = _joint_state_vel_handles[i].get().get_value();
    }
    pinocchio::forwardKinematics(_pin_model, _pin_data, _q, _qd);
}

void CartesianAdaptiveComplianceController::_initialize_variables() {
    // Tank initialisation
    _x_tank = get_node()->get_parameter("tank.initial_state").as_double();

    // Minimum and maximum stiffness
    std::vector<double> Kmin_vals
            = get_node()->get_parameter("Qp.K_min").as_double_array();
    std::vector<double> Kmax_vals
            = get_node()->get_parameter("Qp.K_max").as_double_array();
    if (Kmin_vals.size() != 6) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "K_min must have 6 elements, switching to default values"
        );
        Kmin_vals = defaults::K_min;
    }
    if (Kmax_vals.size() != 6) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "K_max must have 6 elements, switching to default values"
        );
        Kmax_vals = defaults::K_max;
    }
    _Kmin = Vector6d(Kmin_vals.data());
    _Kmax = Vector6d(Kmax_vals.data());
    _K    = _Kmin.asDiagonal();
    _update_damping();
    m_stiffness = _Kmin.asDiagonal();

    // Weight matrices for the QP problem
    auto Q_weights = get_node()->get_parameter("Qp.Q_weights").as_double_array();
    auto R_weights = get_node()->get_parameter("Qp.R_weights").as_double_array();
    if (Q_weights.size() != 6) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "Q_weights must have 6 elements, switching to default values"
        );
        Q_weights = defaults::Q_weights;
    }
    if (R_weights.size() != 6) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "R_weights must have 6 elements, switching to default values"
        );
        R_weights = defaults::R_weights;
    }
    _Q = Vector6d(Q_weights.data()).asDiagonal();
    _R = Vector6d(R_weights.data()).asDiagonal();

    auto F_min = get_node()->get_parameter("Qp.F_min").as_double_array();
    auto F_max = get_node()->get_parameter("Qp.F_max").as_double_array();
    if (F_min.size() != 6) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "F_min must have 6 elements, switching to default values (got %lu)",
                F_min.size()
        );
        F_min = defaults::F_min;
    }
    if (F_max.size() != 6) {
        RCLCPP_WARN(
                get_node()->get_logger(),
                "F_max must have 6 elements, switching to default values (got %lu)",
                F_max.size()
        );
        F_max = defaults::F_max;
    }
    _F_min = Vector6d(F_min.data());
    _F_max = Vector6d(F_max.data());
}

void CartesianAdaptiveComplianceController::_initialize_qp_problem() {
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

//  __  __       _              _        _
// |  \/  | __ _(_)_ __     ___| |_ _ __| |
// | |\/| |/ _` | | '_ \   / __| __| '__| |
// | |  | | (_| | | | | | | (__| |_| |  | |
// |_|  |_|\__,_|_|_| |_|  \___|\__|_|  |_|
//
//   __                  _   _
//  / _|_   _ _ __   ___| |_(_) ___  _ __
// | |_| | | | '_ \ / __| __| |/ _ \| '_ \
// |  _| |_| | | | | (__| |_| | (_) | | | |
// |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|
//
bool CartesianAdaptiveComplianceController::_update_stiffness() {
    using pinocchio::SE3;
    pinocchio::updateFramePlacements(_pin_model, _pin_data);
    const SE3& ee_frame         = _pin_data.oMf[_ee_link_id];
    const SE3& compliance_frame = _pin_data.oMf[_compliance_link_id];

    const SE3 world_to_compliance = compliance_frame.inverse();

    // Get the current and desired position and orientation (in world frame)
    const Vector3d   target_pos = get_position(MotionBase::m_target_frame);
    const Quaternion target_ori = get_quaternion(MotionBase::m_target_frame);
    const Vector3d   curr_pos   = ee_frame.translation();
    const Quaternion curr_ori(ee_frame.rotation());

    // Cartesian error (in world frame)
    const Vector3d pos_err = target_pos - curr_pos;
    const Vector3d ori_err = quat_logarithmic_map(target_ori * curr_ori.conjugate());

    // Transform the error to the compliance frame
    const Vector6d x_tilde
            = rotate_6d_vec(world_to_compliance, stack_vectors(pos_err, ori_err));

    // EE velocity in world frame
    const Vector6d ee_vel_world = pinocchio::getFrameVelocity(
            _pin_model, _pin_data, _ee_link_id, pinocchio::WORLD
    );
    // Compute desired velocity in world frame
    const auto     des_vel_frame = get_frame(_pin_model, _pin_data, _des_vel_link);
    const Vector6d des_vel_world = rotate_6d_vec(des_vel_frame, _des_vel);
    if (!des_vel_frame.has_value()) {
        // Notify the user that the desired velocity frame was not found
        RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Desired velocity frame (%s) not found",
                _des_vel_link.c_str()
        );
    }

    // Cartesian velocity error (in compliance frame)
    const Vector6d xd_tilde = rotate_6d_vec(
            world_to_compliance.rotation(), des_vel_world - ee_vel_world
    );

    // Project desired wrench to compliance frame
    const auto des_wrench_frame = get_frame(_pin_model, _pin_data, _des_wrench_link);
    const Vector6d des_wrench_world = rotate_6d_vec(des_wrench_frame, _des_wrench);
    const Vector6d des_wrench
            = rotate_6d_vec(world_to_compliance.rotation(), des_wrench_world);
    if (!des_wrench_frame.has_value()) {
        RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Desired wrench frame (%s) not found",
                _des_wrench_link.c_str()
        );
    }

    const Matrix6d X_tilde  = x_tilde.asDiagonal();  // pos tracking error diag matrix
    const Matrix6d Xd_tilde = xd_tilde.asDiagonal();

    const Vector6d d = _D * xd_tilde;
    const Vector6d f = d - des_wrench;

    const Vector6d A_bot_row = -Xd_tilde * x_tilde;

    const double T     = _tankEnergy();
    const double Tmin  = get_node()->get_parameter("tank.minimum_energy").as_double();
    const double sigma = (T <= 1.0) ? 1.0 : 0.0;
    const double eta = get_node()->get_parameter("tank.eta").as_double();

    const double k_tmp1 = sigma * xd_tilde.transpose() * _D * xd_tilde;
    const double k_tmp2 = x_tilde.transpose() * _Kmin.asDiagonal() * xd_tilde;
    const double k_tmp3 = k_tmp1 - k_tmp2;

    const double k1 = k_tmp3 + (T - Tmin) / _dt;
    const double k2 = k_tmp3 - eta;
    const double k3 = k_tmp3 - 0.2;


    // Fill the QP problem
    _qp_H                       = X_tilde.transpose() * _Q * X_tilde + _R;
    _qp_g                       = X_tilde.transpose() * _Q * f - _R * _Kmin;
    _qp_A.topLeftCorner<6, 6>() = X_tilde;
    _qp_A.row(6)                = A_bot_row;
    _qp_A.row(7)                = A_bot_row;
    _qp_A.row(8)                = A_bot_row;
    _qp_x_lb                    = _Kmin;
    _qp_x_ub                    = _Kmax;
    _qp_A_lb.topRows<6>()       = _F_min - d;
    _qp_A_ub.topRows<6>()       = _F_max - d;
    _qp_A_lb.bottomRows<3>()    = Vector3d({-1e9, -1e9, k3});
    _qp_A_ub.bottomRows<3>()    = Vector3d({k1, k2, 1e9});


    // Solve the QP problem:
    qpOASES::int_t       nWSR = 20;
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
#if 0
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
#endif
    } else {
        // Update the stiffness matrix
        _K = _qp_x_sol.asDiagonal();
        _update_damping();
        m_stiffness = _K;
    }

    // Integrate energy tank
    const Matrix6d Kmin = _Kmin.asDiagonal();
    Vector6d       w    = -(_K - Kmin) * x_tilde;
    if (T < Tmin) w = Vector6d::Zero();
    const double dx_tank_1 = sigma / _x_tank * xd_tilde.transpose() * _D * xd_tilde;
    const double dx_tank_2 = -(1 / _x_tank) * w.transpose() * xd_tilde;
    const double dx_tank   = dx_tank_1 + dx_tank_2;
    _x_tank += dx_tank * _dt;

    // Log data
    Float64MultiArray tank_state_msg;
    Float64MultiArray stiffness_msg;
    Float64MultiArray damping_msg;
    Float64MultiArray xtilde_msg;
    Float64MultiArray dxtilde_msg;

    tank_state_msg.data.resize(5);
    stiffness_msg.data.resize(6);
    damping_msg.data.resize(6);
    xtilde_msg.data.resize(6);
    dxtilde_msg.data.resize(6);

    for (int i = 0; i < 6; i++) {
        stiffness_msg.data[i] = _K(i, i);
        damping_msg.data[i]   = _D(i, i);
        xtilde_msg.data[i]    = x_tilde(i);
        dxtilde_msg.data[i]   = xd_tilde(i);
    }
    tank_state_msg.data[0] = _x_tank;
    tank_state_msg.data[1] = dx_tank;
    tank_state_msg.data[2] = _tankEnergy();
    tank_state_msg.data[3] = qp_solve_status == qpOASES::SUCCESSFUL_RETURN;
    tank_state_msg.data[4] = dx_tank_2;

    _tank_state_pub->publish(tank_state_msg);
    _stiffness_pub->publish(stiffness_msg);
    _damping_pub->publish(damping_msg);
    _xtilde_pub->publish(xtilde_msg);
    _dxtilde_pub->publish(dxtilde_msg);

    m_stiffness = Matrix6d::Zero();
    m_stiffness = Vector6d(100, 100, 100, 10, 10, 10).asDiagonal();

    _t += _dt;
    return true;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
        cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController,
        controller_interface::ControllerInterface
)
