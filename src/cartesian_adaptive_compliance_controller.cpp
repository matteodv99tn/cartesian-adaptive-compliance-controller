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
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>

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
#include "std_msgs/msg/float64_multi_array.hpp"

#ifdef LOGGING
#include "fmt/core.h"
#include "fmt/os.h"
#include "fmt/ostream.h"
#endif

using cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using std_msgs::msg::Float64MultiArray;

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

//  _                      _
// | |    ___   __ _  __ _(_)_ __   __ _
// | |   / _ \ / _` |/ _` | | '_ \ / _` |
// | |__| (_) | (_| | (_| | | | | | (_| |
// |_____\___/ \__, |\__, |_|_| |_|\__, |
//             |___/ |___/         |___/
// The following are macros that are expanded at compile time to have a less verbose
// code while logging variables to a csv file
//
// For instance calling
//    log_vector<6>(*_logfile, _Kmin);
// will expand to
//    _logfile->print("{}", _Kmin(0));
//    _logfile->print("{}", _Kmin(1));
//    ... (up to 6 elements)
//
// Similarly
//   log_vector_heading<6>(*_logfile, "K");
// will expand to
//   _logfile->print("K{}", 0);
//   _logfile->print("K{}", 1);
//   ... (up to 6 elements)
#ifdef LOGGING
template <int Size, typename Stream, typename VectorType, int... Is>
void __log_vec(Stream& stream, const VectorType& vec, std::integer_sequence<int, Is...>) {
    (..., (stream.print("{},", vec(Is))));
}

template <int Size, typename Stream, typename VectorType>
void log_vector(Stream& stream, const VectorType& vec) {
    __log_vec<Size, Stream, VectorType>(
            static_cast<Stream&>(stream), vec, std::make_integer_sequence<int, Size>()
    );
}

template <int Size, typename Stream, int... Is>
void __log_vec_heading(Stream& stream, const std::string& prefix, std::integer_sequence<int, Is...>) {
    (..., (stream.print("{}{},", prefix, Is)));
}

template <int Size, typename Stream>
void log_vector_heading(Stream& stream, const std::string& prefix) {
    __log_vec_heading<Size, Stream>(
            stream, prefix, std::make_integer_sequence<int, Size>()
    );
}
#endif

const Eigen::Vector3d quat_logarithmic_map(const Eigen::Quaterniond& q) {
    const Eigen::Quaterniond q_normalized = q.normalized();
    const Eigen::Vector3d    u            = q_normalized.vec();
    const double             nu           = q_normalized.w();

    if (u.norm() < 1e-9) return Eigen::Vector3d::Zero();
    else return std::acos(nu) * u.normalized();
}

CartesianAdaptiveComplianceController::CartesianAdaptiveComplianceController() :
        CartesianComplianceController() {
#ifdef LOGGING
    _logfile = std::make_unique<fmt::v8::ostream>(fmt::output_file("controller_data.csv"
    ));
    _configfile = std::make_unique<fmt::v8::ostream>(
            fmt::output_file("controller_configuration.txt")
    );
#endif
}

CartesianAdaptiveComplianceController::~CartesianAdaptiveComplianceController() {
#ifdef LOGGING
    _logfile->close();
    _configfile->close();

    log_vector_heading<6>(*_logfile, "K");
    _logfile->print("t,");
    _logfile->print("x_tank,");
    _logfile->print("dx_tank,");
    _logfile->print("tank_energy,");
    _logfile->print("sigma,");
    log_vector_heading<6>(*_logfile, "K");
    _logfile->print("x_tilde,y_tilde,z_tilde,rx_tilde,ry_tilde,rz_tilde,");
    _logfile->print("dx_tilde,dy_tilde,dz_tilde,drx_tilde,dry_tilde,drz_tilde,");
    _logfile->print("x_curr,y_curr,z_curr,qx_curr,qy_curr,qz_curr,qw_curr,");
    _logfile->print("x_des,y_des,z_des,qx_des,qy_des,qz_des,qw_des,");
    _logfile->print("vx_curr,vy_curr,vz_curr,wx_curr,wy_curr,wz_curr,");
    _logfile->print("vx_des,vy_des,vz_des,wx_des,wy_des,wz_des,");
    _logfile->print("Fx_des,Fy_des,Fz_des,Tx_des,Ty_des,Tz_des,");
    _logfile->print("\n");
#endif
}

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
    _des_vel = decltype(_des_vel)::Zero();
    auto on_target_velocity_received
            = [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                  this->_des_vel = ctrl::Vector6D(
                          {msg->twist.linear.x,
                           msg->twist.linear.y,
                           msg->twist.linear.z,
                           msg->twist.angular.x,
                           msg->twist.angular.y,
                           msg->twist.angular.z}
                  );
              };
    _twist_sub = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            get_node()->get_parameter("target_velocity_topic").as_string(),
            10,
            on_target_velocity_received
    );

    // Create subscription for desired wrench
    _des_wrench = decltype(_des_wrench)::Zero();
    auto on_target_wrench_received
            = [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                  this->_des_wrench = ctrl::Vector6D(
                          {msg->wrench.force.x,
                           msg->wrench.force.y,
                           msg->wrench.force.z,
                           msg->wrench.torque.x,
                           msg->wrench.torque.y,
                           msg->wrench.torque.z}
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
    _t  = 0;
    logParameters();
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

void CartesianAdaptiveComplianceController::logParameters() const {
    const auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "Sampling period: %f", _dt);
    RCLCPP_INFO(logger, "Tank state (energy): %f (%f)", _x_tank, _tankEnergy());
    RCLCPP_INFO(
            logger,
            "Minimum stiffness: %f, %f, %f, %f, %f, %f",
            _Kmin(0),
            _Kmin(1),
            _Kmin(2),
            _Kmin(3),
            _Kmin(4),
            _Kmin(5)
    );
    RCLCPP_INFO(
            logger,
            "Maximum stiffness: %f, %f, %f, %f, %f, %f",
            _Kmax(0),
            _Kmax(1),
            _Kmax(2),
            _Kmax(3),
            _Kmax(4),
            _Kmax(5)
    );
    RCLCPP_INFO(
            logger,
            "Minimum wrench: %f, %f, %f, %f, %f, %f",
            _F_min(0),
            _F_min(1),
            _F_min(2),
            _F_min(3),
            _F_min(4),
            _F_min(5)
    );
    RCLCPP_INFO(
            logger,
            "Maximum wrench: %f, %f, %f, %f, %f, %f",
            _F_max(0),
            _F_max(1),
            _F_max(2),
            _F_max(3),
            _F_max(4),
            _F_max(5)
    );
    RCLCPP_INFO(
            logger,
            "Q weights: %f, %f, %f, %f, %f, %f",
            _Q(0, 0),
            _Q(1, 1),
            _Q(2, 2),
            _Q(3, 3),
            _Q(4, 4),
            _Q(5, 5)
    );
    RCLCPP_INFO(
            logger,
            "R weights: %f, %f, %f, %f, %f, %f",
            _R(0, 0),
            _R(1, 1),
            _R(2, 2),
            _R(3, 3),
            _R(4, 4),
            _R(5, 5)
    );
    _configfile->print(
            "R weights: {}, {}, {}, {}, {}, {}\n",
            _R(0, 0),
            _R(1, 1),
            _R(2, 2),
            _R(3, 3),
            _R(4, 4),
            _R(5, 5)
    );
    ;
    _configfile->print(
            "Q weights: {}, {}, {}, {}, {}, {}\n",
            _Q(0, 0),
            _Q(1, 1),
            _Q(2, 2),
            _Q(3, 3),
            _Q(4, 4),
            _Q(5, 5)
    );
    ;
    _configfile->print(
            "Minimum stiffnesses: {}, {}, {}, {}, {}, {}\n",
            _Kmin(0),
            _Kmin(1),
            _Kmin(2),
            _Kmin(3),
            _Kmin(4),
            _Kmin(5)
    );
    _configfile->print(
            "Maximum stiffnesses: {}, {}, {}, {}, {}, {}\n",
            _Kmax(0),
            _Kmin(1),
            _Kmin(2),
            _Kmin(3),
            _Kmin(4),
            _Kmin(5)
    );
    _configfile->print(
            "Minimum wrench: {}, {}, {}, {}, {}, {}\n",
            _F_min(0),
            _F_min(1),
            _F_min(2),
            _F_min(3),
            _F_min(4),
            _F_min(5)
    );
    _configfile->print(
            "Maximum wrench: {}, {}, {}, {}, {}, {}\n",
            _F_max(0),
            _F_max(1),
            _F_max(2),
            _F_max(3),
            _F_max(4),
            _F_max(5)
    );
    _configfile->print("Sampling period: {}\n", _dt);
    _configfile->print(
            "Tank initial state : {}\n",
            get_node()->get_parameter("tank.initial_state").as_double()
    );
    _configfile->print(
            "Tank minimum energy: {}\n",
            get_node()->get_parameter("tank.minimum_energy").as_double()
    );
    _configfile->print(
            "Tank eta: {}\n", get_node()->get_parameter("tank.eta").as_double()
    );
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
    std::vector<double> Kmin_vals
            = get_node()->get_parameter("Qp.K_min").as_double_array();
    std::cout << "Loaded Kmin" << std::endl;
    std::vector<double> Kmax_vals
            = get_node()->get_parameter("Qp.K_max").as_double_array();
    std::cout << "Loaded Kmax" << std::endl;
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
    _Kmin = ctrl::Vector6D(Kmin_vals.data());
    _Kmax = ctrl::Vector6D(Kmax_vals.data());
    _K    = _Kmin.asDiagonal();
    _updateDamping();
    m_stiffness = _Kmin.asDiagonal();

    // Weight matrices for the QP problem
    auto Q_weights = get_node()->get_parameter("Qp.Q_weights").as_double_array();
    std::cout << "Loaded Q weights" << std::endl;
    auto R_weights = get_node()->get_parameter("Qp.R_weights").as_double_array();
    std::cout << "Loaded R weights" << std::endl;
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
    _Q = ctrl::Vector6D(Q_weights.data()).asDiagonal();
    _R = ctrl::Vector6D(R_weights.data()).asDiagonal();

    // Force limits
    auto F_min = get_node()->get_parameter("Qp.F_min").as_double_array();
    std::cout << "Loaded F_min" << std::endl;
    auto F_max = get_node()->get_parameter("Qp.F_max").as_double_array();
    std::cout << "Loaded F_max" << std::endl;
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
    _F_min = ctrl::Vector6D(F_min.data());
    _F_max = ctrl::Vector6D(F_max.data());
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
    const ctrl::Vector3D pos_curr{// Current position
                                  frame_vel.p.p.x(),
                                  frame_vel.p.p.y(),
                                  frame_vel.p.p.z()};
    Eigen::Quaterniond   quat_curr;
    frame_vel.M.R.GetQuaternion(
            quat_curr.x(), quat_curr.y(), quat_curr.z(), quat_curr.w()
    );

    const ctrl::Vector6D xd{// Current velocity
                            frame_vel.p.v.x(),
                            frame_vel.p.v.y(),
                            frame_vel.p.v.z(),
                            frame_vel.M.w.x(),
                            frame_vel.M.w.y(),
                            frame_vel.M.w.z()};

    const ctrl::Vector3D pos_des{// Desired position
                                 MotionBase::m_target_frame.p.x(),
                                 MotionBase::m_target_frame.p.y(),
                                 MotionBase::m_target_frame.p.z()};
    Eigen::Quaterniond   quat_des;
    MotionBase::m_target_frame.M.GetQuaternion(
            quat_des.x(), quat_des.y(), quat_des.z(), quat_des.w()
    );

    ctrl::Vector6D x_tilde;  // pos tracking error
    x_tilde.head<3>() = pos_des - pos_curr;
    x_tilde.tail<3>() = quat_logarithmic_map(quat_des * quat_curr.inverse());

    const ctrl::Vector6D xd_tilde = _des_vel - xd;  // vel tracking error

    const ctrl::Matrix6D X_tilde
            = x_tilde.asDiagonal();  // pos tracking error diag matrix
    const ctrl::Matrix6D Xd_tilde = xd_tilde.asDiagonal();

    const ctrl::Vector6D d = _D * xd_tilde;
    const ctrl::Vector6D f = d - _des_wrench;

    const ctrl::Vector6D A_bot_row = -Xd_tilde * x_tilde;

    const double T     = _tankEnergy();
    const double Tmin  = get_node()->get_parameter("tank.minimum_energy").as_double();
    const double sigma = T >= 1.0 ? 1 : 0;
    const double eta   = get_node()->get_parameter("tank.eta").as_double();

    const double k_tmp1 = sigma * xd_tilde.transpose() * _D * xd_tilde;
    const double k_tmp2 = x_tilde.transpose() * _Kmin.asDiagonal() * xd_tilde;
    const double k_tmp3 = k_tmp1 - k_tmp2;

    const double k1 = k_tmp3 + (T - Tmin) / _dt;
    const double k2 = k_tmp3 - eta;


    // Fill the QP problem
    _qp_H                       = X_tilde.transpose() * _Q * X_tilde + _R;
    _qp_g                       = X_tilde.transpose() * _Q * f - _R * _Kmin;
    _qp_A.topLeftCorner<6, 6>() = X_tilde;
    _qp_A.row(6)                = A_bot_row;
    _qp_A.row(7)                = A_bot_row;
    _qp_x_lb                    = _Kmin;
    _qp_x_ub                    = _Kmax;
    _qp_A_lb.topRows<6>()       = _F_min - d;
    _qp_A_ub.topRows<6>()       = _F_max - d;
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
    m_stiffness = _K;

    // Integrate energy tank
    const ctrl::Matrix6D Kmin    = _Kmin.asDiagonal();
    const ctrl::Vector6D w       = -(_K - Kmin) * x_tilde;
    double               dx_tank = sigma / T * xd_tilde.transpose() * _D * xd_tilde;
    dx_tank += -1 / T * w.transpose() * xd_tilde;
    _x_tank += dx_tank * _dt;

    // Log the data
#ifdef LOGGING
    _logfile->print("{},", _t);
    _logfile->print("{},", _x_tank);
    _logfile->print("{},", dx_tank);
    _logfile->print("{},", _tankEnergy());
    _logfile->print("{},", sigma);
    log_vector<6>(*_logfile, _K.diagonal());
    log_vector<6>(*_logfile, x_tilde);
    log_vector<6>(*_logfile, xd_tilde);
    log_vector<3>(*_logfile, pos_curr);
    log_vector<4>(*_logfile, quat_curr.coeffs());
    log_vector<3>(*_logfile, pos_des);
    log_vector<4>(*_logfile, quat_des.coeffs());
    log_vector<6>(*_logfile, xd);
    log_vector<6>(*_logfile, _des_vel);
    log_vector<6>(*_logfile, _des_wrench);
    _logfile->print("\n");
#endif

    Float64MultiArray tank_state_msg;
    Float64MultiArray stiffness_msg;
    Float64MultiArray damping_msg;
    Float64MultiArray xtilde_msg;
    Float64MultiArray dxtilde_msg;

    tank_state_msg.data.reserve(3);
    stiffness_msg.data.reserve(6);
    damping_msg.data.reserve(6);

    for (int i = 0; i < 6; i++) {
        tank_state_msg.data[i] = _K(i, i);
        damping_msg.data[i] = _D(i, i);
        xtilde_msg.data[i] = x_tilde(i);
        dxtilde_msg.data[i] = xd_tilde(i);
    }
    tank_state_msg.data[0] = _x_tank;
    tank_state_msg.data[1] = dx_tank;
    tank_state_msg.data[2] = _tankEnergy();

    _tank_state_pub->publish(tank_state_msg);
    _stiffness_pub->publish(stiffness_msg);
    _damping_pub->publish(damping_msg);
    _xtilde_pub->publish(xtilde_msg);
    _dxtilde_pub->publish(dxtilde_msg);

    _t += _dt;
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
