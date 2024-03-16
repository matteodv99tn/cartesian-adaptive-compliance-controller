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
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <memory>
#include <new>
#include <utility>
#include <vector>

#include <kdl/treefksolverpos_recursive.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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

#ifdef LOGGING
#include "fmt/core.h"
#include "fmt/os.h"
#include "fmt/ostream.h"
#endif

using cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using std_msgs::msg::Float64MultiArray;

using Transform3d = Eigen::Transform<double, 3, Eigen::Affine>;

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


int get_segment_number(const KDL::Chain& chain, const std::string segment_name) {
    for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
        if (chain.getSegment(i).getName() == segment_name) { return i; }
    }
    return -1;
}

Eigen::Vector3d get_position(const KDL::FrameVel& frame) {
    return {frame.p.p.x(), frame.p.p.y(), frame.p.p.z()};
}

Eigen::Vector3d get_position(const KDL::Frame& frame) {
    return {frame.p.x(), frame.p.y(), frame.p.z()};
}

Eigen::Quaterniond get_quaternion(const KDL::FrameVel& frame) {
    Eigen::Quaterniond quat;
    frame.M.R.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    return quat;
}

Eigen::Quaterniond get_quaternion(const KDL::Frame& frame) {
    Eigen::Quaterniond quat;
    frame.M.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    return quat;
}

Eigen::Vector3d get_linear_vel(const KDL::FrameVel& frame) {
    return {frame.p.v.x(), frame.p.v.y(), frame.p.v.z()};
}

Eigen::Vector3d get_angular_vel(const KDL::FrameVel& frame) {
    return {frame.M.w.x(), frame.M.w.y(), frame.M.w.z()};
}

Transform3d compute_transform(const KDL::FrameVel& from, const KDL::FrameVel& to) {
    const Eigen::Vector3d    from_pos = get_position(from);
    const Eigen::Vector3d    to_pos   = get_position(to);
    const Eigen::Quaterniond from_ori = get_quaternion(from);
    const Eigen::Quaterniond to_ori   = get_quaternion(to);

    Transform3d from_transform, to_transform;
    from_transform = Eigen::Translation3d(from_pos) * from_ori;
    to_transform   = Eigen::Translation3d(to_pos) * to_ori;
    return to_transform.inverse() * from_transform;
}

ctrl::Vector6D stack_vector(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    return ctrl::Vector6D({v1(0), v1(1), v1(2), v2(0), v2(1), v2(2)});
}

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

CartesianAdaptiveComplianceController::~CartesianAdaptiveComplianceController() {
#ifdef LOGGING
    _logfile->close();
    _configfile->close();

    // log_vector_heading<6>(*_logfile, "K");

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
    _q.resize(Base::m_joint_state_pos_handles.size());
    _qd.resize(Base::m_joint_state_pos_handles.size());

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
    pinocchio::urdf::buildModelFromXML(
            get_node()->get_parameter("robot_description").as_string(), _pin_model
    );
    _pin_data = pinocchio::Data(_pin_model);

    _base_link_name = get_node()->get_parameter("robot_base_link").as_string();
    _compliance_link_name
            = get_node()->get_parameter("compliance_ref_link").as_string();
    _ee_link_name = get_node()->get_parameter("end_effector_link").as_string();

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
    _synchronisePinocchioModel();
    _updateStiffness();

    // --- Same control loop of the cartesian compliance controller
    for (int i = 0; i < Base::m_iterations; ++i) {
        auto           internal_period = rclcpp::Duration::from_seconds(0.02);
        ctrl::Vector6D error
                = CartesianAdaptiveComplianceController::computeComplianceError();
        Base::computeJointControlCmds(error, internal_period);
    }
    Base::writeJointControlCmds();

    return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianAdaptiveComplianceController::computeComplianceError() {
    ctrl::Vector6D net_force =
            // Spring force in base orientation
            Base::displayInBaseLink(m_stiffness, m_compliance_ref_link)
                    * MotionBase::computeMotionError()
            - Base::displayInBaseLink(_D, m_compliance_ref_link) * _ee_vel
            // Sensor and target force in base orientation
            + ForceBase::computeForceError();
    return net_force;
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
#ifdef LOGGING
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
#endif
}

void CartesianAdaptiveComplianceController::_synchronisePinocchioModel() {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(_pin_model.nq);
    for (std::size_t i = 0; i < _joint_state_vel_handles.size(); ++i) {
        _joint_velocities(i) = _joint_state_vel_handles[i].get().get_value();
        q(i)                 = Base::m_joint_state_pos_handles[i].get().get_value();
    }
    pinocchio::forwardKinematics(_pin_model, _pin_data, q, _joint_velocities);
    _q = q;
    _qd = _joint_velocities;
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

    const std::string base_link
            = get_node()->get_parameter("robot_base_link").as_string();
    const std::string ee_link
            = get_node()->get_parameter("end_effector_link").as_string();
    const std::string compliance_link
            = get_node()->get_parameter("compliance_ref_link").as_string();
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

    pinocchio::forwardKinematics(_pin_model, _pin_data, _q, _qd);
    pinocchio::updateFramePlacements(_pin_model, _pin_data);

    int frame_id = _pin_model.getFrameId("tool0");
    int max_id = _pin_model.frames.size();
    // pinocchio::SE3 frame = pinocchio::updateFramePlacement(_pin_model, _pin_data, frame_id);
    pinocchio::SE3 frame = _pin_data.oMf[frame_id];



    static int i = 0;
    if (i % 100000 == 0) {
        std::cout << "=================================\n";
        std::cout << "q: " << _q.transpose() << std::endl;
        std::cout << "qd: " << _qd.transpose() << std::endl;
        std::cout << "Frame id: " << frame_id << std::endl;
        std::cout << frame << std::endl;
        if (frame_id == max_id) {
            std::cout << "Frame id is max id" << std::endl;
            for (int j = 0; j < max_id; j++) {
                std::cout << "Frame " << j << ": " << _pin_model.frames[j].name << std::endl;
            }
        }
    }
    i++;

    /*
    const ctrl::Matrix6D X_tilde
            = x_tilde.asDiagonal();  // pos tracking error diag matrix
    const ctrl::Matrix6D Xd_tilde = xd_tilde.asDiagonal();

    const ctrl::Vector6D d = _D * xd_tilde;
    const ctrl::Vector6D f = d - _des_wrench;

    const ctrl::Vector6D A_bot_row = -Xd_tilde * x_tilde;

    const double T     = _tankEnergy();
    const double Tmin  = get_node()->get_parameter("tank.minimum_energy").as_double();
    const double sigma = (T <= 1.0) ? 1.0 : 0.0;
    // const double sigma = 0;
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
    _qp_A_lb.bottomRows<3>()    = Eigen::Vector3d({-1e9, -1e9, k3});
    _qp_A_ub.bottomRows<3>()    = Eigen::Vector3d({k1, k2, 1e9});
    // _qp_A_ub.bottomRows<2>() = 1e9 * Eigen::Vector2d::Ones();


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
        // return;

    } else {
        // Update the stiffness matrix
        _K = _qp_x_sol.asDiagonal();
        // _K = _Kmin.asDiagonal();
        _updateDamping();
        m_stiffness = _K;
    }

    // Integrate energy tank
    const ctrl::Matrix6D Kmin = _Kmin.asDiagonal();
    ctrl::Vector6D       w    = -(_K - Kmin) * x_tilde;
    if (T < Tmin) w = ctrl::Vector6D::Zero();
    const double dx_tank_1 = sigma / _x_tank * xd_tilde.transpose() * _D * xd_tilde;
    const double dx_tank_2 = -(1 / _x_tank) * w.transpose() * xd_tilde;
    const double dx_tank   = dx_tank_1 + dx_tank_2;
    _x_tank += dx_tank * _dt;

    // Log the data
#ifdef LOGGING
   //_logfile->print("{},", _t);
   // _logfile->print("{},", _x_tank);
   // _logfile->print("{},", dx_tank);
   // _logfile->print("{},", _tankEnergy());
   // _logfile->print("{},", sigma);
   // log_vector<6>(*_logfile, _K.diagonal());
   // log_vector<6>(*_logfile, x_tilde);
   // log_vector<6>(*_logfile, xd_tilde);
   // log_vector<3>(*_logfile, pos_curr);
   // log_vector<4>(*_logfile, quat_curr.coeffs());
   // log_vector<3>(*_logfile, pos_des);
   // log_vector<4>(*_logfile, quat_des.coeffs());
   // log_vector<6>(*_logfile, xd);
   // log_vector<6>(*_logfile, _des_vel);
   // log_vector<6>(*_logfile, _des_wrench);
   // _logfile->print("\n");
#endif

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
    */

    m_stiffness = ctrl::Matrix6D::Zero();
    m_stiffness = ctrl::Vector6D(100, 100, 100, 10, 10, 10).asDiagonal();

    _t += _dt;
}

void CartesianAdaptiveComplianceController::_updateDamping() {
    _D = 2.0 * _K.diagonal().cwiseSqrt().asDiagonal();
    // _D = 0.02 * _K.diagonal().cwiseSqrt().asDiagonal();
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
        cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController,
        controller_interface::ControllerInterface
)
