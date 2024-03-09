#ifndef CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_HPP__
#define CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_HPP__


#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>

#include "cartesian_compliance_controller/cartesian_compliance_controller.h"
#include "cartesian_controller_base/Utility.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/framevel.hpp"
#include "qpOASES.hpp"
#include "qpOASES/QProblem.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace cartesian_adaptive_compliance_controller {
using QpMatrix = Eigen::
        Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using QpVector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;

class CartesianAdaptiveComplianceController
        : public cartesian_compliance_controller::CartesianComplianceController {
public:
    CartesianAdaptiveComplianceController() = default;

    // Note:
    // This way the controller won't work with the Foxy version of ROS2
    virtual LifecycleNodeInterface::CallbackReturn on_init() override;

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

    /**
     * @brief reads joint velocity state interfaces and updates the variable
     * _joint_velocities
     *
     */
    void _synchroniseJointVelocities();

    /**
     * @brief Initialise some variables of the controller reading from the parameter
     * server.
     *
     * For instance:
     *  - minimum stiffness
     *
     */
    void _initializeVariables();

    /**
     * @brief Initialize variables and solvers for the QP problem
     *
     */
    void _initializeQpProblem();

    /**
     * @brief fills the variable of the QP problem (based on the current state), solves
     * it and writes back the m_stiffness matrix (in the CartesianComplianceController
     * parent class)
     *
     */
    void _updateStiffness();

    /**
     * @brief updates the damping matrix according to the current stiffness
     *
     */
    void _updateDamping();

    /**
     * @brief Returns the end-effector frame velocity
     *
     */
    KDL::FrameVel _getEndEffectorFrameVel() const;


    //   ___  ____    ____            _     _
    //  / _ \|  _ \  |  _ \ _ __ ___ | |__ | | ___ _ __ ___
    // | | | | |_) | | |_) | '__/ _ \| '_ \| |/ _ \ '_ ` _ \
    // | |_| |  __/  |  __/| | | (_) | |_) | |  __/ | | | | |
    //  \__\_\_|     |_|   |_|  \___/|_.__/|_|\___|_| |_| |_|
    //
    /**
     * Note: qpOASES solves minimisation problem of the following form:
     *
     *     min  0.5 * x^T * H * x + x^T * g
     *    s.t.  lbA <= A * x <= ubA
     *          lb <= x <= ub
     *
     */
    qpOASES::QProblem _qp_prob;
    QpMatrix          _qp_H;
    QpVector          _qp_g;
    QpMatrix          _qp_A;
    QpVector          _qp_A_lb, _qp_A_ub;
    QpVector          _qp_x_lb, _qp_x_ub;
    QpVector          _qp_x_sol;

    ctrl::Matrix3D _Q;
    ctrl::Matrix3D _R;

    //  _____           _
    // |_   _|_ _ _ __ | | __
    //   | |/ _` | '_ \| |/ /
    //   | | (_| | | | |   <
    //   |_|\__,_|_| |_|_|\_\
    //
    double _x_tank;
    double _dt;

    ctrl::Matrix3D _D;            // Damping matrix
    ctrl::Matrix3D _K;            // Stiffness matrix
    ctrl::Vector3D _Kmin, _Kmax;  // min/max diag. elems
    ctrl::Vector3D _F_min, _F_max;
    std::unique_ptr<KDL::ChainFkSolverVel_recursive> _kin_solver;

    double inline _tankEnergy() const { return 0.5 * _x_tank * _x_tank; };

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr  _twist_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr _wrench_sub;
    ctrl::Vector3D                                                     _des_vel;
    ctrl::Vector3D                                                     _des_wrench;
};

}  // namespace cartesian_adaptive_compliance_controller

#endif  // CARTESIAN_ADAPTIVE_COMPLIVECTOR_HPP__
