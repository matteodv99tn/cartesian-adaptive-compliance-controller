#ifndef CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_HPP__
#define CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_HPP__


#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <vector>

#include "cartesian_compliance_controller/cartesian_compliance_controller.h"
#include "cartesian_controller_base/Utility.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "qpOASES.hpp"
#include "qpOASES/QProblem.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace cartesian_adaptive_compliance_controller {

template <int ROWS = Eigen::Dynamic, int COLS = Eigen::Dynamic>
using QpMatrix = Eigen::Matrix<qpOASES::real_t, ROWS, COLS, Eigen::RowMajor>;

template <int SIZE = Eigen::Dynamic>
using QpVector = Eigen::Matrix<qpOASES::real_t, SIZE, 1>;

class CartesianAdaptiveComplianceController
        : public cartesian_compliance_controller::CartesianComplianceController {
public:
    // Note:
    // This way the controller won't work with the Foxy version of ROS2
    virtual LifecycleNodeInterface::CallbackReturn on_init() override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    void logParameters() const;

private:
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            _joint_state_vel_handles;

    /**
     * @brief reads joint velocity state interfaces and updates the variable
     * _joint_velocities
     *
     */
    void _synchronise_pinocchio_model();

    /**
     * @brief Initialise some variables of the controller reading from the parameter
     * server.
     *
     * For instance:
     *  - minimum stiffness
     *
     */
    void _initialize_variables();

    /**
     * @brief Initialize variables and solvers for the QP problem
     *
     */
    void _initialize_qp_problem();

    /**
     * @brief fills the variable of the QP problem (based on the current state), solves
     * it and writes back the m_stiffness matrix (in the CartesianComplianceController
     * parent class)
     *
     */
    bool _update_stiffness();

    /**
     * @brief updates the damping matrix according to the current stiffness
     *
     */
    void _update_damping();

    /**
     * @brief Computes the compliance error for the forward dynamic solver
     *
     */
    ctrl::Vector6D _compute_compliance_error();


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
    static constexpr int nv = 6;  // number of variables of the qp problem
    static constexpr int nc = 9;  // number of constraings of the qp problem

    qpOASES::QProblem _qp_prob;
    QpMatrix<nv, nv>  _qp_H;
    QpVector<nv>      _qp_g;
    QpMatrix<nc, nv>  _qp_A;
    QpVector<nc>      _qp_A_lb, _qp_A_ub;
    QpVector<nv>      _qp_x_lb, _qp_x_ub;
    QpVector<nv>      _qp_x_sol;


    ctrl::Matrix6D _Q;
    ctrl::Matrix6D _R;

    pinocchio::Model _pin_model;
    pinocchio::Data  _pin_data;
    Eigen::VectorXd  _q, _qd;
    int              _ee_link_id, _compliance_link_id, _base_link_id;

    std::string _base_link_name;
    std::string _ee_link_name;
    std::string _compliance_link_name;

    //  _____           _
    // |_   _|_ _ _ __ | | __
    //   | |/ _` | '_ \| |/ /
    //   | | (_| | | | |   <
    //   |_|\__,_|_| |_|_|\_\
    //
    double _x_tank;
    double _dt;
    double _t;

    ctrl::Matrix6D _D;            // Damping matrix
    ctrl::Matrix6D _K;            // Stiffness matrix
    ctrl::Vector6D _Kmin, _Kmax;  // min/max diag. elems
    ctrl::Vector6D _F_min, _F_max;

    double inline _tankEnergy() const { return 0.5 * _x_tank * _x_tank; };

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr  _twist_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr _wrench_sub;
    ctrl::Vector6D                                                     _des_vel;
    ctrl::Vector6D                                                     _des_wrench;
    std::string                                                        _des_vel_link;
    std::string                                                        _des_wrench_link;

    void _onDesiredTwistReceived(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void _onDesiredWrenchReceived(const geometry_msgs::msg::WrenchStamped::SharedPtr msg
    );

    ctrl::Vector6D _ee_vel;  // end-effector velocity

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _tank_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _stiffness_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _damping_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _xtilde_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _dxtilde_pub;
};

}  // namespace cartesian_adaptive_compliance_controller

#endif  // CARTESIAN_ADAPTIVE_COMPLIVECTOR_HPP__
