#ifndef CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_DMP_HPP__
#define CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_DMP_HPP__

#include <Eigen/Dense>
#include <vector>

#define RECURSIVE_METHOD 3
// 1 -> full matrix form
// 2 -> element-wise form
// 3 -> element-wise form with P as a vector

namespace dmp {
using RwMatrixXd
        = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

RwMatrixXd loadTrainingTrajectory();

class PeriodicDmp {
public:
    /**
     * @brief Constructor
     *
     * @param[in] N Numbers of basis functions for each DMP
     */
    PeriodicDmp(const int N = 25, const double alpha = 48.0, const double lambda = 0.1);

    void train();
    void trainRecursive();
    Eigen::VectorXd simulate() const;

    Eigen::VectorXd getWeights() const { return _w; }
private:
// public:

    int _N;
    double _dt;
    double _tau;
    double _alpha;
    double _beta;
    double _lambda;
    double _Omega;
    double _g;
    double _r;

    Eigen::VectorXd _w;
    Eigen::VectorXd _h;
    Eigen::VectorXd _c;
    Eigen::VectorXd _fd_store;

    RwMatrixXd _train_data;

    Eigen::VectorXd evaluateBasisFunctions(const double phi) const;

    Eigen::Vector3d getPosition(const int i) const;
    Eigen::Vector4d getOrientation(const int i) const;
    Eigen::Vector3d getVelocity(const int i) const;
    Eigen::Vector3d getAngularVelocity(const int i) const;
    Eigen::Vector3d getAcceleration(const int i) const;
    Eigen::Vector3d getAngularAcceleration(const int i) const;

};
}  // namespace dmp

#endif  // CARTESIAN_ADAPTIVE_COMPLIANCE_CONTROLLER_DMP_HPP__
