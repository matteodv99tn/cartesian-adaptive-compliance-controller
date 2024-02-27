#include "cartesian_adaptive_compliance_controller/dmp.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/src/Core/Matrix.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <thread>

#include "range/v3/all.hpp"

#define TIME_COLID       0
#define POSX_COLID       1
#define POSY_COLID       2
#define POSZ_COLID       3
#define QUATX_COLID      4
#define QUATY_COLID      5
#define QUATZ_COLID      6
#define QUATW_COLID      7
#define VELX_COLID       8
#define VELY_COLID       9
#define VELZ_COLID       10
#define OMEGAX_COLID     11
#define OMEGAY_COLID     12
#define OMEGAZ_COLID     13
#define ACCLINX_COLID    14
#define ACCLINY_COLID    15
#define ACCLINZ_COLID    16
#define ACCROTX_COLID    17
#define ACCROTY_COLID    18
#define ACCROTZ_COLID    19

#define NUM_COLS         20


using dmp::PeriodicDmp;
using dmp::RwMatrixXd;

RwMatrixXd dmp::loadTrainingTrajectory() {
    const std::string home      = std::getenv("HOME");
    const std::string file_path = home + "/dmps/data/end_effector_states.csv";

    std::ifstream in(file_path);
    std::string   content(
            (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()
    );

    std::vector<double> elems = content | ranges::views::transform([](char c) {
                                    return ((c == ',') || (c == '\n')) ? ' ' : c;
                                })
                                | ranges::views::split(' ')
                                | ranges::views::transform([](auto&& rng) -> double {
                                      return std::stod(rng | ranges::to<std::string>);
                                  })
                                | ranges::to<std::vector<double>>;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data(
            elems.size() / NUM_COLS, NUM_COLS
    );
    ranges::copy(elems, data.data());

    return data;
}

PeriodicDmp::PeriodicDmp(const int N, const double alpha, const double lambda) {
    _N      = N;
    _alpha  = alpha;
    _beta   = _alpha / 4.0;
    _lambda = lambda;
    _dt     = 0.002;
    _r      = 1.0;

    _train_data = loadTrainingTrajectory();
    _tau        = _train_data(_train_data.rows() - 1, TIME_COLID) / (2 * M_PI);
    // _Omega  = 1/_train_data(_train_data.rows() - 1, TIME_COLID);
    // _Omega = 1 / (_tau * _tau);
    _Omega = 1 / (_tau);

    // _w = Eigen::VectorXd::(_N);
    // _h = Eigen::VectorXd::Ones(_N);
    _h = Eigen::VectorXd::Ones(_N) * 1.5 * _N;
    _c = Eigen::VectorXd::LinSpaced(_N, 0, 2 * M_PI - 2 * M_PI / _N);
    // _g = _train_data.col(POSX_COLID).mean();
    _g = 0;
}

void PeriodicDmp::train() {
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(_train_data.rows(), _N);
    Eigen::VectorXd fd  = Eigen::VectorXd::Zero(_train_data.rows());

    for (int i : ranges::views::iota(0, _train_data.rows())) {
        const double          phi             = i * _dt / _tau;
        const Eigen::VectorXd basis_functions = evaluateBasisFunctions(phi);
        Phi.row(i) = basis_functions.transpose() / basis_functions.sum() * _r;
    }

    const Eigen::VectorXd y   = _train_data.col(POSX_COLID);
    const Eigen::VectorXd yd  = _train_data.col(VELX_COLID);
    const Eigen::VectorXd ydd = _train_data.col(ACCLINX_COLID);
    fd = ydd / (_Omega * _Omega) - _alpha * (_beta * (-y) - yd / _Omega);

    _fd_store = fd;
    _w        = Phi.colPivHouseholderQr().solve(fd);
}

#if RECURSIVE_METHOD == 1
void PeriodicDmp::trainRecursive() {
    Eigen::MatrixXd P    = Eigen::MatrixXd::Identity(_N, _N);
    Eigen::MatrixXd Pnew = Eigen::MatrixXd::Zero(_N, _N);
    _w                   = Eigen::VectorXd::Zero(_N);

    const Eigen::VectorXd y   = _train_data.col(POSX_COLID);
    const Eigen::VectorXd yd  = _train_data.col(VELX_COLID);
    const Eigen::VectorXd ydd = _train_data.col(ACCLINX_COLID);

    const int n_samples = _train_data.rows();
    for (int j : ranges::views::iota(0, n_samples)) {
        double          phi = j * 2 * M_PI / n_samples;
        Eigen::VectorXd psi = evaluateBasisFunctions(phi);
        Eigen::VectorXd Phi;
        if (psi.sum() < 1e-8) {
            Phi = Eigen::VectorXd::Zero(_N);
        } else {
            Phi = psi / psi.sum();
        }

        double fd = ydd(j) / (_Omega * _Omega)
                    - _alpha * (_beta * (-y(j)) - yd(j) / _Omega);

        const Eigen::MatrixXd num = P * Phi * Phi.transpose() * P;
        const double          den = _lambda + Phi.transpose() * P * Phi;
        Pnew                      = 1 / _lambda * (P - num / den);
        P                         = Pnew;

        _w += (fd - Phi.transpose() * _w) * P * Phi;
    }
}
#endif

#if RECURSIVE_METHOD == 2
void PeriodicDmp::trainRecursive() {
    Eigen::VectorXd P    = Eigen::VectorXd::Ones(_N);
    Eigen::VectorXd Pnew = Eigen::VectorXd::Zero(_N);
    _w                   = Eigen::VectorXd::Zero(_N);

    const Eigen::VectorXd y   = _train_data.col(POSX_COLID);
    const Eigen::VectorXd yd  = _train_data.col(VELX_COLID);
    const Eigen::VectorXd ydd = _train_data.col(ACCLINX_COLID);

    const int n_samples = _train_data.rows();

    for (int j : ranges::views::iota(0, n_samples)) {
        double          phi = j * 2 * M_PI / n_samples;
        Eigen::VectorXd psi = evaluateBasisFunctions(phi);
        Eigen::VectorXd Phi;
        if (psi.sum() < 1e-8) {
            Phi = Eigen::VectorXd::Zero(_N);
        } else {
            Phi = psi / psi.sum();
        }

        Eigen::VectorXd num = P.cwiseAbs2().cwiseProduct(Phi.cwiseAbs2());
        double          den = _lambda + Phi.transpose() * Phi.cwiseProduct(P);
        Pnew                = (P - num / den);


        double fd = ydd(j) / (_Omega * _Omega)
                    - _alpha * (_beta * (-y(j)) - yd(j) / _Omega);

        _w = _w + (fd - Phi.transpose() * _w) * P.cwiseProduct(Phi);
        P  = Pnew;
    }
}
#endif

#if RECURSIVE_METHOD == 3
void PeriodicDmp::trainRecursive() {
    Eigen::VectorXd P = Eigen::VectorXd::Ones(_N);
    _w                = Eigen::VectorXd::Zero(_N);

    const Eigen::VectorXd y   = _train_data.col(POSX_COLID);
    const Eigen::VectorXd yd  = _train_data.col(VELX_COLID);
    const Eigen::VectorXd ydd = _train_data.col(ACCLINX_COLID);

    double Pnew = 0;

    const int n_samples = _train_data.rows();

    for (int i : ranges::views::iota(0, n_samples)) {
        double phi = i * 2 * M_PI / n_samples;
        Eigen::VectorXd psi = evaluateBasisFunctions(phi);

        double fd = ydd(i) / (_Omega * _Omega)
                    - _alpha * (_beta * (-y(i)) - yd(i) / _Omega);

        for (int j : ranges::views::iota(0, _N)) {
            Pnew  = (P(j) - P(j) * P(j) / (P(j) + _lambda / psi(j))) / _lambda;
            _w(j) = _w(j) + psi(j) * Pnew * (fd - _w(j));
            P(j)  = Pnew;
        }
    }
}
#endif

Eigen::VectorXd PeriodicDmp::simulate() const {
    const int       nreps = 4;
    Eigen::VectorXd y     = Eigen::VectorXd::Zero(_train_data.rows() * nreps);
    Eigen::VectorXd z     = Eigen::VectorXd::Zero(_train_data.rows() * nreps);
    Eigen::VectorXd zd    = Eigen::VectorXd::Zero(_train_data.rows() * nreps);

    y(0)  = _train_data(0, POSX_COLID);
    z(0)  = _train_data(0, VELX_COLID);
    zd(0) = _train_data(0, ACCLINX_COLID);

    double phi = 0;

    for (int i : ranges::views::iota(0, _train_data.rows() * nreps - 1)) {
        const Eigen::VectorXd basis_functions = evaluateBasisFunctions(phi);
        const double          den             = basis_functions.sum();
        const double          f = (basis_functions.transpose() * _w)(0, 0) / den;

        const double dz_dt = _Omega * (_alpha * (_beta * (-y(i)) - z(i) / _Omega) + f);

        z(i + 1) = z(i) + dz_dt * _dt;
        y(i + 1) = y(i) + z(i) * _Omega * _dt;
        phi += _dt * _Omega;
    }

    return y;
}

Eigen::VectorXd PeriodicDmp::evaluateBasisFunctions(const double phi) const {
    Eigen::VectorXd res = Eigen::VectorXd::Zero(_N);
    for (int i : ranges::views::iota(0, _N)) {
        double cos_val = std::cos(phi - _c(i));
        res(i)         = std::exp(_h(i) * (cos_val - 1));
    }
    return res;
}

//   ____      _   _
//  / ___| ___| |_| |_ ___ _ __ ___
// | |  _ / _ \ __| __/ _ \ '__/ __|
// | |_| |  __/ |_| ||  __/ |  \__ \
//  \____|\___|\__|\__\___|_|  |___/
//


Eigen::Vector3d PeriodicDmp::getPosition(const int i) const {
    return _train_data.block(i, POSX_COLID, 1, 3).transpose();
}

Eigen::Vector4d PeriodicDmp::getOrientation(const int i) const {
    return _train_data.block(i, QUATX_COLID, 1, 4).transpose();
}

Eigen::Vector3d PeriodicDmp::getVelocity(const int i) const {
    return _train_data.block(i, VELX_COLID, 1, 3).transpose();
}

Eigen::Vector3d PeriodicDmp::getAngularVelocity(const int i) const {
    return _train_data.block(i, OMEGAX_COLID, 1, 3).transpose();
}

Eigen::Vector3d PeriodicDmp::getAcceleration(const int i) const {
    return _train_data.block(i, ACCLINX_COLID, 1, 3).transpose();
}

Eigen::Vector3d PeriodicDmp::getAngularAcceleration(const int i) const {
    return _train_data.block(i, ACCROTX_COLID, 1, 3).transpose();
}
