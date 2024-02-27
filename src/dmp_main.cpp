#include <cmath>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

#include "cartesian_adaptive_compliance_controller/dmp.hpp"

int main() {
    std::cout << "Creating DMP" << std::endl;
#if RECURSIVE_METHOD == 1
    dmp::PeriodicDmp dmp(55, 48.0, 0.999);
#elif RECURSIVE_METHOD == 2
    dmp::PeriodicDmp dmp(55, 48.0, 0.2);
#else
    dmp::PeriodicDmp dmp(55, 48.0, 0.9999);
#endif
    std::cout << "Training DMP" << std::endl;
    dmp.trainRecursive();
    std::cout << "DMP trained" << std::endl;

    std::ofstream wout("weights.csv");
    wout << dmp.getWeights()<< std::endl;
    wout.close();

    std::cout << "Simulating DMP" << std::endl;
    std::ofstream out("dmp.csv");
    Eigen::VectorXd sim = dmp.simulate();
    out << sim << std::endl;
    out.close();


    Eigen::VectorXd w_rec = dmp.getWeights();

    dmp.train();
    Eigen::VectorXd w_batch = dmp.getWeights();
    Eigen::VectorXd rel_error = (w_rec - w_batch).cwiseProduct(w_batch.cwiseInverse());

    // std::cout << "Recursive training:\n" << dmp._w << std::endl;
    // std::cout << "Batch training:\n" << dmp._w << std::endl;
    // std::cout << "Relative error:\n" << -rel_error * 100 << std::endl;
    std::cout << "Average absolute error (%):\n" << rel_error.cwiseAbs().mean() * 100 << std::endl;

    return 0;
}
