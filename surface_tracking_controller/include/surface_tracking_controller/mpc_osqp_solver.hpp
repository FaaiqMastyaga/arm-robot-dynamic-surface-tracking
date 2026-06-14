#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

namespace surface_tracking_controller {

class MpcOsqpSolver {
public:
    MpcOsqpSolver(int horizon, int state_dim, int input_dim);
    ~MpcOsqpSolver() = default;

    void updateHorizon(int new_horizon, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    void setWeights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

    // Call this at each control step to solve for the optimal control input
    bool solve(const Eigen::MatrixXd& A_k,
               const Eigen::MatrixXd& B,
               const Eigen::VectorXd& x_0,
               const Eigen::MatrixXd& x_ref,
               const Eigen::VectorXd& u_min,
               const Eigen::VectorXd& u_max,
               Eigen::VectorXd& u_optimal_out);

private:
    int N_;  // Prediction horizon
    int nx_; // State dimension
    int nu_; // Input dimension

    Eigen::MatrixXd Q_bar_;
    Eigen::MatrixXd R_bar_;

    OsqpEigen::Solver solver_;
    bool solver_initialized_ = false;

    // Helper function for Dense Matrix Lifting
    void computeDenseMatrices(const Eigen::MatrixXd& A_k,
                              const Eigen::MatrixXd& B,
                              Eigen::MatrixXd& F,
                              Eigen::MatrixXd& G);
};

} // namespace surface_tracking_controller