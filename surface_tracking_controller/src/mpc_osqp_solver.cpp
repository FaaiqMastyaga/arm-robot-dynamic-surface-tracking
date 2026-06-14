#include "surface_tracking_controller/mpc_osqp_solver.hpp"
#include <iostream>

namespace surface_tracking_controller {

MpcOsqpSolver::MpcOsqpSolver(int horizon, int state_dim, int input_dim)
    : N_(horizon), nx_(state_dim), nu_(input_dim), solver_initialized_(false) {}

void MpcOsqpSolver::updateHorizon(int new_horizon, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    if (N_ == new_horizon) return; // Ignore if it's the same

    N_ = new_horizon;
    
    // Rebuild Q_bar and R_bar to match the new size
    setWeights(Q, R); 

    // CRITICAL: Wipe both the OSQP C-workspace AND the C++ data wrapper caches
    solver_.clearSolver(); 
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    
    // Force the solve() loop to trigger the initSolver() branch on the next tick
    solver_initialized_ = false; 
}

void MpcOsqpSolver::setWeights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    // Construct Block Diagonal Q_bar and R_bar for the entire prediction horizon
    Q_bar_ = Eigen::MatrixXd::Zero(N_ * nx_, N_ * nx_);
    R_bar_ = Eigen::MatrixXd::Zero(N_ * nu_, N_ * nu_);

    for (int i = 0; i < N_; ++i) {
        Q_bar_.block(i * nx_, i * nx_, nx_, nx_) = Q;
        R_bar_.block(i * nu_, i * nu_, nu_, nu_) = R;
    }
}

void MpcOsqpSolver::computeDenseMatrices(const Eigen::MatrixXd& A_k,
                                         const Eigen::MatrixXd& B,
                                         Eigen::MatrixXd& F,
                                         Eigen::MatrixXd& G) {

    F = Eigen::MatrixXd::Zero(N_ * nx_, nx_);
    G = Eigen::MatrixXd::Zero(N_ * nx_, N_ * nu_);

    // Precompute powers of A to drop computation from O(N^3) down to O(N)
    std::vector<Eigen::MatrixXd> A_powers(N_ + 1);
    A_powers[0] = Eigen::MatrixXd::Identity(nx_, nx_);
    for (int i = 1; i <= N_; ++i) {
        A_powers[i] = A_powers[i - 1] * A_k;
    }

    for (int i = 0; i < N_; ++i) {
        // F matrix block: A^(i+1)
        F.block(i * nx_, 0, nx_, nx_) = A_powers[i + 1];

        // G matrix blocks
        for (int j = 0; j <= i; ++j) {
            // G block: A^(i-j) * B
            G.block(i * nx_, j * nu_, nx_, nu_) = A_powers[i - j] * B;
        }
    }   
}

bool MpcOsqpSolver::solve(const Eigen::MatrixXd& A_k, const Eigen::MatrixXd& B,
                          const Eigen::VectorXd& x_0, const Eigen::MatrixXd& x_ref,
                          const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max,
                          Eigen::VectorXd& u_optimal_out) {

    // Compute Dense Matrices F and G for the lifted system
    Eigen::MatrixXd F, G;
    computeDenseMatrices(A_k, B, F, G);

    // Flatten the reference matrix into a single column vector
    Eigen::VectorXd r_flat(N_ * nx_);
    for (int i = 0; i < N_; ++i) {
        r_flat.segment(i * nx_, nx_) = x_ref.col(i);
    }

    // 1. Compute OSQP Hesian (P) and Gradient (q) for the QP problem
    Eigen::MatrixXd H = 2.0 * (G.transpose() * Q_bar_ * G + R_bar_);
    H = 0.5 * (H + H.transpose());

    Eigen::VectorXd f = 2.0 * G.transpose() * Q_bar_ * (F * x_0 - r_flat);

    // 2. Compute constraints
    Eigen::SparseMatrix<double> A_cons(N_ * nu_, N_ * nu_);
    A_cons.setIdentity();

    Eigen::VectorXd lower_bound = u_min.replicate(N_, 1);
    Eigen::VectorXd upper_bound = u_max.replicate(N_, 1);

    // Convert Dense H to sparse format for OSQP
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    
    // Force Eigen to remove unused memory gaps before OSQP reads it
    H_sparse.makeCompressed();
    A_cons.makeCompressed();

    // 3. Initialize or Update OSQP Solver
    if (!solver_initialized_) {
        solver_.settings()->setVerbosity(false);
        solver_.settings()->setWarmStart(true);

        // --- ADAPTIVE SCALING AND RELAXATION ---
        solver_.settings()->setAdaptiveRho(true);
        solver_.settings()->setAdaptiveRhoInterval(25);
        solver_.settings()->setScaling(10); 
        solver_.settings()->setMaxIteration(4000);

        solver_.data()->setNumberOfVariables(N_ * nu_);
        solver_.data()->setNumberOfConstraints(N_ * nu_);

        if (!solver_.data()->setHessianMatrix(H_sparse) ||
            !solver_.data()->setGradient(f) ||
            !solver_.data()->setLinearConstraintsMatrix(A_cons) ||
            !solver_.data()->setLowerBound(lower_bound) ||
            !solver_.data()->setUpperBound(upper_bound)) {

            return false;
        }

        solver_.initSolver();
        solver_initialized_ = true;
    } else {
        // LTV-MPC changes H and f every step, so we must update them
        // Safely remove and reset the value
        solver_.clearSolver();

        solver_.data()->clearHessianMatrix();
        solver_.data()->clearLinearConstraintsMatrix();
        
        solver_.data()->setHessianMatrix(H_sparse);
        solver_.data()->setGradient(f);
        solver_.data()->setLinearConstraintsMatrix(A_cons);
        solver_.data()->setLowerBound(lower_bound);
        solver_.data()->setUpperBound(upper_bound);
        
        // Re-initialize the workspace with the new matrices
        solver_.initSolver();
    }

    // 4. Solve the QP problem
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "OSQP solver failed to find a solution!" << std::endl;
        return false;
    }

    Eigen::VectorXd QPSolution = solver_.getSolution();

    // We only care about the first control input in the optimal sequence
    u_optimal_out = QPSolution.head(nu_);
    return true;
}

} // namespace surface_tracking_controller