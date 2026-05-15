#include "surface_tracking_estimator/kalman_filter.hpp"
#include <stdexcept>

namespace surface_tracking_estimator {

KalmanFilter::KalmanFilter(double q_multiplier, double r_multiplier)
    : is_initialized_(false), n_(12), m_(6)
{
    // Initialize state vector and matrices
    x_ = Eigen::VectorXd::Zero(n_);
    P_ = Eigen::MatrixXd::Identity(n_, n_); // Large initial uncertainty

    // Process Noise (Q): How much we trust our model
    Q_ = Eigen::MatrixXd::Identity(n_, n_) * q_multiplier; // Process noise covariance

    // Measurement Noise (R): How noisy our measurements are
    R_ = Eigen::MatrixXd::Identity(m_, m_) * r_multiplier; // Measurement noise covariance

    // State transition matrix (assuming constant velocity model)
    A_ = Eigen::MatrixXd::Identity(n_, n_);

    // Measurement matrix (we measure position only)
    H_ = Eigen::MatrixXd::Zero(m_, n_);
    for (int i = 0; i < m_; ++i) {
        H_(i, i) = 1.0;
    }

    I_ = Eigen::MatrixXd::Identity(n_, n_);
}

std::vector<double> KalmanFilter::update(const std::vector<double>& raw_velocity, double dt)
{
    if (raw_velocity.size() != static_cast<size_t>(m_)) {
        throw std::invalid_argument("Raw velocity vector must have exactly 6 elements [x, y, z, roll, pitch, yaw]");
    }

    Eigen::VectorXd z = Eigen::VectorXd::Map(raw_velocity.data(), m_);

    if (!is_initialized_) {
        // Initialize state with the first measurement, zero acceleration
        x_.head(m_) = z; // Set initial position from measurement
        x_.tail(m_).setZero(); // Initial velocity is zero

        is_initialized_ = true;
        return raw_velocity;
    }

    // --- PREDICTION STEP ---
    // Update state transition matrix A wwith current dt
    for (int i = 0; i < m_; ++i) {
        A_(i, i + m_) = dt; // velocity = velocity + acceleration * dt, but we assume constant velocity (acceleration = 0)
    }

    // Predict state: x_k|k-1 = A * x_k-1|k-1
    x_ = A_ * x_; // State prediction

    // Predict covariance: P_k|k-1 = A * P_k-1|k-1 * A^T + Q
    P_ = A_ * P_ * A_.transpose() + Q_;

    // --- UPDATE STEP ---
    // Innovation (Residual): y = z - H * x_k|k-1
    Eigen::VectorXd y = z - (H_ * x_);

    // Innovation covariance: S = H * P_k|k-1 * H^T + R
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;

    // Kalman Gain: K = P_k|k-1 * H^T * S^-1
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // Update state: x_k|k = x_k|k-1 + K * y 
    x_ = x_ + K * y;

    // Update covariance: P_k|k = (I - K * H) * P_k|k-1
    P_ = (I_ - K * H_) * P_;

    // Extract filtered velocity from state vector
    std::vector<double> filtered_velocity(m_);
    Eigen::VectorXd::Map(&filtered_velocity[0], m_) = x_.head(m_);

    return filtered_velocity;
}

void KalmanFilter::reset()
{
    is_initialized_ = false;
    x_.setZero();
    P_.setIdentity();
}

} // namespace surface_tracking_estimator