#ifndef SURFACE_TRACKING_ESTIMATOR_KALMAN_FILTER_HPP
#define SURFACE_TRACKING_ESTIMATOR_KALMAN_FILTER_HPP

#include "surface_tracking_estimator/velocity_filter_base.hpp"
#include <Eigen/Dense>
#include <vector>

namespace surface_tracking_estimator {
    
class KalmanFilter : public VelocityFilterBase 
{
public:
    KalmanFilter(double q_multiplier, double r_multiplier);
    std::vector<double> update(const std::vector<double>& raw_velocity, double dt) override;
    void reset() override;

private:
    bool is_initialized_;

    // Dimensions
    int n_; // State dimension (12)
    int m_; // Measurement dimension (6)

    // Matrices
    Eigen::VectorXd x_; // State vector [12x1]
    Eigen::MatrixXd P_; // Estimate error covariance [12x12]
    Eigen::MatrixXd Q_; // Process noise covariance [12x12]
    Eigen::MatrixXd R_; // Measurement noise covariance [6x6]
    Eigen::MatrixXd A_; // State transition matrix [12x12]
    Eigen::MatrixXd H_; // Measurement matrix [6x12]
    Eigen::MatrixXd I_; // Identity matrix [12x12]  
};

} // namespace surface_tracking_estimator

#endif // SURFACE_TRACKING_ESTIMATOR_KALMAN_FILTER_HPP