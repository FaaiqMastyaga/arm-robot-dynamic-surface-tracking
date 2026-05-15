#include "surface_tracking_controller/pid_controller.hpp"

namespace surface_tracking_controller {

PidController::PidController(double kp, double ki, double kd, double max_integral)
    : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral)
{
    reset();
}

double PidController::update(double state, double setpoint, double dt)
{
    if (dt <= 0.0) return 0.0; // Safety againts division by zero

    // Calculate error
    double error = setpoint - state;

    // Proportional term
    double p_term = kp_ * error;

    // Integral term
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);
    double i_term = ki_ * integral_;

    // Derivative term
    double derivative = (error - previous_error_) / dt;
    double d_term = kd_ * derivative;

    // Save current error for next loop
    previous_error_ = error;

    // Output velocity command
    return p_term + i_term + d_term;
}

void PidController::reset()
{
    integral_ = 0.0;
    previous_error_ = 0.0;
}

} // namespace surface_tracking_controller