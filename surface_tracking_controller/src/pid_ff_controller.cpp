#include "surface_tracking_controller/pid_ff_controller.hpp"

namespace surface_tracking_controller {

PidFeedforwardController::PidFeedforwardController(double kp, double ki, double kd, double k_ff, double max_integral)
    : kp_(kp), ki_(ki), kd_(kd), k_ff_(k_ff), max_integral_(max_integral)
{
    reset();
}

double PidFeedforwardController::update_with_ff(double state, double setpoint, double target_velocity, double dt)
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

    // Feedforward term
    double ff_term = k_ff_ * target_velocity;

    // Save current error for next loop
    previous_error_ = error;

    // Output velocity command
    return p_term + i_term + d_term + ff_term;
}

double PidFeedforwardController::update(double state, double setpoint, double dt)
{
    return update_with_ff(state, setpoint, 0.0, dt);
}

void PidFeedforwardController::reset()
{
    integral_ = 0.0;
    previous_error_ = 0.0;
}

} // namespace surface_tracking_controller