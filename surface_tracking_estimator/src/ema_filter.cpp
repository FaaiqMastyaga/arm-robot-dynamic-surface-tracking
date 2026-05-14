#include "surface_tracking_estimator/ema_filter.hpp"
#include <stdexcept>

namespace surface_tracking_estimator {

EMAFilter::EMAFilter(double alpha)
    : alpha_(alpha), is_initialized_(false)
{
    prev_velocity_.assign(6, 0.0);
}

std::vector<double> EMAFilter::update(const std::vector<double>& raw_velocity, double dt)
{
    if (raw_velocity.size() != 6) {
        throw std::invalid_argument("Raw velocity vector must have exactly 6 elements [x, y, z, roll, pitch, yaw]");
    }

    std::vector<double> current_velocity(6, 0.0);

    if (!is_initialized_) {
        prev_velocity_ = raw_velocity;
        is_initialized_ = true;
        return raw_velocity;
    }

    // Apply Exponential Moving Average filter
    for (size_t i = 0; i < 6; ++i) {
        current_velocity[i] = (alpha_ * raw_velocity[i]) + ((1.0 - alpha_) * prev_velocity_[i]);
    }

    // Store for next iteration
    prev_velocity_ = current_velocity;

    return current_velocity;
}

void EMAFilter::reset()
{
    is_initialized_ = false;
    prev_velocity_.assign(6, 0.0);

}

} // namespace surface_tracking_estimator