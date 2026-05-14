#ifndef SURFACE_TRACKING_ESTIMATOR_VELOCITY_FILTER_BASE_HPP
#define SURFACE_TRACKING_ESTIMATOR_VELOCITY_FILTER_BASE_HPP

#include <vector>

namespace surface_tracking_estimator {

class VelocityFilterBase 
{
public:
    virtual ~VelocityFilterBase() = default;

    /**
     * @brief Updates the filter with a new pose and returns the estimated velocities.
     * 
     * @param raw_velocity A vector containing [x, y, z, roll, pitch, yaw]
     * @param dt The time step since the last update
     * @return std::vector<double> The estimated velocities [vx, vy, vz, wx, wy, wz]
     */
    virtual std::vector<double> update(const std::vector<double>& raw_velocity, double dt) = 0;

    // Resets the internal state
    virtual void reset() = 0;
};

} // namespace surface_tracking_estimator

#endif // SURFACE_TRACKING_ESTIMATOR_VELOCITY_FILTER_BASE_HPP