#ifndef SURFACE_TRACKING_CONTROLLER_MIMO_CONTROLLER_BASE_HPP
#define SURFACE_TRACKING_CONTROLLER_MIMO_CONTROLLER_BASE_HPP

#include <vector>

namespace surface_tracking_controller {

class MIMOControllerBase 
{
public:
    virtual ~MIMOControllerBase() = default;

    /**
     * @brief Computes control effort for the entire system simultaneously.
     * @param current_state [x, y, z, roll, pitch, yaw]
     * @param target_state [x, y, z, roll, pitch, yaw]
     * @param target_velocity (Feedforward from Kalman) [vx, vy, vz, wx, wy, wz]
     * @param dt Time step
     * @return Control commands for all 6 DOF
     */
    virtual std::vector<double> update(
        const std::vector<double>& current_state,
        const std::vector<double>& target_state,
        const std::vector<double>& target_velocity,
        double dt) = 0;

    virtual void reset() = 0;
};

} // namespace surface_tracking_controller

#endif // SURFACE_TRACKING_CONTROLLER_MIMO_CONTROLLER_BASE_HPP