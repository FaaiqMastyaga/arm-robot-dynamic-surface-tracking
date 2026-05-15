#ifndef DYNAMIC_TRACKING_PID_FEEDFORWARD_CONTROLLER_HPP
#define DYNAMIC_TRACKING_PID_FEEDFORWARD_CONTROLLER_HPP

#include "surface_tracking_controller/siso_controller_base.hpp"
#include <algorithm>

namespace surface_tracking_controller {

class PidFeedforwardController : public SISOControllerBase
{
public:
    PidFeedforwardController(double kp, double ki, double kd, double k_ff, double max_integral = 1.0);
    double update_with_ff(double state, double setpoint, double target_velocity, double dt) override;
    double update(double state, double setpoint, double dt) override;
    void reset() override;

private:
    // Gains
    double kp_, ki_, kd_, k_ff_;
    
    // Limits
    double max_integral_;

    // Internal State
    double integral_, previous_error_;
};

} // namespace surface_tracking_controller

#endif // DYNAMIC_TRACKING_PID_FEEDFORWARD_CONTROLLER_HPP