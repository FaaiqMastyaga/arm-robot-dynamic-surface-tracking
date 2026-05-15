#ifndef DYNAMIC_TRACKING_PID_CONTROLLER_HPP
#define DYNAMIC_TRACKING_PID_CONTROLLER_HPP

#include "surface_tracking_controller/siso_controller_base.hpp"
#include <algorithm>

namespace surface_tracking_controller {

class PidController : public SISOControllerBase
{
public:
    PidController(double kp, double ki, double kd, double max_integral = 1.0);
    double update(double state, double setpoint, double dt) override;
    void reset() override;

private:
    // Gains
    double kp_, ki_, kd_;
    
    // Limits
    double max_integral_;

    // Internal State
    double integral_, previous_error_;
};

} // namespace surface_tracking_controller

#endif // DYNAMIC_TRACKING_PID_CONTROLLER_HPP