#ifndef SURFACE_TRACKING_CONTROLLER_SISO_CONTROLLER_BASE_HPP
#define SURFACE_TRACKING_CONTROLLER_SISO_CONTROLLER_BASE_HPP

namespace surface_tracking_controller {

class SISOControllerBase 
{
public:
    virtual ~SISOControllerBase() = default;

    // Standard update
    virtual double update(double state, double setpoint, double dt) = 0;

    // Feedforward update (default simply ignores the target_velocity and calls standard update)
    virtual double update_with_ff(double state, double setpoint, double target_velocity, double dt)
    {
        (void)target_velocity;
        return update(state, setpoint, dt);
    }

    virtual void reset() = 0;
};

} // namespace surface_tracking_controller

#endif // SURFACE_TRACKING_CONTROLLER_SISO_CONTROLLER_BASE_HPP