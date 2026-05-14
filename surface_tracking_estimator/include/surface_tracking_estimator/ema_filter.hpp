#ifndef SURFACE_TRACKING_ESTIMATOR_EMA_FILTER_HPP
#define SURFACE_TRACKING_ESTIMATOR_EMA_FILTER_HPP

#include "surface_tracking_estimator/velocity_filter_base.hpp"
#include <vector>

namespace surface_tracking_estimator {

class EMAFilter : public VelocityFilterBase 
{
public:
    explicit EMAFilter(double alpha = 0.2);
    std::vector<double> update(const std::vector<double>& raw_velocity, double dt) override;
    void reset() override;

private:
    double alpha_;
    bool is_initialized_;
    std::vector<double> prev_velocity_;
};

} // namespace surface_tracking_estimator

#endif // SURFACE_TRACKING_ESTIMATOR_EMA_FILTER_HPP