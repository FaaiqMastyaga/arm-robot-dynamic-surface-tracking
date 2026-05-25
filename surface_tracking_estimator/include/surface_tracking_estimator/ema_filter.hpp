#ifndef SURFACE_TRACKING_ESTIMATOR_EMA_FILTER_HPP
#define SURFACE_TRACKING_ESTIMATOR_EMA_FILTER_HPP

#include <vector>

namespace surface_tracking_estimator {

class EMAFilter
{
public:
    explicit EMAFilter(double alpha = 0.2);
    void setAlpha(double alpha);
    std::vector<double> update(const std::vector<double>& raw_velocity, double dt);
    void reset();

private:
    double alpha_;
    bool is_initialized_;
    std::vector<double> prev_velocity_;
};

} // namespace surface_tracking_estimator

#endif // SURFACE_TRACKING_ESTIMATOR_EMA_FILTER_HPP