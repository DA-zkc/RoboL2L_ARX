#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <vector>

// A timestamped reference, not a controller. Coefficients come from a clamped
// cubic spline over the public trajectory samples. Both installed controllers
// evaluate this same curve in their existing realtime loop.
namespace station {
using Sample = std::array<double, 7>;
using Polynomial = std::array<Sample, 4>;
class SampledJointCurve {
 public:
  std::vector<Polynomial> coefficients;
  double dt, duration;
  SampledJointCurve(const std::vector<Polynomial>& values, double interval)
      : coefficients(values), dt(interval), duration(values.size()*interval) {
    if (values.empty() || !std::isfinite(dt) || dt <= 0 || !std::isfinite(duration))
      throw std::invalid_argument("invalid_sampled_trajectory");
    for (const auto& row : values) for (const auto& coefficient : row)
      for (double value : coefficient)
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite_trajectory_coefficient");
  }
  std::tuple<Sample, Sample, Sample> sample(double time) const {
    if (!std::isfinite(time)) throw std::invalid_argument("invalid_sample_time");
    const double t = std::max(0., std::min(duration, time));
    const size_t i = std::min(coefficients.size()-1, static_cast<size_t>(t/dt));
    const double local = t-i*dt;
    const auto& c = coefficients[i];
    Sample q{}, v{}, a{};
    for (size_t j=0; j<7; ++j) {
      q[j]=((c[0][j]*local+c[1][j])*local+c[2][j])*local+c[3][j];
      v[j]=(3*c[0][j]*local+2*c[1][j])*local+c[2][j];
      a[j]=6*c[0][j]*local+2*c[1][j];
    }
    return {q,v,a};
  }
};
}
