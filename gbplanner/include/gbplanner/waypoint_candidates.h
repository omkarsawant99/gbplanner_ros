#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace inspection_waypoints {
// Finite lattice search, nearest offsets first; the original pose is first.
inline std::vector<std::array<double, 3>> candidateOffsets(double radius, double step) {
  if (!std::isfinite(radius) || !std::isfinite(step) || radius < 0 || step <= 0 ||
      radius / step > 30)
    throw std::invalid_argument("Invalid inspection waypoint search radius/resolution");
  std::vector<std::array<double, 3>> offsets;
  const int n = static_cast<int>(std::floor(radius / step));
  for (int x = -n; x <= n; ++x)
    for (int y = -n; y <= n; ++y)
      for (int z = -n; z <= n; ++z) {
        if ((x*x + y*y + z*z)*step*step <= radius*radius + 1e-10)
          offsets.push_back({x*step, y*step, z*step});
      }
  auto squared = [](const std::array<double, 3>& p) { return p[0]*p[0]+p[1]*p[1]+p[2]*p[2]; };
  std::sort(offsets.begin(), offsets.end(), [&](const auto& a, const auto& b) {
    const double da = squared(a), db = squared(b);
    return da == db ? a < b : da < db;
  });
  return offsets;
}
}  // namespace inspection_waypoints
