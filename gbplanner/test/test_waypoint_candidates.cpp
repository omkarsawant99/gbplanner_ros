#include "gbplanner/waypoint_candidates.h"
#include <iostream>

int main() {
  auto require = [](bool ok) { if (!ok) throw std::runtime_error("candidate search regression"); };
  auto offsets = inspection_waypoints::candidateOffsets(1.0, 0.1);
  require(offsets.front() == std::array<double, 3>{0, 0, 0});
  double previous = -1;
  for (const auto& p : offsets) {
    double d = p[0]*p[0]+p[1]*p[1]+p[2]*p[2];
    require(d >= previous && d <= 1.0 + 1e-10);
    previous = d;
  }
  // A target at x=4.13 overlaps a wall at 4.22. Require a 0.3 m
  // half-box clearance and retain the side before the wall.
  bool found = false;
  for (const auto& p : offsets) {
    if (4.13 + p[0] + 0.3 < 4.22) {
      require(std::abs(p[0] + 0.3) < 1e-9 && p[1] == 0 && p[2] == 0);
      found = true;
      break;
    }
  }
  require(found);
  require(inspection_waypoints::candidateOffsets(0, 0.1).size() == 1);
  for (const auto& config : {std::array<double,2>{1,0}, {-1,0.1}, {1,0.001}}) {
    bool rejected = false;
    try { inspection_waypoints::candidateOffsets(config[0],config[1]); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
  }
  std::cout << "Waypoint candidate search tests passed\n";
}
