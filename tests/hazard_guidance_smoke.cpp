#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "astro_navigation/navigation/hazard_guidance.hpp"

namespace {

int fail(const std::string& message) {
  std::cerr << "hazard_guidance_smoke: " << message << '\n';
  return 1;
}

bool containsCell(const std::vector<astro::navigation::GridCell>& cells,
                  const astro::navigation::GridCell target) {
  for (const auto cell : cells) {
    if (cell == target) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main() {
  astro::navigation::HazardCostMap map;
  map.width = 12;
  map.height = 8;
  map.resolution_m = 2.0;
  map.origin_xy_m = Eigen::Vector2d(100.0, 200.0);
  map.costs.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height), 1.0);

  // A high-cost wall with one traversable gap. The planner should route through
  // the gap instead of crossing the blocked cells.
  for (int y = 0; y < map.height; ++y) {
    if (y == 4) {
      continue;
    }
    map.costs[map.index({5, y})] = 1.0e9;
  }
  map.costs[map.index({5, 4})] = 1.2;

  astro::navigation::HazardPlannerOptions options;
  options.blocked_cost = 1000.0;
  options.snap_radius_cells = 2;

  const auto route = astro::navigation::planHazardAwareRoute(map, {1, 3}, {10, 3}, options);
  if (!route.success()) {
    return fail("route planner failed on a map with a clear gap");
  }
  if (route.cells.front() != astro::navigation::GridCell{1, 3}) {
    return fail("route should keep the requested traversable start cell");
  }
  if (route.cells.back() != astro::navigation::GridCell{10, 3}) {
    return fail("route should keep the requested traversable goal cell");
  }
  if (!containsCell(route.cells, {5, 4})) {
    return fail("route should pass through the only safe wall gap");
  }
  for (const auto cell : route.cells) {
    if (!map.traversable(cell, options.blocked_cost)) {
      return fail("route includes a blocked hazard cell");
    }
  }
  if (route.waypoints_xy_m.size() != route.cells.size()) {
    return fail("route waypoints should mirror the grid-cell route");
  }
  if (!std::isfinite(route.total_cost) || route.total_cost <= 0.0) {
    return fail("route should report a finite positive total cost");
  }
  if (route.metrics.route_length_m <= route.metrics.straight_line_length_m) {
    return fail("hazard route should be longer than the straight-line distance around the wall");
  }
  if (route.metrics.detour_ratio <= 1.0) {
    return fail("hazard route should report a detour ratio greater than one");
  }
  if (route.metrics.mean_cost <= 1.0 || route.metrics.max_cost < 1.2) {
    return fail("hazard route cost metrics should include the higher-cost wall gap");
  }
  if (!std::isfinite(route.metrics.min_clearance_cells) ||
      std::abs(route.metrics.min_clearance_cells - 1.0) > 1.0e-12 ||
      std::abs(route.metrics.min_clearance_m - 2.0) > 1.0e-12) {
    return fail("hazard route should report one-cell minimum clearance from blocked hazards");
  }

  const Eigen::Vector2d start_xy = map.cellCenterToWorld({1, 3});
  const Eigen::Vector2d goal_xy = map.cellCenterToWorld({10, 3});
  const auto meter_route =
      astro::navigation::planHazardAwareRouteMeters(map, start_xy, goal_xy, options);
  if (!meter_route.success() || meter_route.cells != route.cells) {
    return fail("meter-coordinate route should match the grid-coordinate route");
  }

  const auto snapped_route = astro::navigation::planHazardAwareRoute(map, {5, 0}, {10, 3}, options);
  if (!snapped_route.success()) {
    return fail("planner should snap a blocked start cell to a nearby traversable cell");
  }
  if (!map.traversable(snapped_route.cells.front(), options.blocked_cost)) {
    return fail("snapped route should start on a traversable cell");
  }

  const auto sampled = astro::navigation::resampleRoute(route.cells, 5U);
  if (sampled.size() != 5U) {
    return fail("resampled route should have requested size");
  }
  if (sampled.front() != route.cells.front() || sampled.back() != route.cells.back()) {
    return fail("resampled route should preserve endpoints");
  }

  return 0;
}
