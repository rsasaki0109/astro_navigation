#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

namespace astro::navigation {

struct GridCell {
  int x{0};
  int y{0};

  [[nodiscard]] bool operator==(const GridCell& other) const = default;
};

struct HazardCostMap {
  int width{0};
  int height{0};
  double resolution_m{1.0};
  Eigen::Vector2d origin_xy_m{Eigen::Vector2d::Zero()};
  std::vector<double> costs;

  [[nodiscard]] bool valid() const;
  [[nodiscard]] bool inBounds(GridCell cell) const;
  [[nodiscard]] std::size_t index(GridCell cell) const;
  [[nodiscard]] double costAt(GridCell cell) const;
  [[nodiscard]] bool traversable(GridCell cell, double blocked_cost) const;
  [[nodiscard]] GridCell worldToCell(const Eigen::Vector2d& xy_m) const;
  [[nodiscard]] Eigen::Vector2d cellCenterToWorld(GridCell cell) const;
  [[nodiscard]] GridCell nearestTraversable(GridCell cell, double blocked_cost,
                                            int radius_cells) const;
};

struct HazardPlannerOptions {
  double blocked_cost{1.0e9};
  double heuristic_weight{1.25};
  int snap_radius_cells{8};
  bool allow_diagonal{true};
};

struct HazardRouteMetrics {
  double route_length_m{0.0};
  double straight_line_length_m{0.0};
  double detour_ratio{1.0};
  double mean_cost{0.0};
  double max_cost{0.0};
  double min_clearance_cells{0.0};
  double min_clearance_m{0.0};
};

struct HazardRoute {
  std::vector<GridCell> cells;
  std::vector<Eigen::Vector2d> waypoints_xy_m;
  double total_cost{0.0};
  HazardRouteMetrics metrics;
  std::string message;

  [[nodiscard]] bool success() const { return !cells.empty(); }
};

[[nodiscard]] HazardRoute planHazardAwareRoute(const HazardCostMap& map, GridCell start,
                                               GridCell goal,
                                               const HazardPlannerOptions& options = {});

[[nodiscard]] HazardRoute planHazardAwareRouteMeters(const HazardCostMap& map,
                                                     const Eigen::Vector2d& start_xy_m,
                                                     const Eigen::Vector2d& goal_xy_m,
                                                     const HazardPlannerOptions& options = {});

[[nodiscard]] HazardRouteMetrics computeHazardRouteMetrics(const HazardCostMap& map,
                                                           const std::vector<GridCell>& route,
                                                           double blocked_cost);

[[nodiscard]] double clearanceToNearestBlockedCell(const HazardCostMap& map, GridCell cell,
                                                   double blocked_cost);

[[nodiscard]] std::vector<GridCell> resampleRoute(const std::vector<GridCell>& route,
                                                  std::size_t target_count);

}  // namespace astro::navigation
