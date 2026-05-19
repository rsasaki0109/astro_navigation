#include "astro_navigation/navigation/hazard_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace astro::navigation {
namespace {

struct QueueItem {
  double priority{0.0};
  std::size_t sequence{0};
  GridCell cell;
};

struct QueueItemGreater {
  [[nodiscard]] bool operator()(const QueueItem& lhs, const QueueItem& rhs) const {
    if (lhs.priority == rhs.priority) {
      return lhs.sequence > rhs.sequence;
    }
    return lhs.priority > rhs.priority;
  }
};

struct NeighborStep {
  int dx{0};
  int dy{0};
  double distance{1.0};
};

[[nodiscard]] double euclideanCells(const GridCell a, const GridCell b) {
  const auto dx = static_cast<double>(a.x - b.x);
  const auto dy = static_cast<double>(a.y - b.y);
  return std::hypot(dx, dy);
}

[[nodiscard]] std::vector<NeighborStep> neighborSteps(const bool allow_diagonal) {
  std::vector<NeighborStep> steps{{-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0}};
  if (allow_diagonal) {
    const double diagonal = std::sqrt(2.0);
    steps.push_back({-1, -1, diagonal});
    steps.push_back({1, -1, diagonal});
    steps.push_back({-1, 1, diagonal});
    steps.push_back({1, 1, diagonal});
  }
  return steps;
}

[[nodiscard]] std::vector<Eigen::Vector2d> cellsToWaypoints(const HazardCostMap& map,
                                                            const std::vector<GridCell>& cells) {
  std::vector<Eigen::Vector2d> waypoints;
  waypoints.reserve(cells.size());
  for (const auto cell : cells) {
    waypoints.push_back(map.cellCenterToWorld(cell));
  }
  return waypoints;
}

[[nodiscard]] bool blockedCell(const HazardCostMap& map,
                               const GridCell cell,
                               const double blocked_cost) {
  if (!map.inBounds(cell)) {
    return false;
  }
  const double cost = map.costAt(cell);
  return !std::isfinite(cost) || cost >= blocked_cost;
}

}  // namespace

bool HazardCostMap::valid() const {
  if (width <= 0 || height <= 0 || resolution_m <= 0.0) {
    return false;
  }
  const auto expected_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  return costs.size() == expected_size;
}

bool HazardCostMap::inBounds(const GridCell cell) const {
  return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height;
}

std::size_t HazardCostMap::index(const GridCell cell) const {
  if (!inBounds(cell)) {
    throw std::out_of_range("hazard grid cell is out of bounds");
  }
  return static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(width) +
         static_cast<std::size_t>(cell.x);
}

double HazardCostMap::costAt(const GridCell cell) const {
  return costs.at(index(cell));
}

bool HazardCostMap::traversable(const GridCell cell, const double blocked_cost) const {
  if (!inBounds(cell)) {
    return false;
  }
  const double cost = costAt(cell);
  return std::isfinite(cost) && cost < blocked_cost;
}

GridCell HazardCostMap::worldToCell(const Eigen::Vector2d& xy_m) const {
  if (resolution_m <= 0.0) {
    throw std::invalid_argument("hazard cost map resolution must be positive");
  }
  const Eigen::Vector2d local = (xy_m - origin_xy_m) / resolution_m;
  return {
      static_cast<int>(std::floor(local.x())),
      static_cast<int>(std::floor(local.y())),
  };
}

Eigen::Vector2d HazardCostMap::cellCenterToWorld(const GridCell cell) const {
  if (!inBounds(cell)) {
    throw std::out_of_range("hazard grid cell is out of bounds");
  }
  return origin_xy_m +
         resolution_m * Eigen::Vector2d(static_cast<double>(cell.x) + 0.5,
                                        static_cast<double>(cell.y) + 0.5);
}

GridCell HazardCostMap::nearestTraversable(const GridCell cell,
                                           const double blocked_cost,
                                           const int radius_cells) const {
  if (!valid()) {
    throw std::invalid_argument("hazard cost map dimensions do not match cost storage");
  }
  const int radius = std::max(0, radius_cells);
  GridCell best = cell;
  double best_score = std::numeric_limits<double>::infinity();

  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const GridCell candidate{std::clamp(cell.x + dx, 0, width - 1),
                               std::clamp(cell.y + dy, 0, height - 1)};
      if (!traversable(candidate, blocked_cost)) {
        continue;
      }
      const double score = costAt(candidate) + 0.01 * std::hypot(static_cast<double>(dx),
                                                                 static_cast<double>(dy));
      if (score < best_score) {
        best = candidate;
        best_score = score;
      }
    }
  }

  if (!std::isfinite(best_score)) {
    throw std::runtime_error("no traversable hazard grid cell found near requested point");
  }
  return best;
}

HazardRoute planHazardAwareRoute(const HazardCostMap& map,
                                 GridCell start,
                                 GridCell goal,
                                 const HazardPlannerOptions& options) {
  if (!map.valid()) {
    throw std::invalid_argument("hazard cost map dimensions do not match cost storage");
  }
  if (options.blocked_cost <= 0.0 || !std::isfinite(options.blocked_cost)) {
    throw std::invalid_argument("blocked cost threshold must be finite and positive");
  }
  if (options.heuristic_weight < 0.0 || !std::isfinite(options.heuristic_weight)) {
    throw std::invalid_argument("heuristic weight must be finite and non-negative");
  }

  start = map.nearestTraversable(start, options.blocked_cost, options.snap_radius_cells);
  goal = map.nearestTraversable(goal, options.blocked_cost, options.snap_radius_cells);

  const auto steps = neighborSteps(options.allow_diagonal);
  const auto cell_count = static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  std::vector<double> best_cost(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemGreater> frontier;

  const auto start_index = map.index(start);
  const auto goal_index = map.index(goal);
  best_cost[start_index] = 0.0;
  frontier.push({0.0, 0U, start});

  std::size_t sequence = 0U;
  bool reached_goal = false;
  while (!frontier.empty()) {
    const QueueItem current = frontier.top();
    frontier.pop();
    const auto current_index = map.index(current.cell);
    if (current.priority >
        best_cost[current_index] + options.heuristic_weight * euclideanCells(current.cell, goal) *
                                       map.resolution_m) {
      continue;
    }
    if (current.cell == goal) {
      reached_goal = true;
      break;
    }

    for (const auto step : steps) {
      const GridCell next{current.cell.x + step.dx, current.cell.y + step.dy};
      if (!map.traversable(next, options.blocked_cost)) {
        continue;
      }
      const auto next_index = map.index(next);
      const double transition_cost =
          step.distance * map.resolution_m * 0.5 * (map.costAt(current.cell) + map.costAt(next));
      const double candidate_cost = best_cost[current_index] + transition_cost;
      if (candidate_cost >= best_cost[next_index]) {
        continue;
      }
      best_cost[next_index] = candidate_cost;
      parent[next_index] = static_cast<int>(current_index);
      ++sequence;
      const double priority =
          candidate_cost + options.heuristic_weight * euclideanCells(next, goal) * map.resolution_m;
      frontier.push({priority, sequence, next});
    }
  }

  if (!reached_goal) {
    HazardRoute failed_route;
    failed_route.total_cost = std::numeric_limits<double>::infinity();
    failed_route.message = "no route found";
    return failed_route;
  }

  std::vector<GridCell> cells;
  auto current_index = static_cast<int>(goal_index);
  while (current_index >= 0) {
    const int x = current_index % map.width;
    const int y = current_index / map.width;
    cells.push_back({x, y});
    current_index = parent[static_cast<std::size_t>(current_index)];
  }
  std::reverse(cells.begin(), cells.end());

  HazardRoute route;
  route.cells = std::move(cells);
  route.waypoints_xy_m = cellsToWaypoints(map, route.cells);
  route.total_cost = best_cost[goal_index];
  route.metrics = computeHazardRouteMetrics(map, route.cells, options.blocked_cost);
  route.message = "route planned";
  return route;
}

HazardRoute planHazardAwareRouteMeters(const HazardCostMap& map,
                                       const Eigen::Vector2d& start_xy_m,
                                       const Eigen::Vector2d& goal_xy_m,
                                       const HazardPlannerOptions& options) {
  return planHazardAwareRoute(map, map.worldToCell(start_xy_m), map.worldToCell(goal_xy_m), options);
}

double clearanceToNearestBlockedCell(const HazardCostMap& map,
                                     const GridCell cell,
                                     const double blocked_cost) {
  if (!map.valid()) {
    throw std::invalid_argument("hazard cost map dimensions do not match cost storage");
  }
  if (!map.inBounds(cell)) {
    throw std::out_of_range("hazard grid cell is out of bounds");
  }

  double best = std::numeric_limits<double>::infinity();
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const GridCell candidate{x, y};
      if (!blockedCell(map, candidate, blocked_cost)) {
        continue;
      }
      best = std::min(best, euclideanCells(cell, candidate));
    }
  }
  return best;
}

HazardRouteMetrics computeHazardRouteMetrics(const HazardCostMap& map,
                                             const std::vector<GridCell>& route,
                                             const double blocked_cost) {
  if (!map.valid()) {
    throw std::invalid_argument("hazard cost map dimensions do not match cost storage");
  }

  HazardRouteMetrics metrics;
  if (route.empty()) {
    metrics.min_clearance_cells = std::numeric_limits<double>::infinity();
    metrics.min_clearance_m = std::numeric_limits<double>::infinity();
    return metrics;
  }

  double total_cell_cost = 0.0;
  metrics.max_cost = -std::numeric_limits<double>::infinity();
  metrics.min_clearance_cells = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index < route.size(); ++index) {
    const GridCell cell = route.at(index);
    const double cost = map.costAt(cell);
    total_cell_cost += cost;
    metrics.max_cost = std::max(metrics.max_cost, cost);
    metrics.min_clearance_cells =
        std::min(metrics.min_clearance_cells, clearanceToNearestBlockedCell(map, cell, blocked_cost));

    if (index > 0U) {
      metrics.route_length_m += euclideanCells(route.at(index - 1U), cell) * map.resolution_m;
    }
  }

  metrics.mean_cost = total_cell_cost / static_cast<double>(route.size());
  metrics.straight_line_length_m = euclideanCells(route.front(), route.back()) * map.resolution_m;
  metrics.detour_ratio =
      metrics.straight_line_length_m > 0.0
          ? metrics.route_length_m / metrics.straight_line_length_m
          : 1.0;
  metrics.min_clearance_m = metrics.min_clearance_cells * map.resolution_m;
  return metrics;
}

std::vector<GridCell> resampleRoute(const std::vector<GridCell>& route,
                                    const std::size_t target_count) {
  if (target_count == 0U || route.empty()) {
    return {};
  }
  if (route.size() <= target_count) {
    return route;
  }
  if (target_count == 1U) {
    return {route.front()};
  }

  std::vector<GridCell> sampled;
  sampled.reserve(target_count);
  const double last = static_cast<double>(route.size() - 1U);
  const double denominator = static_cast<double>(target_count - 1U);
  for (std::size_t index = 0; index < target_count; ++index) {
    const double t = static_cast<double>(index) / denominator;
    const auto route_index = static_cast<std::size_t>(std::llround(t * last));
    sampled.push_back(route.at(route_index));
  }
  return sampled;
}

}  // namespace astro::navigation
