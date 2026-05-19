#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "astro_navigation/navigation/hazard_guidance.hpp"

namespace {

struct Args {
  std::filesystem::path cost_map;
  std::optional<int> start_cell_x;
  std::optional<int> start_cell_y;
  std::optional<int> goal_cell_x;
  std::optional<int> goal_cell_y;
  std::optional<double> start_x_m;
  std::optional<double> start_y_m;
  std::optional<double> goal_x_m;
  std::optional<double> goal_y_m;
  double resolution_m{1.0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  double blocked_cost{1.0e9};
  double heuristic_weight{1.25};
  int snap_radius_cells{8};
  bool allow_diagonal{true};
  std::filesystem::path output_csv;
  std::filesystem::path output_json;
};

void printUsage() {
  std::cerr
      << "Usage: hazard_route_demo --cost-map cost.csv "
         "(--start-cell-x <x> --start-cell-y <y> --goal-cell-x <x> --goal-cell-y <y> | "
         "--start-x-m <x> --start-y-m <y> --goal-x-m <x> --goal-y-m <y>) "
         "[--resolution-m <m>] [--origin-x-m <m>] [--origin-y-m <m>] "
         "[--blocked-cost <cost>] [--heuristic-weight <w>] [--snap-radius-cells <n>] "
         "[--no-diagonal] [--output-csv route.csv] [--output-json route.json]\n";
}

double parseDouble(const char* value, const std::string& name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
    throw std::invalid_argument("invalid numeric value for " + name + ": " + value);
  }
  return parsed;
}

int parseInt(const char* value, const std::string& name) {
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0') {
    throw std::invalid_argument("invalid integer value for " + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

Args parseArgs(const int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    auto requireValue = [&](const std::string& option) -> const char* {
      if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + option);
      }
      return argv[++i];
    };

    if (key == "--cost-map") {
      args.cost_map = requireValue(key);
    } else if (key == "--start-cell-x") {
      args.start_cell_x = parseInt(requireValue(key), key);
    } else if (key == "--start-cell-y") {
      args.start_cell_y = parseInt(requireValue(key), key);
    } else if (key == "--goal-cell-x") {
      args.goal_cell_x = parseInt(requireValue(key), key);
    } else if (key == "--goal-cell-y") {
      args.goal_cell_y = parseInt(requireValue(key), key);
    } else if (key == "--start-x-m") {
      args.start_x_m = parseDouble(requireValue(key), key);
    } else if (key == "--start-y-m") {
      args.start_y_m = parseDouble(requireValue(key), key);
    } else if (key == "--goal-x-m") {
      args.goal_x_m = parseDouble(requireValue(key), key);
    } else if (key == "--goal-y-m") {
      args.goal_y_m = parseDouble(requireValue(key), key);
    } else if (key == "--resolution-m") {
      args.resolution_m = parseDouble(requireValue(key), key);
    } else if (key == "--origin-x-m") {
      args.origin_x_m = parseDouble(requireValue(key), key);
    } else if (key == "--origin-y-m") {
      args.origin_y_m = parseDouble(requireValue(key), key);
    } else if (key == "--blocked-cost") {
      args.blocked_cost = parseDouble(requireValue(key), key);
    } else if (key == "--heuristic-weight") {
      args.heuristic_weight = parseDouble(requireValue(key), key);
    } else if (key == "--snap-radius-cells") {
      args.snap_radius_cells = parseInt(requireValue(key), key);
    } else if (key == "--no-diagonal") {
      args.allow_diagonal = false;
    } else if (key == "--output-csv") {
      args.output_csv = requireValue(key);
    } else if (key == "--output-json") {
      args.output_json = requireValue(key);
    } else if (key == "--help" || key == "-h") {
      printUsage();
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }

  if (args.cost_map.empty()) {
    throw std::invalid_argument("--cost-map is required");
  }
  if (args.resolution_m <= 0.0) {
    throw std::invalid_argument("--resolution-m must be positive");
  }

  const int cell_count =
      (args.start_cell_x ? 1 : 0) + (args.start_cell_y ? 1 : 0) + (args.goal_cell_x ? 1 : 0) +
      (args.goal_cell_y ? 1 : 0);
  const int meter_count =
      (args.start_x_m ? 1 : 0) + (args.start_y_m ? 1 : 0) + (args.goal_x_m ? 1 : 0) +
      (args.goal_y_m ? 1 : 0);
  if ((cell_count != 0 && cell_count != 4) || (meter_count != 0 && meter_count != 4)) {
    throw std::invalid_argument("start/goal coordinates must be provided as a complete cell or meter set");
  }
  if ((cell_count == 0 && meter_count == 0) || (cell_count != 0 && meter_count != 0)) {
    throw std::invalid_argument("provide exactly one start/goal coordinate mode");
  }

  return args;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream input(line);
  while (std::getline(input, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

astro::navigation::HazardCostMap loadCostMapCsv(const std::filesystem::path& path,
                                                const double resolution_m,
                                                const Eigen::Vector2d& origin_xy_m) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open cost map: " + path.string());
  }

  std::vector<double> costs;
  int width = 0;
  int height = 0;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const auto fields = splitCsvLine(line);
    if (fields.empty()) {
      continue;
    }
    if (width == 0) {
      width = static_cast<int>(fields.size());
    } else if (width != static_cast<int>(fields.size())) {
      throw std::runtime_error("cost map rows must all have the same width");
    }

    for (const auto& field : fields) {
      costs.push_back(parseDouble(field.c_str(), "cost-map cell"));
    }
    ++height;
  }

  astro::navigation::HazardCostMap map;
  map.width = width;
  map.height = height;
  map.resolution_m = resolution_m;
  map.origin_xy_m = origin_xy_m;
  map.costs = std::move(costs);
  if (!map.valid()) {
    throw std::runtime_error("cost map is empty or malformed");
  }
  return map;
}

std::string jsonNumber(const double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(15) << value;
  return output.str();
}

void writeRouteCsv(const std::filesystem::path& path,
                   const astro::navigation::HazardCostMap& map,
                   const astro::navigation::HazardRoute& route,
                   const double blocked_cost) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write route CSV: " + path.string());
  }
  output << "index,cell_x,cell_y,x_m,y_m,cost,cumulative_distance_m,clearance_cells,clearance_m\n";
  output << std::setprecision(15);
  double cumulative_distance_m = 0.0;
  for (std::size_t index = 0; index < route.cells.size(); ++index) {
    const auto cell = route.cells.at(index);
    const auto xy_m = route.waypoints_xy_m.at(index);
    if (index > 0U) {
      const auto previous = route.waypoints_xy_m.at(index - 1U);
      cumulative_distance_m += (xy_m - previous).norm();
    }
    const double clearance_cells =
        astro::navigation::clearanceToNearestBlockedCell(map, cell, blocked_cost);
    const double clearance_m = clearance_cells * map.resolution_m;
    output << index << ',' << cell.x << ',' << cell.y << ',' << xy_m.x() << ',' << xy_m.y() << ','
           << map.costAt(cell) << ',' << cumulative_distance_m << ',' << clearance_cells << ','
           << clearance_m << '\n';
  }
}

void writeRouteJson(const std::filesystem::path& path,
                    const astro::navigation::HazardCostMap& map,
                    const astro::navigation::HazardRoute& route) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write route JSON: " + path.string());
  }
  output << std::setprecision(15);
  output << "{\n";
  output << "  \"success\": " << (route.success() ? "true" : "false") << ",\n";
  output << "  \"message\": \"" << route.message << "\",\n";
  output << "  \"width\": " << map.width << ",\n";
  output << "  \"height\": " << map.height << ",\n";
  output << "  \"resolution_m\": " << map.resolution_m << ",\n";
  output << "  \"origin_xy_m\": [" << map.origin_xy_m.x() << ", " << map.origin_xy_m.y() << "],\n";
  output << "  \"total_cost\": " << jsonNumber(route.total_cost) << ",\n";
  output << "  \"metrics\": {\n";
  output << "    \"route_length_m\": " << jsonNumber(route.metrics.route_length_m) << ",\n";
  output << "    \"straight_line_length_m\": " << jsonNumber(route.metrics.straight_line_length_m)
         << ",\n";
  output << "    \"detour_ratio\": " << jsonNumber(route.metrics.detour_ratio) << ",\n";
  output << "    \"mean_cost\": " << jsonNumber(route.metrics.mean_cost) << ",\n";
  output << "    \"max_cost\": " << jsonNumber(route.metrics.max_cost) << ",\n";
  output << "    \"min_clearance_cells\": " << jsonNumber(route.metrics.min_clearance_cells) << ",\n";
  output << "    \"min_clearance_m\": " << jsonNumber(route.metrics.min_clearance_m) << "\n";
  output << "  },\n";
  output << "  \"waypoints\": [\n";
  for (std::size_t index = 0; index < route.cells.size(); ++index) {
    const auto cell = route.cells.at(index);
    const auto xy_m = route.waypoints_xy_m.at(index);
    output << "    {\"cell\": [" << cell.x << ", " << cell.y << "], \"xy_m\": [" << xy_m.x()
           << ", " << xy_m.y() << "], \"cost\": " << map.costAt(cell) << "}";
    output << (index + 1U == route.cells.size() ? "\n" : ",\n");
  }
  output << "  ]\n";
  output << "}\n";
}

}  // namespace

int main(const int argc, char** argv) {
  try {
    const Args args = parseArgs(argc, argv);
    const auto map =
        loadCostMapCsv(args.cost_map, args.resolution_m, {args.origin_x_m, args.origin_y_m});

    astro::navigation::HazardPlannerOptions options;
    options.blocked_cost = args.blocked_cost;
    options.heuristic_weight = args.heuristic_weight;
    options.snap_radius_cells = args.snap_radius_cells;
    options.allow_diagonal = args.allow_diagonal;

    astro::navigation::HazardRoute route;
    if (args.start_cell_x && args.start_cell_y && args.goal_cell_x && args.goal_cell_y) {
      route = astro::navigation::planHazardAwareRoute(
          map, {*args.start_cell_x, *args.start_cell_y}, {*args.goal_cell_x, *args.goal_cell_y}, options);
    } else {
      route = astro::navigation::planHazardAwareRouteMeters(
          map,
          {*args.start_x_m, *args.start_y_m},
          {*args.goal_x_m, *args.goal_y_m},
          options);
    }

    std::cout
        << "success,message,cells,total_cost,route_length_m,straight_line_length_m,detour_ratio,"
           "mean_cost,max_cost,min_clearance_cells,min_clearance_m,start_cell_x,start_cell_y,"
           "goal_cell_x,goal_cell_y\n";
    if (route.success()) {
      std::cout << "1," << route.message << ',' << route.cells.size() << ',' << route.total_cost << ','
                << route.metrics.route_length_m << ',' << route.metrics.straight_line_length_m << ','
                << route.metrics.detour_ratio << ',' << route.metrics.mean_cost << ','
                << route.metrics.max_cost << ',' << route.metrics.min_clearance_cells << ','
                << route.metrics.min_clearance_m << ',' << route.cells.front().x << ','
                << route.cells.front().y << ',' << route.cells.back().x << ',' << route.cells.back().y
                << '\n';
    } else {
      std::cout << "0," << route.message << ",0," << route.total_cost << ",0,0,0,0,0,0,0,0,0,0,0\n";
    }

    if (!args.output_csv.empty()) {
      writeRouteCsv(args.output_csv, map, route, options.blocked_cost);
    }
    if (!args.output_json.empty()) {
      writeRouteJson(args.output_json, map, route);
    }

    return route.success() ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    printUsage();
    return EXIT_FAILURE;
  }
}
