#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int fail(const std::string& message) {
  std::cerr << "hazard_route_cli_smoke: " << message << '\n';
  return 1;
}

std::string shellQuote(const std::filesystem::path& path) {
  std::string value = path.string();
  std::string quoted = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      quoted += "'\\''";
    } else {
      quoted += ch;
    }
  }
  quoted += "'";
  return quoted;
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void writeCostMap(const std::filesystem::path& path) {
  std::ofstream output(path);
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 12; ++x) {
      double cost = 1.0;
      if (x == 5 && y != 4) {
        cost = 1000000000.0;
      } else if (x == 5 && y == 4) {
        cost = 1.2;
      }
      output << cost << (x + 1 == 12 ? '\n' : ',');
    }
  }
}

}  // namespace

int main(const int argc, char** argv) {
  if (argc != 3) {
    return fail("usage: hazard_route_cli_smoke <hazard-route-demo> <output-dir>");
  }

  const std::filesystem::path app_path(argv[1]);
  const std::filesystem::path output_dir(argv[2]);
  std::filesystem::create_directories(output_dir);

  const auto cost_map_path = output_dir / "cost_map.csv";
  const auto stdout_path = output_dir / "stdout.csv";
  const auto route_csv_path = output_dir / "route.csv";
  const auto route_json_path = output_dir / "route.json";
  writeCostMap(cost_map_path);

  const std::string command =
      shellQuote(app_path) + " --cost-map " + shellQuote(cost_map_path) +
      " --start-cell-x 1 --start-cell-y 3 --goal-cell-x 10 --goal-cell-y 3 "
      "--resolution-m 2 --origin-x-m 100 --origin-y-m 200 --blocked-cost 1000 "
      "--output-csv " +
      shellQuote(route_csv_path) + " --output-json " + shellQuote(route_json_path) + " > " +
      shellQuote(stdout_path);

  const int rc = std::system(command.c_str());
  if (rc != 0) {
    return fail("hazard_route_demo returned non-zero status");
  }

  const std::string stdout_text = readText(stdout_path);
  const std::string csv_text = readText(route_csv_path);
  const std::string json_text = readText(route_json_path);

  if (stdout_text.find("success,message,cells,total_cost") == std::string::npos ||
      stdout_text.find("route_length_m,straight_line_length_m,detour_ratio") == std::string::npos ||
      stdout_text.find("1,route planned,") == std::string::npos ||
      stdout_text.find(",1,3,10,3") == std::string::npos) {
    return fail("stdout summary did not contain expected route fields");
  }
  if (csv_text.find(
          "index,cell_x,cell_y,x_m,y_m,cost,cumulative_distance_m,clearance_cells,clearance_m") ==
          std::string::npos ||
      csv_text.find(",5,4,111,209,1.2") == std::string::npos) {
    return fail("route CSV did not include the safe wall gap waypoint");
  }
  if (json_text.find("\"success\": true") == std::string::npos ||
      json_text.find("\"message\": \"route planned\"") == std::string::npos ||
      json_text.find("\"detour_ratio\"") == std::string::npos ||
      json_text.find("\"min_clearance_cells\": 1") == std::string::npos ||
      json_text.find("\"cell\": [5, 4]") == std::string::npos) {
    return fail("route JSON did not include expected route fields");
  }

  return 0;
}
