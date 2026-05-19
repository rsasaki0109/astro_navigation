#include <Eigen/Core>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

int fail(const std::string& message) {
  std::cerr << "mission_navigation_cli_smoke: " << message << '\n';
  return 1;
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
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

void writeStarFixture(const std::filesystem::path& catalog_path,
                      const std::filesystem::path& observations_path) {
  std::ofstream catalog(catalog_path);
  std::ofstream observations(observations_path);
  catalog << "id,x,y,z\n";
  observations << "id,u,v\n";

  const std::vector<Eigen::Vector2d> pixels = {
      {512.0, 512.0}, {620.0, 500.0}, {440.0, 610.0}, {570.0, 680.0}};
  for (std::size_t index = 0; index < pixels.size(); ++index) {
    const Eigen::Vector3d bearing = Eigen::Vector3d((pixels[index].x() - 512.0) / 1000.0,
                                                    (pixels[index].y() - 512.0) / 1000.0, 1.0)
                                        .normalized();
    catalog << "star_" << index << ',' << bearing.x() << ',' << bearing.y() << ',' << bearing.z()
            << '\n';
    observations << "star_" << index << ',' << pixels[index].x() << ',' << pixels[index].y()
                 << '\n';
  }
}

}  // namespace

int main(const int argc, char** argv) {
  if (argc != 4) {
    return fail(
        "usage: mission_navigation_cli_smoke <mission-navigation-demo> <trn-summary.json> "
        "<output-dir>");
  }

  const std::filesystem::path app_path(argv[1]);
  const std::filesystem::path trn_summary_path(argv[2]);
  const std::filesystem::path output_dir(argv[3]);
  std::filesystem::create_directories(output_dir);

  const auto catalog_path = output_dir / "catalog.csv";
  const auto observations_path = output_dir / "observations.csv";
  const auto json_path = output_dir / "nav_state.json";
  const auto csv_path = output_dir / "nav_state.csv";
  const auto stdout_path = output_dir / "stdout.csv";
  writeStarFixture(catalog_path, observations_path);

  const std::string command = shellQuote(app_path) + " --catalog " + shellQuote(catalog_path) +
                              " --observations " + shellQuote(observations_path) +
                              " --fx 1000 --fy 1000 --cx 512 --cy 512 --trn-summary " +
                              shellQuote(trn_summary_path) +
                              " --localizability-score 0.63 --route-trn-confidence 0.38 "
                              "--output-json " +
                              shellQuote(json_path) + " --output-csv " + shellQuote(csv_path) +
                              " > " + shellQuote(stdout_path);

  const int rc = std::system(command.c_str());
  if (rc != 0) {
    return fail("mission_navigation_demo returned non-zero status");
  }

  const std::string stdout_text = readText(stdout_path);
  const std::string json_text = readText(json_path);
  const std::string csv_text = readText(csv_path);

  if (stdout_text.find("status,status_reason,attitude_lock,position_lock") == std::string::npos ||
      stdout_text.find("DEGRADED,ROUTE_RISK_HIGH,1,1,4,") == std::string::npos ||
      stdout_text.find(",0.63,0.38,0.62,82,11,map,") == std::string::npos ||
      stdout_text.find(",82,11,map,") == std::string::npos) {
    return fail("stdout CSV did not contain expected route-risk navigation row");
  }
  if (json_text.find("\"status\": \"DEGRADED\"") == std::string::npos ||
      json_text.find("\"status_reason\": \"ROUTE_RISK_HIGH\"") == std::string::npos ||
      json_text.find("\"position_frame_id\": \"map\"") == std::string::npos ||
      json_text.find("\"localizability_score\": 0.630000000") == std::string::npos ||
      json_text.find("\"route_trn_confidence\": 0.380000000") == std::string::npos ||
      json_text.find("\"navigation_risk_score\": 0.620000000") == std::string::npos ||
      json_text.find("\"attitude_correspondences\": 4") == std::string::npos) {
    return fail("JSON output did not contain expected navigation fields");
  }
  if (csv_text.find("timestamp,status,status_reason,attitude_lock") == std::string::npos ||
      csv_text.find(",DEGRADED,ROUTE_RISK_HIGH,1,1,4,") == std::string::npos ||
      csv_text.find(",0.630000000,0.380000000,0.620000000,") == std::string::npos) {
    return fail("CSV output did not contain expected navigation row");
  }

  return 0;
}
