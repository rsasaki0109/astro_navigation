#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "astro_navigation/navigation/pipeline.hpp"
#include "astro_navigation/navigation/state.hpp"
#include "astro_navigation/navigation/state_io.hpp"
#include "astro_navigation/navigation/trn_summary.hpp"

namespace {

bool near(const double actual, const double expected, const double tolerance) {
  return std::abs(actual - expected) <= tolerance;
}

int fail(const std::string& message) {
  std::cerr << "navigation_state_smoke: " << message << '\n';
  return 1;
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

}  // namespace

int main(const int argc, char** argv) {
  if (argc != 3) {
    return fail("usage: navigation_state_smoke <trn-summary.json> <output-dir>");
  }

  const std::filesystem::path trn_summary_path(argv[1]);
  const std::filesystem::path output_dir(argv[2]);

  astro::navigation::NavState empty_state;
  astro::navigation::refreshStatus(empty_state);
  if (empty_state.status != astro::navigation::NavStatus::kLost) {
    return fail("empty state should be LOST");
  }
  if (empty_state.status_reason != astro::navigation::NavStatusReason::kNoLocks) {
    return fail("empty state should report NO_LOCKS");
  }

  astro::localization::StarTrackerEstimate attitude;
  attitude.success = true;
  attitude.correspondence_count = 30;
  attitude.rms_direction_error_rad = 0.01;
  attitude.q_camera_inertial = Eigen::Quaterniond::Identity();

  auto state = astro::navigation::fromStarTrackerEstimate(attitude, 12.5);
  if (state.status != astro::navigation::NavStatus::kDegraded) {
    return fail("attitude-only state should be DEGRADED");
  }
  if (state.status_reason != astro::navigation::NavStatusReason::kAttitudeOnly) {
    return fail("attitude-only state should report ATTITUDE_ONLY");
  }
  if (!state.quality.attitude_lock || state.quality.position_lock) {
    return fail("attitude-only lock flags are wrong");
  }
  if (!near(state.covariance(3, 3), 0.0001, 1.0e-12)) {
    return fail("attitude covariance should use squared RMS error");
  }

  const auto trn_lock = astro::navigation::loadTrnSummaryPositionLock(trn_summary_path);
  if (!near(trn_lock.position.x(), 46069.113087615, 1.0e-6) ||
      !near(trn_lock.position.y(), 46097.73073361571, 1.0e-6) ||
      !near(trn_lock.position.z(), 30000.596809295897, 1.0e-6)) {
    return fail("TRN position did not match summary fixture");
  }
  if (!near(trn_lock.sigma_m, 143.9561409500211, 1.0e-9)) {
    return fail("TRN sigma did not match map/reprojection/inlier quality model");
  }
  if (!near(trn_lock.quality_terms.map_resolution_sigma_m, 143.9561409500211, 1.0e-9)) {
    return fail("TRN map-resolution sigma term did not match fixture");
  }
  if (!near(trn_lock.quality_terms.reprojection_sigma_m, 6.911283349000403, 1.0e-9)) {
    return fail("TRN reprojection sigma term did not match fixture");
  }
  if (!near(trn_lock.quality_terms.inlier_geometry_sigma_m, 75.17864273102316, 1.0e-9)) {
    return fail("TRN inlier-geometry sigma term did not match fixture");
  }
  if (!near(trn_lock.evaluation_error_m, 31.92681015364329, 1.0e-9)) {
    return fail("TRN evaluation error did not preserve position_error_m");
  }
  if (trn_lock.match_count != 82 || trn_lock.inlier_count != 11) {
    return fail("TRN match/inlier counts did not match summary fixture");
  }

  astro::navigation::applyPositionLock(state, trn_lock.position, "map", trn_lock.sigma_m);
  if (state.status != astro::navigation::NavStatus::kOk) {
    return fail("attitude + TRN position should be OK");
  }
  if (state.status_reason != astro::navigation::NavStatusReason::kNone) {
    return fail("attitude + TRN position should report NONE reason");
  }
  if (!near(state.covariance(0, 0), trn_lock.sigma_m * trn_lock.sigma_m, 1.0e-6)) {
    return fail("position covariance should use squared TRN sigma");
  }

  std::filesystem::create_directories(output_dir);
  const auto json_path = output_dir / "nav_state.json";
  const auto csv_path = output_dir / "nav_state.csv";
  astro::navigation::writeNavStateJson(json_path, state);
  astro::navigation::writeNavStateCsv(csv_path, state);

  const std::string json = readText(json_path);
  const std::string csv = readText(csv_path);
  if (json.find("\"status\": \"OK\"") == std::string::npos ||
      json.find("\"status_reason\": \"NONE\"") == std::string::npos ||
      json.find("\"position_sigma_m\": 143.956140950") == std::string::npos) {
    return fail("JSON output is missing expected navigation fields");
  }
  if (csv.find("timestamp,status,status_reason,attitude_lock") == std::string::npos ||
      csv.find(",OK,NONE,1,1,30,") == std::string::npos) {
    return fail("CSV output is missing expected navigation fields");
  }

  const auto catalog_path = output_dir / "pipeline_catalog.csv";
  const auto observations_path = output_dir / "pipeline_observations.csv";
  std::ofstream catalog_output(catalog_path);
  std::ofstream observations_output(observations_path);
  catalog_output << "id,x,y,z\n";
  observations_output << "id,u,v\n";
  const std::vector<Eigen::Vector2d> pixels = {
      {512.0, 512.0}, {620.0, 500.0}, {440.0, 610.0}, {570.0, 680.0}};
  for (std::size_t index = 0; index < pixels.size(); ++index) {
    const Eigen::Vector3d bearing =
        Eigen::Vector3d((pixels[index].x() - 512.0) / 1000.0,
                        (pixels[index].y() - 512.0) / 1000.0,
                        1.0)
            .normalized();
    catalog_output << "star_" << index << ',' << bearing.x() << ',' << bearing.y() << ','
                   << bearing.z() << '\n';
    observations_output << "star_" << index << ',' << pixels[index].x() << ',' << pixels[index].y()
                        << '\n';
  }
  catalog_output.close();
  observations_output.close();

  astro::navigation::MissionNavigationInput pipeline_input;
  pipeline_input.catalog_path = catalog_path.string();
  pipeline_input.observations_path = observations_path.string();
  pipeline_input.intrinsics.fx = 1000.0;
  pipeline_input.intrinsics.fy = 1000.0;
  pipeline_input.intrinsics.cx = 512.0;
  pipeline_input.intrinsics.cy = 512.0;
  pipeline_input.trn_summary_path = trn_summary_path.string();

  const auto pipeline_result = astro::navigation::runMissionNavigation(pipeline_input);
  if (pipeline_result.state.status != astro::navigation::NavStatus::kOk) {
    return fail("pipeline should produce OK with star tracker and TRN inputs");
  }
  if (pipeline_result.trn_matches != 82 || pipeline_result.trn_inliers != 11) {
    return fail("pipeline did not preserve TRN quality counts");
  }
  if (!pipeline_result.position_lock) {
    return fail("pipeline did not expose TRN position lock measurement");
  }
  if (!near(pipeline_result.position_lock->quality_terms.map_resolution_sigma_m,
            143.9561409500211,
            1.0e-9)) {
    return fail("pipeline did not expose TRN quality terms");
  }
  if (pipeline_result.state.quality.attitude_correspondences != 4) {
    return fail("pipeline did not preserve star tracker correspondence count");
  }

  return 0;
}
