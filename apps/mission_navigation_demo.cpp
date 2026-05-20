#include <Eigen/Core>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include "astro_navigation/navigation/pipeline.hpp"
#include "astro_navigation/navigation/state_io.hpp"

namespace {

struct Args {
  std::string observations;
  std::string catalog;
  astro::core::CameraIntrinsics intrinsics;
  double timestamp{0.0};
  std::optional<double> position_x;
  std::optional<double> position_y;
  std::optional<double> position_z;
  std::optional<double> position_sigma_m;
  std::string position_frame_id{"map"};
  std::filesystem::path trn_summary;
  std::filesystem::path output_json;
  std::filesystem::path output_csv;
  std::optional<double> localizability_score;
  std::optional<double> route_trn_confidence;
};

void printUsage() {
  std::cerr << "Usage: mission_navigation_demo --observations stars.csv --catalog catalog.csv "
               "--fx <fx> --fy <fy> --cx <cx> --cy <cy> "
               "[--timestamp <sec>] [--position-x <m> --position-y <m> --position-z <m>] "
               "[--position-sigma-m <m>] [--position-frame <name>] [--trn-summary summary.json] "
               "[--localizability-score <0..1>] [--route-trn-confidence <0..1>] "
               "[--output-json nav.json] [--output-csv nav.csv]\n";
}

double parseDouble(const char* value, const std::string& name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
    throw std::invalid_argument("invalid numeric value for " + name + ": " + value);
  }
  return parsed;
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

    if (key == "--observations") {
      args.observations = requireValue(key);
    } else if (key == "--catalog") {
      args.catalog = requireValue(key);
    } else if (key == "--fx") {
      args.intrinsics.fx = parseDouble(requireValue(key), key);
    } else if (key == "--fy") {
      args.intrinsics.fy = parseDouble(requireValue(key), key);
    } else if (key == "--cx") {
      args.intrinsics.cx = parseDouble(requireValue(key), key);
    } else if (key == "--cy") {
      args.intrinsics.cy = parseDouble(requireValue(key), key);
    } else if (key == "--timestamp") {
      args.timestamp = parseDouble(requireValue(key), key);
    } else if (key == "--position-x") {
      args.position_x = parseDouble(requireValue(key), key);
    } else if (key == "--position-y") {
      args.position_y = parseDouble(requireValue(key), key);
    } else if (key == "--position-z") {
      args.position_z = parseDouble(requireValue(key), key);
    } else if (key == "--position-sigma-m") {
      args.position_sigma_m = parseDouble(requireValue(key), key);
    } else if (key == "--position-frame") {
      args.position_frame_id = requireValue(key);
    } else if (key == "--trn-summary") {
      args.trn_summary = requireValue(key);
    } else if (key == "--output-json") {
      args.output_json = requireValue(key);
    } else if (key == "--output-csv") {
      args.output_csv = requireValue(key);
    } else if (key == "--localizability-score") {
      args.localizability_score = parseDouble(requireValue(key), key);
    } else if (key == "--route-trn-confidence") {
      args.route_trn_confidence = parseDouble(requireValue(key), key);
    } else if (key == "--help" || key == "-h") {
      printUsage();
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }

  if (args.observations.empty() || args.catalog.empty()) {
    throw std::invalid_argument("--observations and --catalog are required");
  }
  if (!args.intrinsics.valid()) {
    throw std::invalid_argument("--fx and --fy must be positive");
  }

  const int position_value_count = (args.position_x.has_value() ? 1 : 0) +
                                   (args.position_y.has_value() ? 1 : 0) +
                                   (args.position_z.has_value() ? 1 : 0);
  if (position_value_count != 0 && position_value_count != 3) {
    throw std::invalid_argument(
        "position lock requires --position-x, --position-y, and --position-z");
  }
  if (!args.trn_summary.empty() && position_value_count != 0) {
    throw std::invalid_argument("--trn-summary cannot be combined with manual position arguments");
  }
  return args;
}

}  // namespace

int main(const int argc, char** argv) {
  try {
    const Args args = parseArgs(argc, argv);
    astro::navigation::MissionNavigationInput input;
    input.observations_path = args.observations;
    input.catalog_path = args.catalog;
    input.intrinsics = args.intrinsics;
    input.timestamp = args.timestamp;
    input.position_frame_id = args.position_frame_id;
    input.position_sigma_override_m = args.position_sigma_m;
    input.localizability_score = args.localizability_score;
    input.route_trn_confidence = args.route_trn_confidence;
    if (!args.trn_summary.empty()) {
      input.trn_summary_path = args.trn_summary.string();
    } else if (args.position_x && args.position_y && args.position_z) {
      input.manual_position = astro::navigation::ManualPositionInput{
          Eigen::Vector3d(*args.position_x, *args.position_y, *args.position_z),
          args.position_sigma_m.value_or(100.0)};
    }

    const auto result = astro::navigation::runMissionNavigation(input);
    const auto& state = result.state;

    std::cout
        << "status,status_reason,attitude_lock,position_lock,correspondences,attitude_sigma_rad,"
           "position_sigma_m,localizability_score,route_trn_confidence,navigation_risk_score,"
           "trn_matches,trn_inliers,frame,x,y,z,qx,qy,qz,qw,message\n";
    std::cout << astro::navigation::toString(state.status) << ','
              << astro::navigation::toString(state.status_reason) << ','
              << (state.quality.attitude_lock ? 1 : 0) << ','
              << (state.quality.position_lock ? 1 : 0) << ','
              << state.quality.attitude_correspondences << ',' << state.quality.attitude_sigma_rad
              << ',' << state.quality.position_sigma_m << ','
              << state.quality.localizability_score << ',' << state.quality.route_trn_confidence
              << ',' << state.quality.navigation_risk_score << ',' << result.trn_matches << ','
              << result.trn_inliers << ',' << state.position_frame_id << ',' << state.position.x()
              << ',' << state.position.y() << ',' << state.position.z() << ','
              << state.q_body_reference.x() << ',' << state.q_body_reference.y() << ','
              << state.q_body_reference.z() << ',' << state.q_body_reference.w() << ','
              << state.message << '\n';

    if (!args.output_json.empty()) {
      astro::navigation::writeNavStateJson(args.output_json, state);
    }
    if (!args.output_csv.empty()) {
      astro::navigation::writeNavStateCsv(args.output_csv, state);
    }

    return state.status == astro::navigation::NavStatus::kLost ? EXIT_FAILURE : EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    printUsage();
    return EXIT_FAILURE;
  }
}
