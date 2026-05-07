#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "astro_localization/localization/star_tracker.hpp"

namespace {

struct Args {
  std::string observations;
  std::string catalog;
  astro::core::CameraIntrinsics intrinsics;
};

void printUsage() {
  std::cerr << "Usage: star_tracker_attitude --observations stars.csv --catalog catalog.csv "
               "--fx <fx> --fy <fy> --cx <cx> --cy <cy>\n";
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
  return args;
}

}  // namespace

int main(const int argc, char** argv) {
  try {
    const Args args = parseArgs(argc, argv);
    const auto observations = astro::localization::loadStarObservationsCsv(args.observations);
    const auto catalog = astro::localization::loadStarCatalogCsv(args.catalog);
    const auto estimate =
        astro::localization::estimateStarTrackerAttitude(observations, catalog, args.intrinsics);

    std::cout << "success,correspondences,rms_direction_error_rad,qx,qy,qz,qw,status\n";
    std::cout << (estimate.success ? 1 : 0) << ',' << estimate.correspondence_count << ','
              << estimate.rms_direction_error_rad << ',' << estimate.q_camera_inertial.x() << ','
              << estimate.q_camera_inertial.y() << ',' << estimate.q_camera_inertial.z() << ','
              << estimate.q_camera_inertial.w() << ',' << estimate.message << '\n';
    return estimate.success ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    printUsage();
    return EXIT_FAILURE;
  }
}

