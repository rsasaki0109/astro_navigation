#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "astro_navigation/core/image_sequence.hpp"
#include "astro_navigation/crater/crater_detector.hpp"
#include "astro_navigation/localization/visual_odometry.hpp"
#include "astro_navigation/visualization/trajectory_io.hpp"

namespace {

struct Args {
  std::filesystem::path images;
  std::filesystem::path trajectory{"outputs/trajectory.tum"};
  astro::core::CameraIntrinsics intrinsics;
  astro::localization::VisualOdometryOptions vo;
  bool detect_craters{false};
  bool use_clahe{false};
  double clahe_clip_limit{2.0};
  int clahe_tile_grid_size{8};
};

void printUsage() {
  std::cerr
      << "Usage: lunar_visual_odometry --images <dir|list.txt> --fx <fx> --fy <fy> --cx <cx> "
         "--cy <cy> [--feature orb|sift] [--trajectory outputs/traj.tum] [--detect-craters] "
         "[--clahe] [--clahe-clip-limit 2.0] [--clahe-tile-grid-size 8]\n";
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

cv::Mat preprocessImage(const cv::Mat& image, const Args& args) {
  if (!args.use_clahe) {
    return image;
  }
  cv::Mat output;
  const int grid = std::max(1, args.clahe_tile_grid_size);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(args.clahe_clip_limit, cv::Size(grid, grid));
  clahe->apply(image, output);
  return output;
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

    if (key == "--images") {
      args.images = requireValue(key);
    } else if (key == "--trajectory") {
      args.trajectory = requireValue(key);
    } else if (key == "--fx") {
      args.intrinsics.fx = parseDouble(requireValue(key), key);
    } else if (key == "--fy") {
      args.intrinsics.fy = parseDouble(requireValue(key), key);
    } else if (key == "--cx") {
      args.intrinsics.cx = parseDouble(requireValue(key), key);
    } else if (key == "--cy") {
      args.intrinsics.cy = parseDouble(requireValue(key), key);
    } else if (key == "--feature") {
      args.vo.feature_type = astro::localization::parseFeatureType(requireValue(key));
    } else if (key == "--max-features") {
      args.vo.max_features = parseInt(requireValue(key), key);
    } else if (key == "--detect-craters") {
      args.detect_craters = true;
    } else if (key == "--clahe") {
      args.use_clahe = true;
    } else if (key == "--clahe-clip-limit") {
      args.clahe_clip_limit = parseDouble(requireValue(key), key);
    } else if (key == "--clahe-tile-grid-size") {
      args.clahe_tile_grid_size = parseInt(requireValue(key), key);
    } else if (key == "--help" || key == "-h") {
      printUsage();
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }

  if (args.images.empty()) {
    throw std::invalid_argument("--images is required");
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
    const std::vector<std::filesystem::path> images = astro::core::loadImageSequence(args.images);
    if (images.size() < 2) {
      throw std::runtime_error("at least two images are required for visual odometry");
    }

    astro::localization::VisualOdometry odometry(args.intrinsics, args.vo);
    std::vector<astro::core::PoseStamped> trajectory;
    trajectory.reserve(images.size());

    std::cout << "frames,feature,matches,inliers,status\n";
    for (std::size_t index = 0; index < images.size(); ++index) {
      const cv::Mat image = preprocessImage(astro::core::loadGrayImage(images[index]), args);
      const auto estimate = odometry.process(image, static_cast<double>(index));
      trajectory.push_back(estimate.pose);

      std::cout << estimate.frame_index << ',' << astro::localization::toString(args.vo.feature_type)
                << ',' << estimate.motion.match_count << ',' << estimate.motion.inlier_count << ','
                << estimate.motion.message;
      if (args.detect_craters) {
        const auto craters = astro::crater::detectCircularCraters(image);
        std::cout << ",craters=" << craters.size();
      }
      std::cout << '\n';
    }

    if (args.trajectory.extension() == ".csv") {
      astro::visualization::writeCsvTrajectory(args.trajectory, trajectory);
    } else {
      astro::visualization::writeTumTrajectory(args.trajectory, trajectory);
    }
    std::cerr << "wrote trajectory: " << args.trajectory << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    printUsage();
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
