#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "astro_localization/core/image_sequence.hpp"
#include "astro_localization/localization/stereo_visual_odometry.hpp"
#include "astro_localization/visualization/trajectory_io.hpp"

namespace {

struct Pair {
  std::filesystem::path left;
  std::filesystem::path right;
};

struct Args {
  std::filesystem::path pairs;
  std::filesystem::path trajectory{"outputs/stereo_trajectory.tum"};
  astro::localization::StereoCameraModel camera;
  astro::localization::StereoVisualOdometryOptions vo;
  bool use_clahe{false};
  double clahe_clip_limit{2.0};
  int clahe_tile_grid_size{8};
};

void printUsage() {
  std::cerr << "Usage: stereo_visual_odometry --pairs stereo_pairs.csv "
               "--fx <fx> --fy <fy> --cx <cx> --cy <cy> "
               "--right-fx <fx> --right-fy <fy> --right-cx <cx> --right-cy <cy> "
               "--baseline <meters> [--trajectory outputs/stereo.tum] "
               "[--feature orb|sift] [--max-features <n>] "
               "[--ratio-test 0.75] [--pnp-reproj-error 4.0] "
               "[--max-stereo-y-diff <px>] [--min-disparity <px>] "
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

std::vector<Pair> loadPairs(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open stereo pair list: " + path.string());
  }

  std::vector<Pair> pairs;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.starts_with("#") || line.starts_with("left,")) {
      continue;
    }
    std::stringstream stream(line);
    std::string left;
    std::string right;
    std::getline(stream, left, ',');
    std::getline(stream, right, ',');
    if (left.empty() || right.empty()) {
      continue;
    }
    pairs.push_back({left, right});
  }
  return pairs;
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

    if (key == "--pairs") {
      args.pairs = requireValue(key);
    } else if (key == "--trajectory") {
      args.trajectory = requireValue(key);
    } else if (key == "--fx") {
      args.camera.left.fx = parseDouble(requireValue(key), key);
    } else if (key == "--fy") {
      args.camera.left.fy = parseDouble(requireValue(key), key);
    } else if (key == "--cx") {
      args.camera.left.cx = parseDouble(requireValue(key), key);
    } else if (key == "--cy") {
      args.camera.left.cy = parseDouble(requireValue(key), key);
    } else if (key == "--right-fx") {
      args.camera.right.fx = parseDouble(requireValue(key), key);
    } else if (key == "--right-fy") {
      args.camera.right.fy = parseDouble(requireValue(key), key);
    } else if (key == "--right-cx") {
      args.camera.right.cx = parseDouble(requireValue(key), key);
    } else if (key == "--right-cy") {
      args.camera.right.cy = parseDouble(requireValue(key), key);
    } else if (key == "--baseline") {
      args.camera.t_right_left = cv::Vec3d(-parseDouble(requireValue(key), key), 0.0, 0.0);
    } else if (key == "--feature") {
      args.vo.feature_type = astro::localization::parseFeatureType(requireValue(key));
    } else if (key == "--max-features") {
      args.vo.max_features = parseInt(requireValue(key), key);
    } else if (key == "--ratio-test") {
      args.vo.ratio_test = parseDouble(requireValue(key), key);
    } else if (key == "--pnp-reproj-error") {
      args.vo.pnp_reprojection_error_px = parseDouble(requireValue(key), key);
    } else if (key == "--max-depth") {
      args.vo.max_depth_m = parseDouble(requireValue(key), key);
    } else if (key == "--max-stereo-y-diff") {
      args.vo.max_stereo_y_diff_px = parseDouble(requireValue(key), key);
    } else if (key == "--min-disparity") {
      args.vo.min_disparity_px = parseDouble(requireValue(key), key);
    } else if (key == "--min-pnp-points") {
      args.vo.min_pnp_points = parseInt(requireValue(key), key);
    } else if (key == "--min-pnp-inliers") {
      args.vo.min_pnp_inliers = parseInt(requireValue(key), key);
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

  if (args.pairs.empty()) {
    throw std::invalid_argument("--pairs is required");
  }
  if (!args.camera.left.valid() || !args.camera.right.valid()) {
    throw std::invalid_argument("left and right intrinsics are required");
  }
  return args;
}

}  // namespace

int main(const int argc, char** argv) {
  try {
    const Args args = parseArgs(argc, argv);
    const std::vector<Pair> pairs = loadPairs(args.pairs);
    if (pairs.size() < 2) {
      throw std::runtime_error("at least two stereo pairs are required");
    }

    astro::localization::StereoVisualOdometry odometry(args.camera, args.vo);
    std::vector<astro::core::PoseStamped> trajectory;
    trajectory.reserve(pairs.size());

    std::cout << "frames,stereo_matches,valid_3d_points,temporal_matches,pnp_points,pnp_inliers,status\n";
    for (std::size_t index = 0; index < pairs.size(); ++index) {
      const cv::Mat left = preprocessImage(astro::core::loadGrayImage(pairs[index].left), args);
      const cv::Mat right = preprocessImage(astro::core::loadGrayImage(pairs[index].right), args);
      const auto estimate = odometry.process(left, right, static_cast<double>(index));
      trajectory.push_back(estimate.pose);
      std::cout << estimate.frame_index << ',' << estimate.motion.stereo_match_count << ','
                << estimate.motion.valid_3d_point_count << ',' << estimate.motion.temporal_match_count
                << ',' << estimate.motion.pnp_point_count << ',' << estimate.motion.pnp_inlier_count
                << ',' << estimate.motion.message << '\n';
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
