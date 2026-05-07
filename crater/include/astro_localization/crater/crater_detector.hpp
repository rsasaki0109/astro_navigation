#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace astro::crater {

struct CraterDetectionOptions {
  double dp{1.2};
  double min_dist_px{40.0};
  double canny_threshold{120.0};
  double accumulator_threshold{24.0};
  int min_radius_px{8};
  int max_radius_px{160};
  int blur_kernel{5};
};

struct CraterCandidate {
  cv::Point2f center;
  float radius_px{0.0F};
  float score{0.0F};
};

std::vector<CraterCandidate> detectCircularCraters(const cv::Mat& gray_image,
                                                   const CraterDetectionOptions& options = {});

}  // namespace astro::crater

