#include "astro_navigation/crater/crater_detector.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace astro::crater {

std::vector<CraterCandidate> detectCircularCraters(const cv::Mat& gray_image,
                                                   const CraterDetectionOptions& options) {
  cv::Mat blurred;
  const int kernel = options.blur_kernel % 2 == 1 ? options.blur_kernel : options.blur_kernel + 1;
  cv::GaussianBlur(gray_image, blurred, cv::Size(kernel, kernel), 0.0);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, options.dp, options.min_dist_px,
                   options.canny_threshold, options.accumulator_threshold, options.min_radius_px,
                   options.max_radius_px);

  std::vector<CraterCandidate> candidates;
  candidates.reserve(circles.size());
  for (const auto& circle : circles) {
    candidates.push_back({cv::Point2f(circle[0], circle[1]), circle[2], circle[2]});
  }
  std::ranges::sort(candidates, [](const auto& lhs, const auto& rhs) {
    return lhs.score > rhs.score;
  });
  return candidates;
}

}  // namespace astro::crater

