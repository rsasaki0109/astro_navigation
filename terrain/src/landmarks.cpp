#include "astro_localization/terrain/landmarks.hpp"

#include <opencv2/features2d.hpp>

namespace astro::terrain {

std::vector<TerrainLandmark> extractOrbTerrainLandmarks(const cv::Mat& gray_image,
                                                        const int max_features) {
  cv::Ptr<cv::ORB> orb = cv::ORB::create(max_features);
  std::vector<cv::KeyPoint> keypoints;
  orb->detect(gray_image, keypoints);

  std::vector<TerrainLandmark> landmarks;
  landmarks.reserve(keypoints.size());
  for (const auto& keypoint : keypoints) {
    landmarks.push_back({keypoint.pt, keypoint.response, keypoint.size});
  }
  return landmarks;
}

}  // namespace astro::terrain

