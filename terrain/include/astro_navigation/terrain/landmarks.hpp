#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace astro::terrain {

struct TerrainLandmark {
  cv::Point2f pixel;
  float response{0.0F};
  float scale{0.0F};
};

std::vector<TerrainLandmark> extractOrbTerrainLandmarks(const cv::Mat& gray_image,
                                                        int max_features = 1000);

}  // namespace astro::terrain
