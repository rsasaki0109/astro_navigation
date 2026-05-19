#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <optional>
#include <string>
#include <vector>

#include "astro_navigation/core/types.hpp"

namespace astro::localization {

enum class FeatureType { kOrb, kSift };

struct VisualOdometryOptions {
  FeatureType feature_type{FeatureType::kOrb};
  int max_features{2000};
  double ratio_test{0.75};
  double ransac_threshold_px{1.0};
  double ransac_confidence{0.999};
  int min_matches{40};
  int min_inliers{25};
};

struct MotionEstimate {
  bool success{false};
  int match_count{0};
  int inlier_count{0};
  Eigen::Isometry3d T_previous_current{Eigen::Isometry3d::Identity()};
  std::string message;
};

struct FrameEstimate {
  std::size_t frame_index{0};
  MotionEstimate motion;
  core::PoseStamped pose;
};

FeatureType parseFeatureType(const std::string& name);
std::string toString(FeatureType type);

class VisualOdometry {
 public:
  VisualOdometry(core::CameraIntrinsics intrinsics, VisualOdometryOptions options);

  FrameEstimate process(const cv::Mat& gray_image, double timestamp);

 private:
  struct Features {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
  };

  Features extract(const cv::Mat& gray_image) const;
  MotionEstimate estimateMotion(const Features& previous, const Features& current) const;

  core::CameraIntrinsics intrinsics_;
  VisualOdometryOptions options_;
  cv::Ptr<cv::Feature2D> detector_;
  std::optional<Features> previous_features_;
  Eigen::Isometry3d T_world_camera_{Eigen::Isometry3d::Identity()};
  std::size_t frame_index_{0};
};

}  // namespace astro::localization
