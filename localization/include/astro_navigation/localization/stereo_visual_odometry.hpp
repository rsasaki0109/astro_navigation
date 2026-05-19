#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <optional>
#include <string>
#include <vector>

#include "astro_navigation/core/types.hpp"
#include "astro_navigation/localization/visual_odometry.hpp"

namespace astro::localization {

struct StereoCameraModel {
  core::CameraIntrinsics left;
  core::CameraIntrinsics right;
  cv::Matx33d R_right_left{cv::Matx33d::eye()};
  cv::Vec3d t_right_left{-0.4, 0.0, 0.0};
};

struct StereoVisualOdometryOptions {
  FeatureType feature_type{FeatureType::kOrb};
  int max_features{2500};
  double ratio_test{0.75};
  double min_depth_m{0.1};
  double max_depth_m{100.0};
  int min_stereo_matches{40};
  int min_pnp_points{10};
  int min_pnp_inliers{6};
  double pnp_reprojection_error_px{4.0};
  double pnp_confidence{0.999};
  double max_stereo_y_diff_px{80.0};
  double min_disparity_px{2.0};
};

struct StereoMotionEstimate {
  bool success{false};
  int stereo_match_count{0};
  int temporal_match_count{0};
  int pnp_point_count{0};
  int pnp_inlier_count{0};
  int valid_3d_point_count{0};
  Eigen::Isometry3d T_previous_current{Eigen::Isometry3d::Identity()};
  std::string message;
};

struct StereoFrameEstimate {
  std::size_t frame_index{0};
  StereoMotionEstimate motion;
  core::PoseStamped pose;
};

class StereoVisualOdometry {
 public:
  StereoVisualOdometry(StereoCameraModel camera, StereoVisualOdometryOptions options);

  StereoFrameEstimate process(const cv::Mat& left_gray, const cv::Mat& right_gray,
                              double timestamp);

 private:
  struct Features {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
  };

  struct StereoFrame {
    Features left;
    std::vector<std::optional<cv::Point3f>> left_points_m;
    Features depth_features;
    std::vector<cv::Point3f> depth_points_m;
    int stereo_match_count{0};
    int valid_3d_point_count{0};
  };

  Features extract(const cv::Mat& gray_image) const;
  StereoFrame buildStereoFrame(const cv::Mat& left_gray, const cv::Mat& right_gray) const;
  StereoMotionEstimate estimateMotion(const StereoFrame& previous,
                                      const StereoFrame& current) const;

  StereoCameraModel camera_;
  StereoVisualOdometryOptions options_;
  cv::Ptr<cv::Feature2D> detector_;
  std::optional<StereoFrame> previous_frame_;
  std::optional<StereoFrame> last_good_frame_;
  Eigen::Isometry3d T_world_at_last_good_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_world_camera_{Eigen::Isometry3d::Identity()};
  std::size_t frame_index_{0};
};

}  // namespace astro::localization
