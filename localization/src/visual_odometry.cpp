#include "astro_navigation/localization/visual_odometry.hpp"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

namespace astro::localization {
namespace {

cv::Mat cameraMatrix(const core::CameraIntrinsics& intrinsics) {
  return (cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
          intrinsics.cy, 0.0, 0.0, 1.0);
}

Eigen::Isometry3d toEigenIsometry(const cv::Mat& rotation, const cv::Mat& translation) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      transform.linear()(row, col) = rotation.at<double>(row, col);
    }
    transform.translation()(row) = translation.at<double>(row);
  }
  return transform;
}

}  // namespace

FeatureType parseFeatureType(const std::string& name) {
  if (name == "orb" || name == "ORB") {
    return FeatureType::kOrb;
  }
  if (name == "sift" || name == "SIFT") {
    return FeatureType::kSift;
  }
  throw std::invalid_argument("unknown feature type: " + name);
}

std::string toString(const FeatureType type) {
  switch (type) {
    case FeatureType::kOrb:
      return "ORB";
    case FeatureType::kSift:
      return "SIFT";
  }
  return "unknown";
}

VisualOdometry::VisualOdometry(core::CameraIntrinsics intrinsics, VisualOdometryOptions options)
    : intrinsics_(intrinsics), options_(options) {
  if (!intrinsics_.valid()) {
    throw std::invalid_argument("camera intrinsics must include positive fx and fy");
  }
  if (options_.feature_type == FeatureType::kOrb) {
    detector_ = cv::ORB::create(options_.max_features);
  } else {
    detector_ = cv::SIFT::create(options_.max_features);
  }
}

FrameEstimate VisualOdometry::process(const cv::Mat& gray_image, const double timestamp) {
  Features current = extract(gray_image);
  MotionEstimate motion;

  if (previous_features_.has_value()) {
    motion = estimateMotion(*previous_features_, current);
    if (motion.success) {
      T_world_camera_ = T_world_camera_ * motion.T_previous_current.inverse();
    }
  } else {
    motion.success = true;
    motion.message = "initialized";
  }

  previous_features_ = std::move(current);

  FrameEstimate estimate;
  estimate.frame_index = frame_index_++;
  estimate.motion = motion;
  estimate.pose.timestamp = timestamp;
  estimate.pose.T_world_camera = T_world_camera_;
  return estimate;
}

VisualOdometry::Features VisualOdometry::extract(const cv::Mat& gray_image) const {
  Features features;
  detector_->detectAndCompute(gray_image, cv::noArray(), features.keypoints, features.descriptors);
  return features;
}

MotionEstimate VisualOdometry::estimateMotion(const Features& previous,
                                              const Features& current) const {
  MotionEstimate motion;

  if (previous.descriptors.empty() || current.descriptors.empty()) {
    motion.message = "missing descriptors";
    return motion;
  }

  const int norm = options_.feature_type == FeatureType::kOrb ? cv::NORM_HAMMING : cv::NORM_L2;
  cv::BFMatcher matcher(norm);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher.knnMatch(previous.descriptors, current.descriptors, knn_matches, 2);

  std::vector<cv::DMatch> matches;
  matches.reserve(knn_matches.size());
  for (const auto& pair : knn_matches) {
    if (pair.size() < 2) {
      continue;
    }
    if (pair[0].distance < static_cast<float>(options_.ratio_test) * pair[1].distance) {
      matches.push_back(pair[0]);
    }
  }

  motion.match_count = static_cast<int>(matches.size());
  if (motion.match_count < options_.min_matches) {
    motion.message = "not enough matches";
    return motion;
  }

  std::vector<cv::Point2f> points_previous;
  std::vector<cv::Point2f> points_current;
  points_previous.reserve(matches.size());
  points_current.reserve(matches.size());
  for (const auto& match : matches) {
    points_previous.push_back(previous.keypoints[static_cast<std::size_t>(match.queryIdx)].pt);
    points_current.push_back(current.keypoints[static_cast<std::size_t>(match.trainIdx)].pt);
  }

  cv::Mat inlier_mask;
  const cv::Mat essential =
      cv::findEssentialMat(points_previous, points_current, cameraMatrix(intrinsics_), cv::RANSAC,
                           options_.ransac_confidence, options_.ransac_threshold_px, inlier_mask);
  if (essential.empty()) {
    motion.message = "essential matrix estimation failed";
    return motion;
  }

  cv::Mat rotation;
  cv::Mat translation;
  motion.inlier_count =
      cv::recoverPose(essential, points_previous, points_current, cameraMatrix(intrinsics_),
                      rotation, translation, inlier_mask);
  if (motion.inlier_count < options_.min_inliers) {
    motion.message = "not enough essential matrix inliers";
    return motion;
  }

  motion.success = true;
  motion.T_previous_current = toEigenIsometry(rotation, translation);
  motion.message = "ok";
  return motion;
}

}  // namespace astro::localization
