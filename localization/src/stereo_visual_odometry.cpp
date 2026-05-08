#include "astro_localization/localization/stereo_visual_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace astro::localization {
namespace {

cv::Matx33d cameraMatrix(const core::CameraIntrinsics& intrinsics) {
  return cv::Matx33d(intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy, intrinsics.cy, 0.0,
                     0.0, 1.0);
}

Eigen::Isometry3d toPreviousCurrent(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);

  Eigen::Isometry3d T_current_previous = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      T_current_previous.linear()(row, col) = rotation.at<double>(row, col);
    }
    T_current_previous.translation()(row) = tvec.at<double>(row);
  }
  return T_current_previous.inverse();
}

std::vector<cv::DMatch> ratioMatches(const cv::Mat& query_descriptors, const cv::Mat& train_descriptors,
                                     const double ratio_test) {
  if (query_descriptors.empty() || train_descriptors.empty()) {
    return {};
  }

  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher.knnMatch(query_descriptors, train_descriptors, knn_matches, 2);

  std::vector<cv::DMatch> matches;
  matches.reserve(knn_matches.size());
  for (const auto& pair : knn_matches) {
    if (pair.size() < 2) {
      continue;
    }
    if (pair[0].distance < static_cast<float>(ratio_test) * pair[1].distance) {
      matches.push_back(pair[0]);
    }
  }
  return matches;
}

}  // namespace

StereoVisualOdometry::StereoVisualOdometry(StereoCameraModel camera,
                                           StereoVisualOdometryOptions options)
    : camera_(std::move(camera)), options_(options), detector_(cv::ORB::create(options.max_features)) {
  if (!camera_.left.valid() || !camera_.right.valid()) {
    throw std::invalid_argument("left and right camera intrinsics must be valid");
  }
}

StereoFrameEstimate StereoVisualOdometry::process(const cv::Mat& left_gray, const cv::Mat& right_gray,
                                                  const double timestamp) {
  StereoFrame current = buildStereoFrame(left_gray, right_gray);
  StereoMotionEstimate motion;

  if (previous_frame_.has_value()) {
    motion = estimateMotion(*previous_frame_, current);
    if (motion.success) {
      T_world_camera_ = T_world_camera_ * motion.T_previous_current;
    }
  } else {
    motion.success = true;
    motion.stereo_match_count = current.stereo_match_count;
    motion.valid_3d_point_count = current.valid_3d_point_count;
    motion.message = "initialized";
  }

  previous_frame_ = std::move(current);

  StereoFrameEstimate estimate;
  estimate.frame_index = frame_index_++;
  estimate.motion = motion;
  estimate.pose.timestamp = timestamp;
  estimate.pose.T_world_camera = T_world_camera_;
  return estimate;
}

StereoVisualOdometry::Features StereoVisualOdometry::extract(const cv::Mat& gray_image) const {
  Features features;
  detector_->detectAndCompute(gray_image, cv::noArray(), features.keypoints, features.descriptors);
  return features;
}

StereoVisualOdometry::StereoFrame StereoVisualOdometry::buildStereoFrame(const cv::Mat& left_gray,
                                                                         const cv::Mat& right_gray) const {
  StereoFrame frame;
  frame.left = extract(left_gray);
  const Features right = extract(right_gray);
  frame.left_points_m.resize(frame.left.keypoints.size());

  std::vector<cv::DMatch> stereo_matches =
      ratioMatches(frame.left.descriptors, right.descriptors, options_.ratio_test);
  std::erase_if(stereo_matches, [&](const cv::DMatch& match) {
    const cv::Point2f& left_point = frame.left.keypoints[static_cast<std::size_t>(match.queryIdx)].pt;
    const cv::Point2f& right_point = right.keypoints[static_cast<std::size_t>(match.trainIdx)].pt;
    const float disparity = left_point.x - right_point.x;
    return std::abs(left_point.y - right_point.y) > options_.max_stereo_y_diff_px ||
           disparity < options_.min_disparity_px;
  });
  frame.stereo_match_count = static_cast<int>(stereo_matches.size());
  if (frame.stereo_match_count < options_.min_stereo_matches) {
    return frame;
  }

  cv::Matx<double, 3, 4> P_left = cv::Matx<double, 3, 4>::zeros();
  P_left(0, 0) = camera_.left.fx;
  P_left(0, 2) = camera_.left.cx;
  P_left(1, 1) = camera_.left.fy;
  P_left(1, 2) = camera_.left.cy;
  P_left(2, 2) = 1.0;

  cv::Matx<double, 3, 4> extrinsic_right_left;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      extrinsic_right_left(row, col) = camera_.R_right_left(row, col);
    }
    extrinsic_right_left(row, 3) = camera_.t_right_left(row);
  }
  const cv::Matx<double, 3, 4> P_right = cameraMatrix(camera_.right) * extrinsic_right_left;

  std::vector<cv::Point2d> left_points;
  std::vector<cv::Point2d> right_points;
  left_points.reserve(stereo_matches.size());
  right_points.reserve(stereo_matches.size());
  for (const auto& match : stereo_matches) {
    const cv::Point2f left_point = frame.left.keypoints[static_cast<std::size_t>(match.queryIdx)].pt;
    const cv::Point2f right_point = right.keypoints[static_cast<std::size_t>(match.trainIdx)].pt;
    left_points.emplace_back(left_point.x, left_point.y);
    right_points.emplace_back(right_point.x, right_point.y);
  }

  cv::Mat homogeneous;
  cv::triangulatePoints(P_left, P_right, left_points, right_points, homogeneous);
  for (int col = 0; col < homogeneous.cols; ++col) {
    const double w = homogeneous.at<double>(3, col);
    if (std::abs(w) < 1e-12) {
      continue;
    }
    const double x = homogeneous.at<double>(0, col) / w;
    const double y = homogeneous.at<double>(1, col) / w;
    const double z = homogeneous.at<double>(2, col) / w;
    if (z < options_.min_depth_m || z > options_.max_depth_m) {
      continue;
    }
    const int left_index = stereo_matches[static_cast<std::size_t>(col)].queryIdx;
    frame.left_points_m[static_cast<std::size_t>(left_index)] =
        cv::Point3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    ++frame.valid_3d_point_count;
  }
  return frame;
}

StereoMotionEstimate StereoVisualOdometry::estimateMotion(const StereoFrame& previous,
                                                          const StereoFrame& current) const {
  StereoMotionEstimate motion;
  motion.stereo_match_count = current.stereo_match_count;
  motion.valid_3d_point_count = current.valid_3d_point_count;

  const std::vector<cv::DMatch> temporal_matches =
      ratioMatches(previous.left.descriptors, current.left.descriptors, options_.ratio_test);
  motion.temporal_match_count = static_cast<int>(temporal_matches.size());

  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  object_points.reserve(temporal_matches.size());
  image_points.reserve(temporal_matches.size());
  for (const auto& match : temporal_matches) {
    const auto& point = previous.left_points_m[static_cast<std::size_t>(match.queryIdx)];
    if (!point.has_value()) {
      continue;
    }
    object_points.push_back(*point);
    image_points.push_back(current.left.keypoints[static_cast<std::size_t>(match.trainIdx)].pt);
  }

  motion.pnp_point_count = static_cast<int>(object_points.size());
  if (motion.pnp_point_count < options_.min_pnp_points) {
    motion.message = "not enough 3D-2D correspondences";
    return motion;
  }

  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat inliers;
  const bool solved = cv::solvePnPRansac(
      object_points, image_points, cv::Mat(cameraMatrix(camera_.left)), cv::noArray(), rvec, tvec,
      false, 100, static_cast<float>(options_.pnp_reprojection_error_px), options_.pnp_confidence,
      inliers, cv::SOLVEPNP_EPNP);
  if (!solved) {
    motion.message = "pnp failed";
    return motion;
  }

  motion.pnp_inlier_count = inliers.rows;
  if (motion.pnp_inlier_count < options_.min_pnp_inliers) {
    motion.message = "not enough pnp inliers";
    return motion;
  }

  motion.success = true;
  motion.T_previous_current = toPreviousCurrent(rvec, tvec);
  motion.message = "ok";
  return motion;
}

}  // namespace astro::localization
