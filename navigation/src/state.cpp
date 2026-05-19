#include "astro_navigation/navigation/state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astro::navigation {
namespace {

constexpr double kDefaultPositionVariance = 1.0e12;
constexpr double kDefaultAttitudeVariance = 1.0e6;

double squaredOrFallback(const double value, const double fallback) {
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return value * value;
}

void resetCovariance(NavState& state) {
  state.covariance.setZero();
  const double position_variance =
      squaredOrFallback(state.quality.position_sigma_m, kDefaultPositionVariance);
  const double attitude_variance =
      squaredOrFallback(state.quality.attitude_sigma_rad, kDefaultAttitudeVariance);
  state.covariance.block<3, 3>(0, 0).diagonal().setConstant(position_variance);
  state.covariance.block<3, 3>(3, 3).diagonal().setConstant(attitude_variance);
}

}  // namespace

std::string toString(const NavStatus status) {
  switch (status) {
    case NavStatus::kUnknown:
      return "UNKNOWN";
    case NavStatus::kOk:
      return "OK";
    case NavStatus::kDegraded:
      return "DEGRADED";
    case NavStatus::kLost:
      return "LOST";
    case NavStatus::kRelocalizing:
      return "RELOCALIZING";
  }
  return "UNKNOWN";
}

std::string toString(const NavStatusReason reason) {
  switch (reason) {
    case NavStatusReason::kNone:
      return "NONE";
    case NavStatusReason::kNoLocks:
      return "NO_LOCKS";
    case NavStatusReason::kAttitudeOnly:
      return "ATTITUDE_ONLY";
    case NavStatusReason::kPositionOnly:
      return "POSITION_ONLY";
    case NavStatusReason::kVelocityMissing:
      return "VELOCITY_MISSING";
    case NavStatusReason::kHighAttitudeUncertainty:
      return "HIGH_ATTITUDE_UNCERTAINTY";
    case NavStatusReason::kHighPositionUncertainty:
      return "HIGH_POSITION_UNCERTAINTY";
  }
  return "NONE";
}

NavStatus classifyState(const NavState& state) {
  const bool has_any_lock = state.quality.attitude_lock || state.quality.position_lock;
  if (!has_any_lock) {
    return NavStatus::kLost;
  }
  if (state.quality.attitude_lock && state.quality.position_lock) {
    return NavStatus::kOk;
  }
  return NavStatus::kDegraded;
}

NavStatusReason classifyReason(const NavState& state) {
  if (!state.quality.attitude_lock && !state.quality.position_lock) {
    return NavStatusReason::kNoLocks;
  }
  if (state.quality.attitude_lock && !state.quality.position_lock) {
    return NavStatusReason::kAttitudeOnly;
  }
  if (!state.quality.attitude_lock && state.quality.position_lock) {
    return NavStatusReason::kPositionOnly;
  }
  return NavStatusReason::kNone;
}

NavState fromPoseEstimate(const core::PoseStamped& pose, const std::string& position_frame_id,
                          const double position_sigma_m) {
  NavState state;
  state.timestamp = pose.timestamp;
  state.position_frame_id = position_frame_id;
  state.position = pose.T_world_camera.translation();
  state.q_body_reference = Eigen::Quaterniond(pose.T_world_camera.linear()).normalized();
  state.quality.position_lock = true;
  state.quality.attitude_lock = true;
  state.quality.position_sigma_m = position_sigma_m;
  state.quality.attitude_sigma_rad = 0.0;
  state.message = "pose lock";
  refreshStatus(state);
  return state;
}

NavState fromStarTrackerEstimate(const localization::StarTrackerEstimate& estimate,
                                 const double timestamp, const double attitude_sigma_scale) {
  NavState state;
  state.timestamp = timestamp;
  state.quality.attitude_lock = estimate.success;
  state.quality.attitude_correspondences = estimate.correspondence_count;
  state.quality.attitude_sigma_rad =
      std::max(0.0, estimate.rms_direction_error_rad * attitude_sigma_scale);
  if (estimate.success) {
    state.q_body_reference = estimate.q_camera_inertial.normalized();
    state.message = "attitude lock";
  } else {
    state.message = estimate.message.empty() ? "attitude lost" : estimate.message;
  }
  refreshStatus(state);
  return state;
}

void applyPositionLock(NavState& state, const Eigen::Vector3d& position,
                       const std::string& position_frame_id, const double position_sigma_m) {
  state.position = position;
  state.position_frame_id = position_frame_id;
  state.quality.position_lock = true;
  state.quality.position_sigma_m = position_sigma_m;
  refreshStatus(state);
}

void refreshStatus(NavState& state) {
  resetCovariance(state);
  state.status = classifyState(state);
  state.status_reason = classifyReason(state);
  if (state.status == NavStatus::kOk) {
    state.message = "navigation lock";
  } else if (state.status == NavStatus::kDegraded && state.message.empty()) {
    state.message = "partial navigation lock";
  } else if (state.status == NavStatus::kLost && state.message.empty()) {
    state.message = "navigation lost";
  }
}

}  // namespace astro::navigation
