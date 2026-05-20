#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "astro_navigation/core/types.hpp"
#include "astro_navigation/localization/star_tracker.hpp"

namespace astro::navigation {

enum class NavStatus { kUnknown, kOk, kDegraded, kLost, kRelocalizing };

enum class NavStatusReason {
  kNone,
  kNoLocks,
  kAttitudeOnly,
  kPositionOnly,
  kVelocityMissing,
  kHighAttitudeUncertainty,
  kHighPositionUncertainty,
  kRouteRiskHigh
};

struct NavQuality {
  bool attitude_lock{false};
  bool position_lock{false};
  bool velocity_lock{false};
  double attitude_sigma_rad{0.0};
  double position_sigma_m{0.0};
  double localizability_score{1.0};
  double route_trn_confidence{1.0};
  double navigation_risk_score{0.0};
  int attitude_correspondences{0};
};

struct NavState {
  double timestamp{0.0};
  std::string position_frame_id{"world"};
  std::string attitude_reference_frame_id{"inertial"};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_body_reference{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Identity()};
  NavQuality quality;
  NavStatus status{NavStatus::kUnknown};
  NavStatusReason status_reason{NavStatusReason::kNone};
  std::string message{"uninitialized"};
};

[[nodiscard]] std::string toString(NavStatus status);
[[nodiscard]] std::string toString(NavStatusReason reason);
[[nodiscard]] NavStatus classifyState(const NavState& state);
[[nodiscard]] NavStatusReason classifyReason(const NavState& state);

[[nodiscard]] NavState fromPoseEstimate(const core::PoseStamped& pose,
                                        const std::string& position_frame_id,
                                        double position_sigma_m);

[[nodiscard]] NavState fromStarTrackerEstimate(const localization::StarTrackerEstimate& estimate,
                                               double timestamp, double attitude_sigma_scale = 1.0);

void applyPositionLock(NavState& state, const Eigen::Vector3d& position,
                       const std::string& position_frame_id, double position_sigma_m);

void applyNavigationRisk(NavState& state, double localizability_score, double route_trn_confidence);

void refreshStatus(NavState& state);

}  // namespace astro::navigation
