#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>

#include "astro_navigation/core/types.hpp"
#include "astro_navigation/navigation/state.hpp"
#include "astro_navigation/navigation/trn_summary.hpp"

namespace astro::navigation {

struct ManualPositionInput {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double sigma_m{100.0};
};

struct MissionNavigationInput {
  std::string observations_path;
  std::string catalog_path;
  core::CameraIntrinsics intrinsics;
  double timestamp{0.0};
  std::string position_frame_id{"map"};
  std::optional<ManualPositionInput> manual_position;
  std::optional<std::string> trn_summary_path;
  std::optional<double> position_sigma_override_m;
  std::optional<double> localizability_score;
  std::optional<double> route_trn_confidence;
};

struct MissionNavigationResult {
  NavState state;
  std::optional<PositionLockMeasurement> position_lock;
  int trn_matches{0};
  int trn_inliers{0};
};

[[nodiscard]] MissionNavigationResult runMissionNavigation(const MissionNavigationInput& input);

}  // namespace astro::navigation
