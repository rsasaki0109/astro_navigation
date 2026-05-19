#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "astro_navigation/core/types.hpp"

namespace astro::localization {

struct StarObservation {
  std::string id;
  cv::Point2d pixel;
};

struct StarCatalogEntry {
  std::string id;
  Eigen::Vector3d inertial_direction{Eigen::Vector3d::UnitX()};
};

struct StarTrackerEstimate {
  bool success{false};
  int correspondence_count{0};
  double rms_direction_error_rad{0.0};
  Eigen::Quaterniond q_camera_inertial{Eigen::Quaterniond::Identity()};
  std::string message;
};

std::vector<StarObservation> loadStarObservationsCsv(const std::string& path);
std::vector<StarCatalogEntry> loadStarCatalogCsv(const std::string& path);

StarTrackerEstimate estimateStarTrackerAttitude(const std::vector<StarObservation>& observations,
                                                const std::vector<StarCatalogEntry>& catalog,
                                                const core::CameraIntrinsics& intrinsics);

}  // namespace astro::localization
