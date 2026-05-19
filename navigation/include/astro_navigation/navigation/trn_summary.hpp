#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <string>

namespace astro::navigation {

struct TrnQualityTerms {
  double map_resolution_sigma_m{0.0};
  double reprojection_sigma_m{0.0};
  double inlier_geometry_sigma_m{0.0};
};

struct PositionLockMeasurement {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double sigma_m{0.0};
  double evaluation_error_m{0.0};
  TrnQualityTerms quality_terms;
  int match_count{0};
  int inlier_count{0};
  std::string source;
};

[[nodiscard]] PositionLockMeasurement loadTrnSummaryPositionLock(
    const std::filesystem::path& summary_path);

}  // namespace astro::navigation
