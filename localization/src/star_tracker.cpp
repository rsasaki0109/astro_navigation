#include "astro_navigation/localization/star_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace astro::localization {
namespace {

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

Eigen::Vector3d bearingFromPixel(const cv::Point2d& pixel, const core::CameraIntrinsics& intrinsics) {
  Eigen::Vector3d bearing((pixel.x - intrinsics.cx) / intrinsics.fx,
                          (pixel.y - intrinsics.cy) / intrinsics.fy, 1.0);
  return bearing.normalized();
}

}  // namespace

std::vector<StarObservation> loadStarObservationsCsv(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open star observations: " + path);
  }

  std::vector<StarObservation> observations;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.starts_with("#") || line.starts_with("id,")) {
      continue;
    }
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() < 3) {
      continue;
    }
    observations.push_back({fields[0], cv::Point2d(std::stod(fields[1]), std::stod(fields[2]))});
  }
  return observations;
}

std::vector<StarCatalogEntry> loadStarCatalogCsv(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open star catalog: " + path);
  }

  std::vector<StarCatalogEntry> catalog;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.starts_with("#") || line.starts_with("id,")) {
      continue;
    }
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() < 4) {
      continue;
    }
    Eigen::Vector3d direction(std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3]));
    if (direction.norm() <= 0.0) {
      continue;
    }
    catalog.push_back({fields[0], direction.normalized()});
  }
  return catalog;
}

StarTrackerEstimate estimateStarTrackerAttitude(const std::vector<StarObservation>& observations,
                                                const std::vector<StarCatalogEntry>& catalog,
                                                const core::CameraIntrinsics& intrinsics) {
  StarTrackerEstimate estimate;
  if (!intrinsics.valid()) {
    estimate.message = "invalid camera intrinsics";
    return estimate;
  }

  std::unordered_map<std::string, Eigen::Vector3d> catalog_by_id;
  for (const auto& entry : catalog) {
    catalog_by_id.emplace(entry.id, entry.inertial_direction.normalized());
  }

  std::vector<Eigen::Vector3d> inertial_vectors;
  std::vector<Eigen::Vector3d> camera_vectors;
  for (const auto& observation : observations) {
    const auto found = catalog_by_id.find(observation.id);
    if (found == catalog_by_id.end()) {
      continue;
    }
    inertial_vectors.push_back(found->second);
    camera_vectors.push_back(bearingFromPixel(observation.pixel, intrinsics));
  }

  estimate.correspondence_count = static_cast<int>(camera_vectors.size());
  if (camera_vectors.size() < 3) {
    estimate.message = "at least three identified stars are required";
    return estimate;
  }

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < camera_vectors.size(); ++i) {
    covariance += camera_vectors[i] * inertial_vectors[i].transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0) {
    correction(2, 2) = -1.0;
  }
  const Eigen::Matrix3d R_camera_inertial =
      svd.matrixU() * correction * svd.matrixV().transpose();

  double squared_error_sum = 0.0;
  for (std::size_t i = 0; i < camera_vectors.size(); ++i) {
    const Eigen::Vector3d predicted = R_camera_inertial * inertial_vectors[i];
    const double cosine = std::clamp(predicted.dot(camera_vectors[i]), -1.0, 1.0);
    const double error = std::acos(cosine);
    squared_error_sum += error * error;
  }

  estimate.success = true;
  estimate.q_camera_inertial = Eigen::Quaterniond(R_camera_inertial).normalized();
  estimate.rms_direction_error_rad =
      std::sqrt(squared_error_sum / static_cast<double>(camera_vectors.size()));
  estimate.message = "ok";
  return estimate;
}

}  // namespace astro::localization
