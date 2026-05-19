#include "astro_navigation/navigation/trn_summary.hpp"

#include <cmath>
#include <algorithm>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace astro::navigation {
namespace {

std::string readText(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open TRN summary: " + path.string());
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::optional<std::string> extractArrayBody(const std::string& text, const std::string& key) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*\\[([^\\]]+)\\]");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return std::nullopt;
  }
  return match[1].str();
}

std::optional<double> extractNumber(const std::string& text, const std::string& key) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?\\d+(?:\\.\\d*)?(?:[eE][-+]?\\d+)?)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return std::nullopt;
  }
  return std::stod(match[1].str());
}

std::optional<int> extractInteger(const std::string& text, const std::string& key) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?\\d+)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return std::nullopt;
  }
  return std::stoi(match[1].str());
}

std::vector<double> parseNumberArray(const std::string& body) {
  std::vector<double> values;
  std::stringstream stream(body);
  std::string field;
  while (std::getline(stream, field, ',')) {
    values.push_back(std::stod(field));
  }
  return values;
}

Eigen::Vector3d extractPosition(const std::string& text) {
  std::optional<std::string> body = extractArrayBody(text, "rover_estimated_xyz_m");
  if (!body) {
    body = extractArrayBody(text, "estimated_xyz_m");
  }
  if (!body) {
    throw std::runtime_error("TRN summary does not contain rover_estimated_xyz_m");
  }

  const std::vector<double> values = parseNumberArray(*body);
  if (values.size() != 3) {
    throw std::runtime_error("TRN position must contain exactly three values");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

TrnQualityTerms estimateQualityTerms(const std::string& text, const int inlier_count) {
  TrnQualityTerms terms;
  const auto px_to_m = extractNumber(text, "px_to_m");
  if (!px_to_m || !std::isfinite(*px_to_m) || *px_to_m <= 0.0) {
    terms.map_resolution_sigma_m = 100.0;
    return terms;
  }

  terms.map_resolution_sigma_m = 2.0 * *px_to_m;
  if (const auto median_reproj_px = extractNumber(text, "inlier_median_reproj_px");
      median_reproj_px && std::isfinite(*median_reproj_px) && *median_reproj_px > 0.0) {
    terms.reprojection_sigma_m = *median_reproj_px * *px_to_m;
  }

  const int effective_inliers = std::max(inlier_count, 1);
  terms.inlier_geometry_sigma_m =
      *px_to_m * std::sqrt(12.0 / static_cast<double>(effective_inliers));
  return terms;
}

double estimatePositionSigmaM(const TrnQualityTerms& terms) {
  return std::max({terms.map_resolution_sigma_m,
                   terms.reprojection_sigma_m,
                   terms.inlier_geometry_sigma_m});
}

}  // namespace

PositionLockMeasurement loadTrnSummaryPositionLock(const std::filesystem::path& summary_path) {
  const std::string text = readText(summary_path);

  PositionLockMeasurement measurement;
  measurement.position = extractPosition(text);
  measurement.match_count = extractInteger(text, "match_count").value_or(0);
  measurement.inlier_count = extractInteger(text, "pnp_inliers").value_or(0);
  measurement.quality_terms = estimateQualityTerms(text, measurement.inlier_count);
  measurement.sigma_m = estimatePositionSigmaM(measurement.quality_terms);
  measurement.evaluation_error_m = extractNumber(text, "position_error_m").value_or(0.0);
  measurement.source = summary_path.string();
  return measurement;
}

}  // namespace astro::navigation
