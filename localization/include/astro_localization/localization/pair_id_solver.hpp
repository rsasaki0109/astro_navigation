#pragma once

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "astro_localization/localization/pair_index_loader.hpp"

namespace astro_localization::localization {

// Configuration mirroring the Python identify_stars_with_pair_index.py CLI.
struct LostInSpaceConfig {
  double tolerance_arcsec = 300.0;
  int neighbor_bins = 2;
  double verification_tolerance_arcsec = 600.0;
  double magnitude_prior_arcsec = 15.0;
  int max_observation_triangles = 400;
  int max_candidates_per_observation_triangle = 8;
  int max_verified_hypotheses = 400;
  int pyramid_size = 0;
  int pyramid_restarts = 0;
  double confidence_fraction = 0.5;
  std::uint64_t pyramid_restart_seed = 0;
};

struct LostInSpaceResult {
  // observation_index -> catalog_index
  std::unordered_map<int, int> assignments;
  double best_rms_error_rad = std::numeric_limits<double>::infinity();
  double best_mean_score_rad = std::numeric_limits<double>::infinity();
  int best_attempt_index = -1;
  int attempts_taken = 0;
  std::int64_t triangle_matches = 0;
  std::int64_t candidate_hypotheses = 0;
  std::int64_t verified_hypotheses = 0;
  std::int64_t observation_triangles_evaluated = 0;
  double candidate_generation_seconds = 0.0;
  double pruning_seconds = 0.0;
  double verification_seconds = 0.0;
};

// observation_magnitudes is either empty (legacy: verify_rotation scores by
// catalog magnitude alone) or sized to observations.size() (uses
// |obs_mag - cat_mag| as the magnitude term, sharper discrimination).
LostInSpaceResult identify_lost_in_space(
    const std::vector<Eigen::Vector3d>& observations,
    const std::vector<double>& observation_magnitudes,
    const PairIndex& index,
    const LostInSpaceConfig& config);

}  // namespace astro_localization::localization
