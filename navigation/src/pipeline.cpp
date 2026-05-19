#include "astro_navigation/navigation/pipeline.hpp"

#include <stdexcept>

#include "astro_navigation/localization/star_tracker.hpp"

namespace astro::navigation {

MissionNavigationResult runMissionNavigation(const MissionNavigationInput& input) {
  if (input.observations_path.empty() || input.catalog_path.empty()) {
    throw std::invalid_argument("observations_path and catalog_path are required");
  }
  if (!input.intrinsics.valid()) {
    throw std::invalid_argument("camera intrinsics must have positive fx and fy");
  }
  if (input.manual_position && input.trn_summary_path) {
    throw std::invalid_argument("manual position and TRN summary are mutually exclusive");
  }

  const auto observations = localization::loadStarObservationsCsv(input.observations_path);
  const auto catalog = localization::loadStarCatalogCsv(input.catalog_path);
  const auto attitude =
      localization::estimateStarTrackerAttitude(observations, catalog, input.intrinsics);

  MissionNavigationResult result;
  result.state = fromStarTrackerEstimate(attitude, input.timestamp);

  if (input.trn_summary_path) {
    const auto position_lock = loadTrnSummaryPositionLock(*input.trn_summary_path);
    result.position_lock = position_lock;
    result.trn_matches = position_lock.match_count;
    result.trn_inliers = position_lock.inlier_count;
    applyPositionLock(result.state,
                      position_lock.position,
                      input.position_frame_id,
                      input.position_sigma_override_m.value_or(position_lock.sigma_m));
  } else if (input.manual_position) {
    applyPositionLock(result.state,
                      input.manual_position->position,
                      input.position_frame_id,
                      input.position_sigma_override_m.value_or(input.manual_position->sigma_m));
  }

  return result;
}

}  // namespace astro::navigation
