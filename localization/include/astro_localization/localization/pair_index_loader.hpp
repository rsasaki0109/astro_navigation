#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace astro_localization::localization {

// Flat in-memory representation of the pair-angle catalog index emitted by
// scripts/build_star_pair_index.py with --write-bin. Mirrors the layout of
// the .npz arrays one-to-one so identification can run without any zlib
// dependency on the C++ side.
struct PairIndex {
  // Per-star data, length = star_count.
  std::vector<std::string> star_ids;
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> vectors;
  Eigen::VectorXd magnitudes;

  // Pair-edge bin index. bin_offsets has length n_bins + 1; bin i covers
  // pair_endpoints[bin_offsets[i] .. bin_offsets[i+1]).
  std::vector<std::int32_t> bin_keys;
  std::vector<std::int64_t> bin_offsets;
  std::vector<std::array<std::int32_t, 2>> pair_endpoints;

  double bin_arcsec = 0.0;
  double bin_size_rad = 0.0;
  double min_edge_deg = 0.0;
  double max_edge_deg = 0.0;

  std::int64_t star_count() const { return static_cast<std::int64_t>(star_ids.size()); }
  std::int64_t bin_count() const { return static_cast<std::int64_t>(bin_keys.size()); }
  std::int64_t pair_count() const { return static_cast<std::int64_t>(pair_endpoints.size()); }
};

// Read a pair index from the flat binary format. Throws std::runtime_error
// on missing file, magic mismatch, unsupported version, or truncated payload.
PairIndex load_pair_index_bin(const std::filesystem::path& path);

}  // namespace astro_localization::localization
