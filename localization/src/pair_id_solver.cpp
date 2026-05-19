#include "astro_navigation/localization/pair_id_solver.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace astro_navigation::localization {

namespace {

constexpr double kPi = 3.14159265358979323846;

double to_radians_arcsec(double arcsec) { return (arcsec / 3600.0) * kPi / 180.0; }

int edge_bin(double edge, double bin_size_rad) {
  // Match python: int(round(edge / bin_size_rad)) — banker's rounding in py3,
  // but std::lround uses round-half-away-from-zero. For non-degenerate edges
  // the half-tie case is astronomically unlikely; fall back to nearest int.
  return static_cast<int>(std::lround(edge / bin_size_rad));
}

double angular_distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  double dot = a.dot(b);
  if (dot > 1.0) dot = 1.0;
  if (dot < -1.0) dot = -1.0;
  return std::acos(dot);
}

// Returns the slice [begin, end) of pair_endpoints that lives in bin `key`,
// or empty if the bin is absent. bin_keys is sorted ascending so we can binary
// search for the slot.
struct BinSlice {
  const std::array<std::int32_t, 2>* begin = nullptr;
  std::size_t size = 0;
};

BinSlice find_bin(const PairIndex& index, int key) {
  if (index.bin_keys.empty()) {
    return BinSlice{};
  }
  auto it = std::lower_bound(index.bin_keys.begin(), index.bin_keys.end(),
                             static_cast<std::int32_t>(key));
  if (it == index.bin_keys.end() || *it != static_cast<std::int32_t>(key)) {
    return BinSlice{};
  }
  const auto slot = static_cast<std::size_t>(std::distance(index.bin_keys.begin(), it));
  const std::int64_t start = index.bin_offsets[slot];
  const std::int64_t end = index.bin_offsets[slot + 1];
  BinSlice slice;
  slice.begin = index.pair_endpoints.data() + start;
  slice.size = static_cast<std::size_t>(end - start);
  return slice;
}

// Concatenate pair lists across [key - radius, key + radius] into an output
// vector, mirroring Python's load_candidate_pairs (no internal dedup needed —
// each physical pair lives in exactly one bin).
void load_candidate_pairs(const PairIndex& index, int key, int radius,
                          std::vector<std::array<std::int32_t, 2>>& out) {
  out.clear();
  for (int delta = -radius; delta <= radius; ++delta) {
    BinSlice slice = find_bin(index, key + delta);
    if (slice.size == 0) {
      continue;
    }
    out.insert(out.end(), slice.begin, slice.begin + slice.size);
  }
}

// (a, b, c) -> double score; produced in order. The Python pipeline sorts
// candidates by score (stable) and truncates, so emission order only matters
// for tie-breaking. We follow Python's per-side iteration order so ties on
// score are dominated by the first AB-pair × first AC-pair × matching BC.
struct Candidate {
  double score;
  std::int32_t a;
  std::int32_t b;
  std::int32_t c;
  // tie-break order matches Python's (pre-sort) emission order
  std::uint64_t emission_order;
};

struct PairKey {
  std::int32_t b;
  std::int32_t c;
  bool operator==(const PairKey& other) const noexcept { return b == other.b && c == other.c; }
};

struct PairKeyHash {
  std::size_t operator()(const PairKey& k) const noexcept {
    const auto hi = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.b)) << 32;
    const auto lo = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.c));
    return std::hash<std::uint64_t>{}(hi | lo);
  }
};

// Mirrors Python's candidate_mappings: 3-way merge over the doubled AB / AC /
// BC pair lists, edge-error filter, and edge+magnitude scoring. Adjacency maps
// stand in for pandas's left/right merge; an unordered_set on (b, c) plays the
// role of the second merge.
std::vector<Candidate> candidate_mappings(const std::vector<Eigen::Vector3d>& observations,
                                          const PairIndex& index, int neighbor_bins,
                                          double tolerance_rad, double magnitude_prior_rad,
                                          int obs_a, int obs_b, int obs_c,
                                          std::vector<std::array<std::int32_t, 2>>& scratch_ab,
                                          std::vector<std::array<std::int32_t, 2>>& scratch_ac,
                                          std::vector<std::array<std::int32_t, 2>>& scratch_bc) {
  const double edge_ab = angular_distance(observations[obs_a], observations[obs_b]);
  const double edge_ac = angular_distance(observations[obs_a], observations[obs_c]);
  const double edge_bc = angular_distance(observations[obs_b], observations[obs_c]);

  load_candidate_pairs(index, edge_bin(edge_ab, index.bin_size_rad), neighbor_bins, scratch_ab);
  load_candidate_pairs(index, edge_bin(edge_ac, index.bin_size_rad), neighbor_bins, scratch_ac);
  load_candidate_pairs(index, edge_bin(edge_bc, index.bin_size_rad), neighbor_bins, scratch_bc);
  if (scratch_ab.empty() || scratch_ac.empty() || scratch_bc.empty()) {
    return {};
  }

  // Build adjacency for AB and AC over both directions of every pair.
  std::unordered_map<std::int32_t, std::vector<std::int32_t>> ab_adj;
  ab_adj.reserve(scratch_ab.size() * 2);
  for (const auto& p : scratch_ab) {
    ab_adj[p[0]].push_back(p[1]);
    ab_adj[p[1]].push_back(p[0]);
  }

  std::unordered_map<std::int32_t, std::vector<std::int32_t>> ac_adj;
  ac_adj.reserve(scratch_ac.size() * 2);
  for (const auto& p : scratch_ac) {
    ac_adj[p[0]].push_back(p[1]);
    ac_adj[p[1]].push_back(p[0]);
  }

  std::unordered_set<PairKey, PairKeyHash> bc_set;
  bc_set.reserve(scratch_bc.size() * 2);
  for (const auto& p : scratch_bc) {
    bc_set.insert(PairKey{p[0], p[1]});
    bc_set.insert(PairKey{p[1], p[0]});
  }

  std::vector<Candidate> candidates;
  std::uint64_t emission = 0;

  // Iterate the join: for each shared 'a' between AB and AC adjacency,
  // cross-product AB[a] × AC[a] and keep triples whose (b, c) is also a
  // pair edge in BC, with c != a and c != b.
  for (const auto& [a_value, ab_partners] : ab_adj) {
    auto ac_it = ac_adj.find(a_value);
    if (ac_it == ac_adj.end()) {
      continue;
    }
    const auto& ac_partners = ac_it->second;
    const Eigen::Vector3d va = index.vectors.row(a_value);

    for (const std::int32_t b_value : ab_partners) {
      if (b_value == a_value) {
        continue;
      }
      const Eigen::Vector3d vb = index.vectors.row(b_value);
      double dot_ab = va.dot(vb);
      if (dot_ab > 1.0) dot_ab = 1.0;
      if (dot_ab < -1.0) dot_ab = -1.0;
      const double err_ab = std::abs(edge_ab - std::acos(dot_ab));
      if (err_ab > tolerance_rad) {
        continue;
      }

      for (const std::int32_t c_value : ac_partners) {
        if (c_value == a_value || c_value == b_value) {
          continue;
        }
        if (bc_set.find(PairKey{b_value, c_value}) == bc_set.end()) {
          continue;
        }
        const Eigen::Vector3d vc = index.vectors.row(c_value);
        double dot_ac = va.dot(vc);
        double dot_bc = vb.dot(vc);
        if (dot_ac > 1.0) dot_ac = 1.0;
        if (dot_ac < -1.0) dot_ac = -1.0;
        if (dot_bc > 1.0) dot_bc = 1.0;
        if (dot_bc < -1.0) dot_bc = -1.0;
        const double err_ac = std::abs(edge_ac - std::acos(dot_ac));
        if (err_ac > tolerance_rad) {
          continue;
        }
        const double err_bc = std::abs(edge_bc - std::acos(dot_bc));
        if (err_bc > tolerance_rad) {
          continue;
        }

        const double edge_score =
            std::sqrt((err_ab * err_ab + err_ac * err_ac + err_bc * err_bc) / 3.0);
        const double mag_avg =
            (index.magnitudes(a_value) + index.magnitudes(b_value) + index.magnitudes(c_value)) /
            3.0;
        Candidate cand;
        cand.score = edge_score + magnitude_prior_rad * mag_avg;
        cand.a = a_value;
        cand.b = b_value;
        cand.c = c_value;
        cand.emission_order = emission++;
        candidates.push_back(cand);
      }
    }
  }
  return candidates;
}

// Wahba/Kabsch: R = U V^T (with reflection guard) from sum of outer products.
Eigen::Matrix3d estimate_rotation_camera_inertial(const std::vector<Eigen::Vector3d>& observations,
                                                  const PairIndex& index,
                                                  const std::vector<std::pair<int, int>>& pairs) {
  Eigen::Matrix3d correlation = Eigen::Matrix3d::Zero();
  for (const auto& [obs_index, cat_index] : pairs) {
    correlation += observations[obs_index] * index.vectors.row(cat_index);
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(correlation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  const Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d rotation = U * V.transpose();
  if (rotation.determinant() < 0.0) {
    U.col(2) *= -1.0;
    rotation = U * V.transpose();
  }
  return rotation;
}

struct VerifyResult {
  std::unordered_map<int, int> assignments;
  double rms_error = std::numeric_limits<double>::infinity();
  double mean_score = std::numeric_limits<double>::infinity();
};

// Vectorised verify_rotation: predicted = catalog @ R^T, score each
// observation against all predicted directions, collect (score, error,
// obs, cat) candidates and greedy-assign in score-ascending order.
// observation_magnitudes is either empty (legacy: scoring uses cat_mag) or
// sized to observations.size() (scoring uses |obs_mag - cat_mag|).
VerifyResult verify_rotation(const Eigen::Matrix3d& rotation_camera_inertial,
                             const std::vector<Eigen::Vector3d>& observations,
                             const std::vector<double>& observation_magnitudes,
                             const PairIndex& index, double tolerance_rad,
                             double magnitude_prior_rad,
                             double fov_radius_rad = std::numeric_limits<double>::infinity()) {
  const bool use_obs_mag = !observation_magnitudes.empty();
  const double min_dot = std::cos(tolerance_rad);

  // Sky-cell verification cone: when fov_radius_rad is finite, restrict the
  // catalog to stars whose inertial unit vector lies within the cone of half
  // angle fov_radius_rad around the optical axis (R^T * [0,0,1]).
  Eigen::MatrixXd predicted;
  Eigen::VectorXd magnitude_subset;
  std::vector<int> cat_index_map;
  if (std::isfinite(fov_radius_rad) && index.vectors.rows() > 0) {
    const Eigen::Vector3d axis_inertial =
        rotation_camera_inertial.transpose() * Eigen::Vector3d(0.0, 0.0, 1.0);
    const Eigen::VectorXd axis_dot = index.vectors * axis_inertial;
    const double cone_min_dot = std::cos(fov_radius_rad);
    cat_index_map.reserve(static_cast<std::size_t>(axis_dot.size()));
    for (int i = 0; i < axis_dot.size(); ++i) {
      if (axis_dot(i) >= cone_min_dot) cat_index_map.push_back(i);
    }
    if (cat_index_map.empty()) {
      return {};
    }
    Eigen::MatrixXd subset(cat_index_map.size(), 3);
    magnitude_subset.resize(static_cast<Eigen::Index>(cat_index_map.size()));
    for (std::size_t row = 0; row < cat_index_map.size(); ++row) {
      subset.row(static_cast<Eigen::Index>(row)) = index.vectors.row(cat_index_map[row]);
      magnitude_subset(static_cast<Eigen::Index>(row)) = index.magnitudes(cat_index_map[row]);
    }
    predicted = subset * rotation_camera_inertial.transpose();
  } else {
    predicted = index.vectors * rotation_camera_inertial.transpose();
  }

  auto cat_mag_at = [&](int local_index) {
    return cat_index_map.empty() ? index.magnitudes(local_index) : magnitude_subset(local_index);
  };
  auto cat_index_at = [&](int local_index) {
    return cat_index_map.empty() ? local_index
                                 : cat_index_map[static_cast<std::size_t>(local_index)];
  };

  struct Candidate {
    double score;
    double error;
    int obs;
    int cat;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(observations.size()) * 4);

  for (int obs_index = 0; obs_index < static_cast<int>(observations.size()); ++obs_index) {
    const Eigen::VectorXd dots = predicted * observations[obs_index];
    for (int local_cat = 0; local_cat < dots.size(); ++local_cat) {
      double dot = dots(local_cat);
      if (dot < min_dot) {
        continue;
      }
      if (dot > 1.0) dot = 1.0;
      if (dot < -1.0) dot = -1.0;
      const double error = std::acos(dot);
      const double cat_mag = cat_mag_at(local_cat);
      const double mag_term =
          use_obs_mag
              ? std::abs(cat_mag - observation_magnitudes[static_cast<std::size_t>(obs_index)])
              : cat_mag;
      const double score = error + magnitude_prior_rad * mag_term;
      candidates.push_back(Candidate{score, error, obs_index, cat_index_at(local_cat)});
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
    if (lhs.score != rhs.score) return lhs.score < rhs.score;
    if (lhs.error != rhs.error) return lhs.error < rhs.error;
    if (lhs.obs != rhs.obs) return lhs.obs < rhs.obs;
    return lhs.cat < rhs.cat;
  });

  VerifyResult result;
  std::unordered_set<int> used_cat;
  used_cat.reserve(candidates.size());
  std::vector<double> errors;
  errors.reserve(observations.size());
  double score_sum = 0.0;
  for (const auto& cand : candidates) {
    if (result.assignments.count(cand.obs) != 0) {
      continue;
    }
    if (used_cat.count(cand.cat) != 0) {
      continue;
    }
    result.assignments.emplace(cand.obs, cand.cat);
    used_cat.insert(cand.cat);
    errors.push_back(cand.error);
    score_sum += cand.score;
  }
  if (!errors.empty()) {
    double sum_sq = 0.0;
    for (double e : errors) sum_sq += e * e;
    result.rms_error = std::sqrt(sum_sq / static_cast<double>(errors.size()));
    result.mean_score = score_sum / static_cast<double>(errors.size());
  }
  return result;
}

// Match Python's select_observation_triangles: enumerate combinations(N, 3)
// in lexicographic order, then either keep all or pick max_obs_tris items
// uniformly along the combination axis using rounded interpolation.
std::vector<std::array<int, 3>> select_observation_triangles(int observation_count,
                                                             int max_observation_triangles) {
  std::vector<std::array<int, 3>> all;
  if (observation_count < 3) {
    return all;
  }
  all.reserve(static_cast<std::size_t>(observation_count) *
              static_cast<std::size_t>(observation_count - 1) *
              static_cast<std::size_t>(observation_count - 2) / 6);
  for (int i = 0; i < observation_count - 2; ++i) {
    for (int j = i + 1; j < observation_count - 1; ++j) {
      for (int k = j + 1; k < observation_count; ++k) {
        all.push_back({i, j, k});
      }
    }
  }
  if (max_observation_triangles <= 0 || static_cast<int>(all.size()) <= max_observation_triangles) {
    return all;
  }
  if (max_observation_triangles == 1) {
    return {all[all.size() / 2]};
  }
  const long last_index = static_cast<long>(all.size()) - 1;
  std::set<long> picked;
  for (int n = 0; n < max_observation_triangles; ++n) {
    // python's round() on a float — half-to-even; use std::lrint with default
    // rounding mode (matches python for non-tie cases, off by at most 1 on
    // exact half — same caveat as edge_bin).
    const double t = static_cast<double>(n) * static_cast<double>(last_index) /
                     static_cast<double>(max_observation_triangles - 1);
    picked.insert(std::lrint(t));
  }
  std::vector<std::array<int, 3>> selected;
  selected.reserve(picked.size());
  for (long idx : picked) {
    selected.push_back(all[static_cast<std::size_t>(idx)]);
  }
  return selected;
}

// Fisher-Yates shuffle deterministic in C++ but not bit-compatible with
// Python's random.Random — that's fine for restarts because the bit-exact
// fixture targets succeed on attempt 0 (no shuffle invoked).
void shuffle_permutation(std::vector<int>& permutation, std::mt19937_64& rng) {
  for (int i = static_cast<int>(permutation.size()) - 1; i >= 1; --i) {
    std::uniform_int_distribution<int> dist(0, i);
    const int j = dist(rng);
    std::swap(permutation[i], permutation[j]);
  }
}

bool is_close(double a, double b) {
  // python's math.isclose default rel_tol=1e-9, abs_tol=0.0
  const double diff = std::abs(a - b);
  return diff <= std::max(1e-9 * std::max(std::abs(a), std::abs(b)), 0.0);
}

}  // namespace

LostInSpaceResult identify_lost_in_space(const std::vector<Eigen::Vector3d>& observations,
                                         const std::vector<double>& observation_magnitudes,
                                         const PairIndex& index, const LostInSpaceConfig& config) {
  if (index.bin_size_rad <= 0.0) {
    throw std::runtime_error("identify_lost_in_space: bin_size_rad must be positive");
  }
  if (!observation_magnitudes.empty() && observation_magnitudes.size() != observations.size()) {
    throw std::runtime_error(
        "identify_lost_in_space: observation_magnitudes size must match observations or be empty");
  }

  const double tolerance_rad = to_radians_arcsec(config.tolerance_arcsec);
  const double verification_tolerance_rad = to_radians_arcsec(config.verification_tolerance_arcsec);
  const double magnitude_prior_rad = to_radians_arcsec(config.magnitude_prior_arcsec);

  LostInSpaceResult result;
  std::vector<int> permutation(observations.size());
  std::iota(permutation.begin(), permutation.end(), 0);
  std::mt19937_64 restart_rng(config.pyramid_restart_seed);
  const double confidence_target =
      config.confidence_fraction * static_cast<double>(observations.size());

  std::vector<std::array<std::int32_t, 2>> scratch_ab;
  std::vector<std::array<std::int32_t, 2>> scratch_ac;
  std::vector<std::array<std::int32_t, 2>> scratch_bc;

  using Clock = std::chrono::steady_clock;
  auto seconds_since = [](Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
  };

  for (int attempt = 0; attempt <= config.pyramid_restarts; ++attempt) {
    result.attempts_taken = attempt + 1;

    std::vector<int> pool;
    if (config.pyramid_size > 0) {
      const auto take = std::min(static_cast<int>(permutation.size()), config.pyramid_size);
      pool.assign(permutation.begin(), permutation.begin() + take);
    } else {
      pool = permutation;
    }
    const auto base_triangles = select_observation_triangles(static_cast<int>(pool.size()),
                                                             config.max_observation_triangles);

    std::vector<std::array<int, 3>> observation_triangles;
    observation_triangles.reserve(base_triangles.size());
    for (const auto& tri : base_triangles) {
      observation_triangles.push_back({pool[tri[0]], pool[tri[1]], pool[tri[2]]});
    }
    result.observation_triangles_evaluated +=
        static_cast<std::int64_t>(observation_triangles.size());

    struct Hypothesis {
      double score;
      std::array<int, 3> obs;
      std::array<std::int32_t, 3> cat;
      std::uint64_t order;
    };
    std::vector<Hypothesis> hypotheses;

    std::uint64_t global_order = 0;
    for (const auto& obs_indices : observation_triangles) {
      const auto gen_start = Clock::now();
      auto candidates = candidate_mappings(observations, index, config.neighbor_bins, tolerance_rad,
                                           magnitude_prior_rad, obs_indices[0], obs_indices[1],
                                           obs_indices[2], scratch_ab, scratch_ac, scratch_bc);
      result.candidate_hypotheses += static_cast<std::int64_t>(candidates.size());
      std::stable_sort(
          candidates.begin(), candidates.end(),
          [](const Candidate& lhs, const Candidate& rhs) { return lhs.score < rhs.score; });
      if (config.max_candidates_per_observation_triangle > 0 &&
          static_cast<int>(candidates.size()) > config.max_candidates_per_observation_triangle) {
        candidates.resize(static_cast<std::size_t>(config.max_candidates_per_observation_triangle));
      }
      result.candidate_generation_seconds += seconds_since(gen_start);
      for (const auto& cand : candidates) {
        Hypothesis h;
        h.score = cand.score;
        h.obs = obs_indices;
        h.cat = {cand.a, cand.b, cand.c};
        h.order = global_order++;
        hypotheses.push_back(h);
      }
    }

    const auto pruning_start = Clock::now();
    result.triangle_matches += static_cast<std::int64_t>(hypotheses.size());
    std::stable_sort(
        hypotheses.begin(), hypotheses.end(),
        [](const Hypothesis& lhs, const Hypothesis& rhs) { return lhs.score < rhs.score; });
    if (config.max_verified_hypotheses > 0 &&
        static_cast<int>(hypotheses.size()) > config.max_verified_hypotheses) {
      hypotheses.resize(static_cast<std::size_t>(config.max_verified_hypotheses));
    }
    result.pruning_seconds += seconds_since(pruning_start);

    std::unordered_map<int, int> attempt_assignments;
    double attempt_rms_error = std::numeric_limits<double>::infinity();
    double attempt_mean_score = std::numeric_limits<double>::infinity();

    for (const auto& hyp : hypotheses) {
      const auto verify_start = Clock::now();
      std::vector<std::pair<int, int>> pairs{
          {hyp.obs[0], hyp.cat[0]}, {hyp.obs[1], hyp.cat[1]}, {hyp.obs[2], hyp.cat[2]}};
      const Eigen::Matrix3d rotation =
          estimate_rotation_camera_inertial(observations, index, pairs);
      const VerifyResult v =
          verify_rotation(rotation, observations, observation_magnitudes, index,
                          verification_tolerance_rad, magnitude_prior_rad, config.fov_radius_rad);
      result.verification_seconds += seconds_since(verify_start);
      if (!v.assignments.empty()) {
        result.verified_hypotheses += 1;
      }
      const std::size_t cur = v.assignments.size();
      const std::size_t best = attempt_assignments.size();
      bool replace = false;
      if (cur > best) {
        replace = true;
      } else if (cur == best && cur > 0) {
        if (v.mean_score < attempt_mean_score) {
          replace = true;
        } else if (is_close(v.mean_score, attempt_mean_score) && v.rms_error < attempt_rms_error) {
          replace = true;
        }
      }
      if (replace) {
        attempt_assignments = v.assignments;
        attempt_rms_error = v.rms_error;
        attempt_mean_score = v.mean_score;
      }
    }

    const std::size_t cur = attempt_assignments.size();
    const std::size_t best = result.assignments.size();
    bool promote = false;
    if (cur > best) {
      promote = true;
    } else if (cur == best && cur > 0) {
      if (attempt_mean_score < result.best_mean_score_rad) {
        promote = true;
      } else if (is_close(attempt_mean_score, result.best_mean_score_rad) &&
                 attempt_rms_error < result.best_rms_error_rad) {
        promote = true;
      }
    }
    if (promote) {
      result.assignments = std::move(attempt_assignments);
      result.best_rms_error_rad = attempt_rms_error;
      result.best_mean_score_rad = attempt_mean_score;
      result.best_attempt_index = attempt;
    }

    if (static_cast<double>(result.assignments.size()) >= confidence_target) {
      break;
    }
    shuffle_permutation(permutation, restart_rng);
  }

  return result;
}

}  // namespace astro_navigation::localization
