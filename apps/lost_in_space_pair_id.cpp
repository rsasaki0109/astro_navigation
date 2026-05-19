// C++ port of scripts/identify_stars_with_pair_index.py: loads the flat
// binary pair index, runs candidate_mappings + verify_rotation through the
// pyramid+restart loop, and writes assignments CSV + metadata JSON matching
// the Python reference on the same fixtures.

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "astro_navigation/localization/pair_id_solver.hpp"
#include "astro_navigation/localization/pair_index_loader.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

double parse_double(const char* value, const std::string& name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
    throw std::invalid_argument("invalid numeric value for " + name + ": " + value);
  }
  return parsed;
}

long parse_long(const char* value, const std::string& name) {
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0') {
    throw std::invalid_argument("invalid integer value for " + name + ": " + value);
  }
  return parsed;
}

struct Args {
  std::filesystem::path observations_path;
  std::filesystem::path index_path;
  std::filesystem::path output_path;
  std::filesystem::path calibration_json_path;
  std::optional<double> fx, fy, cx, cy;
  std::optional<double> k1, k2, p1, p2;
  astro_navigation::localization::LostInSpaceConfig config;
};

// Minimal regex-based extractor for the camera-calibration JSON schema written
// by scripts/generate_star_tracker_observations_from_catalog.py: top-level
// object keys "intrinsics" (fx/fy/cx/cy) and optional "distortion"
// (k1/k2/p1/p2). We do not attempt full JSON parsing — each key only appears
// once in the generator's schema so a single regex per key is sufficient and
// keeps the C++ binary dependency-free.
std::optional<double> extract_json_number(const std::string& text, const std::string& key) {
  std::regex pattern("\"" + key + "\"\\s*:\\s*(-?\\d+(?:\\.\\d*)?(?:[eE][-+]?\\d+)?)");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return std::stod(match[1].str());
  }
  return std::nullopt;
}

void print_usage(const char* prog) {
  std::cerr << "usage: " << prog
            << " --observations <obs.csv> --index <pair.bin>"
               " --output <assignments.csv> --fx <fx> --fy <fy> --cx <cx> --cy <cy>"
               " [--tolerance-arcsec ...] [--neighbor-bins ...]"
               " [--verification-tolerance-arcsec ...] [--magnitude-prior-arcsec ...]"
               " [--max-observation-triangles ...] [--max-candidates-per-observation-triangle ...]"
               " [--max-verified-hypotheses ...] [--pyramid-size ...] [--pyramid-restarts ...]"
               " [--confidence-fraction ...] [--pyramid-restart-seed ...]"
               " [--fov-radius-deg ...]\n";
}

Args parse_args(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    auto require_value = [&](const std::string& option) -> const char* {
      if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + option);
      }
      return argv[++i];
    };
    if (key == "--observations") {
      args.observations_path = require_value(key);
    } else if (key == "--index") {
      args.index_path = require_value(key);
    } else if (key == "--output") {
      args.output_path = require_value(key);
    } else if (key == "--fx") {
      args.fx = parse_double(require_value(key), key);
    } else if (key == "--fy") {
      args.fy = parse_double(require_value(key), key);
    } else if (key == "--cx") {
      args.cx = parse_double(require_value(key), key);
    } else if (key == "--cy") {
      args.cy = parse_double(require_value(key), key);
    } else if (key == "--calibration-json") {
      args.calibration_json_path = require_value(key);
    } else if (key == "--tolerance-arcsec") {
      args.config.tolerance_arcsec = parse_double(require_value(key), key);
    } else if (key == "--neighbor-bins") {
      args.config.neighbor_bins = static_cast<int>(parse_long(require_value(key), key));
    } else if (key == "--verification-tolerance-arcsec") {
      args.config.verification_tolerance_arcsec = parse_double(require_value(key), key);
    } else if (key == "--magnitude-prior-arcsec") {
      args.config.magnitude_prior_arcsec = parse_double(require_value(key), key);
    } else if (key == "--max-observation-triangles") {
      args.config.max_observation_triangles = static_cast<int>(parse_long(require_value(key), key));
    } else if (key == "--max-candidates-per-observation-triangle") {
      args.config.max_candidates_per_observation_triangle =
          static_cast<int>(parse_long(require_value(key), key));
    } else if (key == "--max-verified-hypotheses") {
      args.config.max_verified_hypotheses = static_cast<int>(parse_long(require_value(key), key));
    } else if (key == "--pyramid-size") {
      args.config.pyramid_size = static_cast<int>(parse_long(require_value(key), key));
    } else if (key == "--pyramid-restarts") {
      args.config.pyramid_restarts = static_cast<int>(parse_long(require_value(key), key));
    } else if (key == "--confidence-fraction") {
      args.config.confidence_fraction = parse_double(require_value(key), key);
    } else if (key == "--pyramid-restart-seed") {
      args.config.pyramid_restart_seed =
          static_cast<std::uint64_t>(parse_long(require_value(key), key));
    } else if (key == "--distortion-k1") {
      args.k1 = parse_double(require_value(key), key);
    } else if (key == "--distortion-k2") {
      args.k2 = parse_double(require_value(key), key);
    } else if (key == "--distortion-p1") {
      args.p1 = parse_double(require_value(key), key);
    } else if (key == "--distortion-p2") {
      args.p2 = parse_double(require_value(key), key);
    } else if (key == "--fov-radius-deg") {
      args.config.fov_radius_rad = parse_double(require_value(key), key) * M_PI / 180.0;
    } else if (key == "--help" || key == "-h") {
      print_usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }
  if (args.observations_path.empty() || args.index_path.empty() || args.output_path.empty()) {
    throw std::invalid_argument("--observations, --index and --output are required");
  }

  // Reconcile --calibration-json with the individual flags. Explicit CLI flags always win;
  // JSON values fill in anything left as nullopt. After reconciliation fx/fy/cx/cy must be
  // set; distortion defaults to 0.
  if (!args.calibration_json_path.empty()) {
    std::ifstream stream(args.calibration_json_path);
    if (!stream) {
      throw std::invalid_argument("cannot open --calibration-json: " +
                                  args.calibration_json_path.string());
    }
    std::stringstream buffer;
    buffer << stream.rdbuf();
    const std::string text = buffer.str();
    auto fill = [&](std::optional<double>& slot, const std::string& key) {
      if (!slot.has_value()) {
        if (auto found = extract_json_number(text, key)) slot = *found;
      }
    };
    fill(args.fx, "fx");
    fill(args.fy, "fy");
    fill(args.cx, "cx");
    fill(args.cy, "cy");
    fill(args.k1, "k1");
    fill(args.k2, "k2");
    fill(args.p1, "p1");
    fill(args.p2, "p2");
  }
  if (!args.fx || !args.fy || !args.cx || !args.cy) {
    throw std::invalid_argument(
        "--fx, --fy, --cx, --cy must be supplied (directly or via --calibration-json)");
  }
  return args;
}

// Iterative inverse Brown-Conrady: forward distortion matches Python's
// scripts/identify_stars_with_index._undistort_normalized so both identifiers
// produce the same bearings for the same coefficients.
std::pair<double, double> undistort_normalized(double x_d, double y_d, double k1, double k2,
                                               double p1, double p2, int iterations = 8) {
  double x = x_d;
  double y = y_d;
  for (int n = 0; n < iterations; ++n) {
    const double r2 = x * x + y * y;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2;
    const double x_t = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double y_t = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    x = (x_d - x_t) / radial;
    y = (y_d - y_t) / radial;
  }
  return {x, y};
}

struct LoadedObservations {
  std::vector<Eigen::Vector3d> bearings;
  std::vector<double> magnitudes;  // empty when input has no `mag` column
};

LoadedObservations load_observations(const std::filesystem::path& path, double fx, double fy,
                                     double cx, double cy, double k1 = 0.0, double k2 = 0.0,
                                     double p1 = 0.0, double p2 = 0.0) {
  const bool distortion_active = (k1 != 0.0) || (k2 != 0.0) || (p1 != 0.0) || (p2 != 0.0);
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open observations file: " + path.string());
  }

  auto strip_cr = [](std::string& s) {
    if (!s.empty() && s.back() == '\r') s.pop_back();
  };
  auto split_csv = [&strip_cr](const std::string& line) {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      strip_cr(cell);
      cells.push_back(cell);
    }
    return cells;
  };

  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("observations file is empty: " + path.string());
  }
  strip_cr(line);
  // Header parse — find the column indices of "u", "v", and optional "mag".
  std::vector<std::string> header_columns = split_csv(line);
  int u_col = -1, v_col = -1, mag_col = -1;
  for (int i = 0; i < static_cast<int>(header_columns.size()); ++i) {
    if (header_columns[i] == "u")
      u_col = i;
    else if (header_columns[i] == "v")
      v_col = i;
    else if (header_columns[i] == "mag")
      mag_col = i;
  }
  if (u_col < 0 || v_col < 0) {
    throw std::runtime_error("observations CSV must contain 'u' and 'v' columns");
  }

  LoadedObservations loaded;
  const int max_required_col = std::max({u_col, v_col, mag_col});
  while (std::getline(stream, line)) {
    strip_cr(line);
    if (line.empty()) continue;
    std::vector<std::string> cells = split_csv(line);
    if (static_cast<int>(cells.size()) <= max_required_col) {
      throw std::runtime_error("observations CSV row has too few columns: " + line);
    }
    const double u = std::strtod(cells[u_col].c_str(), nullptr);
    const double v = std::strtod(cells[v_col].c_str(), nullptr);
    double x_n = (u - cx) / fx;
    double y_n = (v - cy) / fy;
    if (distortion_active) {
      const auto undistorted = undistort_normalized(x_n, y_n, k1, k2, p1, p2);
      x_n = undistorted.first;
      y_n = undistorted.second;
    }
    Eigen::Vector3d bearing(x_n, y_n, 1.0);
    loaded.bearings.push_back(bearing.normalized());
    if (mag_col >= 0) {
      loaded.magnitudes.push_back(std::strtod(cells[mag_col].c_str(), nullptr));
    }
  }
  return loaded;
}

void write_assignments_csv(const std::filesystem::path& path,
                           const std::map<int, std::string>& assignments) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  // Python's csv.writer defaults to '\r\n' line termination; emit the same so
  // the assignments CSV is byte-exact against the reference fixture.
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open output: " + path.string());
  }
  stream << "observation_index,id\r\n";
  for (const auto& [obs_index, star_id] : assignments) {
    stream << obs_index << ',' << star_id << "\r\n";
  }
}

std::string format_double(double value) {
  std::ostringstream ss;
  ss.precision(17);
  ss << value;
  return ss.str();
}

std::string optional_arcsec(double rad) {
  if (!std::isfinite(rad)) return "null";
  const double arcsec = rad * (180.0 / kPi) * 3600.0;
  return format_double(arcsec);
}

void write_metadata_json(const std::filesystem::path& csv_path,
                         const astro_navigation::localization::LostInSpaceResult& result,
                         const astro_navigation::localization::PairIndex& index, const Args& args,
                         std::size_t observation_count) {
  std::filesystem::path json_path = csv_path;
  json_path.replace_extension(".json");

  std::ostringstream out;
  out << "{\n";
  out << "  \"assigned_observations\": " << result.assignments.size() << ",\n";
  out << "  \"triangle_matches\": " << result.triangle_matches << ",\n";
  out << "  \"observations\": " << observation_count << ",\n";
  out << "  \"observation_triangles_evaluated\": " << result.observation_triangles_evaluated
      << ",\n";
  out << "  \"pyramid_size\": " << args.config.pyramid_size << ",\n";
  out << "  \"pyramid_restarts\": " << args.config.pyramid_restarts << ",\n";
  out << "  \"confidence_fraction\": " << format_double(args.config.confidence_fraction) << ",\n";
  out << "  \"attempts_taken\": " << result.attempts_taken << ",\n";
  out << "  \"winning_attempt_index\": " << result.best_attempt_index << ",\n";
  out << "  \"index_stars\": " << index.star_count() << ",\n";
  out << "  \"index_pairs\": " << index.pair_count() << ",\n";
  out << "  \"candidate_hypotheses\": " << result.candidate_hypotheses << ",\n";
  out << "  \"pruned_hypotheses\": " << result.triangle_matches << ",\n";
  out << "  \"tolerance_arcsec\": " << format_double(args.config.tolerance_arcsec) << ",\n";
  out << "  \"neighbor_bins\": " << args.config.neighbor_bins << ",\n";
  out << "  \"verification_tolerance_arcsec\": "
      << format_double(args.config.verification_tolerance_arcsec) << ",\n";
  out << "  \"magnitude_prior_arcsec\": " << format_double(args.config.magnitude_prior_arcsec)
      << ",\n";
  if (std::isfinite(args.config.fov_radius_rad)) {
    out << "  \"fov_radius_deg\": " << format_double(args.config.fov_radius_rad * 180.0 / M_PI)
        << ",\n";
  } else {
    out << "  \"fov_radius_deg\": null,\n";
  }
  out << "  \"max_candidates_per_observation_triangle\": "
      << args.config.max_candidates_per_observation_triangle << ",\n";
  out << "  \"max_verified_hypotheses\": " << args.config.max_verified_hypotheses << ",\n";
  out << "  \"verified_hypotheses\": " << result.verified_hypotheses << ",\n";
  out << "  \"best_rms_error_arcsec\": " << optional_arcsec(result.best_rms_error_rad) << ",\n";
  out << "  \"best_mean_score_arcsec\": " << optional_arcsec(result.best_mean_score_rad) << ",\n";
  out << "  \"candidate_generation_seconds\": "
      << format_double(result.candidate_generation_seconds) << ",\n";
  out << "  \"pruning_seconds\": " << format_double(result.pruning_seconds) << ",\n";
  out << "  \"verification_seconds\": " << format_double(result.verification_seconds) << ",\n";
  out << "  \"index_format\": \"bin\"\n";
  out << "}\n";

  std::ofstream json_stream(json_path);
  if (!json_stream) {
    throw std::runtime_error("cannot open metadata json: " + json_path.string());
  }
  json_stream << out.str();
  std::cout << out.str();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = parse_args(argc, argv);
    const auto index = astro_navigation::localization::load_pair_index_bin(args.index_path);
    const auto loaded = load_observations(args.observations_path, *args.fx, *args.fy, *args.cx,
                                          *args.cy, args.k1.value_or(0.0), args.k2.value_or(0.0),
                                          args.p1.value_or(0.0), args.p2.value_or(0.0));

    const auto result = astro_navigation::localization::identify_lost_in_space(
        loaded.bearings, loaded.magnitudes, index, args.config);

    std::map<int, std::string> assignments_by_id;
    for (const auto& [obs_index, cat_index] : result.assignments) {
      assignments_by_id.emplace(obs_index, index.star_ids[static_cast<std::size_t>(cat_index)]);
    }
    write_assignments_csv(args.output_path, assignments_by_id);
    write_metadata_json(args.output_path, result, index, args, loaded.bearings.size());
  } catch (const std::exception& ex) {
    std::cerr << "lost_in_space_pair_id: " << ex.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
