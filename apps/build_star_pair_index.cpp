// C++ port of scripts/build_star_pair_index.py. Reads a converted star catalog
// CSV (id, x, y, z, [mag]), enumerates pairs whose angular separation falls in
// [min_edge_deg, max_edge_deg], bins them by angular distance, and writes the
// flat binary format consumed by localization::load_pair_index_bin().
//
// Output is gated bit-exact against the Python --write-bin emitter on the
// 500-star fixture so the deployment pipeline can drop Python entirely.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr char kMagic[8] = {'A', 'S', 'T', 'R', 'O', 'I', 'D', 'X'};
constexpr std::uint32_t kBinVersion = 1;

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
  std::filesystem::path catalog_path;
  std::filesystem::path output_path;
  long limit = 1000;
  double bin_arcsec = 120.0;
  double min_edge_deg = 0.2;
  double max_edge_deg = 80.0;
};

void print_usage(const char* prog) {
  std::cerr << "usage: " << prog
            << " --catalog <catalog.csv> --output <out.bin>"
               " [--limit N] [--bin-arcsec F] [--min-edge-deg F] [--max-edge-deg F]\n"
               "  --limit 0 disables the row cap (default 1000 matches Python).\n";
}

Args parse_args(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    auto require_value = [&](const std::string& option) -> const char* {
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + option);
      return argv[++i];
    };
    if (key == "--catalog") {
      args.catalog_path = require_value(key);
    } else if (key == "--output") {
      args.output_path = require_value(key);
    } else if (key == "--limit") {
      args.limit = parse_long(require_value(key), key);
    } else if (key == "--bin-arcsec") {
      args.bin_arcsec = parse_double(require_value(key), key);
    } else if (key == "--min-edge-deg") {
      args.min_edge_deg = parse_double(require_value(key), key);
    } else if (key == "--max-edge-deg") {
      args.max_edge_deg = parse_double(require_value(key), key);
    } else if (key == "--help" || key == "-h") {
      print_usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }
  if (args.catalog_path.empty() || args.output_path.empty()) {
    throw std::invalid_argument("--catalog and --output are required");
  }
  return args;
}

struct Star {
  std::string id;
  Eigen::Vector3d direction;
  double magnitude;
};

void strip_cr(std::string& s) {
  if (!s.empty() && s.back() == '\r') s.pop_back();
}

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    strip_cr(cell);
    cells.push_back(cell);
  }
  return cells;
}

double parse_double_or_default(const std::string& s, double fallback) {
  if (s.empty()) return fallback;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') return fallback;
  return v;
}

std::vector<Star> load_catalog(const std::filesystem::path& path, long limit) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open catalog: " + path.string());
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("catalog is empty: " + path.string());
  }
  strip_cr(line);
  const auto headers = split_csv(line);
  int id_col = -1, x_col = -1, y_col = -1, z_col = -1, mag_col = -1;
  for (int i = 0; i < static_cast<int>(headers.size()); ++i) {
    if (headers[i] == "id") id_col = i;
    else if (headers[i] == "x") x_col = i;
    else if (headers[i] == "y") y_col = i;
    else if (headers[i] == "z") z_col = i;
    else if (headers[i] == "mag") mag_col = i;
  }
  if (id_col < 0 || x_col < 0 || y_col < 0 || z_col < 0) {
    throw std::runtime_error("catalog must contain id,x,y,z columns");
  }

  std::vector<Star> stars;
  while (std::getline(stream, line)) {
    strip_cr(line);
    if (line.empty()) continue;
    const auto cells = split_csv(line);
    const int max_col = std::max({id_col, x_col, y_col, z_col, mag_col});
    if (static_cast<int>(cells.size()) <= max_col) {
      throw std::runtime_error("catalog row has too few columns: " + line);
    }
    Star star;
    star.id = cells[id_col];
    Eigen::Vector3d v(
        std::strtod(cells[x_col].c_str(), nullptr),
        std::strtod(cells[y_col].c_str(), nullptr),
        std::strtod(cells[z_col].c_str(), nullptr));
    // Match Python's `vector / np.linalg.norm(vector)` (no special-case for zero).
    star.direction = v / v.norm();
    if (mag_col >= 0) {
      star.magnitude = parse_double_or_default(cells[mag_col], 0.0);
    } else {
      star.magnitude = 0.0;
    }
    stars.push_back(std::move(star));
    if (limit > 0 && static_cast<long>(stars.size()) >= limit) break;
  }
  return stars;
}

template <typename T>
void write_pod(std::ofstream& stream, const T& value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

void write_pair_index_bin(
    const std::filesystem::path& path,
    const std::vector<Star>& stars,
    const std::vector<std::int32_t>& bin_keys,
    const std::vector<std::int64_t>& bin_offsets,
    const std::vector<std::array<std::int32_t, 2>>& pair_endpoints,
    double bin_arcsec, double bin_size_rad,
    double min_edge_deg, double max_edge_deg) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open output: " + path.string());

  const std::int64_t n_stars = static_cast<std::int64_t>(stars.size());
  const std::int64_t n_bins = static_cast<std::int64_t>(bin_keys.size());
  const std::int64_t n_pairs = static_cast<std::int64_t>(pair_endpoints.size());

  stream.write(kMagic, sizeof(kMagic));
  write_pod<std::uint32_t>(stream, kBinVersion);
  write_pod<std::int64_t>(stream, n_stars);
  write_pod<std::int64_t>(stream, n_bins);
  write_pod<std::int64_t>(stream, n_pairs);
  write_pod<double>(stream, bin_arcsec);
  write_pod<double>(stream, bin_size_rad);
  write_pod<double>(stream, min_edge_deg);
  write_pod<double>(stream, max_edge_deg);
  // Pad to 72-byte header (8 + 4 + 8*3 + 8*4 = 68 → 4 bytes pad).
  const std::uint32_t pad = 0;
  write_pod<std::uint32_t>(stream, pad);

  // vectors: n_stars × 3 × float64
  for (const auto& star : stars) {
    const double xyz[3] = {star.direction.x(), star.direction.y(), star.direction.z()};
    stream.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
  // magnitudes: n_stars × float64
  for (const auto& star : stars) {
    write_pod<double>(stream, star.magnitude);
  }
  // bin_keys: n_bins × int32
  if (n_bins > 0) {
    stream.write(reinterpret_cast<const char*>(bin_keys.data()),
                 static_cast<std::streamsize>(n_bins * sizeof(std::int32_t)));
  }
  // bin_offsets: (n_bins + 1) × int64
  stream.write(reinterpret_cast<const char*>(bin_offsets.data()),
               static_cast<std::streamsize>((n_bins + 1) * sizeof(std::int64_t)));
  // pair_endpoints: n_pairs × 2 × int32
  if (n_pairs > 0) {
    stream.write(reinterpret_cast<const char*>(pair_endpoints.data()),
                 static_cast<std::streamsize>(n_pairs * 2 * sizeof(std::int32_t)));
  }
  // star_ids_blob: int64 size + concatenation of (uint16 LE length, utf-8 bytes)
  std::vector<std::uint8_t> blob;
  for (const auto& star : stars) {
    const auto len = star.id.size();
    if (len > 0xFFFFu) {
      throw std::runtime_error("star id too long for uint16 length prefix: " + star.id);
    }
    const std::uint16_t prefix = static_cast<std::uint16_t>(len);
    blob.push_back(static_cast<std::uint8_t>(prefix & 0xFF));
    blob.push_back(static_cast<std::uint8_t>((prefix >> 8) & 0xFF));
    blob.insert(blob.end(), star.id.begin(), star.id.end());
  }
  const std::int64_t blob_size = static_cast<std::int64_t>(blob.size());
  write_pod<std::int64_t>(stream, blob_size);
  if (blob_size > 0) {
    stream.write(reinterpret_cast<const char*>(blob.data()),
                 static_cast<std::streamsize>(blob_size));
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = parse_args(argc, argv);
    const auto stars = load_catalog(args.catalog_path, args.limit);
    const auto star_count = static_cast<std::int32_t>(stars.size());

    const double bin_size_rad = (args.bin_arcsec / 3600.0) * kPi / 180.0;
    const double min_edge_rad = args.min_edge_deg * kPi / 180.0;
    const double max_edge_rad = args.max_edge_deg * kPi / 180.0;

    // Pair enumeration uses a 2-pass bucket fill so we never materialise the
    // intermediate (lhs, rhs, bin) triple buffer. Pass 1 counts pairs per bin
    // and derives bin_keys + bin_offsets; pass 2 re-walks the same (i, j > i)
    // order and writes each pair into its slot via a per-bin cursor. Within
    // any bin, pairs are written in visitation order (i ascending, j ascending),
    // which matches numpy's argsort(kind="stable") on the prior in-memory path
    // and keeps the .bin output byte-exact against the Python reference.
    auto compute_bin = [&](std::int32_t i, std::int32_t j) -> std::int32_t {
      double dot = stars[i].direction.dot(stars[j].direction);
      if (dot > 1.0) dot = 1.0;
      if (dot < -1.0) dot = -1.0;
      const double edge = std::acos(dot);
      if (edge < min_edge_rad || edge > max_edge_rad) {
        return std::numeric_limits<std::int32_t>::min();
      }
      // np.round on float is half-to-even; std::nearbyint with default
      // FE_TONEAREST is also half-to-even, so the bin assignment matches
      // the Python reference at every sample.
      return static_cast<std::int32_t>(std::nearbyint(edge / bin_size_rad));
    };

    std::unordered_map<std::int32_t, std::int64_t> bin_counts;
    bin_counts.reserve(static_cast<std::size_t>(
        std::max<double>(8.0, max_edge_rad / bin_size_rad)));
    std::int64_t pair_count = 0;
    for (std::int32_t i = 0; i < star_count - 1; ++i) {
      for (std::int32_t j = i + 1; j < star_count; ++j) {
        const auto bin = compute_bin(i, j);
        if (bin == std::numeric_limits<std::int32_t>::min()) continue;
        bin_counts[bin] += 1;
        pair_count += 1;
      }
    }

    std::vector<std::int32_t> bin_keys;
    bin_keys.reserve(bin_counts.size());
    for (const auto& [key, _count] : bin_counts) bin_keys.push_back(key);
    std::sort(bin_keys.begin(), bin_keys.end());

    std::vector<std::int64_t> bin_offsets(bin_keys.size() + 1, 0);
    for (std::size_t k = 0; k < bin_keys.size(); ++k) {
      bin_offsets[k + 1] = bin_offsets[k] + bin_counts[bin_keys[k]];
    }
    if (bin_offsets.back() != pair_count) {
      throw std::runtime_error("pair count mismatch between count pass and offsets");
    }

    std::unordered_map<std::int32_t, std::int32_t> bin_slot;
    bin_slot.reserve(bin_keys.size() * 2);
    for (std::int32_t k = 0; k < static_cast<std::int32_t>(bin_keys.size()); ++k) {
      bin_slot.emplace(bin_keys[static_cast<std::size_t>(k)], k);
    }

    std::vector<std::array<std::int32_t, 2>> pair_endpoints(static_cast<std::size_t>(pair_count));
    std::vector<std::int64_t> cursor(bin_keys.size());
    for (std::size_t k = 0; k < bin_keys.size(); ++k) {
      cursor[k] = bin_offsets[k];
    }

    for (std::int32_t i = 0; i < star_count - 1; ++i) {
      for (std::int32_t j = i + 1; j < star_count; ++j) {
        const auto bin = compute_bin(i, j);
        if (bin == std::numeric_limits<std::int32_t>::min()) continue;
        const auto slot = static_cast<std::size_t>(bin_slot.at(bin));
        pair_endpoints[static_cast<std::size_t>(cursor[slot])] = {i, j};
        cursor[slot] += 1;
      }
    }

    write_pair_index_bin(
        args.output_path, stars, bin_keys, bin_offsets, pair_endpoints,
        args.bin_arcsec, bin_size_rad, args.min_edge_deg, args.max_edge_deg);

    std::cout << "{\n"
              << "  \"catalog_path\": \"" << args.catalog_path.string() << "\",\n"
              << "  \"stars\": " << star_count << ",\n"
              << "  \"pairs\": " << pair_count << ",\n"
              << "  \"bins\": " << bin_keys.size() << ",\n"
              << "  \"bin_arcsec\": " << args.bin_arcsec << ",\n"
              << "  \"min_edge_deg\": " << args.min_edge_deg << ",\n"
              << "  \"max_edge_deg\": " << args.max_edge_deg << ",\n"
              << "  \"bin_path\": \"" << args.output_path.string() << "\"\n"
              << "}\n";
  } catch (const std::exception& ex) {
    std::cerr << "build_star_pair_index: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
