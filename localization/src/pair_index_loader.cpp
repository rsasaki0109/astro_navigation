#include "astro_navigation/localization/pair_index_loader.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace astro_navigation::localization {

namespace {

constexpr char kMagic[8] = {'A', 'S', 'T', 'R', 'O', 'I', 'D', 'X'};
constexpr std::uint32_t kSupportedVersion = 1;
constexpr std::size_t kHeaderBytes = 72;

template <typename T>
void read_into(std::ifstream& stream, T& target) {
  stream.read(reinterpret_cast<char*>(&target), sizeof(T));
  if (!stream) {
    throw std::runtime_error("pair_index: unexpected EOF while reading scalar");
  }
}

void read_bytes(std::ifstream& stream, void* destination, std::size_t bytes) {
  if (bytes == 0) {
    return;
  }
  stream.read(reinterpret_cast<char*>(destination), static_cast<std::streamsize>(bytes));
  if (!stream || static_cast<std::size_t>(stream.gcount()) != bytes) {
    throw std::runtime_error("pair_index: unexpected EOF while reading payload");
  }
}

}  // namespace

PairIndex load_pair_index_bin(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("pair_index: cannot open " + path.string());
  }

  std::array<char, kHeaderBytes> header_bytes{};
  read_bytes(stream, header_bytes.data(), header_bytes.size());

  if (std::memcmp(header_bytes.data(), kMagic, sizeof(kMagic)) != 0) {
    throw std::runtime_error("pair_index: magic mismatch in " + path.string());
  }

  std::uint32_t version = 0;
  std::memcpy(&version, header_bytes.data() + 8, sizeof(version));
  if (version != kSupportedVersion) {
    throw std::runtime_error("pair_index: unsupported version " + std::to_string(version));
  }

  std::int64_t n_stars = 0;
  std::int64_t n_bins = 0;
  std::int64_t n_pairs = 0;
  std::memcpy(&n_stars, header_bytes.data() + 12, sizeof(n_stars));
  std::memcpy(&n_bins, header_bytes.data() + 20, sizeof(n_bins));
  std::memcpy(&n_pairs, header_bytes.data() + 28, sizeof(n_pairs));
  if (n_stars < 0 || n_bins < 0 || n_pairs < 0) {
    throw std::runtime_error("pair_index: negative count in header");
  }

  PairIndex index;
  std::memcpy(&index.bin_arcsec, header_bytes.data() + 36, sizeof(double));
  std::memcpy(&index.bin_size_rad, header_bytes.data() + 44, sizeof(double));
  std::memcpy(&index.min_edge_deg, header_bytes.data() + 52, sizeof(double));
  std::memcpy(&index.max_edge_deg, header_bytes.data() + 60, sizeof(double));

  index.vectors.resize(n_stars, 3);
  read_bytes(stream, index.vectors.data(), static_cast<std::size_t>(n_stars) * 3 * sizeof(double));

  index.magnitudes.resize(n_stars);
  read_bytes(stream, index.magnitudes.data(), static_cast<std::size_t>(n_stars) * sizeof(double));

  index.bin_keys.resize(static_cast<std::size_t>(n_bins));
  read_bytes(stream, index.bin_keys.data(), static_cast<std::size_t>(n_bins) * sizeof(std::int32_t));

  index.bin_offsets.resize(static_cast<std::size_t>(n_bins) + 1);
  read_bytes(stream, index.bin_offsets.data(),
             (static_cast<std::size_t>(n_bins) + 1) * sizeof(std::int64_t));

  index.pair_endpoints.resize(static_cast<std::size_t>(n_pairs));
  read_bytes(stream, index.pair_endpoints.data(),
             static_cast<std::size_t>(n_pairs) * 2 * sizeof(std::int32_t));

  std::int64_t blob_size = 0;
  read_into(stream, blob_size);
  if (blob_size < 0) {
    throw std::runtime_error("pair_index: negative star_ids_blob size");
  }

  std::vector<std::uint8_t> blob(static_cast<std::size_t>(blob_size));
  read_bytes(stream, blob.data(), blob.size());

  index.star_ids.reserve(static_cast<std::size_t>(n_stars));
  std::size_t cursor = 0;
  for (std::int64_t i = 0; i < n_stars; ++i) {
    if (cursor + 2 > blob.size()) {
      throw std::runtime_error("pair_index: truncated star id length prefix");
    }
    std::uint16_t length = 0;
    std::memcpy(&length, blob.data() + cursor, sizeof(length));
    cursor += 2;
    if (cursor + length > blob.size()) {
      throw std::runtime_error("pair_index: truncated star id payload");
    }
    index.star_ids.emplace_back(reinterpret_cast<const char*>(blob.data() + cursor),
                                static_cast<std::size_t>(length));
    cursor += length;
  }
  if (cursor != blob.size()) {
    throw std::runtime_error("pair_index: trailing bytes in star_ids_blob");
  }

  return index;
}

}  // namespace astro_navigation::localization
