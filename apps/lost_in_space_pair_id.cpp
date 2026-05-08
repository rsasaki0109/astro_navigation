// First-cut C++ entry point for the lost-in-space pair-angle pipeline. This
// session ships only the binary loader plus a metadata print so the C++ side
// can be wired into CI alongside the Python prototype. Subsequent sessions
// will add candidate_mappings, verify_rotation, and the pyramid + restart
// loop in Eigen, matching the Python output bit-exactly.

#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "astro_localization/localization/pair_index_loader.hpp"

namespace {

void print_usage(const char* prog) {
  std::cerr << "usage: " << prog << " <pair_index.bin>\n"
            << "       Loads the flat binary pair index emitted by\n"
            << "       scripts/build_star_pair_index.py --write-bin and\n"
            << "       prints a summary matching the Python build's JSON.\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  std::filesystem::path index_path = argv[1];
  try {
    const auto index = astro_localization::localization::load_pair_index_bin(index_path);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "{\n"
              << "  \"index_path\": \"" << index_path.string() << "\",\n"
              << "  \"stars\": " << index.star_count() << ",\n"
              << "  \"bins\": " << index.bin_count() << ",\n"
              << "  \"pairs\": " << index.pair_count() << ",\n"
              << "  \"bin_arcsec\": " << index.bin_arcsec << ",\n"
              << "  \"bin_size_rad\": " << index.bin_size_rad << ",\n"
              << "  \"min_edge_deg\": " << index.min_edge_deg << ",\n"
              << "  \"max_edge_deg\": " << index.max_edge_deg << ",\n";

    std::cout << "  \"first_star\": ";
    if (!index.star_ids.empty()) {
      std::cout << "\"" << index.star_ids.front() << "\" (mag "
                << index.magnitudes(0) << "),\n";
    } else {
      std::cout << "null,\n";
    }

    std::cout << "  \"first_bin\": ";
    if (!index.bin_keys.empty()) {
      std::cout << index.bin_keys.front() << ",\n";
    } else {
      std::cout << "null,\n";
    }

    std::cout << "  \"first_pair\": ";
    if (!index.pair_endpoints.empty()) {
      const auto& pair = index.pair_endpoints.front();
      std::cout << "[" << pair[0] << ", " << pair[1] << "]\n";
    } else {
      std::cout << "null\n";
    }
    std::cout << "}\n";
  } catch (const std::exception& ex) {
    std::cerr << "lost_in_space_pair_id: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
