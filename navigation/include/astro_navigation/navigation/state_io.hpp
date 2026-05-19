#pragma once

#include <filesystem>

#include "astro_navigation/navigation/state.hpp"

namespace astro::navigation {

void writeNavStateJson(const std::filesystem::path& output_path, const NavState& state);
void writeNavStateCsv(const std::filesystem::path& output_path, const NavState& state);

}  // namespace astro::navigation
