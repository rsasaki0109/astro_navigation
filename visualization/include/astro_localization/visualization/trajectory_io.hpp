#pragma once

#include <filesystem>
#include <vector>

#include "astro_localization/core/types.hpp"

namespace astro::visualization {

void writeTumTrajectory(const std::filesystem::path& output_path,
                        const std::vector<core::PoseStamped>& poses);

void writeCsvTrajectory(const std::filesystem::path& output_path,
                        const std::vector<core::PoseStamped>& poses);

}  // namespace astro::visualization

