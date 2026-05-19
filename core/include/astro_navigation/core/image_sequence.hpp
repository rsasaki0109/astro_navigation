#pragma once

#include <filesystem>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace astro::core {

bool isImageFile(const std::filesystem::path& path);

std::vector<std::filesystem::path> loadImageSequence(const std::filesystem::path& input);

cv::Mat loadGrayImage(const std::filesystem::path& path);

}  // namespace astro::core
