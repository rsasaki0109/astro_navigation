#include "astro_localization/core/image_sequence.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace astro::core {
namespace {

std::string lowerExtension(const std::filesystem::path& path) {
  std::string ext = path.extension().string();
  std::ranges::transform(ext, ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext;
}

}  // namespace

bool isImageFile(const std::filesystem::path& path) {
  const std::string ext = lowerExtension(path);
  return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" ||
         ext == ".tif" || ext == ".tiff" || ext == ".pgm";
}

std::vector<std::filesystem::path> loadImageSequence(const std::filesystem::path& input) {
  std::vector<std::filesystem::path> images;

  if (std::filesystem::is_directory(input)) {
    for (const auto& entry : std::filesystem::directory_iterator(input)) {
      if (entry.is_regular_file() && isImageFile(entry.path())) {
        images.push_back(entry.path());
      }
    }
  } else if (lowerExtension(input) == ".txt" || lowerExtension(input) == ".csv") {
    std::ifstream file(input);
    if (!file) {
      throw std::runtime_error("failed to open image list: " + input.string());
    }
    const auto base = input.parent_path();
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line.starts_with("#")) {
        continue;
      }
      const auto comma = line.find(',');
      const auto token = line.substr(0, comma);
      std::filesystem::path path(token);
      images.push_back(path.is_absolute() ? path : base / path);
    }
  } else if (isImageFile(input)) {
    images.push_back(input);
  }

  std::ranges::sort(images);
  return images;
}

cv::Mat loadGrayImage(const std::filesystem::path& path) {
  cv::Mat image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    throw std::runtime_error("failed to read image: " + path.string());
  }
  return image;
}

}  // namespace astro::core

