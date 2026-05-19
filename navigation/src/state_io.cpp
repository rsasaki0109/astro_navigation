#include "astro_navigation/navigation/state_io.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace astro::navigation {
namespace {

std::ofstream openOutput(const std::filesystem::path& output_path) {
  if (output_path.has_parent_path()) {
    std::filesystem::create_directories(output_path.parent_path());
  }
  std::ofstream output(output_path);
  if (!output) {
    throw std::runtime_error("failed to open navigation output: " + output_path.string());
  }
  output << std::fixed << std::setprecision(9);
  return output;
}

}  // namespace

void writeNavStateJson(const std::filesystem::path& output_path, const NavState& state) {
  std::ofstream output = openOutput(output_path);
  output << "{\n";
  output << "  \"timestamp\": " << state.timestamp << ",\n";
  output << "  \"status\": \"" << toString(state.status) << "\",\n";
  output << "  \"status_reason\": \"" << toString(state.status_reason) << "\",\n";
  output << "  \"message\": \"" << state.message << "\",\n";
  output << "  \"position_frame_id\": \"" << state.position_frame_id << "\",\n";
  output << "  \"attitude_reference_frame_id\": \"" << state.attitude_reference_frame_id << "\",\n";
  output << "  \"position_m\": [" << state.position.x() << ", " << state.position.y() << ", "
         << state.position.z() << "],\n";
  output << "  \"velocity_mps\": [" << state.velocity.x() << ", " << state.velocity.y() << ", "
         << state.velocity.z() << "],\n";
  output << "  \"q_body_reference_xyzw\": [" << state.q_body_reference.x() << ", "
         << state.q_body_reference.y() << ", " << state.q_body_reference.z() << ", "
         << state.q_body_reference.w() << "],\n";
  output << "  \"quality\": {\n";
  output << "    \"attitude_lock\": " << (state.quality.attitude_lock ? "true" : "false") << ",\n";
  output << "    \"position_lock\": " << (state.quality.position_lock ? "true" : "false") << ",\n";
  output << "    \"velocity_lock\": " << (state.quality.velocity_lock ? "true" : "false") << ",\n";
  output << "    \"attitude_sigma_rad\": " << state.quality.attitude_sigma_rad << ",\n";
  output << "    \"position_sigma_m\": " << state.quality.position_sigma_m << ",\n";
  output << "    \"attitude_correspondences\": " << state.quality.attitude_correspondences << "\n";
  output << "  },\n";
  output << "  \"covariance_6x6\": [\n";
  for (int row = 0; row < 6; ++row) {
    output << "    [";
    for (int col = 0; col < 6; ++col) {
      output << state.covariance(row, col);
      if (col + 1 < 6) {
        output << ", ";
      }
    }
    output << "]";
    if (row + 1 < 6) {
      output << ',';
    }
    output << '\n';
  }
  output << "  ]\n";
  output << "}\n";
}

void writeNavStateCsv(const std::filesystem::path& output_path, const NavState& state) {
  std::ofstream output = openOutput(output_path);
  output
      << "timestamp,status,status_reason,attitude_lock,position_lock,attitude_correspondences,"
         "attitude_sigma_rad,position_sigma_m,frame,x,y,z,qx,qy,qz,qw,message\n";
  output << state.timestamp << ',' << toString(state.status) << ',' << toString(state.status_reason)
         << ','
         << (state.quality.attitude_lock ? 1 : 0) << ','
         << (state.quality.position_lock ? 1 : 0) << ','
         << state.quality.attitude_correspondences << ',' << state.quality.attitude_sigma_rad << ','
         << state.quality.position_sigma_m << ',' << state.position_frame_id << ','
         << state.position.x() << ',' << state.position.y() << ',' << state.position.z() << ','
         << state.q_body_reference.x() << ',' << state.q_body_reference.y() << ','
         << state.q_body_reference.z() << ',' << state.q_body_reference.w() << ','
         << state.message << '\n';
}

}  // namespace astro::navigation
