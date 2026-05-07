#include "astro_localization/visualization/trajectory_io.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace astro::visualization {
namespace {

std::ofstream openOutput(const std::filesystem::path& output_path) {
  std::filesystem::create_directories(output_path.parent_path());
  std::ofstream output(output_path);
  if (!output) {
    throw std::runtime_error("failed to open trajectory output: " + output_path.string());
  }
  output << std::fixed << std::setprecision(9);
  return output;
}

}  // namespace

void writeTumTrajectory(const std::filesystem::path& output_path,
                        const std::vector<core::PoseStamped>& poses) {
  std::ofstream output = openOutput(output_path);
  for (const auto& pose : poses) {
    const Eigen::Quaterniond q(pose.T_world_camera.linear());
    const Eigen::Vector3d t = pose.T_world_camera.translation();
    output << pose.timestamp << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' ' << q.x()
           << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
}

void writeCsvTrajectory(const std::filesystem::path& output_path,
                        const std::vector<core::PoseStamped>& poses) {
  std::ofstream output = openOutput(output_path);
  output << "timestamp,tx,ty,tz,qx,qy,qz,qw\n";
  for (const auto& pose : poses) {
    const Eigen::Quaterniond q(pose.T_world_camera.linear());
    const Eigen::Vector3d t = pose.T_world_camera.translation();
    output << pose.timestamp << ',' << t.x() << ',' << t.y() << ',' << t.z() << ',' << q.x()
           << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
  }
}

}  // namespace astro::visualization

