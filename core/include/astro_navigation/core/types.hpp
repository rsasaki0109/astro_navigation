#pragma once

#include <Eigen/Geometry>

namespace astro::core {

struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  [[nodiscard]] bool valid() const { return fx > 0.0 && fy > 0.0; }
};

struct PoseStamped {
  double timestamp{0.0};
  Eigen::Isometry3d T_world_camera{Eigen::Isometry3d::Identity()};
};

}  // namespace astro::core
