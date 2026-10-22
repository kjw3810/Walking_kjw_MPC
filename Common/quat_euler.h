// Quaternion <-> roll/pitch/yaw (XYZ body axes, ZYX Tait-Bryan extraction)
// conversion shared by Controller (Eigen-based) and Mujoco (plain doubles),
// so both sides agree on the same convention for the IMU orientation carried
// in RefMsg/SenMsg without linking Eigen into the Mujoco simulate target.
#pragma once

#include <algorithm>
#include <cmath>

namespace walking_kjw {

// quat = (w, x, y, z), unit. Returns roll (X), pitch (Y), yaw (Z) in radians.
inline void EulerFromQuat(double w, double x, double y, double z, double* roll,
                           double* pitch, double* yaw) {
  *roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  *pitch = std::asin(sinp);
  *yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// Inverse of EulerFromQuat: roll/pitch/yaw (radians) -> unit quaternion (w,x,y,z).
inline void QuatFromEuler(double roll, double pitch, double yaw, double* w,
                           double* x, double* y, double* z) {
  double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  *w = cr * cp * cy + sr * sp * sy;
  *x = sr * cp * cy - cr * sp * sy;
  *y = cr * sp * cy + sr * cp * sy;
  *z = cr * cp * sy - sr * sp * cy;
}

}  // namespace walking_kjw
