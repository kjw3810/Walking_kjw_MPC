// 다리 하나(6자유도) 감쇠 Gauss-Newton IK. KinematicsThread와 tools/ik_check가 공유.
#pragma once

#include <array>

#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace walking_kjw {

struct LegContext {
  pinocchio::FrameIndex foot_frame;
  std::array<int, 6> joints;  // hip_pitch..ankle_roll
};

pinocchio::JointIndex JointId(const pinocchio::Model& model, int joint_index);

// 고정 반복 + warm-start (상세: 코드 해설 문서).
void SolveLegIk(const pinocchio::Model& model, pinocchio::Data& data, const LegContext& leg,
                const pinocchio::SE3& target, Eigen::VectorXd& q);

}  // namespace walking_kjw
