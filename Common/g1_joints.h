// Canonical 13-joint order shared by Controller (Pinocchio/URDF side) and
// Mujoco (simulate side), so both processes agree on array layout without
// duplicating the list. Order matches the revolute joints in
// Controller/model/g1/g1.urdf (see Controller/docs/architecture.md section 9).
#pragma once

#include <array>

#include "ipc_protocol.h"

namespace walking_kjw {

enum JointIndex {
  kLeftHipPitch = 0,
  kLeftHipRoll,
  kLeftHipYaw,
  kLeftKnee,
  kLeftAnklePitch,
  kLeftAnkleRoll,
  kRightHipPitch,
  kRightHipRoll,
  kRightHipYaw,
  kRightKnee,
  kRightAnklePitch,
  kRightAnkleRoll,
  kWaistYaw,
};

inline const std::array<const char*, kNumJoints>& G1JointNames() {
  static const std::array<const char*, kNumJoints> kNames = {
      "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
      "left_knee_joint",       "left_ankle_pitch_joint", "left_ankle_roll_joint",
      "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
      "right_knee_joint",      "right_ankle_pitch_joint", "right_ankle_roll_joint",
      "waist_yaw_joint",
  };
  return kNames;
}

// Per-leg joint index groups, used by IK (Jacobian columns) and ID (J^T*wrench).
inline constexpr std::array<int, 6> kLeftLegJoints = {
    kLeftHipPitch, kLeftHipRoll, kLeftHipYaw, kLeftKnee, kLeftAnklePitch, kLeftAnkleRoll};
inline constexpr std::array<int, 6> kRightLegJoints = {
    kRightHipPitch, kRightHipRoll, kRightHipYaw, kRightKnee, kRightAnklePitch, kRightAnkleRoll};

inline constexpr const char* kLeftFootFrame = "left_ankle_roll_link";
inline constexpr const char* kRightFootFrame = "right_ankle_roll_link";
inline constexpr const char* kPelvisFrame = "pelvis";
inline constexpr const char* kImuPelvisFrame = "imu_in_pelvis";

// Pelvis nominal standing height. 0.793(g1.xml "stand" 키프레임 기준)에서 5cm 낮춤: 0.793에서는
// 발 프레임 z=0 목표가 다리 최대 뻗음을 넘어 IK가 도달 불가능했다 (Controller/tools/ik_check).
// MotionGenThread의 LIPM COM 높이와 task-space 서 있는 높이에 쓰인다.
// ankle_roll_link 프레임 원점에서 발바닥(g1.xml foot geom: z=-0.03, 반지름 0.005)까지 거리.
// 발 프레임 z=0이 곧 지면이 아니다: 프레임은 지면에서 이만큼 위에 있다. IK 발 목표와 오도메트리에서 보정.
inline constexpr double kFootSoleOffsetM = 0.035;

// pelvis 프레임 원점의 서 있는 높이 (IK 도달 가능한 값. 관절 크라우치 WalkReady/각종 오프라인 도구의 기준).
inline constexpr double kNominalStandHeightM = 0.743;

// pelvis 링크 질량중심의 pelvis 프레임 기준 z 오프셋 (g1.urdf <inertial><origin xyz="0 0 -0.0760300603">):
// 프레임 원점보다 7.6cm 아래. IK root 목표의 **초기 추정**에만 쓰이고(Kinematics가 IK 해의 전신 COM으로 매 틱 갱신),
// URDF 값과 일치하는지 검증하는 데도 쓰인다.
inline constexpr double kPelvisComOffsetZ = -0.0760300603;

// 걷기 준비 자세(kNominalStandHeightM, 관절 크라우치 q=0.394)에서 전신 COM의 pelvis 프레임 기준 z (FK 계산: -0.0790).
// 프레임 원점을 COM으로 쓰면 전신 COM 대비 z로 7.9cm, x로 약 4.4cm 어긋난다. Kinematics는 전신 COM이 COM 목표에 오도록
// root를 정하므로, "COM 목표"는 pelvis 링크가 아니라 **전신 COM**의 월드 위치다.
inline constexpr double kWholeComOffsetZ = -0.0790;

// MotionGen의 COM 목표 높이 = LIPM의 h = 전신 COM의 서 있는 높이(0.664m). 같은 자세(pelvis 프레임 0.743m)를 유지한다.
inline constexpr double kNominalComHeightM = kNominalStandHeightM + kWholeComOffsetZ;

// 발바닥 접지 범위 (g1.xml foot geom: 발목 프레임 기준 x -0.05(뒤꿈치) ~ +0.12(발끝), y ±0.025(뒤)/±0.03(앞), 구 반지름 0.005).
// 발 중앙은 발목보다 3.5cm 앞이다 -> 보행 계획의 ZMP/COM 기준점과 CoP 한계는 발목이 아니라 이 형상을 따라야 한다
// (COM을 발목 바로 위에 두면 뒤꿈치까지 5cm뿐이라 뒤로 미는 외란에 약하다).
inline constexpr double kFootCenterOffsetX = 0.035;  // 발 중앙의 발목 대비 앞쪽 오프셋
inline constexpr double kFootHeelM = 0.045;          // CoP 뒤쪽 한계 (발목 기준, 5mm 여유)
inline constexpr double kFootToeM = 0.115;           // CoP 앞쪽 한계
inline constexpr double kFootHalfWidthM = 0.025;     // CoP 좌우 한계

}  // namespace walking_kjw
