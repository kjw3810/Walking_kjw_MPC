// Shared System V message queue protocol between Controller and Mujoco.
// Included by both sides so the keys and message layouts can't drift apart.
#pragma once

#include <sys/ipc.h>
#include <sys/msg.h>

#include <cstdlib>

namespace walking_kjw {

// msg_ref: Controller -> Mujoco (position reference command)
// msg_sen: Mujoco -> Controller (angle/velocity feedback)
constexpr key_t kMsgRefKey = 0x574B4A31;  // "WKJ1"
constexpr key_t kMsgSenKey = 0x574B4A32;  // "WKJ2"
// 환경변수 WKJ_IPC_KEY(정수)가 있으면 그 값부터 쓴다 (오프라인 테스트 하네스가 실행 중인 시뮬과
// 큐를 공유하지 않게). 없으면 위 기본값 -- 평소 사용에는 영향 없음.
inline key_t MsgRefKey() {
  const char* e = std::getenv("WKJ_IPC_KEY");
  return e ? static_cast<key_t>(std::strtol(e, nullptr, 0)) : kMsgRefKey;
}
inline key_t MsgSenKey() {
  const char* e = std::getenv("WKJ_IPC_KEY");
  return e ? static_cast<key_t>(std::strtol(e, nullptr, 0)) + 1 : kMsgSenKey;
}

// G1 target design: 12 leg joints (hip_pitch/roll/yaw, knee, ankle_pitch/roll
// x2) + waist_yaw, in the revolute-joint order of Controller/model/g1/g1.urdf.
// See Controller/docs/architecture.md section 9 for the field-by-field
// rationale.
constexpr int kNumJoints = 13;

// Single source of truth for the joint PD gain default, shared by two
// otherwise-unrelated call sites that must not drift apart:
//   - RefMsg::kp_joint/kd_joint's own default member initializer below,
//     which only matters as a fail-safe for a RefMsg sender other than
//     KinematicsThread (e.g. a debug/test harness) that forgets to set them
//     explicitly -- kinematics_thread.cpp itself always overwrites these
//     from DataWarehouse before every send, so this default never fires on
//     the normal Controller -> Simulate path.
//   - DataWarehouse's kp_joint_/kd_joint_ construction
//     (Controller/include/data_warehouse.h), which *is* the value actually
//     used on that normal path.
// Keeping both pinned to the same constant means there's only one number to
// tune, instead of two that silently disagree.
constexpr double kDefaultKpJoint = 500.0;
constexpr double kDefaultKdJoint = 10.0;

// Default pelvis_hold weld height, matching Mujoco/model/g1/g1.xml's
// <weld ... anchor="0 0 0.89".../>. RefMsg::pelvis_hold_height_m carries the
// *current* commanded height for that weld every tick (KinematicsThread's
// drop sequence ramps it down from here); pid_ipc.cc writes whatever it
// receives into the weld's live anchor via mjModel::eq_data (see its
// comment for why that needs a const_cast). Only takes effect while the
// weld is active (drop_robot_sim == false).
constexpr double kWeldHoldHeightM = 0.89;

struct RefMsg {
  long mtype = 1;
  double joint_targ_pos[kNumJoints] = {};
  double joint_targ_vel[kNumJoints] = {};
  double joint_targ_torq[kNumJoints] = {};
  double kp_joint = kDefaultKpJoint;  // shared position gain across all joints
  double kd_joint = kDefaultKdJoint;  // shared velocity gain across all joints
  bool drop_robot_sim = false;
  double pelvis_hold_height_m = kWeldHoldHeightM;
  // 외력 푸시 (디버그/시험용): pelvis에 월드 좌표계 힘 (push_fx, push_fy) [N]을 push_dur_s 초 동안.
  // push_seq가 바뀔 때마다 새 푸시 1회로 본다 (같은 값을 매 틱 재전송해도 한 번만 적용).
  double push_fx = 0.0;
  double push_fy = 0.0;
  double push_dur_s = 0.0;
  long push_seq = 0;
};

struct SenMsg {
  long mtype = 1;
  double joint_pos[kNumJoints] = {};
  double joint_vel[kNumJoints] = {};
  double joint_torq[kNumJoints] = {};
  double imu_ang[3] = {};
  double imu_angvel[3] = {};
  double sim_time_s = 0.0;
};

}  // namespace walking_kjw
