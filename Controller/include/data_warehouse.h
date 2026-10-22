// Global blackboard shared by all Controller threads (Kinematics, MotionGen,
// CommandInput, WholeBodyControl, Logging). Every field is private and
// reachable only through its own get/set accessor, each guarded by its own
// mutex (see Controller/docs/architecture.md section 2: "필드별 락").
//
// Trade-off (documented in architecture.md section 2): per-field locking is
// cheap for the 1kHz threads that touch one or two fields per tick, but a
// consumer that needs several fields at a consistent instant (LoggingThread)
// will see values from slightly different ticks. Acceptable for now; revisit
// with a dedicated snapshot lock if logging precision becomes an issue.
#pragma once

#include <array>
#include <mutex>

#include <Eigen/Dense>

#include "ipc_protocol.h"
#include "mpc_config.h"

namespace walking_kjw {

enum class MotionMode { kWalk, kRun, kUpAndDown, kWalkReady };
enum class ControlMode { kTaskSpace = 1, kJointSpace = 2 };
enum class SaveFlag { kIdle, kStart, kEnd };

struct Pose3d {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
};

// Mpc command: Fx,Fy,Fz,Tx,Ty (architecture.md section 9.1).
struct ContactWrench {
  double fx = 0.0;
  double fy = 0.0;
  double fz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
};

struct FootTargetWorld {
  Eigen::Vector3d left = Eigen::Vector3d::Zero();
  Eigen::Vector3d right = Eigen::Vector3d::Zero();
};

// 보행 패턴 디버그 값 (MotionGen 출력, 로깅용).
struct WalkDebug {
  int phase = 0;    // 0 Init, 1 SSP, 2 DSP
  int support = 0;  // 0 왼발, 1 오른발
  double dx = 0.0;  // 발 위치 보정 (m)
  double dy = 0.0;
};

// MPC가 보는 향후 보행 계획 (MotionGen 출력). 점 k는 현재+ k*kMpcDtS 시점.
// 발 위치는 발바닥(지면 기준) 월드 좌표, COM은 xy (z는 com_targ 높이).
struct PreviewPoint {
  Eigen::Vector3d left = Eigen::Vector3d::Zero();
  Eigen::Vector3d right = Eigen::Vector3d::Zero();
  Eigen::Vector2d com_xy = Eigen::Vector2d::Zero();
  Eigen::Vector2d com_acc = Eigen::Vector2d::Zero();  // 계획 COM 가속도
  double w_left = 0.5;  // 왼발 접촉 비중 (오른발 = 1 - w_left)
};
struct WalkPreview {
  bool valid = false;
  std::array<PreviewPoint, kMpcHorizon> pts;
};

// 시뮬레이터 pelvis 외력 푸시 요청 ("push" 명령). seq가 바뀌면 새 푸시.
struct PushRequest {
  double fx = 0.0, fy = 0.0, dur = 0.1;
  long seq = 0;
};

namespace detail {
// One mutex per field, as architecture.md section 2 specifies. Kept private
// inside DataWarehouse; callers only ever see the named Get/Set methods.
template <typename T>
class Guarded {
 public:
  explicit Guarded(T initial = T()) : value_(std::move(initial)) {}

  T Get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  void Set(T new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(new_value);
  }

 private:
  mutable std::mutex mutex_;
  T value_;
};
}  // namespace detail

class DataWarehouse {
 public:
  DataWarehouse()
      : joint_pos_sen_(Eigen::VectorXd::Zero(kNumJoints)),
        joint_vel_sen_(Eigen::VectorXd::Zero(kNumJoints)),
        joint_torq_sen_(Eigen::VectorXd::Zero(kNumJoints)),
        joint_targ_pos_(Eigen::VectorXd::Zero(kNumJoints)),
        joint_targ_torq_(Eigen::VectorXd::Zero(kNumJoints)),
        // [kp_roll, kd_roll, kp_pitch, kd_pitch, kp_height, (reserved)]
        kp_wbc_((Eigen::VectorXd(6) << 300.0, 30.0, 300.0, 30.0, 2000.0, 0.0).finished()) {}

  // 실측 (FK 입력) — Simulate로부터 msg_sen으로 수신
  Eigen::VectorXd GetJointPosSen() const { return joint_pos_sen_.Get(); }
  void SetJointPosSen(const Eigen::VectorXd& v) { joint_pos_sen_.Set(v); }

  Eigen::VectorXd GetJointVelSen() const { return joint_vel_sen_.Get(); }
  void SetJointVelSen(const Eigen::VectorXd& v) { joint_vel_sen_.Set(v); }

  Eigen::VectorXd GetJointTorqSen() const { return joint_torq_sen_.Get(); }
  void SetJointTorqSen(const Eigen::VectorXd& v) { joint_torq_sen_.Set(v); }

  // pelvis 자세 + 위치. 위치는 pelvis 프레임 원점이 아니라 pelvis **링크 질량중심**(프레임 원점보다 7.6cm 아래, 전신 COM에 가까운 점).
  Pose3d GetTorsoPoseMeasWorld() const { return torso_pose_meas_world_.Get(); }
  void SetTorsoPoseMeasWorld(const Pose3d& p) { torso_pose_meas_world_.Set(p); }

  Eigen::Vector3d GetImuAng() const { return imu_ang_.Get(); }
  void SetImuAng(const Eigen::Vector3d& v) { imu_ang_.Set(v); }

  Eigen::Vector3d GetImuAngVel() const { return imu_angvel_.Get(); }
  void SetImuAngVel(const Eigen::Vector3d& v) { imu_angvel_.Set(v); }

  std::array<bool, 2> GetFootContact() const { return foot_contact_.Get(); }
  void SetFootContact(const std::array<bool, 2>& v) { foot_contact_.Set(v); }

  // IK — Kinematics(Th1) 결과
  Eigen::VectorXd GetJointTargPos() const { return joint_targ_pos_.Get(); }
  void SetJointTargPos(const Eigen::VectorXd& v) { joint_targ_pos_.Set(v); }

  // ID — Kinematics(Th1) 결과
  Eigen::VectorXd GetJointTargTorq() const { return joint_targ_torq_.Get(); }
  void SetJointTargTorq(const Eigen::VectorXd& v) { joint_targ_torq_.Set(v); }

  // 모션생성 — MotionGen(Th2) 출력
  FootTargetWorld GetFootTargWorld() const { return foot_targ_world_.Get(); }
  void SetFootTargWorld(const FootTargetWorld& v) { foot_targ_world_.Set(v); }

  Eigen::Vector3d GetComTargWorld() const { return com_targ_world_.Get(); }
  void SetComTargWorld(const Eigen::Vector3d& v) { com_targ_world_.Set(v); }

  // 허리 yaw 목표 [rad] (MotionGen 출력, Kinematics task-space IK가 사용). 보행 아닐 때는 0으로 수렴.
  double GetWaistYawTarg() const { return waist_yaw_targ_.Get(); }
  void SetWaistYawTarg(double v) { waist_yaw_targ_.Set(v); }

  // 상체 카운터 로테이션 게인 (`gain waist <k> [max]`): 허리 yaw = -k * (오른발x - 왼발x), |yaw| <= max.
  double GetWaistK() const { return waist_k_.Get(); }
  void SetWaistK(double v) { waist_k_.Set(v); }
  double GetWaistMax() const { return waist_max_.Get(); }
  void SetWaistMax(double v) { waist_max_.Set(v); }

  // 몸통 목표 **자세**(방향만. 단위 쿼터니언 = 수평). 위치는 ComTargWorld 하나로만 전달한다 (예전엔 Pose3d에 같은 위치를
  // 한 번 더 넣었지만 아무도 읽지 않았다).
  Eigen::Quaterniond GetTorsoQuatTarg() const { return torso_quat_targ_.Get(); }
  void SetTorsoQuatTarg(const Eigen::Quaterniond& q) { torso_quat_targ_.Set(q); }

  WalkDebug GetWalkDebug() const { return walk_debug_.Get(); }
  void SetWalkDebug(const WalkDebug& v) { walk_debug_.Set(v); }

  WalkPreview GetWalkPreview() const { return walk_preview_.Get(); }
  void SetWalkPreview(const WalkPreview& v) { walk_preview_.Set(v); }

  // 실측 발 프레임(발목) 월드 위치 — Kinematics(Th1) FK/오도메트리 결과, MPC의 COM-발 상대 위치 보정용
  std::array<Eigen::Vector3d, 2> GetFootPosMeasWorld() const { return foot_pos_meas_world_.Get(); }
  void SetFootPosMeasWorld(const std::array<Eigen::Vector3d, 2>& v) { foot_pos_meas_world_.Set(v); }

  // 실측 전신 COM (pinocchio centerOfMass, 월드). pelvis 링크 질량중심과 x로 2~5cm 다르다 — MPC 기하에 필수. COM 목표(ComTargWorld)도 이 전신 COM의 목표다.
  Eigen::Vector3d GetComMeasWorld() const { return com_meas_world_.Get(); }
  void SetComMeasWorld(const Eigen::Vector3d& v) { com_meas_world_.Set(v); }

  // WBC(Th4) 출력 — 발별(좌/우) ContactWrench
  std::array<ContactWrench, 2> GetWbcForceTorque() const { return wbc_force_torque_.Get(); }
  void SetWbcForceTorque(const std::array<ContactWrench, 2>& v) { wbc_force_torque_.Set(v); }

  // 커맨드
  MotionMode GetMotionMode() const { return motion_mode_.Get(); }
  void SetMotionMode(MotionMode v) { motion_mode_.Set(v); }

  Pose3d GetComPosOriCal() const { return com_pos_ori_cal_.Get(); }
  void SetComPosOriCal(const Pose3d& v) { com_pos_ori_cal_.Set(v); }

  double GetVelCmd() const { return vel_cmd_.Get(); }
  void SetVelCmd(double v) { vel_cmd_.Set(v); }

  // 모드/플래그
  // 실제 적용되는 WBC 플래그: MotionGen이 매 틱 (walk 모드 && 사용자 허용)으로 갱신한다.
  bool GetWbcFlag() const { return wbc_enabled_.Get(); }
  void SetWbcFlag(bool v) { wbc_enabled_.Set(v); }
  // 사용자 허용 ("wbc on/off" 명령, 기본 on). walk 모드에서 on이면 WBC가 켜지고, "wbc off"를 주면 꺼진 채 유지된다.
  bool GetStepAdapt() const { return step_adapt_.Get(); }
  void SetStepAdapt(bool v) { step_adapt_.Set(v); }
  bool GetWbcUserEnabled() const { return wbc_user_enabled_.Get(); }
  void SetWbcUserEnabled(bool v) { wbc_user_enabled_.Set(v); }

  ControlMode GetControlMode() const { return control_mode_.Get(); }
  void SetControlMode(ControlMode v) { control_mode_.Set(v); }

  // 게인: [kp_roll, kd_roll, kp_pitch, kd_pitch, kp_height, (reserved)].
  // wbc_thread.cpp's attitude-to-zero PD + height PD (see its top comment).
  Eigen::VectorXd GetKpWbc() const { return kp_wbc_.Get(); }
  void SetKpWbc(const Eigen::VectorXd& v) { kp_wbc_.Set(v); }

  double GetKpFoot() const { return kp_foot_.Get(); }
  void SetKpFoot(double v) { kp_foot_.Set(v); }
  double GetKdFoot() const { return kd_foot_.Get(); }
  void SetKdFoot(double v) { kd_foot_.Set(v); }

  double GetKpJoint() const { return kp_joint_.Get(); }
  void SetKpJoint(double v) { kp_joint_.Set(v); }
  double GetKdJoint() const { return kd_joint_.Get(); }
  void SetKdJoint(double v) { kd_joint_.Set(v); }

  // 시뮬레이터 연동
  bool GetDropRobot() const { return drop_robot_.Get(); }
  void SetDropRobot(bool v) { drop_robot_.Set(v); }

  bool GetDropSequenceRequested() const { return drop_sequence_requested_.Get(); }
  void SetDropSequenceRequested(bool v) { drop_sequence_requested_.Set(v); }
  bool GetDropDescending() const { return drop_descending_.Get(); }
  void SetDropDescending(bool v) { drop_descending_.Set(v); }

  PushRequest GetPushRequest() const { return push_request_.Get(); }
  void SetPushRequest(const PushRequest& v) { push_request_.Set(v); }

  // 로깅
  SaveFlag GetSaveFlag() const { return save_flag_.Get(); }
  void SetSaveFlag(SaveFlag v) { save_flag_.Set(v); }

 private:
  detail::Guarded<Eigen::VectorXd> joint_pos_sen_;
  detail::Guarded<Eigen::VectorXd> joint_vel_sen_;
  detail::Guarded<Eigen::VectorXd> joint_torq_sen_;
  detail::Guarded<Pose3d> torso_pose_meas_world_;
  detail::Guarded<Eigen::Vector3d> imu_ang_{Eigen::Vector3d::Zero()};
  detail::Guarded<Eigen::Vector3d> imu_angvel_{Eigen::Vector3d::Zero()};
  detail::Guarded<std::array<bool, 2>> foot_contact_{{false, false}};

  detail::Guarded<Eigen::VectorXd> joint_targ_pos_;
  detail::Guarded<Eigen::Quaterniond> torso_quat_targ_{Eigen::Quaterniond::Identity()};
  detail::Guarded<Eigen::VectorXd> joint_targ_torq_;

  detail::Guarded<FootTargetWorld> foot_targ_world_;
  detail::Guarded<Eigen::Vector3d> com_targ_world_{Eigen::Vector3d::Zero()};
  detail::Guarded<double> waist_yaw_targ_{0.0};
  detail::Guarded<double> waist_k_{0.7};  // 폐루프 시험으로 정함: 0.7 rad/m(S=0.2 m에서 ±8°). 0이면 꺼짐
  detail::Guarded<double> waist_max_{0.15};

  detail::Guarded<WalkDebug> walk_debug_;
  detail::Guarded<WalkPreview> walk_preview_;
  detail::Guarded<Eigen::Vector3d> com_meas_world_{Eigen::Vector3d::Zero()};
  detail::Guarded<std::array<Eigen::Vector3d, 2>> foot_pos_meas_world_{
      {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
  detail::Guarded<std::array<ContactWrench, 2>> wbc_force_torque_{{ContactWrench{}, ContactWrench{}}};

  detail::Guarded<MotionMode> motion_mode_{MotionMode::kWalkReady};
  detail::Guarded<Pose3d> com_pos_ori_cal_;
  detail::Guarded<double> vel_cmd_{0.2};  // 기본 보행 속도 명령 0.2 m/s

  detail::Guarded<bool> wbc_enabled_{false};
  detail::Guarded<bool> wbc_user_enabled_{true};
  detail::Guarded<bool> step_adapt_{true};  // 외란 적응(조기 착지 + 스텝 주기 단축), "sa on|off"
  detail::Guarded<ControlMode> control_mode_{ControlMode::kTaskSpace};

  detail::Guarded<Eigen::VectorXd> kp_wbc_;
  detail::Guarded<double> kp_foot_{0.0};  // 0 = 보정 꺼짐. 스윕 결과 0.37/0.10도 푸시 없이 자발 낙하(2/8) -> docs/mpc_plan.md 3.5
  detail::Guarded<double> kd_foot_{0.0};
  // Shared with RefMsg's own default in Common/ipc_protocol.h -- see the
  // comment on kDefaultKpJoint there for why the same constant backs both.
  detail::Guarded<double> kp_joint_{kDefaultKpJoint};
  detail::Guarded<double> kd_joint_{kDefaultKdJoint};

  detail::Guarded<bool> drop_robot_{false};
  // "drop auto" sequence (see KinematicsThread): CommandInput sets
  // drop_sequence_requested_ true; Kinematics consumes it (resets to false)
  // and sets drop_descending_ true while it ramps the weld height down,
  // clearing it again once a foot touchdown triggers the real drop_robot_.
  detail::Guarded<bool> drop_sequence_requested_{false};
  detail::Guarded<bool> drop_descending_{false};

  detail::Guarded<PushRequest> push_request_;

  detail::Guarded<SaveFlag> save_flag_{SaveFlag::kIdle};
};

}  // namespace walking_kjw
