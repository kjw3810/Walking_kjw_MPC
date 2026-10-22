#include "motion_gen_thread.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <Eigen/Dense>

#include "g1_joints.h"
#include "lipm_gait.h"
#include "loop_timer.h"
#include "mpc_config.h"
#include "precise_sleep.h"

namespace walking_kjw {
namespace {

constexpr auto kMotionGenControlPeriod = std::chrono::milliseconds(1);  // 1kHz
constexpr double kDtS = 0.001;

constexpr double kStanceHalfWidthM = 0.09;  // pelvis 중심~발 좌우 거리 (g1.urdf 기준 근사)
constexpr double kStepHeightM = 0.05;       // 스윙 발 최고 높이

// Sit and stand (UpAndDown, 명칭 유지).
constexpr double kSquatDepthM = 0.15;
constexpr double kSquatPeriodS = 2.0;
constexpr int kSquatCycles = 5;

// WalkReady (task-space): 1회성 하강. Walk의 COM 높이(kNominalComHeightM)와 같게 맞추려고 0 (walk 진입 시 높이 점프 없음).
constexpr double kWalkReadyDepthM = 0.0;
constexpr double kWalkReadyDurationS = 2.0;

// WalkReady (joint-space, ControlMode::kJointSpace 전용): 현재 측정 관절각에서
// 아래 목표 관절각까지 일정 속도(선형 보간)로 이동하는 데 걸리는 시간.
constexpr double kWalkReadyJointDurationS = 3.0;
// 좌우 대칭, sagittal 평면(hip_pitch/knee/ankle_pitch)만 사용. hip_roll/yaw,
// ankle_roll, waist_yaw는 0 유지 -- task-space IK가 이 방향으로 수렴시키는
// 값은 다리 하나당 6DOF의 중복 자유도 때문에 실행마다 달라져(비유일해) 목표로
// 삼기엔 신뢰할 수 없다. 아래 값은 손으로 고른 대략적인 크라우치 각도이며,
// 실기 튜닝으로 조정될 것을 전제로 한다.
// 부호: hip_pitch는 음수가 허벅지 앞으로. 허벅지 -q + 무릎 2q + 발목 -q = 0 이라 발바닥이 지면과 평행.
// q=0.394 rad: 이때 pelvis 프레임 높이(발바닥 기준)가 kNominalStandHeightM(0.743 m), 즉 전신 COM 높이가
// Walk의 COM 목표 kNominalComHeightM(0.664 m)과 같다 (FK로 계산).
constexpr double kWalkReadyJointHipPitch = -0.394;
constexpr double kWalkReadyJointKnee = 0.788;
constexpr double kWalkReadyJointAnklePitch = -0.394;

// Walk 패턴 파라미터 (수식/근거: Controller/docs/walking_pattern_plan.md).
constexpr double kWalkStepTimeS = 0.7;      // 한 스텝 (SSP + DSP)
constexpr double kWalkDspRatio = 0.2;       // DSP 비율 (0이면 슬라이드 원형)
constexpr double kWalkSwingLandRatio = 0.85;  // 스윙 x가 목표에 도착하는 진행률

double SquatHeightOffsetM(double squat_t) {
  const double total_duration_s = kSquatPeriodS * kSquatCycles;
  const double t = std::min(squat_t, total_duration_s);
  return -0.5 * kSquatDepthM * (1.0 - std::cos(2.0 * M_PI * t / kSquatPeriodS));
}

double WalkReadyHeightOffsetM(double walkready_t) {
  const double frac = std::clamp(walkready_t / kWalkReadyDurationS, 0.0, 1.0);
  const double eased = 0.5 * (1.0 - std::cos(M_PI * frac));
  return -kWalkReadyDepthM * eased;
}

// 관절제어모드 WalkReady의 목표 관절각 (13차원, hip_roll/yaw·ankle_roll·waist_yaw는 0).
Eigen::VectorXd WalkReadyJointTarget() {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(kNumJoints);
  q[kLeftHipPitch] = kWalkReadyJointHipPitch;
  q[kLeftKnee] = kWalkReadyJointKnee;
  q[kLeftAnklePitch] = kWalkReadyJointAnklePitch;
  q[kRightHipPitch] = kWalkReadyJointHipPitch;
  q[kRightKnee] = kWalkReadyJointKnee;
  q[kRightAnklePitch] = kWalkReadyJointAnklePitch;
  return q;
}

// ---- Sit and stand: 고정 중앙 스탠스, 높이만 반복 진동 ----
void TickSitAndStand(DataWarehouse& dw, double& squat_t) {
  FootTargetWorld foot_targ;
  foot_targ.left = Eigen::Vector3d(0.0, kStanceHalfWidthM, 0.0);
  foot_targ.right = Eigen::Vector3d(0.0, -kStanceHalfWidthM, 0.0);
  dw.SetFootTargWorld(foot_targ);

  const double height = kNominalComHeightM + SquatHeightOffsetM(squat_t);
  // COM x는 발목(0)이 아니라 발 접지 중앙(발목 + kFootCenterOffsetX) 위. 발목 위면 뒤꿈치 쪽에 치우쳐 뒤로 넘어진다.
  dw.SetComTargWorld(Eigen::Vector3d(kFootCenterOffsetX, 0.0, height));

  dw.SetTorsoQuatTarg(Eigen::Quaterniond::Identity());  // 몸통 수평

  squat_t += kDtS;
}

// ---- WalkReady: 고정 중앙 스탠스, 1회성 하강 후 유지 ----
void TickWalkReady(DataWarehouse& dw, double& walkready_t) {
  FootTargetWorld foot_targ;
  foot_targ.left = Eigen::Vector3d(0.0, kStanceHalfWidthM, 0.0);
  foot_targ.right = Eigen::Vector3d(0.0, -kStanceHalfWidthM, 0.0);
  dw.SetFootTargWorld(foot_targ);

  const double height = kNominalComHeightM + WalkReadyHeightOffsetM(walkready_t);
  // COM x는 발목(0)이 아니라 발 접지 중앙(발목 + kFootCenterOffsetX) 위. 발목 위면 뒤꿈치 쪽에 치우쳐 뒤로 넘어진다.
  dw.SetComTargWorld(Eigen::Vector3d(kFootCenterOffsetX, 0.0, height));

  dw.SetTorsoQuatTarg(Eigen::Quaterniond::Identity());  // 몸통 수평

  walkready_t += kDtS;
}

// TickWalkReadyJointSpace가 틱 사이에 들고 가는 상태: 이동을 시작한 시점의
// 측정 관절각(start_q)을 한 번만 캡처해서 그 각도부터 목표까지 선형 보간한다.
struct JointRampState {
  double t = 0.0;
  bool started = false;
  Eigen::VectorXd start_q;
};

// ---- WalkReady (관절제어모드): 현재 측정 관절각 -> 목표 관절각으로 일정
// 속도(선형 보간) 이동. IK/task-space를 전혀 거치지 않는다. ----
void TickWalkReadyJointSpace(DataWarehouse& dw, JointRampState& s) {
  if (!s.started) {
    s.start_q = dw.GetJointPosSen();  // 이동 시작 시점의 측정 관절각을 출발점으로 캡처
    s.started = true;
  }
  const Eigen::VectorXd target_q = WalkReadyJointTarget();
  const double frac = std::clamp(s.t / kWalkReadyJointDurationS, 0.0, 1.0);
  dw.SetJointTargPos(s.start_q + frac * (target_q - s.start_q));  // 선형 보간 = 등속

  s.t += kDtS;
}

// ---- Sit and stand (관절제어모드): WalkReady 자세를 중심으로 sagittal 관절(hip_pitch/knee/
// ankle_pitch)을 같은 비율로 스케일해 앉았다 일어남. scale = 1 + 0.5*sin -> [0.5, 1.5], 시작/끝은
// WalkReady 자세(scale=1)라 연속. hip+ankle+knee 합이 항상 0이라 발바닥은 지면과 평행 유지. ----
constexpr double kSquatJointScaleAmp = 0.5;

void TickSitAndStandJointSpace(DataWarehouse& dw, double& squat_t) {
  const double t = std::min(squat_t, kSquatPeriodS * kSquatCycles);
  const double scale = 1.0 + kSquatJointScaleAmp * std::sin(2.0 * M_PI * t / kSquatPeriodS);
  dw.SetJointTargPos(WalkReadyJointTarget() * scale);
  squat_t += kDtS;
}

// ---- Run: 미구현 (빈 스텁) ----
void TickRun() {
  // TODO: run 모션. 구현 전까지 아무 것도 하지 않음 (직전 목표값 유지).
}

gait::GaitParams WalkBaseParams() {
  gait::GaitParams p;
  p.h = kNominalComHeightM;  // LIPM 높이 = 전신 COM의 서 있는 높이
  p.T = kWalkStepTimeS;
  p.rho = kWalkDspRatio;
  p.y0 = kStanceHalfWidthM;
  p.h_step = kStepHeightM;
  p.s_land = kWalkSwingLandRatio;
  p.zmp_dx = kFootCenterOffsetX;  // ZMP/COM 기준점 = 발 접지 중앙(발목보다 3.5cm 앞)
  return p;
}

// ---- Walking: LIPM 해석해 COM + DSP Hermite + cosine 스윙발 + 몸통 기울기 발 위치 보정 ----
// 패턴 생성 본체는 lipm_gait.h (tools/gait_check가 같은 코드를 검증). 여기는 DataWarehouse와의 접착만.
void TickWalking(DataWarehouse& dw, gait::WalkPatternGenerator& gen, int& preview_tick, double& waist_cmd) {
  const Eigen::Vector3d imu_ang = dw.GetImuAng();
  const Eigen::Vector3d imu_angvel = dw.GetImuAngVel();

  gait::WalkInput in;
  in.vel = dw.GetVelCmd();
  in.roll = imu_ang[0];
  in.pitch = imu_ang[1];
  in.wx = imu_angvel[0];
  in.wy = imu_angvel[1];
  in.kp_foot = dw.GetKpFoot();  // Kθ (기본 0 = 보정 꺼짐)
  in.kd_foot = dw.GetKdFoot();  // Kω
  in.waist_k = dw.GetWaistK();      // 상체 카운터 로테이션 게인 (0 = 꺼짐)
  in.waist_max = dw.GetWaistMax();
  in.adapt = dw.GetStepAdapt();
  const Eigen::Vector3d com_meas = dw.GetComMeasWorld();
  in.meas_valid = true;
  in.cx = com_meas.x();
  in.cy = com_meas.y();
  // MPC용 향후 계획: 현재 상태에서 kMpcDtS 간격으로 N점 (보정 입력 고정, kMpcSolvePeriodTicks마다 갱신).
  // 틱 이전 상태에서 복사해 미리 보므로 이번 틱의 Tick()보다 먼저 호출.
  if (preview_tick++ % kMpcSolvePeriodTicks == 0) {
    gait::WalkOutput pv[kMpcHorizon];
    gen.Preview(in, kDtS, kMpcDtS, kMpcHorizon, pv);
    WalkPreview wp;
    wp.valid = true;
    for (int k = 0; k < kMpcHorizon; ++k) {
      wp.pts[k].left = Eigen::Vector3d(pv[k].left.x, pv[k].left.y, pv[k].left.z);
      wp.pts[k].right = Eigen::Vector3d(pv[k].right.x, pv[k].right.y, pv[k].right.z);
      wp.pts[k].com_xy = Eigen::Vector2d(pv[k].com_x, pv[k].com_y);
      wp.pts[k].com_acc = Eigen::Vector2d(pv[k].com_ax, pv[k].com_ay);
      wp.pts[k].w_left = pv[k].w_left;
    }
    dw.SetWalkPreview(wp);
  }
  const gait::WalkOutput out = gen.Tick(in, kDtS);
  waist_cmd = out.waist_yaw;

  FootTargetWorld foot_targ;
  foot_targ.left = Eigen::Vector3d(out.left.x, out.left.y, out.left.z);
  foot_targ.right = Eigen::Vector3d(out.right.x, out.right.y, out.right.z);
  dw.SetFootTargWorld(foot_targ);

  dw.SetWalkDebug({static_cast<int>(out.phase), out.support, out.dx, out.dy});

  const Eigen::Vector3d com(out.com_x, out.com_y, kNominalComHeightM);
  dw.SetComTargWorld(com);

  // 몸통은 항상 수평 유지 (균형은 WBC 담당, 여기서 겹쳐 명령하면 서로 충돌).
  dw.SetTorsoQuatTarg(Eigen::Quaterniond::Identity());
}

}  // namespace

void RunMotionGenThread(DataWarehouse& dw, std::atomic<bool>& stop, std::FILE* timing_log) {
  MotionMode prev_mode = MotionMode::kWalk;
  ControlMode prev_control_mode = dw.GetControlMode();
  double squat_t = 0.0;
  double walkready_t = 0.0;
  JointRampState walkready_joint_state;
  gait::WalkPatternGenerator walking_gen(WalkBaseParams());
  int walking_preview_tick = 0;
  double waist_cmd = 0.0;  // 허리 yaw 목표 (보행 중 생성기 값, 그 외 모드에서는 0으로 수렴)
  LoopTimer loop_timer("MotionGen", kMotionGenControlPeriod.count(), std::chrono::seconds(1),
                        timing_log);

  while (!stop.load()) {
    const auto tick_start = std::chrono::steady_clock::now();
    loop_timer.TickStart();
    const MotionMode mode = dw.GetMotionMode();
    const ControlMode control_mode = dw.GetControlMode();

    if (mode != prev_mode) {
      if (mode == MotionMode::kUpAndDown) squat_t = 0.0;
      if (mode == MotionMode::kWalkReady) walkready_t = 0.0;
      if (mode == MotionMode::kWalk) {
        walking_gen.Reset();
        walking_preview_tick = 0;
      }
      if (mode != MotionMode::kWalk) dw.SetWalkPreview(WalkPreview{});  // walk 진입마다 Init부터 다시 시작
      if (mode == MotionMode::kWalkReady) walkready_joint_state = JointRampState{};
      prev_mode = mode;
    }
    // WalkReady 도중에 controlmode를 taskspace<->jointspace로 바꿔도, 관절제어
    // 이동은 그 시점의 측정 관절각부터 다시 시작하도록 리셋.
    if (control_mode != prev_control_mode) {
      if (mode == MotionMode::kWalkReady && control_mode == ControlMode::kJointSpace) {
        walkready_joint_state = JointRampState{};
      }
      if (mode == MotionMode::kUpAndDown) squat_t = 0.0;
      prev_control_mode = control_mode;
    }

    // WBC는 walk 모드이면서 사용자 허용이 켜져 있을 때만 켜진다 (기본 on, "wbc off"로 끔).
    dw.SetWbcFlag(mode == MotionMode::kWalk && dw.GetWbcUserEnabled());

    switch (mode) {
      case MotionMode::kUpAndDown:
        if (control_mode == ControlMode::kJointSpace) {
          TickSitAndStandJointSpace(dw, squat_t);
        } else {
          TickSitAndStand(dw, squat_t);
        }
        break;
      case MotionMode::kWalkReady:
        if (control_mode == ControlMode::kJointSpace) {
          TickWalkReadyJointSpace(dw, walkready_joint_state);
        } else {
          TickWalkReady(dw, walkready_t);
        }
        break;
      case MotionMode::kRun:
        TickRun();
        break;
      case MotionMode::kWalk:
      default:
        TickWalking(dw, walking_gen, walking_preview_tick, waist_cmd);
        break;
    }

    if (mode != MotionMode::kWalk) {  // 보행이 아니면 허리를 0으로 부드럽게 되돌린다 (1 rad/s)
      waist_cmd -= std::clamp(waist_cmd, -kDtS, kDtS);
    }
    dw.SetWaistYawTarg(waist_cmd);

    loop_timer.ComputeDone();
    SleepUntilPrecise(tick_start + kMotionGenControlPeriod);
  }
}

}  // namespace walking_kjw
