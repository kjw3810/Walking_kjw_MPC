#include "kinematics_thread.h"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "g1_joints.h"
#include "leg_ik.h"
#include "leg_odometry.h"
#include "ipc_protocol.h"
#include "loop_timer.h"
#include "precise_sleep.h"
#include "quat_euler.h"

namespace walking_kjw {
namespace {

constexpr auto kKinematicsControlPeriod = std::chrono::milliseconds(1);  // 1kHz

// drop auto: 5cm/s로 서서히 낮추다 계산된 높이(com_targ.z + margin)에서 해제.
// 접촉 센서 없음 (실기와 동일). kDropMinHeightM은 안전 하한 backstop.
constexpr double kDropDescendRateMps = 0.05;
constexpr double kDropMinHeightM = 0.3;
constexpr double kDropReleaseMarginM = 0.02;

// 측정 관절각 + IMU 자세 + pelvis 월드 위치로 q 조립.
Eigen::VectorXd BuildMeasuredQ(const pinocchio::Model& model, const Eigen::VectorXd& joint_pos_sen,
                                const Eigen::Vector3d& imu_ang, const Eigen::Vector3d& pelvis_pos) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  q.segment<3>(0) = pelvis_pos;
  double qw, qx, qy, qz;
  QuatFromEuler(imu_ang[0], imu_ang[1], imu_ang[2], &qw, &qx, &qy, &qz);
  q[3] = qx;
  q[4] = qy;
  q[5] = qz;
  q[6] = qw;
  for (int i = 0; i < kNumJoints; ++i) {
    q[model.idx_qs[JointId(model, i)]] = joint_pos_sen[i];
  }
  return q;
}

struct MeasuredState {
  Eigen::VectorXd q;
  std::array<bool, 2> contact;
  std::array<Eigen::Vector3d, 2> foot_pos;  // 발 프레임(발목) 월드 위치
  Eigen::Vector3d com;                      // 전신 COM 월드 위치 (pelvis와 다름)
  Pose3d torso_pose_world;  // pos = pelvis **링크 질량중심**(프레임 원점이 아님), quat = pelvis 자세
};

// FK: pelvis 월드 위치 + 발 접촉 판정(WBC용) + torso 자세. 종료 시 data는 q 기준으로 갱신돼 있다.
// weld가 잡고 있는 동안(drop 전/하강 중)은 pelvis 위치를 아는 값(hold_height_m, XY=0)으로 두고,
// 해제 후에는 다리 오도메트리로 추정한다.
MeasuredState SolveMeasuredStateFK(const pinocchio::Model& model, pinocchio::Data& data,
                                    const std::array<const LegContext*, 2>& legs,
                                    const Eigen::VectorXd& joint_pos_sen,
                                    const Eigen::Vector3d& imu_ang, bool weld_active,
                                    double hold_height_m, const Eigen::Vector3d& pelvis_com_local,
                                    LegOdometry& odometry) {
  MeasuredState s;
  s.q = BuildMeasuredQ(model, joint_pos_sen, imu_ang, Eigen::Vector3d::Zero());
  if (weld_active) {
    odometry.Reset();  // 해제 시점부터 XY 원점 = 0
    s.q.segment<3>(0) = Eigen::Vector3d(0.0, 0.0, hold_height_m);
    pinocchio::forwardKinematics(model, data, s.q);
    pinocchio::updateFramePlacements(model, data);
    for (int i = 0; i < 2; ++i) {
      s.contact[i] =
          data.oMf[legs[i]->foot_frame].translation()[2] - kFootSoleOffsetM < kFootContactHeightM;
    }
  } else {
    pinocchio::forwardKinematics(model, data, s.q);  // pelvis 원점 기준 발 상대 위치
    pinocchio::updateFramePlacements(model, data);
    const std::array<Eigen::Vector3d, 2> foot_rel = {data.oMf[legs[0]->foot_frame].translation(),
                                                     data.oMf[legs[1]->foot_frame].translation()};
    s.q.segment<3>(0) = odometry.Update(foot_rel, &s.contact);
    pinocchio::forwardKinematics(model, data, s.q);
    pinocchio::updateFramePlacements(model, data);
  }
  s.com = pinocchio::centerOfMass(model, data, pinocchio::POSITION, false);
  for (int i = 0; i < 2; ++i) s.foot_pos[i] = data.oMf[legs[i]->foot_frame].translation();
  // "pelvis 위치"는 URDF의 pelvis 프레임 원점이 아니라 pelvis 링크 질량중심(원점에서 pelvis_com_local만큼, 자세 반영).
  // 프레임 원점은 전신 COM보다 z로 약 8cm 위지만 링크 질량중심은 약 0.3cm 차이라 COM 근사로 훨씬 낫다.
  s.torso_pose_world.quat = Eigen::Quaterniond(s.q[6], s.q[3], s.q[4], s.q[5]);
  s.torso_pose_world.pos = s.q.segment<3>(0) + s.torso_pose_world.quat * pelvis_com_local;
  return s;
}

// ID: 관절 피드포워드 토크 = 중력 보상 g(q) - J^T·WBC wrench.
// 정적으로 서 있을 때 필요한 관절 토크 = g(q) - J^T w (w = 지면이 로봇에 가하는 힘).
// MuJoCo 정적 평형과 대조해 오차 0 확인 (tools/id_check).
// IK가 data를 덮어쓰므로 여기서 q_meas 기준으로 다시 갱신한다.
Eigen::VectorXd ComputeFeedforwardTorqueID(const pinocchio::Model& model, pinocchio::Data& data,
                                         const std::array<const LegContext*, 2>& legs,
                                         const Eigen::VectorXd& q_meas,
                                         const std::array<ContactWrench, 2>& wrench) {
  pinocchio::forwardKinematics(model, data, q_meas);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, q_meas);

  Eigen::VectorXd torq = Eigen::VectorXd::Zero(kNumJoints);
  const Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, q_meas);
  for (int i = 0; i < kNumJoints; ++i) {
    torq[i] = gravity[model.idx_vs[JointId(model, i)]];
  }

  for (int side = 0; side < 2; ++side) {
    const LegContext& leg = *legs[side];
    const ContactWrench& w = wrench[side];
    Eigen::Matrix<double, 6, 1> wrench_vec;
    wrench_vec << w.fx, w.fy, w.fz, w.tx, w.ty, 0.0;

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, leg.foot_frame, pinocchio::LOCAL_WORLD_ALIGNED, J);

    for (int k = 0; k < 6; ++k) {
      // 부호: 로봇이 지면에 가하는 힘은 -w 이므로 관절 토크는 -J^T w (id_check: +부호는 틀림).
      torq[leg.joints[k]] -= J.col(model.idx_vs[JointId(model, leg.joints[k])]).dot(wrench_vec);
    }
  }
  return torq;
}

}  // namespace

void RunKinematicsThread(DataWarehouse& dw, const pinocchio::Model& model, int msgref_id,
                          int msgsen_id, std::atomic<bool>& stop, std::FILE* timing_log) {
  pinocchio::Data data(model);

  const LegContext left_leg{model.getFrameId(kLeftFootFrame), kLeftLegJoints};
  const LegContext right_leg{model.getFrameId(kRightFootFrame), kRightLegJoints};
  const std::array<const LegContext*, 2> legs = {&left_leg, &right_leg};

  // 자유 관절(root, pelvis 링크)의 질량중심: pelvis 프레임 기준 (URDF <inertial><origin>). root 목표의 초기 추정으로 쓴다.
  const Eigen::Vector3d pelvis_com_local = model.inertias[1].lever();
  // root(pelvis 프레임 원점)에서 **전신 COM**까지의 월드 벡터. 처음엔 pelvis 링크 질량중심으로 근사하고, task-space IK를
  // 풀 때마다 그 해 자세의 전신 COM으로 갱신한다(전신 COM은 자세에 따라 pelvis 링크와 x로 2~5cm 달라짐: 다리 폄 1.8 -> 크라우치 4.4).
  Eigen::Vector3d com_off = pelvis_com_local;
  if (std::abs(pelvis_com_local.z() - kPelvisComOffsetZ) > 1e-4 || pelvis_com_local.head<2>().norm() > 1e-4) {
    std::fprintf(stderr,
                 "Kinematics: pelvis COM offset (%.4f %.4f %.4f) != kPelvisComOffsetZ (%.4f) -- 상수 확인 필요\n",
                 pelvis_com_local.x(), pelvis_com_local.y(), pelvis_com_local.z(), kPelvisComOffsetZ);
  }

  Eigen::VectorXd q_ik = pinocchio::neutral(model);  // warm-start
  // 다리를 완전히 편 neutral은 IK 특이점이라, 살짝 굽힌 seed에서 출발.
  for (const int* leg_joints : {kLeftLegJoints.data(), kRightLegJoints.data()}) {
    q_ik[model.idx_qs[JointId(model, leg_joints[0])]] = -0.3;  // hip_pitch
    q_ik[model.idx_qs[JointId(model, leg_joints[3])]] = 0.6;   // knee
    q_ik[model.idx_qs[JointId(model, leg_joints[4])]] = -0.3;  // ankle_pitch
  }
  Eigen::VectorXd prev_targ_pos = Eigen::VectorXd::Zero(kNumJoints);
  bool have_prev_targ = false;
  LoopTimer loop_timer("Kinematics", kKinematicsControlPeriod.count(), std::chrono::seconds(1),
                        timing_log);

  // drop auto 진행 상태 (이 스레드 로컬).
  bool descending = false;
  std::chrono::steady_clock::time_point descent_start;
  double hold_height_m = kWeldHoldHeightM;
  LegOdometry odometry;

  while (!stop.load()) {
    const auto tick_start = std::chrono::steady_clock::now();
    loop_timer.TickStart();

    // ---- 최신 msg_sen 수신 (드레인), DataWarehouse 갱신 ----
    SenMsg sen;
    bool got_sen = false;
    while (msgrcv(msgsen_id, &sen, sizeof(SenMsg) - sizeof(long), 1, IPC_NOWAIT) > 0) {
      got_sen = true;
    }
    Eigen::VectorXd joint_pos_sen(kNumJoints), joint_vel_sen(kNumJoints), joint_torq_sen(kNumJoints);
    Eigen::Vector3d imu_ang, imu_angvel;
    if (got_sen) {
      for (int i = 0; i < kNumJoints; ++i) {
        joint_pos_sen[i] = sen.joint_pos[i];
        joint_vel_sen[i] = sen.joint_vel[i];
        joint_torq_sen[i] = sen.joint_torq[i];
      }
      imu_ang = Eigen::Vector3d(sen.imu_ang[0], sen.imu_ang[1], sen.imu_ang[2]);
      imu_angvel = Eigen::Vector3d(sen.imu_angvel[0], sen.imu_angvel[1], sen.imu_angvel[2]);
      dw.SetJointPosSen(joint_pos_sen);
      dw.SetJointVelSen(joint_vel_sen);
      dw.SetJointTorqSen(joint_torq_sen);
      dw.SetImuAng(imu_ang);
      dw.SetImuAngVel(imu_angvel);
    } else {
      joint_pos_sen = dw.GetJointPosSen();
      joint_vel_sen = dw.GetJointVelSen();
      joint_torq_sen = dw.GetJointTorqSen();
      imu_ang = dw.GetImuAng();
      imu_angvel = dw.GetImuAngVel();
    }

    // ---- MotionGen 목표 (drop 해제 판정 + IK 양쪽에 필요) ----
    const Eigen::Quaterniond torso_quat_targ = dw.GetTorsoQuatTarg();
    const Eigen::Vector3d com_targ = dw.GetComTargWorld();
    const FootTargetWorld foot_targ = dw.GetFootTargWorld();

    // ---- drop auto: 요청 시 시작, 진행 중엔 높이 램프 + 계산된 높이에서 해제 ----
    if (!descending && !dw.GetDropRobot() && dw.GetDropSequenceRequested()) {
      descending = true;
      descent_start = tick_start;
      hold_height_m = kWeldHoldHeightM;
      dw.SetDropSequenceRequested(false);
      dw.SetDropDescending(true);
    }
    if (descending) {
      const double elapsed_s = std::chrono::duration<double>(tick_start - descent_start).count();
      hold_height_m = std::max(kWeldHoldHeightM - kDropDescendRateMps * elapsed_s, kDropMinHeightM);
      // weld가 잡는 것은 pelvis 프레임 원점이므로, 해제 높이는 COM 목표가 아니라 그에 대응하는 프레임 원점 높이 기준.
      const double root_targ_z = (com_targ - com_off).z();
      if (hold_height_m <= root_targ_z + kDropReleaseMarginM) {
        descending = false;
        dw.SetDropDescending(false);
        dw.SetDropRobot(true);
      }
    }
    // ---- FK ----
    const MeasuredState meas = SolveMeasuredStateFK(model, data, legs, joint_pos_sen, imu_ang,
                                                     !dw.GetDropRobot(), hold_height_m, pelvis_com_local, odometry);
    const Eigen::VectorXd& q_meas = meas.q;
    dw.SetFootContact(meas.contact);
    dw.SetFootPosMeasWorld(meas.foot_pos);
    dw.SetComMeasWorld(meas.com);
    dw.SetTorsoPoseMeasWorld(meas.torso_pose_world);

    Eigen::VectorXd joint_targ_pos(kNumJoints);
    if (dw.GetControlMode() == ControlMode::kJointSpace) {
      // 관절제어모드: MotionGen이 목표 관절각을 직접 계산해서 이미
      // DataWarehouse에 써넣었다 (예: WalkReady의 관절공간 등속 이동). IK를
      // 건너뛰고 그 값을 그대로 가져다 쓴다 -- q_ik/SolveLegIk는 실행하지 않음.
      joint_targ_pos = dw.GetJointTargPos();
      // 작업공간 모드로 돌아올 때 IK가 마지막 관절 목표에서 출발하도록 warm-start 동기화
      // (neutral=다리 완전히 폄은 특이점이라 거기서 출발하면 IK가 못 풀린다: tools/ik_check).
      for (int i = 0; i < kNumJoints; ++i) {
        q_ik[model.idx_qs[JointId(model, i)]] = joint_targ_pos[i];
      }
    } else {
      // ---- IK: MotionGen 목표 -> joint_targ_pos ----
      // COM 목표는 **전신 COM**의 위치다. root(pelvis 프레임 원점)는 그 COM에서 root->전신COM 벡터(com_off)만큼 뺀 곳에 둔다
      // (똑바로 서면 원점이 COM 목표보다 약 7.9cm 위, x로는 약 4.4cm 뒤). com_off는 직전 틱 IK 해에서 계산한 값.
      q_ik.segment<3>(0) = com_targ - com_off;
      q_ik[3] = torso_quat_targ.x();
      q_ik[4] = torso_quat_targ.y();
      q_ik[5] = torso_quat_targ.z();
      q_ik[6] = torso_quat_targ.w();

      // MotionGen 발 목표는 발바닥(지면 기준). IK 프레임은 발목이라 sole offset만큼 올려서 전달.
      const Eigen::Vector3d sole_up(0.0, 0.0, kFootSoleOffsetM);
      const pinocchio::SE3 left_target(Eigen::Matrix3d::Identity(), foot_targ.left + sole_up);
      const pinocchio::SE3 right_target(Eigen::Matrix3d::Identity(), foot_targ.right + sole_up);
      SolveLegIk(model, data, left_leg, left_target, q_ik);
      SolveLegIk(model, data, right_leg, right_target, q_ik);
      q_ik[model.idx_qs[JointId(model, kWaistYaw)]] = dw.GetWaistYawTarg();  // 상체 카운터 로테이션 (보행 아니면 0)
      // 이번 해 자세의 전신 COM으로 root->COM 벡터 갱신 (다음 틱 root 목표에 쓰임. 스윙 다리가 움직여 COM이 바뀌면 따라간다).
      com_off = pinocchio::centerOfMass(model, data, q_ik) - q_ik.head<3>();

      for (int i = 0; i < kNumJoints; ++i) {
        joint_targ_pos[i] = q_ik[model.idx_qs[JointId(model, i)]];
      }
      dw.SetJointTargPos(joint_targ_pos);
    }

    // ---- ID. 관절 PD는 Simulate 쪽에서만. ----
    const double kp_joint = dw.GetKpJoint();
    const double kd_joint = dw.GetKdJoint();
    Eigen::VectorXd joint_targ_torq = Eigen::VectorXd::Zero(kNumJoints);
    if (dw.GetControlMode() == ControlMode::kTaskSpace) {
      joint_targ_torq = ComputeFeedforwardTorqueID(model, data, legs, q_meas, dw.GetWbcForceTorque());
    }
    // 관절제어모드: 순수 관절 PD(Simulate)만 사용. IMU-PD/WBC 토크를 얹으면 접지 시 자세를 흔든다.
    dw.SetJointTargTorq(joint_targ_torq);

    // ---- SendRefToSim ----
    RefMsg ref;
    for (int i = 0; i < kNumJoints; ++i) {
      ref.joint_targ_pos[i] = joint_targ_pos[i];
      ref.joint_targ_vel[i] =
          have_prev_targ ? (joint_targ_pos[i] - prev_targ_pos[i]) / 0.001 : 0.0;
      ref.joint_targ_torq[i] = joint_targ_torq[i];
    }
    ref.kp_joint = kp_joint;
    ref.kd_joint = kd_joint;
    ref.drop_robot_sim = dw.GetDropRobot();
    ref.pelvis_hold_height_m = hold_height_m;
    const PushRequest push = dw.GetPushRequest();
    ref.push_fx = push.fx;
    ref.push_fy = push.fy;
    ref.push_dur_s = push.dur;
    ref.push_seq = push.seq;
    msgsnd(msgref_id, &ref, sizeof(RefMsg) - sizeof(long), IPC_NOWAIT);  // 블로킹 금지, 최신값 우선
    prev_targ_pos = joint_targ_pos;
    have_prev_targ = true;

    loop_timer.ComputeDone();
    SleepUntilPrecise(tick_start + kKinematicsControlPeriod);
  }
}

}  // namespace walking_kjw
