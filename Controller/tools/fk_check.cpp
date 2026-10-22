// FK/오도메트리 오프라인 검증 (시뮬레이터 GUI 불필요). 사용: Controller/build/fk_check
//  1. Pinocchio(URDF) FK == MuJoCo(g1.xml) FK 인가 (관절 순서, 쿼터니언 순서, 프레임 정의)
//  2. IMU 오일러 왕복 + MuJoCo 센서 쿼터니언 == 실제 pelvis 자세
//  3. 발바닥 offset (MuJoCo 발 geom 최저점 vs 발목 프레임)
//  4. LegOdometry: 발 고정 걸음 시나리오에서 pelvis 추정 vs 정답
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <random>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "g1_joints.h"
#include "g1_model.h"
#include "leg_ik.h"
#include "leg_odometry.h"
#include "quat_euler.h"

using namespace walking_kjw;

namespace {

Eigen::VectorXd BuildQ(const pinocchio::Model& model, const Eigen::VectorXd& joints,
                       const Eigen::Vector3d& pos, const Eigen::Vector3d& rpy) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  q.segment<3>(0) = pos;
  double w, x, y, z;
  QuatFromEuler(rpy[0], rpy[1], rpy[2], &w, &x, &y, &z);
  q[3] = x; q[4] = y; q[5] = z; q[6] = w;
  for (int i = 0; i < kNumJoints; ++i) q[model.idx_qs[JointId(model, i)]] = joints[i];
  return q;
}

}  // namespace

int main() {
  const pinocchio::Model model = LoadG1Model();
  pinocchio::Data data(model);

  char err[1000] = "";
  mjModel* m = mj_loadXML(MUJOCO_G1_XML, nullptr, err, sizeof(err));
  if (!m) { std::printf("MuJoCo 로드 실패: %s\n", err); return 1; }
  mjData* d = mj_makeData(m);

  int qadr[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) {
    const int j = mj_name2id(m, mjOBJ_JOINT, G1JointNames()[i]);
    qadr[i] = m->jnt_qposadr[j];
  }
  const int lb = mj_name2id(m, mjOBJ_BODY, "left_ankle_roll_link");
  const int rb = mj_name2id(m, mjOBJ_BODY, "right_ankle_roll_link");
  const pinocchio::FrameIndex lf = model.getFrameId(kLeftFootFrame), rf = model.getFrameId(kRightFootFrame);

  // ---- 1. Pinocchio FK vs MuJoCo FK ----
  {
    std::mt19937 rng(7);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    double max_pos = 0, max_rot = 0, max_com = 0;
    for (int n = 0; n < 1000; ++n) {
      Eigen::VectorXd joints(kNumJoints);
      for (int i = 0; i < kNumJoints; ++i) {
        const int qi = model.idx_qs[JointId(model, i)];
        joints[i] = U(std::max(model.lowerPositionLimit[qi], -2.0), std::min(model.upperPositionLimit[qi], 2.0));
      }
      const Eigen::Vector3d pos(U(-1, 1), U(-1, 1), U(0.3, 1.2));
      const Eigen::Vector3d rpy(U(-0.5, 0.5), U(-0.5, 0.5), U(-3, 3));
      const Eigen::VectorXd q = BuildQ(model, joints, pos, rpy);

      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      double w, x, y, z;
      QuatFromEuler(rpy[0], rpy[1], rpy[2], &w, &x, &y, &z);
      mju_zero(d->qpos, m->nq);
      d->qpos[0] = pos[0]; d->qpos[1] = pos[1]; d->qpos[2] = pos[2];
      d->qpos[3] = w; d->qpos[4] = x; d->qpos[5] = y; d->qpos[6] = z;  // MuJoCo: wxyz
      for (int i = 0; i < kNumJoints; ++i) d->qpos[qadr[i]] = joints[i];
      mj_kinematics(m, d);

      for (int side = 0; side < 2; ++side) {
        const int b = side == 0 ? lb : rb;
        const pinocchio::SE3& P = data.oMf[side == 0 ? lf : rf];
        const Eigen::Vector3d pm(d->xpos[3 * b], d->xpos[3 * b + 1], d->xpos[3 * b + 2]);
        const Eigen::Quaterniond qm(d->xquat[4 * b], d->xquat[4 * b + 1], d->xquat[4 * b + 2], d->xquat[4 * b + 3]);
        max_pos = std::max(max_pos, (P.translation() - pm).norm());
        max_rot = std::max(max_rot, Eigen::AngleAxisd(P.rotation().transpose() * qm.toRotationMatrix()).angle());
      }
    }
    std::printf("[1 FK 일치 (Pinocchio URDF vs MuJoCo g1.xml), 무작위 1000자세, 양발]\n"
                "  최대 위치 오차 = %.2e m, 최대 자세 오차 = %.2e rad\n", max_pos, max_rot);
    (void)max_com;
  }

  // ---- 2. IMU 오일러 왕복 + 센서 쿼터니언 ----
  {
    std::mt19937 rng(3);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    double max_rt = 0, max_sensor = 0;
    const int qsid = mj_name2id(m, mjOBJ_SENSOR, "imu-pelvis-orientation");
    const int sadr = qsid >= 0 ? m->sensor_adr[qsid] : -1;
    for (int n = 0; n < 1000; ++n) {
      const Eigen::Vector3d rpy(U(-1.2, 1.2), U(-1.2, 1.2), U(-3, 3));
      double w, x, y, z, r, p, yw;
      QuatFromEuler(rpy[0], rpy[1], rpy[2], &w, &x, &y, &z);
      EulerFromQuat(w, x, y, z, &r, &p, &yw);
      max_rt = std::max({max_rt, std::abs(r - rpy[0]), std::abs(p - rpy[1]), std::abs(std::remainder(yw - rpy[2], 2 * M_PI))});
      if (sadr >= 0) {
        mju_zero(d->qpos, m->nq);
        d->qpos[2] = 0.8;
        d->qpos[3] = w; d->qpos[4] = x; d->qpos[5] = y; d->qpos[6] = z;
        mj_forward(m, d);
        const double dot = std::abs(w * d->sensordata[sadr] + x * d->sensordata[sadr + 1] +
                                    y * d->sensordata[sadr + 2] + z * d->sensordata[sadr + 3]);
        max_sensor = std::max(max_sensor, 2 * std::acos(std::min(1.0, dot)));
      }
    }
    std::printf("[2 IMU] 오일러 왕복 최대 오차 = %.2e rad, MuJoCo imu 센서 쿼터니언 vs pelvis 자세 = %.2e rad%s\n",
                max_rt, max_sensor, sadr < 0 ? " (센서 없음)" : "");
  }

  // ---- 3. 발바닥 offset ----
  {
    mju_zero(d->qpos, m->nq);
    d->qpos[2] = 1.0;
    d->qpos[3] = 1;
    mj_kinematics(m, d);
    double lowest = 1e9;
    for (int g = 0; g < m->ngeom; ++g) {
      if (m->geom_bodyid[g] != lb || m->geom_type[g] != mjGEOM_SPHERE) continue;
      lowest = std::min(lowest, d->geom_xpos[3 * g + 2] - m->geom_size[3 * g]);
    }
    std::printf("[3 발바닥] 발목 프레임 z - 발 geom 최저점 z = %.4f m (코드 상수 kFootSoleOffsetM = %.4f)\n",
                d->xpos[3 * lb + 2] - lowest, kFootSoleOffsetM);
  }

  // ---- 4. 오도메트리 시나리오: 발은 월드에 고정/스윙, pelvis는 이동 ----
  {
    const LegContext L{model.getFrameId(kLeftFootFrame), kLeftLegJoints};
    const LegContext R{model.getFrameId(kRightFootFrame), kRightLegJoints};
    Eigen::VectorXd q = pinocchio::neutral(model);
    const double ready[6] = {-0.3, 0, 0, 0.6, -0.3, 0};
    for (const LegContext* leg : {&L, &R})
      for (int k = 0; k < 6; ++k) q[model.idx_qs[JointId(model, leg->joints[k])]] = ready[k];

    auto smooth = [](double f) { f = std::clamp(f, 0.0, 1.0); return 0.5 * (1 - std::cos(M_PI * f)); };
    const double h = kNominalStandHeightM;
    // 단계: 0=양발 지지(pelvis x 0->0.03), 1=왼발 지지(오른발 0->0.12 스윙, pelvis ->0.09),
    //       2=양발 지지(pelvis ->0.06+... ), 3=오른발 지지(왼발 0->0.24 스윙)
    const int N = 800;
    double lx = 0, rx = 0;  // 발바닥 월드 x
    double lz = 0, rz = 0;
    double px = 0;
    LegOdometry odo;
    double max_xy = 0, max_z = 0, end_xy = 0;
    int flips = 0;
    std::array<bool, 2> prev_contact = {true, true};
    for (int phase = 0; phase < 4; ++phase) {
      for (int t = 0; t < N; ++t) {
        const double f = double(t) / N;
        if (phase == 0) px = 0.03 * smooth(f);
        if (phase == 1) { px = 0.03 + 0.06 * smooth(f); rx = 0.12 * smooth(f); rz = 0.05 * std::sin(M_PI * smooth(f)); }
        if (phase == 2) { px = 0.09 + 0.03 * smooth(f); }
        if (phase == 3) { px = 0.12 + 0.06 * smooth(f); lx = 0.24 * smooth(f); lz = 0.05 * std::sin(M_PI * smooth(f)); }
        const Eigen::Vector3d rpy(0.0, 0.05 * std::sin(2 * M_PI * f), 0.0);
        const Eigen::Vector3d pos(px, 0.0, h);
        double w, x, y, z;
        QuatFromEuler(rpy[0], rpy[1], rpy[2], &w, &x, &y, &z);
        q.segment<3>(0) = pos; q[3] = x; q[4] = y; q[5] = z; q[6] = w;
        const pinocchio::SE3 tl(Eigen::Matrix3d::Identity(), Eigen::Vector3d(lx, 0.09, lz + kFootSoleOffsetM));
        const pinocchio::SE3 tr(Eigen::Matrix3d::Identity(), Eigen::Vector3d(rx, -0.09, rz + kFootSoleOffsetM));
        for (int it = 0; it < 3; ++it) { SolveLegIk(model, data, L, tl, q); SolveLegIk(model, data, R, tr, q); }

        // 컨트롤러가 보는 것: 관절각 + IMU 오일러 -> pelvis 원점 기준 FK -> 오도메트리
        Eigen::VectorXd joints(kNumJoints);
        for (int i = 0; i < kNumJoints; ++i) joints[i] = q[model.idx_qs[JointId(model, i)]];
        const Eigen::VectorXd qm = BuildQ(model, joints, Eigen::Vector3d::Zero(), rpy);
        pinocchio::forwardKinematics(model, data, qm);
        pinocchio::updateFramePlacements(model, data);
        std::array<bool, 2> contact;
        const Eigen::Vector3d est = odo.Update({data.oMf[L.foot_frame].translation(), data.oMf[R.foot_frame].translation()}, &contact);
        if (contact != prev_contact) ++flips;
        prev_contact = contact;

        const double exy = std::hypot(est.x() - (px - 0.0), est.y() - 0.0);  // pelvis y 정답 = 0, 시작 x = 0
        max_xy = std::max(max_xy, exy);
        max_z = std::max(max_z, std::abs(est.z() - h));
        end_xy = exy;
      }
    }
    std::printf("[4 오도메트리, 걸음 시나리오(양발-왼발지지-양발-오른발지지), pelvis x 0->0.18m, 스윙 발 높이 5cm]\n"
                "  XY 최대 오차 = %.4f m (종료 시 %.4f m), Z 최대 오차 = %.4f m, 접촉 플래그 변화 횟수 = %d\n",
                max_xy, end_xy, max_z, flips);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
