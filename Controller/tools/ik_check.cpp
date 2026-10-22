// IK 오프라인 검증 (시뮬레이터 불필요). 사용: Controller/build/ik_check
#include <cmath>
#include <cstdio>
#include <random>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "g1_joints.h"
#include "g1_model.h"
#include "leg_ik.h"

using namespace walking_kjw;

namespace {

pinocchio::Model g_model;

void SetRoot(Eigen::VectorXd& q, const Eigen::Vector3d& pos) {
  q.segment<3>(0) = pos;
  q[3] = q[4] = q[5] = 0.0;
  q[6] = 1.0;
}

void SetLeg(Eigen::VectorXd& q, const LegContext& leg, const double v[6]) {
  for (int k = 0; k < 6; ++k) q[g_model.idx_qs[JointId(g_model, leg.joints[k])]] = v[k];
}

pinocchio::SE3 FootPose(pinocchio::Data& d, const Eigen::VectorXd& q, const LegContext& leg) {
  pinocchio::forwardKinematics(g_model, d, q);
  pinocchio::updateFramePlacements(g_model, d);
  return d.oMf[leg.foot_frame];
}

double PoseErr(pinocchio::Data& d, const Eigen::VectorXd& q, const LegContext& leg,
               const pinocchio::SE3& target) {
  return pinocchio::log6(FootPose(d, q, leg).actInv(target)).toVector().norm();
}

double MaxAbsLeg(const Eigen::VectorXd& q, const LegContext& leg) {
  double m = 0;
  for (int k = 0; k < 6; ++k) m = std::max(m, std::abs(q[g_model.idx_qs[JointId(g_model, leg.joints[k])]]));
  return m;
}

double LegJointDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b, const LegContext& leg) {
  double m = 0;
  for (int k = 0; k < 6; ++k) {
    const int i = g_model.idx_qs[JointId(g_model, leg.joints[k])];
    m = std::max(m, std::abs(a[i] - b[i]));
  }
  return m;
}

// 한 다리를 ticks번(틱당 4회 반복) 돌리며 오차 추이를 출력.
void RunTicks(const char* name, pinocchio::Data& d, const LegContext& leg, const pinocchio::SE3& target,
              Eigen::VectorXd q, const Eigen::VectorXd* q_true, int ticks) {
  std::printf("[%s]\n  tick   pose_err   max|q_leg|%s\n", name, q_true ? "   max|q-q_true|" : "");
  int marks[] = {0, 1, 2, 5, 10, 50, 200, 1000};
  int mi = 0;
  for (int t = 0; t <= ticks; ++t) {
    if (t == marks[mi]) {
      std::printf("  %4d  %9.2e  %9.3e", t, PoseErr(d, q, leg, target), MaxAbsLeg(q, leg));
      if (q_true) std::printf("  %9.3e", LegJointDiff(q, *q_true, leg));
      std::printf("\n");
      if (++mi >= 8 || marks[mi] > ticks) break;
    }
    SolveLegIk(g_model, d, leg, target, q);
  }
}

}  // namespace

int main() {
  g_model = LoadG1Model();
  pinocchio::Data d(g_model);
  const LegContext left{g_model.getFrameId(kLeftFootFrame), kLeftLegJoints};
  const Eigen::Vector3d root(0, 0, kNominalStandHeightM);

  // A. 왕복: walkready 관절각 -> FK -> 목표 -> IK가 같은 관절각을 복원하는가.
  {
    Eigen::VectorXd q_true = pinocchio::neutral(g_model);
    SetRoot(q_true, root);
    const double ready[6] = {-0.3, 0, 0, 0.6, -0.3, 0};
    SetLeg(q_true, left, ready);
    const pinocchio::SE3 target = FootPose(d, q_true, left);
    std::printf("walkready 발 위치(월드): %.4f %.4f %.4f\n", target.translation().x(),
                target.translation().y(), target.translation().z());
    Eigen::VectorXd q0 = pinocchio::neutral(g_model);
    SetRoot(q0, root);
    RunTicks("A1 왕복, 시작=neutral(다리 완전히 폄, 특이점)", d, left, target, q0, &q_true, 1000);
    Eigen::VectorXd q1 = q_true;
    const double near[6] = {-0.25, 0, 0, 0.5, -0.25, 0};
    SetLeg(q1, left, near);
    RunTicks("A2 왕복, 시작=근처 자세(틱간 warm-start와 유사)", d, left, target, q1, &q_true, 200);
  }

  // B. 실제 런타임 목표: 발 (0,+-0.09,0) 자세 identity, 루트 z=nominal, 시작=neutral.
  {
    Eigen::VectorXd q = pinocchio::neutral(g_model);
    SetRoot(q, root);
    const pinocchio::SE3 target(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.09, 0.0));
    RunTicks("B1 런타임 목표(발 z=0, 루트 z=nominal), 시작=neutral", d, left, target, q, nullptr, 1000);
    Eigen::VectorXd q2 = pinocchio::neutral(g_model);
    SetRoot(q2, Eigen::Vector3d(0, 0, 0.60));
    RunTicks("B2 루트를 낮춤(z=0.60), 시작=neutral", d, left, target, q2, nullptr, 200);

    // 굽힌 seed(walkready)에서 시작: 특이점 회피 시 실제로 풀리는가.
    const double ready[6] = {-0.3, 0, 0, 0.6, -0.3, 0};
    Eigen::VectorXd q3 = pinocchio::neutral(g_model);
    SetRoot(q3, root);
    SetLeg(q3, left, ready);
    RunTicks("B3 런타임 목표(루트 z=nominal), 시작=walkready", d, left, target, q3, nullptr, 200);
    Eigen::VectorXd q4 = pinocchio::neutral(g_model);
    SetRoot(q4, Eigen::Vector3d(0, 0, 0.60));
    SetLeg(q4, left, ready);
    RunTicks("B4 루트 z=0.60, 시작=walkready", d, left, target, q4, nullptr, 200);
    Eigen::VectorXd q5 = pinocchio::neutral(g_model);
    SetRoot(q5, Eigen::Vector3d(0, 0, 0.693));
    SetLeg(q5, left, ready);
    RunTicks("B5 루트 z=nominal-0.05(nominal-0.05), 시작=walkready", d, left, target, q5, nullptr, 200);
  }

  // C. 무작위 도달 가능 목표 (관절 범위 내), 시작=walkready 근처.
  {
    std::mt19937 rng(1);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    int n = 200, ok = 0, diverged = 0;
    double worst = 0;
    Eigen::VectorXd q_ready = pinocchio::neutral(g_model);
    SetRoot(q_ready, root);
    const double ready[6] = {-0.3, 0, 0, 0.6, -0.3, 0};
    SetLeg(q_ready, left, ready);
    for (int i = 0; i < n; ++i) {
      Eigen::VectorXd qt = q_ready;
      const double v[6] = {U(-0.6, 0.3), U(-0.2, 0.2), U(-0.2, 0.2), U(0.05, 1.2), U(-0.6, 0.3), U(-0.2, 0.2)};
      SetLeg(qt, left, v);
      const pinocchio::SE3 target = FootPose(d, qt, left);
      Eigen::VectorXd q = q_ready;
      for (int t = 0; t < 100; ++t) SolveLegIk(g_model, d, left, target, q);
      const double e = PoseErr(d, q, left, target);
      if (!(e == e) || MaxAbsLeg(q, left) > 10) ++diverged;
      else if (e < 1e-3) ++ok;
      if (e == e) worst = std::max(worst, e);
    }
    std::printf("[C 무작위 %d개, 100틱 후] 수렴(<1e-3): %d, 발산: %d, 최악 오차: %.2e\n", n, ok, diverged, worst);
  }

  // E. 연속 추종: updown(루트 높이 0~-0.15m, 주기 2s)을 두 발 모두, 1kHz 틱당 4회 반복으로 추종.
  {
    Eigen::VectorXd q = pinocchio::neutral(g_model);
    SetRoot(q, root);
    const double ready[6] = {-0.3, 0, 0, 0.6, -0.3, 0};
    const LegContext right{g_model.getFrameId(kRightFootFrame), kRightLegJoints};
    SetLeg(q, left, ready);
    SetLeg(q, right, ready);
    const pinocchio::SE3 tl(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0.09, 0));
    const pinocchio::SE3 tr(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, -0.09, 0));
    for (double pitch : {0.0, 0.1}) {
      double worst = 0, worst_q = 0;
      Eigen::VectorXd qq = q;
      for (int t = 0; t < 10000; ++t) {
        const double h = kNominalStandHeightM - 0.075 * (1.0 - std::cos(2.0 * M_PI * t / 2000.0));
        SetRoot(qq, Eigen::Vector3d(0, 0, h));
        const Eigen::Quaterniond quat(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
        qq[3] = quat.x(); qq[4] = quat.y(); qq[5] = quat.z(); qq[6] = quat.w();
        SolveLegIk(g_model, d, left, tl, qq);
        SolveLegIk(g_model, d, right, tr, qq);
        if (t > 20) {
          worst = std::max({worst, PoseErr(d, qq, left, tl), PoseErr(d, qq, right, tr)});
          worst_q = std::max({worst_q, MaxAbsLeg(qq, left), MaxAbsLeg(qq, right)});
        }
      }
      std::printf("[E updown 추종 10s, 몸통 pitch=%.1f] 최악 pose_err=%.2e, max|q|=%.3f\n", pitch, worst, worst_q);
    }
  }

  // D. 도달 불가능 목표 (발이 루트 아래 1.2m): 발산하지 않고 유한한 값에 머무는가.
  {
    Eigen::VectorXd q = pinocchio::neutral(g_model);
    SetRoot(q, root);
    const pinocchio::SE3 target(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0.09, -0.4));
    RunTicks("D 도달 불가능 목표(발 z=-0.4), 시작=neutral", d, left, target, q, nullptr, 200);
    const double ready[6] = {-0.3, 0, 0, 0.6, -0.3, 0};
    SetLeg(q, left, ready);
    RunTicks("D2 도달 불가능 목표, 시작=walkready (유한하게 머무는가)", d, left, target, q, nullptr, 1000);
  }
  return 0;
}
