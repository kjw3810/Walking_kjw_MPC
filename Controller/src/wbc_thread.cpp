#include "wbc_thread.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include "attitude_mpc.h"
#include "g1_joints.h"
#include "loop_timer.h"
#include "mpc_config.h"
#include "precise_sleep.h"

namespace walking_kjw {
namespace {

constexpr auto kWbcControlPeriod = std::chrono::milliseconds(1);  // 1kHz
constexpr double kGravityMs2 = 9.81;

// 접촉 제약 (보수적). CoP 한계는 발목 프레임 기준 실제 발 접지 범위 안쪽(g1_joints.h: 뒤꿈치 4.5cm, 발끝 11.5cm, 좌우 2.5cm).
constexpr double kFrictionMu = 0.5;

// 입력 가중 R = [Fx, Fy, Fz, τx, τy] (기준 u_ref=[m*a_com, m*g, 0] 대비 편차에 대한 가중).
// Fx/Fy는 병진 동역학이 모델에 없어 기준(계획 가속도) 근처로 묶는다. Fz는 높이 PD 기준 근처 유지.
const Eigen::Matrix<double, 5, 1> kInputWeight = (Eigen::Matrix<double, 5, 1>() << 50.0, 50.0, 0.2, 0.1, 0.1).finished();
constexpr double kTerminalScale = 5.0;

// 발 위치 계획+실측 융합: k=0의 실측-계획 상대위치 오차를 향후 스텝에 감쇠시키며 더한다 (plan 참고).
constexpr double kOffsetDecayPerStep = 0.85;
constexpr double kOffsetClampM = 0.05;

}  // namespace

void RunWholeBodyControlThread(DataWarehouse& dw, const WbcBodyParams& body, std::atomic<bool>& stop,
                                std::FILE* timing_log) {
  AttitudeMpcConfig cfg;
  cfg.dt = kMpcDtS;
  cfg.inertia_xx = body.inertia_xx;
  cfg.inertia_yy = body.inertia_yy;
  cfg.friction_mu = kFrictionMu;
  cfg.cop_heel = kFootHeelM;
  cfg.cop_toe = kFootToeM;
  cfg.cop_half_width = kFootHalfWidthM;
  cfg.fz_max = body.mass_kg * kGravityMs2 * 2;
  AttitudeMpc mpc(cfg);

  const double weight = body.mass_kg * kGravityMs2;
  ContactWrench u_hold{0.0, 0.0, weight, 0.0, 0.0};  // 직전 MPC 해 (처음엔 중력 보상만)
  std::array<double, 2> share = {0.0, 0.0};          // 발별 하중 비중 (좌, 우)
  bool warned = false;
  long tick = 0;
  LoopTimer loop_timer("WBC", kWbcControlPeriod.count(), std::chrono::seconds(1), timing_log);

  while (!stop.load()) {
    const auto tick_start = std::chrono::steady_clock::now();
    loop_timer.TickStart();

    if (!dw.GetWbcFlag()) {
      dw.SetWbcForceTorque({ContactWrench{}, ContactWrench{}});
      u_hold = ContactWrench{0.0, 0.0, weight, 0.0, 0.0};
      share = {0.0, 0.0};
      tick = 0;
      loop_timer.ComputeDone();
      SleepUntilPrecise(tick_start + kWbcControlPeriod);
      continue;
    }

    if (tick++ % kMpcSolvePeriodTicks == 0) {
      const Eigen::Vector3d com_targ = dw.GetComTargWorld();
      const Eigen::Vector3d imu_ang = dw.GetImuAng();
      const Eigen::Vector3d imu_angvel = dw.GetImuAngVel();
      const std::array<bool, 2> contact = dw.GetFootContact();
      const std::array<Eigen::Vector3d, 2> foot_meas = dw.GetFootPosMeasWorld();
      const WalkPreview preview = dw.GetWalkPreview();
      // [kp_roll, kd_roll, kp_pitch, kd_pitch, kp_height, (예비)] -- 앞 4개는 상태 가중의 제곱근
      // (q = gain^2 -> 대략 u = -gain*x 정도의 세기), kp_height는 Fz 높이 PD.
      const Eigen::VectorXd gain = dw.GetKpWbc();

      // COM - 발목 프레임: 슬라이드의 (x,y,z). 현재는 실측, 향후는 계획+실측 오프셋.
      // COM은 **전신 COM**이다 (pelvis 링크 질량중심과 x로 약 4cm 다르므로 pelvis를 쓰면 정적 모멘트 m·g·x가 틀려
      // 오히려 기존 PD보다 나빠진다: pinocchio 정적 평형으로 확인). MotionGen의 COM 계획도 Kinematics IK가 전신 COM에
      // 맞추므로 같은 기준이라 별도 변환 없이 쓴다.
      const Eigen::Vector3d com_meas = dw.GetComMeasWorld();
      const Eigen::Vector3d r_meas[2] = {com_meas - foot_meas[0], com_meas - foot_meas[1]};
      const Eigen::Vector3d sole_up(0.0, 0.0, kFootSoleOffsetM);
      Eigen::Vector3d offset[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

      const double fz_ref =
          std::max(0.0, weight + gain[4] * (com_targ.z() - com_meas.z()));
      std::array<Eigen::Vector3d, kMpcHorizon> r_eff, f_ref;
      std::array<double, 2> share_now = {0.0, 0.0};
      bool no_contact = false;
      for (int k = 0; k < kMpcHorizon; ++k) {
        Eigen::Vector3d r_i[2];
        double w_left;
        if (preview.valid) {
          const PreviewPoint& pt = preview.pts[k];
          const Eigen::Vector3d com(pt.com_xy.x(), pt.com_xy.y(), com_targ.z());
          const Eigen::Vector3d r_plan[2] = {com - (pt.left + sole_up), com - (pt.right + sole_up)};
          if (k == 0) {
            for (int i = 0; i < 2; ++i) {
              offset[i] = (r_meas[i] - r_plan[i]).cwiseMax(-kOffsetClampM).cwiseMin(kOffsetClampM);
            }
          }
          const double decay = std::pow(kOffsetDecayPerStep, k);
          for (int i = 0; i < 2; ++i) r_i[i] = r_plan[i] + decay * offset[i];
          w_left = pt.w_left;
        } else {  // 계획이 없으면 실측 기하 고정 + 접촉 센서 비중
          r_i[0] = r_meas[0];
          r_i[1] = r_meas[1];
          w_left = 0.5;
        }
        if (preview.valid) {
          // 접촉 비중 = 계획 ZMP 기준: ZMP = COM - (h/g)*a (LIPM)를 두 발 y 사이에서 선형 분배 (SSP는 ZMP=지지발 -> 1).
          // 시간 기준 교대(DSP 중간, preview의 w_left)는 COM이 아직 새 지지발 위에 오기 전에 이전 발 하중을 빼서
          // 피드포워드가 실제 하중 분배와 어긋나 넘어졌다 (tools/walk_sim: 1.4초 만에 낙하). ZMP 기준은 안정.
          const PreviewPoint& pt = preview.pts[k];
          const double h = std::max(r_i[0].z(), 0.3);
          const double zmp_y = pt.com_xy.y() - (h / kGravityMs2) * pt.com_acc.y();
          const double denom = pt.left.y() - pt.right.y();
          if (std::abs(denom) > 1e-3) w_left = std::clamp((zmp_y - pt.right.y()) / denom, 0.0, 1.0);
        }
        double wl = w_left, wr = 1.0 - w_left;
        if (k == 0) {  // 현재는 실제 접촉 판정으로 마스킹 (이륙/착지 지연 대응)
          if (!contact[0]) wl = 0.0;
          if (!contact[1]) wr = 0.0;
          const double sum = wl + wr;
          if (sum <= 1e-6) {
            no_contact = true;
          } else {
            wl /= sum;
            wr /= sum;
          }
          share_now = {wl, wr};
        }
        r_eff[k] = wl * r_i[0] + wr * r_i[1];
        // 힘 기준: 수평 = m * 계획 COM 가속도 (LIPM 지면반력), 수직 = m*g + 높이 PD. 계획이 없으면 0.
        f_ref[k] = Eigen::Vector3d(0.0, 0.0, fz_ref);
        if (preview.valid) {
          f_ref[k].x() = body.mass_kg * preview.pts[k].com_acc.x();
          f_ref[k].y() = body.mass_kg * preview.pts[k].com_acc.y();
        }
      }

      if (no_contact) {
        u_hold = ContactWrench{};
        share = {0.0, 0.0};
      } else {
        AttitudeMpcWeights w;
        w.q << gain[0] * gain[0], gain[1] * gain[1], gain[2] * gain[2], gain[3] * gain[3];
        w.r = kInputWeight;
        w.terminal_scale = kTerminalScale;
        const Eigen::Vector4d x0(imu_ang[0], imu_angvel[0], imu_ang[1], imu_angvel[1]);
        ContactWrench u;
        if (mpc.Solve(x0, r_eff, f_ref, w, &u)) {
          u_hold = u;
        } else if (!warned) {
          std::fprintf(stderr, "WBC: MPC solve failed -- holding previous wrench\n");
          warned = true;
        }
        share = share_now;
      }
    }

    std::array<ContactWrench, 2> wrench = {ContactWrench{}, ContactWrench{}};
    for (int i = 0; i < 2; ++i) {
      wrench[i] = ContactWrench{share[i] * u_hold.fx, share[i] * u_hold.fy, share[i] * u_hold.fz,
                                 share[i] * u_hold.tx, share[i] * u_hold.ty};
    }
    dw.SetWbcForceTorque(wrench);

    loop_timer.ComputeDone();
    SleepUntilPrecise(tick_start + kWbcControlPeriod);
  }
}

}  // namespace walking_kjw
