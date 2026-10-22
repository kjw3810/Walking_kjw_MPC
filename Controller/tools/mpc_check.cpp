// 자세 MPC 오프라인 검증 (시뮬레이터 불필요). 사용: Controller/build/mpc_check
// 모델: 슬라이드의 몸통(관성 I) + 질량 없는 다리. 같은 연속식으로 1kHz 적분하는 플랜트에 MPC(100Hz)를 걸어
// 외란/기울기 초기값에서 θ->0, 제약 만족, 풀이 시간을 확인한다.
#include <chrono>
#include <cmath>
#include <cstdio>

#include "attitude_mpc.h"

using namespace walking_kjw;

namespace {
int g_fail = 0;
void Check(bool ok, const char* name, const char* fmt = "", double a = 0, double b = 0) {
  std::printf("[%s] %s", ok ? "PASS" : "FAIL", name);
  if (fmt[0]) { std::printf("  ("); std::printf(fmt, a, b); std::printf(")"); }
  std::printf("\n");
  if (!ok) ++g_fail;
}

constexpr double kMass = 35.0, kG = 9.81, kIxx = 1.5, kIyy = 1.5;

struct Result { double max_theta_end, max_cop_ratio_viol, min_fz, max_solve_us, mean_solve_us; bool all_ok; double fz_end, tauy_end; };

// r_fn(t): 접촉점(COM 기준) r = COM - foot. x0 초기 상태.
template <typename RFn>
Result Run(const Eigen::Vector4d& x_init, RFn r_fn, double duration, double kp, double kd) {
  AttitudeMpcConfig cfg;
  cfg.inertia_xx = kIxx; cfg.inertia_yy = kIyy; cfg.fz_max = 2 * kMass * kG;
  AttitudeMpc mpc(cfg);
  AttitudeMpcWeights w;
  w.q << kp * kp, kd * kd, kp * kp, kd * kd;
  w.r << 50.0, 50.0, 0.2, 0.1, 0.1;

  Eigen::Vector4d x = x_init;
  ContactWrench u{0, 0, kMass * kG, 0, 0};
  Result res{0, 0, 1e9, 0, 0, true, 0, 0};
  double solve_sum = 0; int solves = 0;
  const double dt = 0.001;
  for (int k = 0; k < static_cast<int>(duration / dt); ++k) {
    const double t = k * dt;
    if (k % kMpcSolvePeriodTicks == 0) {
      std::array<Eigen::Vector3d, kMpcHorizon> r, fref;
      for (int i = 0; i < kMpcHorizon; ++i) {
        r[i] = r_fn(t + i * cfg.dt);
        fref[i] = Eigen::Vector3d(0, 0, kMass * kG);
      }
      const auto t0 = std::chrono::steady_clock::now();
      ContactWrench un;
      const bool ok = mpc.Solve(x, r, fref, w, &un);
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      solve_sum += us; ++solves; res.max_solve_us = std::max(res.max_solve_us, us);
      if (ok) u = un; else res.all_ok = false;
    }
    // 플랜트 (연속식, 1 kHz 오일러)
    const Eigen::Vector3d rr = r_fn(t);
    const double acc_x = (rr.z() * u.fy - rr.y() * u.fz + u.tx) / kIxx;
    const double acc_y = (rr.x() * u.fz - rr.z() * u.fx + u.ty) / kIyy;
    x[0] += dt * x[1]; x[1] += dt * acc_x;
    x[2] += dt * x[3]; x[3] += dt * acc_y;
    res.min_fz = std::min(res.min_fz, u.fz);
    res.max_cop_ratio_viol = std::max({res.max_cop_ratio_viol, u.ty - 0.045 * u.fz - 5e-2, -u.ty - 0.115 * u.fz - 5e-2, std::abs(u.tx) - 0.025 * u.fz - 5e-2,
                                       std::abs(u.fx) - 0.5 * u.fz - 5e-2, std::abs(u.fy) - 0.5 * u.fz - 5e-2});
    if (k > static_cast<int>(0.8 * duration / dt)) res.max_theta_end = std::max({res.max_theta_end, std::abs(x[0]), std::abs(x[2])});
    res.fz_end = u.fz; res.tauy_end = u.ty;
  }
  res.mean_solve_us = solve_sum / solves;
  return res;
}
}  // namespace

int main() {
  const double kp = 300, kd = 30;
  std::printf("=== 1. 기울기 초기값 (pitch 0.1, roll -0.08), 접촉점 COM 바로 아래 ===\n");
  auto r0 = Run({-0.08, 0.0, 0.1, 0.0}, [](double) { return Eigen::Vector3d(0, 0, 0.7); }, 3.0, kp, kd);
  Check(r0.all_ok, "모든 QP 풀이 성공");
  Check(r0.max_theta_end < 1e-3, "θ -> 0 수렴", "|θ| end=%.2e", r0.max_theta_end);
  Check(r0.max_cop_ratio_viol <= 0, "마찰/CoP 제약 만족", "viol=%.2e", r0.max_cop_ratio_viol);
  Check(r0.min_fz >= 0, "Fz >= 0", "min Fz=%.1f", r0.min_fz);
  std::printf("풀이 시간 mean=%.0f us, max=%.0f us\n", r0.mean_solve_us, r0.max_solve_us);

  std::printf("\n=== 2. COM이 지지발 앞 0.06 m (정적 CoP 이동 필요: τy ≈ -m g x) ===\n");
  auto r1 = Run({0, 0, 0, 0}, [](double) { return Eigen::Vector3d(0.06, 0, 0.7); }, 3.0, kp, kd);
  Check(r1.all_ok && r1.max_theta_end < 1e-3, "θ=0 유지", "|θ| end=%.2e", r1.max_theta_end);
  std::printf("fz_end=%.1f tauy_end=%.2f\n", r1.fz_end, r1.tauy_end);
  Check(std::abs(r1.tauy_end + kMass * kG * 0.06) < 0.5, "τy ≈ -m g x", "τy=%.2f (기대 %.2f)", r1.tauy_end, -kMass * kG * 0.06);
  Check(r1.max_cop_ratio_viol <= 0, "제약 만족");

  std::printf("\n=== 3. 시변 r(t): COM이 발 안쪽 범위에서 sway (x 0.01~0.07, y ±0.02) ===\n");
  auto r2 = Run({0.0, 0.0, 0.05, 0.0}, [](double t) { return Eigen::Vector3d(0.04 + 0.03 * std::sin(2 * M_PI * t / 0.7), 0.02 * std::cos(2 * M_PI * t / 0.7), 0.7); }, 4.0, kp, kd);
  Check(r2.all_ok && r2.max_theta_end < 5e-3, "시변 접촉 기하에서도 θ ~ 0", "|θ| end=%.2e", r2.max_theta_end);
  Check(r2.max_cop_ratio_viol <= 0 && r2.min_fz >= 0, "제약 만족");

  std::printf("\n=== 4. 큰 기울기(제약 활성): pitch 0.5 ===\n");
  auto r3 = Run({0, 0, 0.5, 0}, [](double) { return Eigen::Vector3d(0, 0, 0.7); }, 3.0, kp, kd);
  std::printf("ok=%d viol=%.3e minfz=%.2f theta_end=%.4f\n", r3.all_ok, r3.max_cop_ratio_viol, r3.min_fz, r3.max_theta_end);
  Check(r3.all_ok && r3.max_cop_ratio_viol <= 0 && r3.min_fz >= 0, "제약 활성에서도 풀이 성공/제약 만족", "θ_end=%.3f", r3.max_theta_end);

  std::printf("\n%s (%d fail)\n", g_fail == 0 ? "ALL PASS" : "SOME FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
