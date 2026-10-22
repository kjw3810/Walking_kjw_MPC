// 보행 패턴 오프라인 검증 (시뮬레이터 불필요). 사용: Controller/build/gait_check [out.csv]
// plan: Controller/docs/walking_pattern_plan.md 7~8절.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <tuple>
#include <vector>

#include "lipm_gait.h"

using namespace walking_kjw::gait;

namespace {

int g_fail = 0;

void Check(bool ok, const char* name, const char* fmt = "", double a = 0, double b = 0) {
  std::printf("[%s] %s", ok ? "PASS" : "FAIL", name);
  if (fmt[0]) {
    std::printf("  (");
    std::printf(fmt, a, b);
    std::printf(")");
  }
  std::printf("\n");
  if (!ok) ++g_fail;
}

constexpr double kDt = 0.001;

struct Sample {
  double t;
  WalkOutput o;
};

// 1ms 틱으로 duration_s 동안 생성. vel_fn(t)로 속도 명령, pitch_fn(t, prev_out)로 인위 pitch 입력.
template <typename VelFn, typename PitchFn>
std::vector<Sample> Run(WalkPatternGenerator& gen, double duration_s, VelFn vel_fn,
                        PitchFn pitch_fn, double kp = 0.0, double kd = 0.0) {
  std::vector<Sample> v;
  WalkOutput prev;
  for (int k = 0; k < static_cast<int>(duration_s / kDt); ++k) {
    const double t = k * kDt;
    WalkInput in;
    in.vel = vel_fn(t);
    in.pitch = pitch_fn(t, prev);
    in.kp_foot = kp;
    in.kd_foot = kd;
    prev = gen.Tick(in, kDt);
    v.push_back({t, prev});
  }
  return v;
}

double FootCoord(const Vec3& f, int axis) { return axis == 0 ? f.x : (axis == 1 ? f.y : f.z); }

// 연속성: COM 속도 점프, 위치-속도 일관성, 발 속도 상한. (max 값을 반환)
void ContinuityChecks(const std::vector<Sample>& s, const char* tag, size_t from = 0) {
  double max_dv = 0, max_pv_err = 0, max_foot_speed = 0, max_acc = 0;
  for (size_t k = from + 1; k < s.size(); ++k) {
    const auto& a = s[k - 1].o;
    const auto& b = s[k].o;
    const double dv = std::hypot(b.com_vx - a.com_vx, b.com_vy - a.com_vy);
    max_dv = std::max(max_dv, dv);
    max_acc = std::max(max_acc, dv / kDt);
    const double ex = (b.com_x - a.com_x) / kDt - b.com_vx;
    const double ey = (b.com_y - a.com_y) / kDt - b.com_vy;
    max_pv_err = std::max(max_pv_err, std::hypot(ex, ey));
    for (int ax = 0; ax < 3; ++ax) {
      max_foot_speed = std::max(max_foot_speed, std::abs(FootCoord(b.left, ax) - FootCoord(a.left, ax)) / kDt);
      max_foot_speed = std::max(max_foot_speed, std::abs(FootCoord(b.right, ax) - FootCoord(a.right, ax)) / kDt);
    }
  }
  char name[96];
  std::snprintf(name, sizeof name, "%s: COM 속도 연속 (틱당 |dv|<0.01 m/s)", tag);
  Check(max_dv < 0.01, name, "max|dv|=%.5f, max acc=%.2f m/s^2", max_dv, max_acc);
  std::snprintf(name, sizeof name, "%s: COM 위치-속도 일관", tag);
  Check(max_pv_err < 0.02, name, "max err=%.5f m/s", max_pv_err);
  std::snprintf(name, sizeof name, "%s: 발 위치 연속 (순간이동 없음, <2 m/s)", tag);
  Check(max_foot_speed < 2.0, name, "max foot speed=%.3f m/s", max_foot_speed);
}

struct Landing {
  int swing;
  double x, y;
  double t;
};

std::vector<Landing> Landings(const std::vector<Sample>& s) {
  std::vector<Landing> L;
  for (size_t k = 1; k < s.size(); ++k) {
    if (s[k - 1].o.phase == Phase::kSsp && s[k].o.phase == Phase::kDsp) {
      const int swing = 1 - s[k - 1].o.support;
      const Vec3& f = swing == 0 ? s[k].o.left : s[k].o.right;
      L.push_back({swing, f.x, f.y, s[k].t});
    }
  }
  return L;
}

}  // namespace

int main(int argc, char** argv) {
  std::printf("=== 1. 파생값 (plan 3절 손계산: T=.7 rho=.2 v0=.25) ===\n");
  GaitParams hand;
  hand.v0 = 0.25;
  const GaitDerived hg = Derive(hand);
  std::printf("omega=%.4f a=%.4f x0=%.4f S=%.4f y1=%.4f y_s=%.4f ydot_e=%.4f sway(2*y2)=%.4f\n",
              hg.omega, hg.a, hg.x0, hg.S, hg.y1, hg.y_s, hg.ydot_e, 2 * (hand.y0 - hg.y1));
  Check(std::abs(hg.omega - 3.634) < 0.01, "omega ~ 3.634", "%.4f", hg.omega);
  Check(std::abs(hg.x0 - 0.053) < 0.003 && std::abs(hg.S - 0.141) < 0.003, "x0~0.053, S~0.141",
        "x0=%.4f S=%.4f", hg.x0, hg.S);
  Check(std::abs(hg.y1 - 0.048) < 0.003 && std::abs(hg.y_s - 0.075) < 0.003, "y1~0.048, y_s~0.075",
        "y1=%.4f y_s=%.4f", hg.y1, hg.y_s);
  Check(std::abs(2 * (hand.y0 - hg.y_s) - hg.ydot_e * hg.Td) < 1e-9, "DSP 이동거리 = ydot_e*Td (등속 직선)",
        "%.3e", 2 * (hand.y0 - hg.y_s) - hg.ydot_e * hg.Td);

  std::printf("\n=== 2. 슬라이드 3 (Stage 2) 식 vs 유도식: ẏ(T/2)=0 잔차 (rho=0) ===\n");
  {
    GaitParams p0;
    p0.rho = 0;
    const GaitDerived g0 = Derive(p0);
    const double A = g0.a, y0 = p0.y0;
    const double a_slide = -std::pow(std::sinh(A), 2) / std::cosh(A) * y0;
    const double a_derived = -y0 * std::tanh(A);
    // y(τ)=y0*cosh(ωτ)+a*sinh(ωτ) 의 τ=T/2 속도 / ω
    const double res_slide = a_slide * std::cosh(A) + y0 * std::sinh(A);
    const double res_derived = a_derived * std::cosh(A) + y0 * std::sinh(A);
    std::printf("slide a=%.5f residual=%.5f | derived a=%.5f residual=%.2e\n", a_slide, res_slide,
                a_derived, res_derived);
    Check(std::abs(res_derived) < 1e-12, "유도식은 ẏ(T/2)=0 만족");
    Check(std::abs(res_slide) > 1e-3, "슬라이드 식은 불만족 (plan 2.3의 지적 확인)");
    // SSP 대칭 형태의 주기성: d(0)=d(Ts), 극단 y1 = d(Ts/2)
    const SspState b = SspRel(p0, g0, 0), e = SspRel(p0, g0, g0.Ts), m = SspRel(p0, g0, 0.5 * g0.Ts);
    Check(std::abs(b.d - e.d) < 1e-12 && std::abs(b.vd + e.vd) < 1e-12 && std::abs(m.vd) < 1e-12 &&
              std::abs(m.d - g0.y1) < 1e-12,
          "SSP 대칭: d(0)=d(Ts), 중앙 속도 0, 중앙 d=y1");
    Check(std::abs(e.d - y0) < 1e-12, "rho=0: SSP 끝 d = y0 (발 간격 절반, 슬라이드 원형)", "%.6f", e.d);
  }

  std::printf("\n=== 3. 정상 보행 vel=0.25, 30초 ===\n");
  {
    WalkPatternGenerator gen;
    auto s = Run(gen, 30.0, [](double) { return 0.25; }, [](double, const WalkOutput&) { return 0.0; });
    ContinuityChecks(s, "walk");

    const auto L = Landings(s);
    const GaitDerived g = gen.derived();
    // 정상 상태(후반)의 보폭 = S
    double max_step_err = 0;
    for (size_t i = L.size() / 2 + 1; i < L.size(); ++i) {
      const double step = L[i].x - L[i - 1].x;  // 서로 다른 발: 지지발 대비 보폭이 아니라 발 간 x 차이
      max_step_err = std::max(max_step_err, std::abs(step - g.S));
    }
    Check(L.size() > 30 && max_step_err < 1e-6, "정상 상태 착지 간격 = S", "S=%.4f, max err=%.2e", g.S,
          max_step_err);

    // 착지: 발 z=0, 스윙 최고 높이 = h_step, 착지 직전 z속도
    double zmax = 0, z_land_err = 0;
    for (const auto& k : s) {
      zmax = std::max({zmax, k.o.left.z, k.o.right.z});
    }
    for (size_t k = 1; k < s.size(); ++k) {
      if (s[k - 1].o.phase == Phase::kSsp && s[k].o.phase == Phase::kDsp) {
        const int sw = 1 - s[k - 1].o.support;
        z_land_err = std::max(z_land_err, std::abs(sw == 0 ? s[k].o.left.z : s[k].o.right.z));
      }
    }
    Check(std::abs(zmax - 0.05) < 1e-3, "스윙 최고 높이 = h_step", "%.5f", zmax);
    Check(z_land_err < 1e-9, "착지 시 z = 0", "%.2e", z_land_err);

    // 착지/이륙 속도 0, 끌림 위험(z가 아주 작은 구간): 한 스윙을 따로 뽑아서 점검
    {
      const double Ts = g.Ts;
      const Vec3 f05 = SwingFoot(0.05, 0, 0, 0.14, 0, 0.05, 0.85);
      const Vec3 f95 = SwingFoot(0.95, 0, 0, 0.14, 0, 0.05, 0.85);
      const Vec3 f1 = SwingFoot(1.0, 0, 0, 0.14, 0, 0.05, 0.85);
      const Vec3 f1m = SwingFoot(1.0 - kDt / Ts, 0, 0, 0.14, 0, 0.05, 0.85);
      std::printf("z(s=0.05)=%.4f m, z(s=0.95)=%.4f m, 착지 직전 zdot=%.4f m/s\n", f05.z, f95.z,
                  (f1.z - f1m.z) / kDt);
      Check(std::abs((f1.z - f1m.z) / kDt) < 0.01, "착지 직전 z속도 ~ 0");
    }

    // DSP 구간 COM이 정상 상태에서 등속 직선 (공칭 y1/S 조건)
    double max_dsp_vdev = 0;
    for (size_t k = s.size() / 2; k < s.size(); ++k) {
      if (s[k].o.phase != Phase::kDsp) continue;
      // DSP 내부에서 속도 변화
      const auto& a = s[k - 1].o;
      if (a.phase == Phase::kDsp)
        max_dsp_vdev = std::max(max_dsp_vdev, std::hypot(s[k].o.com_vx - a.com_vx, s[k].o.com_vy - a.com_vy));
    }
    Check(max_dsp_vdev < 1e-6, "정상 상태 DSP는 등속 직선", "max |dv|/tick=%.2e", max_dsp_vdev);

    double ymin = 1e9, ymax = -1e9;
    for (size_t k = s.size() / 2; k < s.size(); ++k) {
      ymin = std::min(ymin, s[k].o.com_y);
      ymax = std::max(ymax, s[k].o.com_y);
    }
    std::printf("정상 상태 COM y 범위 [%.4f, %.4f], sway 폭=%.4f m, 보폭 S=%.4f, v0=%.4f\n", ymin, ymax,
                ymax - ymin, g.S, gen.params().v0);

    if (argc > 1) {
      if (FILE* f = std::fopen(argv[1], "w")) {
        std::fprintf(f, "t,phase,support,com_x,com_y,com_vx,com_vy,lx,ly,lz,rx,ry,rz,dx,dy\n");
        for (const auto& k : s)
          std::fprintf(f, "%.3f,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", k.t,
                       static_cast<int>(k.o.phase), k.o.support, k.o.com_x, k.o.com_y, k.o.com_vx,
                       k.o.com_vy, k.o.left.x, k.o.left.y, k.o.left.z, k.o.right.x, k.o.right.y,
                       k.o.right.z, k.o.dx, k.o.dy);
        std::fclose(f);
        std::printf("CSV 저장: %s\n", argv[1]);
      }
    }
  }

  std::printf("\n=== 4. vel=0 제자리 걸음 ===\n");
  {
    WalkPatternGenerator gen;
    auto s = Run(gen, 10.0, [](double) { return 0.0; }, [](double, const WalkOutput&) { return 0.0; });
    ContinuityChecks(s, "in-place");
    double xmax = 0;
    for (const auto& k : s) xmax = std::max({xmax, std::abs(k.o.left.x), std::abs(k.o.right.x), std::abs(k.o.com_x)});
    Check(xmax < 1e-9, "제자리 걸음: x 이동 없음", "%.2e", xmax);
  }

  std::printf("\n=== 5. 속도 명령 변화 (0.25 -> 0.1 -> 0.3) ===\n");
  {
    WalkPatternGenerator gen;
    auto s = Run(
        gen, 30.0, [](double t) { return t < 10 ? 0.25 : (t < 20 ? 0.1 : 0.3); },
        [](double, const WalkOutput&) { return 0.0; });
    ContinuityChecks(s, "vel change");
  }

  std::printf("\n=== 6. rho=0 (슬라이드 원형, DSP 없음, 정상 상태) ===\n");
  {
    GaitParams p;
    p.rho = 0.0;
    WalkPatternGenerator gen(p);
    auto s = Run(gen, 30.0, [](double) { return 0.1; }, [](double, const WalkOutput&) { return 0.0; });
    // DSP가 없으면 v0 변화(x0 변화)를 흡수할 구간이 없어 스텝 경계에서 COM x가 점프한다 (슬라이드 원형의
    // 한계). v0가 수렴한 뒤(정상 상태)만 연속성을 검사한다.
    ContinuityChecks(s, "rho=0 steady", 15000);
  }

  std::printf("\n=== 7. 발 위치 보정 ===\n");
  {
    // 부호/제한/데드밴드/변화율
    auto settle = [](double roll, double pitch, double wx, double wy, double kp, double kd, double sgn) {
      FootCorrector c;
      double max_rate = 0, prev_dx = 0, prev_dy = 0;
      for (int i = 0; i < 2000; ++i) {
        c.Update(roll, pitch, wx, wy, kp, kd, sgn, 0.18, kDt);
        max_rate = std::max({max_rate, std::abs(c.dx() - prev_dx) / kDt, std::abs(c.dy() - prev_dy) / kDt});
        prev_dx = c.dx();
        prev_dy = c.dy();
      }
      return std::array<double, 3>{c.dx(), c.dy(), max_rate};
    };
    const double kp = 0.37, kd = 0.10;
    auto r1 = settle(0, 0.1, 0, 0, kp, kd, +1);
    Check(r1[0] > 0 && std::abs(r1[0] - kp * (0.1 - 0.02)) < 1e-6, "앞으로 기울면(pitch>0) dx>0", "dx=%.4f (기대 %.4f)", r1[0], kp * 0.08);
    auto r2 = settle(0, 1.0, 0, 0, kp, kd, +1);
    Check(std::abs(r2[0] - 0.12) < 1e-9, "dx 상한 +0.12 m", "%.4f", r2[0]);
    auto r3 = settle(0, -1.0, 0, 0, kp, kd, +1);
    Check(std::abs(r3[0] + 0.05) < 1e-9, "dx 하한 -0.05 m", "%.4f", r3[0]);
    auto r4 = settle(0, 0.01, 0, 0.05, kp, kd, +1);
    Check(std::abs(r4[0]) < 1e-12, "데드밴드 (1° 미만, 0.1rad/s 미만) -> 0", "%.2e", r4[0]);
    auto r5 = settle(0.1, 0, 0, 0, kp, kd, -1);  // roll>0 오른쪽 기울기, 오른발 스윙 -> 오른쪽(-y)
    Check(r5[1] < 0, "오른쪽으로 기울면(roll>0) 오른발 dy<0 (바깥)", "dy=%.4f", r5[1]);
    auto r6 = settle(1.0, 0, 0, 0, kp, kd, -1);
    Check(std::abs(r6[1] + 0.06) < 1e-9, "바깥쪽 상한 0.06 m", "%.4f", r6[1]);
    auto r7 = settle(1.0, 0, 0, 0, kp, kd, +1);  // 왼발 스윙인데 오른쪽 기움 -> 안쪽(-y) 제한 0.03
    Check(std::abs(r7[1] + 0.03) < 1e-9, "안쪽 상한 0.03 m", "%.4f", r7[1]);
    Check(r1[2] <= 1.0 + 1e-9 && r2[2] <= 1.0 + 1e-9, "변화율 <= 1 m/s", "max rate=%.3f", std::max(r1[2], r2[2]));
    auto r8 = settle(0, 0.1, 0, 0, 0.0, 0.0, +1);
    Check(r8[0] == 0 && r8[1] == 0, "게인 0이면 보정 꺼짐");

    // 생성기 통합: 항상 앞으로 기운 상태 -> 착지가 공칭보다 앞, 잠금 후 입력은 착지에 영향 없음
    WalkPatternGenerator g0, g1, g2;
    auto none = [](double, const WalkOutput&) { return 0.0; };
    auto s0 = Run(g0, 12.0, [](double) { return 0.25; }, none);
    auto s1 = Run(g1, 12.0, [](double) { return 0.25; }, [](double, const WalkOutput&) { return 0.1; }, kp, kd);
    const auto L0 = Landings(s0), L1 = Landings(s1);
    Check(L1.size() > 6 && L1[6].x > L0[6].x + 0.01, "pitch>0 지속 시 착지가 공칭보다 앞", "nominal=%.4f corrected=%.4f", L0[6].x, L1[6].x);

    // 잠금: 6번째 스텝 SSP의 진행률 0.75 이후에만 pitch 펄스 -> 그 스텝 착지 x는 펄스 없는 경우와 동일
    auto pulse = [&](double, const WalkOutput& prev) {
      static int steps = 0;
      static Phase last = Phase::kInit;
      if (last == Phase::kSsp && prev.phase == Phase::kDsp) ++steps;
      last = prev.phase;
      return (steps == 5 && prev.phase == Phase::kSsp && prev.swing_progress > 0.75) ? 0.3 : 0.0;
    };
    auto s2 = Run(g2, 12.0, [](double) { return 0.25; }, pulse, kp, kd);
    const auto L2 = Landings(s2);
    Check(L2.size() > 6 && std::abs(L2[5].x - L0[5].x) < 1e-12 && std::abs(L2[5].y - L0[5].y) < 1e-12,
          "s>0.7 잠금: 이후 입력은 해당 스텝 착지에 영향 없음", "dx=%.2e", L2[5].x - L0[5].x);
    ContinuityChecks(s1, "corrected walk");
  }

  std::printf("\n=== 8. MPC용 접촉 비중(w_left)과 Preview ===\n");
  {
    WalkPatternGenerator gen;
    auto s = Run(gen, 12.0, [](double) { return 0.25; }, [](double, const WalkOutput&) { return 0.0; });
    // 비중 연속성, DSP 중간 교대, SSP 지지발 1
    double max_dw = 0, ssp_err = 0, mid_err = 0;
    for (size_t k = 1; k < s.size(); ++k) {
      max_dw = std::max(max_dw, std::abs(s[k].o.w_left - s[k - 1].o.w_left));
      if (s[k].o.phase == Phase::kSsp)
        ssp_err = std::max(ssp_err, std::abs(s[k].o.w_left - (s[k].o.support == 0 ? 1.0 : 0.0)));
    }
    // 정상 상태 DSP 중간 시각의 w_left는 0.5 (대칭 교대)
    const double Td = gen.derived().Td;
    int checked = 0;
    for (size_t k = s.size() / 2; k < s.size(); ++k) {
      if (s[k].o.phase != Phase::kDsp || s[k - 1].o.phase != Phase::kSsp) continue;
      const size_t mid = k + static_cast<size_t>(0.5 * Td / kDt + 0.5);
      if (mid < s.size()) { mid_err = std::max(mid_err, std::abs(s[mid].o.w_left - 0.5)); ++checked; }
    }
    Check(max_dw < 0.05, "접촉 비중 연속 (틱당 변화 작음)", "max dw/tick=%.4f", max_dw);
    Check(ssp_err < 1e-12, "SSP에서는 지지발 비중 1");
    Check(checked > 5 && mid_err < 0.02, "DSP 정확히 중간에서 비중 0.5 (교대 시점)", "max err=%.4f, n=%.0f", mid_err, double(checked));

    // Preview: 이후 실제 틱의 출력과 일치 (보정 입력 고정이므로 kp=0이면 동일)
    WalkPatternGenerator g;
    WalkInput in; in.vel = 0.25;
    for (int i = 0; i < 3000; ++i) g.Tick(in, kDt);
    WalkOutput pv[10];
    g.Preview(in, kDt, 0.02, 10, pv);
    double max_err = 0;
    WalkOutput actual;
    int step = 0;
    for (int i = 1; i <= 181; ++i) {
      actual = g.Tick(in, kDt);
      if (i == 1 + 20 * step && step < 10) {
        max_err = std::max({max_err, std::abs(actual.com_x - pv[step].com_x), std::abs(actual.com_y - pv[step].com_y),
                            std::abs(actual.right.x - pv[step].right.x), std::abs(actual.left.z - pv[step].left.z),
                            std::abs(actual.w_left - pv[step].w_left)});
        ++step;
      }
    }
    Check(step == 10 && max_err < 1e-12, "Preview == 실제 진행 (20ms 간격 10점)", "n=%.0f max err=%.2e", double(step), max_err);
  }

  std::printf("\n=== 9. 상체 카운터 로테이션 (허리 yaw) ===\n");
  {
    auto run_waist = [&](double vel, double k, double kmax) {
      WalkPatternGenerator g;
      std::vector<double> w;
      double max_rate = 0, prev = 0;
      std::vector<double> lead;
      for (int i = 0; i < 20000; ++i) {
        WalkInput in;
        in.vel = vel;
        in.waist_k = k;
        in.waist_max = kmax;
        const WalkOutput o = g.Tick(in, kDt);
        w.push_back(o.waist_yaw);
        lead.push_back(o.right.x - o.left.x);
        if (i > 0) max_rate = std::max(max_rate, std::abs(o.waist_yaw - prev) / kDt);
        prev = o.waist_yaw;
      }
      return std::make_tuple(w, lead, max_rate);
    };
    auto [w0, l0, r0] = run_waist(0.0, 0.35, 0.15);
    double max0 = 0;
    for (double v : w0) max0 = std::max(max0, std::abs(v));
    Check(max0 < 1e-9, "제자리 걸음(S=0)에서는 허리 yaw 0", "max|yaw|=%.2e", max0);

    auto [w1, l1, r1] = run_waist(0.2, 0.35, 0.15);
    double max1 = 0, min1 = 0;
    for (size_t i = 10000; i < w1.size(); ++i) { max1 = std::max(max1, w1[i]); min1 = std::min(min1, w1[i]); }
    Check(max1 > 0.04 && min1 < -0.04 && max1 <= 0.15 + 1e-9 && min1 >= -0.15 - 1e-9, "속도 0.2: 허리 yaw가 좌우로 진동하고 상한 이내", "범위 [%.3f, %.3f] rad", min1, max1);
    Check(r1 <= 1.0 + 1e-6, "허리 yaw 변화율 <= 1 rad/s", "max rate=%.3f rad/s", r1);
    // 오른발이 앞서면 허리는 반대(음수)
    double sign_ok = 0, n = 0;
    for (size_t i = 10000; i < w1.size(); ++i) if (std::abs(l1[i]) > 0.1) { ++n; if (w1[i] * l1[i] < 0) ++sign_ok; }
    Check(n > 100 && sign_ok / n > 0.95, "오른발이 앞서면 허리 yaw는 반대 부호(카운터 로테이션)", "일치 %.0f%%", 100.0 * sign_ok / n);

    auto [w2, l2, r2] = run_waist(0.2, 5.0, 0.08);
    double max2 = 0;
    for (double v : w2) max2 = std::max(max2, std::abs(v));
    Check(max2 <= 0.08 + 1e-9, "큰 게인에서도 상한 clamp", "max=%.3f", max2);
    auto [w3, l3, r3] = run_waist(0.2, 0.0, 0.15);
    double max3 = 0;
    for (double v : w3) max3 = std::max(max3, std::abs(v));
    Check(max3 == 0.0, "게인 0이면 꺼짐");
  }

  std::printf("\n%s (%d fail)\n", g_fail == 0 ? "ALL PASS" : "SOME FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
