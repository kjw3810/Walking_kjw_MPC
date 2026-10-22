// LIPM 해석해 + 양발지지(DSP) Hermite + cosine 스윙발 + 몸통 기울기 발 위치 보정.
// 설계/수식 근거: Controller/docs/walking_pattern_plan.md.
// header-only, Eigen/DataWarehouse 비의존 -> MotionGenThread와 tools/gait_check가 같은 코드를 쓴다.
//
// 좌표: x 앞, y 왼쪽(월드). 왼발 y=+y0, 오른발 y=-y0, 발 z=0(발바닥 기준).
// 지지발 기준 측면 거리 d는 지지발에서 몸 중심선 쪽으로 잰 값(항상 > 0): y_com = y_지지발 - sgn*d,
// sgn = 왼발 +1 / 오른발 -1.
#pragma once

#include <algorithm>
#include <cmath>

namespace walking_kjw {
namespace gait {

inline constexpr double kGravity = 9.81;

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct GaitParams {
  double h = 0.743;      // COM 높이 (LIPM)
  double T = 0.7;        // 한 스텝 시간 (SSP + DSP)
  double rho = 0.2;      // DSP 비율: Td = rho*T. 0이면 슬라이드 원형
  double y0 = 0.09;      // 발 좌우 거리 절반
  double v0 = 0.0;       // SSP 양 끝에서의 전진 속도
  double h_step = 0.05;  // 스윙 높이
  double s_land = 0.85;  // 스윙 x가 목표에 먼저 도착하는 진행률 (이후는 z만 하강)
  double zmp_dx = 0.0;   // LIPM의 ZMP(지지 기준점)가 지지발(발목 프레임)보다 앞쪽에 있는 거리 = 발 접지 중앙 오프셋
};

// 파생값. 공칭(보정 없음) DSP가 정확히 등속 직선이 되도록 y1, 보폭 S를 정한다 (plan 3절).
struct GaitDerived {
  double omega = 0, Td = 0, Ts = 0, a = 0;
  double x0 = 0;      // SSP 반 구간 전진량 (지지발 기준 -x0 -> +x0)
  double y1 = 0;      // 지지발에서 sway 극단까지 거리
  double y_s = 0;     // SSP 시작/끝에서 지지발~COM 측면 거리
  double ydot_e = 0;  // SSP 끝 횡속도 크기
  double S = 0;       // 보폭 = 2*x0 + v0*Td
};

inline GaitDerived Derive(const GaitParams& p) {
  GaitDerived g;
  g.omega = std::sqrt(kGravity / p.h);
  g.Td = p.rho * p.T;
  g.Ts = p.T - g.Td;
  g.a = g.omega * g.Ts / 2.0;
  g.x0 = p.v0 * std::tanh(g.a) / g.omega;
  g.y1 = 2.0 * p.y0 / (2.0 * std::cosh(g.a) + g.omega * g.Td * std::sinh(g.a));
  g.y_s = g.y1 * std::cosh(g.a);
  g.ydot_e = g.omega * g.y1 * std::sinh(g.a);
  g.S = 2.0 * g.x0 + p.v0 * g.Td;
  return g;
}

// vel(평균 보행 속도, S/T=vel) -> v0.
inline double V0FromVel(double vel, const GaitParams& p) {
  const GaitDerived g = Derive(p);
  return vel * p.T / (2.0 * std::tanh(g.a) / g.omega + g.Td);
}

// SSP 해석해 (지지발 기준). t in [0,Ts], 대칭 형태: 중앙(t=Ts/2)에서 d 최소(=y1), x=0.
struct SspState {
  double x, vx;  // 전진 위치/속도 (지지발 기준)
  double d, vd;  // 측면 거리/속도 (지지발->중심선 방향 양수)
};

inline SspState SspRel(const GaitParams& p, const GaitDerived& g, double t) {
  const double w = g.omega * (t - 0.5 * g.Ts);
  return {p.v0 * std::sinh(w) / (g.omega * std::cosh(g.a)), p.v0 * std::cosh(w) / std::cosh(g.a),
          g.y1 * std::cosh(w), g.omega * g.y1 * std::sinh(w)};
}

// 3차 Hermite (위치+속도 양 끝 일치). t in [0,Td].
struct PosVel {
  double p, v, a;
};

inline PosVel Hermite(double pa, double va, double pb, double vb, double Td, double t) {
  const double u = std::clamp(t / Td, 0.0, 1.0);
  const double u2 = u * u, u3 = u2 * u;
  const double p = (2 * u3 - 3 * u2 + 1) * pa + (u3 - 2 * u2 + u) * Td * va +
                   (-2 * u3 + 3 * u2) * pb + (u3 - u2) * Td * vb;
  const double v = ((6 * u2 - 6 * u) * pa + (3 * u2 - 4 * u + 1) * Td * va +
                    (-6 * u2 + 6 * u) * pb + (3 * u2 - 2 * u) * Td * vb) /
                   Td;
  const double a = ((12 * u - 6) * pa + (6 * u - 4) * Td * va + (-12 * u + 6) * pb + (6 * u - 2) * Td * vb) /
                   (Td * Td);
  return {p, v, a};
}

// 스윙발: x,y는 cosine 보간(s_land에서 도착), z는 (1-cos(2*pi*s))/2 * h_step.
inline Vec3 SwingFoot(double s, double sx, double sy, double tx, double ty, double h_step,
                      double s_land) {
  s = std::clamp(s, 0.0, 1.0);
  const double s_x = std::min(s / s_land, 1.0);
  const double w = 0.5 * (1.0 - std::cos(M_PI * s_x));
  return {sx + (tx - sx) * w, sy + (ty - sy) * w, 0.5 * h_step * (1.0 - std::cos(2.0 * M_PI * s))};
}

// ---- 몸통 기울기 기반 발 위치 보정 (plan 6절) ----
// Δx =  Kθ*pitch + Kω*ω_y,  Δy = -(Kθ*roll + Kω*ω_x). LPF -> 데드밴드 -> 식 -> 제한 -> 변화율 제한.
// 데드밴드는 문턱을 빼는 연속형(문턱에서 값이 튀지 않음).
struct FootCorrectionConfig {
  double fc_angle_hz = 30.0, fc_rate_hz = 15.0;
  double db_angle = 0.02, db_rate = 0.1;
  double dx_min = -0.05, dx_max = 0.12;
  double dy_out_max = 0.06, dy_in_max = 0.03;  // 스윙발 기준 바깥/안쪽
  double min_foot_sep = 0.12;                  // 좌우 발 간격 최소
  double max_rate = 1.0;                       // m/s
};

class FootCorrector {
 public:
  void Reset() { *this = FootCorrector(); }

  // swing_sgn: 스윙발이 왼발이면 +1, 오른발이면 -1. nominal_sep: 보정 전 좌우 발 간격.
  void Update(double roll, double pitch, double wx, double wy, double kp, double kd,
              double swing_sgn, double nominal_sep, double dt) {
    Lpf(roll_, roll, cfg.fc_angle_hz, dt);
    Lpf(pitch_, pitch, cfg.fc_angle_hz, dt);
    Lpf(wx_, wx, cfg.fc_rate_hz, dt);
    Lpf(wy_, wy, cfg.fc_rate_hz, dt);

    const double dx_raw = kp * Deadband(pitch_, cfg.db_angle) + kd * Deadband(wy_, cfg.db_rate);
    const double dy_raw = -(kp * Deadband(roll_, cfg.db_angle) + kd * Deadband(wx_, cfg.db_rate));

    const double dx_cmd = std::clamp(dx_raw, cfg.dx_min, cfg.dx_max);
    const double out_min = std::max(-cfg.dy_in_max, cfg.min_foot_sep - nominal_sep);
    const double dy_cmd = swing_sgn * std::clamp(swing_sgn * dy_raw, out_min, cfg.dy_out_max);

    const double step = cfg.max_rate * dt;
    dx_ += std::clamp(dx_cmd - dx_, -step, step);
    dy_ += std::clamp(dy_cmd - dy_, -step, step);
  }

  double dx() const { return dx_; }
  double dy() const { return dy_; }

  FootCorrectionConfig cfg;

 private:
  static void Lpf(double& state, double in, double fc_hz, double dt) {
    const double alpha = dt / (dt + 1.0 / (2.0 * M_PI * fc_hz));
    state += alpha * (in - state);
  }
  static double Deadband(double v, double th) { return v > th ? v - th : (v < -th ? v + th : 0.0); }

  double roll_ = 0, pitch_ = 0, wx_ = 0, wy_ = 0;
  double dx_ = 0, dy_ = 0;
};

// ---- 외란 적응: 조기 착지 + 안정될 때까지 스텝 주기 단축 ----
// 측정 COM의 캡처포인트 ξ = p + v/ω 가 계획 ξ에서 벗어나면(정적 오차는 느린 기준선으로 제거)
//  1) 시간 진행 배율 rate를 올려 스윙(조기 착지)/DSP/COM 계획을 함께 앞당기고 (스텝 주기 T/rate),
//  2) 스윙발 착지 목표를 ξ 오차만큼 같은 방향으로 옮긴다 (스윙 90%까지 계속 갱신, 변화율 제한).
// ξ 오차가 데드밴드 아래로 돌아오면 hold_tau 후 rate가 1로 천천히 복귀한다.
struct StepAdaptConfig {
  double xi_db = 0.015, xi_full = 0.06;  // ξ 오차 데드밴드 / 최대 긴급도에 도달하는 크기 [m]
  double rate_max = 1.5;                  // 시간 진행 배율 상한 (스텝 주기 0.7 s -> 약 0.47 s)
  double rate_up = 8.0, rate_down = 1.0;  // rate 변화율 한계 [1/s]
  double hold_tau = 0.5;                  // 긴급도 유지 시간: 오차가 사라진 뒤 이 시간에 걸쳐 감소
  double k_xi = 1.0;                      // 착지 이동 = k_xi * (ξ 오차 - 데드밴드)
  double vel_fc_hz = 10.0;                // COM 속도 추정 LPF
  double base_tau_s = 1.5, base_tau_warm_s = 0.2, warm_s = 2.0;  // 정적 오차 기준선 시정수
  double enable_after_s = 1.5;            // 보행(SSP) 시작 후 이 시간 뒤부터 동작
  double lock_progress = 0.9;             // 스윙 진행률이 이 값을 넘으면 착지 목표 고정
  double foot_rate = 2.0;                 // 착지 목표 변화율 한계 [m/s]
  double dx_min = -0.08, dx_max = 0.15;
  double dy_out_max = 0.10, dy_in_max = 0.03, min_foot_sep = 0.12;
};

// ---- 보행 패턴 생성기 (plan 5절 상태머신) ----
// Init(Hermite) -> SSP_L -> DSP -> SSP_R -> DSP -> ... 첫 지지발은 왼발(오른발 먼저 스윙).
// vel은 DSP 시작(및 Init 시작)에 한 번 읽어 다음 스텝 v0로 확정 (SSP 중 파라미터 변경 없음).

struct WalkInput {
  double vel = 0.0;  // 평균 보행 속도 명령
  double roll = 0, pitch = 0, wx = 0, wy = 0;  // IMU (보정용)
  double kp_foot = 0.0, kd_foot = 0.0;         // Kθ, Kω (0이면 보정 꺼짐)
  double waist_k = 0.0;     // 상체 카운터 로테이션 게인 [rad/m] (0이면 꺼짐)
  double waist_max = 0.15;  // 허리 yaw 상한 [rad]
  bool adapt = false;       // 외란 적응(조기 착지/스텝 주기 단축) 사용
  bool meas_valid = false;  // cx, cy 유효
  double cx = 0, cy = 0;    // 측정 전신 COM 월드 위치 (속도는 내부에서 미분)
};

enum class Phase { kInit = 0, kSsp = 1, kDsp = 2 };

struct WalkOutput {
  double com_x = 0, com_y = 0, com_vx = 0, com_vy = 0;
  double com_ax = 0, com_ay = 0;  // 계획 COM 가속도 (접촉 수평력 m*a의 기준)
  Vec3 left, right;  // 발바닥 목표 (월드)
  Phase phase = Phase::kInit;
  int support = 0;  // 0 왼발, 1 오른발
  double swing_progress = 0.0;
  double dx = 0, dy = 0;  // 적용 중인 발 위치 보정 (스윙 중 잠금 후 값)
  double w_left = 0.5;    // 접촉 하중 비중 (왼발). 오른발 = 1 - w_left. MPC 접촉 교대용.
  double waist_yaw = 0.0;  // 허리 yaw 목표 [rad]: 오른발이 앞이면 −, 왼발이 앞이면 + (상체 카운터 로테이션)
};

// 지지발 교대 비중: DSP(및 Init)의 정확히 중간에서 새 지지발로 넘긴다. width>0이면 중간을 중심으로
// smoothstep으로 width 초에 걸쳐 넘겨 wrench 계단을 없앤다 (0이면 순간 교대). 반환: 새 지지발 비중 0->1.
inline constexpr double kHandoverWidthS = 0.04;
inline double HandoverWeight(double t, double Td) {
  const double width = std::min(kHandoverWidthS, Td);
  if (width <= 1e-9) return t >= 0.5 * Td ? 1.0 : 0.0;
  const double x = std::clamp((t - (0.5 * Td - 0.5 * width)) / width, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

class WalkPatternGenerator {
 public:
  static constexpr double kInitTimeS = 0.5;
  static constexpr double kMaxV0StepMps = 0.05;  // 한 스텝당 v0 변화 한계
  static constexpr double kLockProgress = 0.7;   // s_lock: 이후 발 보정 Δ 고정
  static constexpr double kWaistMaxRateRadS = 1.0;  // 허리 yaw 목표 변화율 한계

  explicit WalkPatternGenerator(const GaitParams& base = GaitParams()) : base_(base) { Reset(); }

  void Reset() {
    initialized_ = false;
    cur_ = base_;
    cur_.v0 = 0.0;
    corr_.Reset();
    waist_yaw_ = 0.0;
    fx_[0] = fx_[1] = 0.0;
    fy_[0] = base_.y0;
    fy_[1] = -base_.y0;
    com_x_ = base_.zmp_dx;  // WalkReady와 같은 발 중앙 위에서 시작 (walk 진입 시 COM x 점프 없음)
    com_y_ = 0.0;
    com_vx_ = com_vy_ = 0.0;
    have_meas_ = false;
    meas_vx_ = meas_vy_ = base_ex_ = base_ey_ = 0.0;
    urgency_ = 0.0;
    rate_ = 1.0;
    xi_dx_ = xi_dy_ = 0.0;
    adapt_time_ = 0.0;
  }

  // 현재 상태에서 보정 입력 없이(보정값 고정) 앞으로 진행했을 때의 출력을 dt_sample 간격으로 n개 채운다.
  // out[0]은 호출 시점 직후 틱의 출력. MPC가 향후 발 위치/접촉 비중을 미리 보는 데 쓴다. 이 객체는 불변.
  void Preview(const WalkInput& in, double dt, double dt_sample, int n, WalkOutput* out) const {
    WalkPatternGenerator copy = *this;
    copy.freeze_corrector_ = true;
    const int every = std::max(1, static_cast<int>(std::lround(dt_sample / dt)));
    for (int k = 0; k < n; ++k) {
      for (int i = 0; i < (k == 0 ? 0 : every - 1); ++i) copy.Tick(in, dt);
      out[k] = copy.Tick(in, dt);
    }
  }

  FootCorrector& corrector() { return corr_; }
  StepAdaptConfig adapt_cfg;
  double time_rate() const { return rate_; }
  double urgency() const { return urgency_; }
  const GaitParams& params() const { return cur_; }
  const GaitDerived& derived() const { return g_; }

  WalkOutput Tick(const WalkInput& in, double dt) {
    if (!initialized_) {
      initialized_ = true;
      LatchNext(in.vel);
      cur_ = next_;
      g_ = gn_;
      BeginInit();
    }

    // 위상 전이 (남은 시간은 이월)
    for (int guard = 0; guard < 4; ++guard) {
      if (phase_ == Phase::kInit && t_ >= kInitTimeS) {
        t_ -= kInitTimeS;
        BeginSsp();
      } else if (phase_ == Phase::kSsp && t_ >= g_.Ts) {
        t_ -= g_.Ts;
        EndSspBeginDsp(in.vel);
      } else if (phase_ == Phase::kDsp && t_ >= g_.Td) {
        t_ -= g_.Td;
        cur_ = next_;
        g_ = gn_;
        BeginSsp();
      } else {
        break;
      }
    }

    const int swing = 1 - support_;
    const double swing_sgn = swing == 0 ? 1.0 : -1.0;
    const double ty_nom = swing == 0 ? base_.y0 : -base_.y0;
    if (!freeze_corrector_) {
      corr_.Update(in.roll, in.pitch, in.wx, in.wy, in.kp_foot, in.kd_foot, swing_sgn,
                   std::abs(ty_nom - fy_[support_]), dt);
    }

    WalkOutput out;
    out.phase = phase_;
    out.support = support_;
    Vec3 foot[2] = {{fx_[0], fy_[0], 0.0}, {fx_[1], fy_[1], 0.0}};

    if (phase_ == Phase::kSsp) {
      const double s = t_ / g_.Ts;
      const SspState rel = SspRel(cur_, g_, t_);
      const double sgn = support_ == 0 ? 1.0 : -1.0;
      out.com_x = fx_[support_] + cur_.zmp_dx + rel.x;  // 기준점 = 발 중앙(발목 + zmp_dx)
      out.com_y = fy_[support_] - sgn * rel.d;
      out.com_vx = rel.vx;
      out.com_vy = -sgn * rel.vd;
      out.com_ax = g_.omega * g_.omega * rel.x;  // LIPM: 지지발(ZMP)에서 멀어지는 방향
      out.com_ay = -sgn * g_.omega * g_.omega * rel.d;

      if (s <= kLockProgress) {  // 잠금 전: 보정을 따라간다
        lock_dx_ = corr_.dx();
        lock_dy_ = corr_.dy();
      }
      tx_ = tx_nom_ + lock_dx_ + xi_dx_;
      ty_ = ty_nom_ + lock_dy_ + xi_dy_;
      foot[swing] = SwingFoot(s, sx_, sy_, tx_, ty_, base_.h_step, base_.s_land);
      out.swing_progress = s;
    } else {
      const double Td = phase_ == Phase::kInit ? kInitTimeS : g_.Td;
      const PosVel px = Hermite(pa_x_, va_x_, pb_x_, vb_x_, Td, t_);
      const PosVel py = Hermite(pa_y_, va_y_, pb_y_, vb_y_, Td, t_);
      out.com_x = px.p;
      out.com_y = py.p;
      out.com_vx = px.v;
      out.com_vy = py.v;
      out.com_ax = px.a;
      out.com_ay = py.a;
    }
    if (phase_ == Phase::kSsp) {
      out.w_left = support_ == 0 ? 1.0 : 0.0;
    } else if (phase_ == Phase::kDsp) {
      const double w_new = HandoverWeight(t_, g_.Td);  // support_는 이미 새 지지발
      out.w_left = support_ == 0 ? w_new : 1.0 - w_new;
    } else {  // Init: 양발 균등에서 시작해 중간에 첫 지지발(왼발)로 넘어간다
      out.w_left = 0.5 + 0.5 * HandoverWeight(t_, kInitTimeS);
    }
    out.left = foot[0];
    out.right = foot[1];

    // 상체 카운터 로테이션: 두 발 x 차이(정상 걸음에서 -S <-> +S로 매끄럽게 진동)에 비례해 허리를 반대로 비튼다.
    // 제자리 걸음(S=0)에서는 0. 상한 clamp + 변화율 제한(kWaistMaxRateRadS).
    {
      const double lead = foot[1].x - foot[0].x;  // 오른발이 앞서면 +
      const double target = std::clamp(-in.waist_k * lead, -in.waist_max, in.waist_max);
      const double step = kWaistMaxRateRadS * dt;
      waist_yaw_ += std::clamp(target - waist_yaw_, -step, step);
      out.waist_yaw = waist_yaw_;
    }
    out.dx = lock_dx_;
    out.dy = lock_dy_;
    com_x_ = out.com_x;
    com_y_ = out.com_y;

    // 시간 진행 배율: 계획 전체가 rate배 빨리 진행 (속도 x rate, 가속도 x rate^2)
    out.com_vx *= rate_;
    out.com_vy *= rate_;
    out.com_ax *= rate_ * rate_;
    out.com_ay *= rate_ * rate_;
    if (!freeze_corrector_) UpdateAdapt(in, out, dt);

    t_ += dt * rate_;
    return out;
  }

  void UpdateAdapt(const WalkInput& in, const WalkOutput& out, double dt) {
    const StepAdaptConfig& c = adapt_cfg;
    // 측정 COM 속도 (미분 + LPF)
    if (in.meas_valid) {
      if (have_meas_) {
        const double a = dt / (dt + 1.0 / (2.0 * M_PI * c.vel_fc_hz));
        meas_vx_ += a * ((in.cx - prev_cx_) / dt - meas_vx_);
        meas_vy_ += a * ((in.cy - prev_cy_) / dt - meas_vy_);
      }
      prev_cx_ = in.cx;
      prev_cy_ = in.cy;
      have_meas_ = true;
    }
    if (phase_ != Phase::kInit) adapt_time_ += dt;
    const bool active = in.adapt && have_meas_ && phase_ != Phase::kInit && adapt_time_ >= c.enable_after_s;

    // ξ 오차 (정적 오차 기준선 제거)
    const double w = g_.omega;
    double ex = 0.0, ey = 0.0;
    if (have_meas_) {
      ex = (in.cx + meas_vx_ / w) - (out.com_x + out.com_vx / w);
      ey = (in.cy + meas_vy_ / w) - (out.com_y + out.com_vy / w);
      if (urgency_ < 0.05) {  // 외란 중에는 기준선을 갱신하지 않는다
        const double tau = adapt_time_ < c.warm_s ? c.base_tau_warm_s : c.base_tau_s;
        const double a = dt / (dt + tau);
        base_ex_ += a * (ex - base_ex_);
        base_ey_ += a * (ey - base_ey_);
      }
      ex -= base_ex_;
      ey -= base_ey_;
    }

    const double mag = std::hypot(ex, ey);
    const double u_inst = active ? std::clamp((mag - c.xi_db) / (c.xi_full - c.xi_db), 0.0, 1.0) : 0.0;
    urgency_ = std::max(u_inst, urgency_ - dt / c.hold_tau);
    const double rate_targ = 1.0 + (c.rate_max - 1.0) * urgency_;
    rate_ += std::clamp(rate_targ - rate_, -c.rate_down * dt, c.rate_up * dt);

    // 착지 목표 이동 (ξ 오차 방향). 스윙 후반(lock_progress~)에는 고정.
    const bool locked = phase_ == Phase::kSsp && t_ / g_.Ts > c.lock_progress;
    if (!locked) {
      const int swing = 1 - support_;
      const double swing_sgn = swing == 0 ? 1.0 : -1.0;
      const double ty_nom = swing == 0 ? base_.y0 : -base_.y0;
      double dx_cmd = 0.0, dy_cmd = 0.0;
      if (active) {
        auto db = [&](double v) { return v > c.xi_db ? v - c.xi_db : (v < -c.xi_db ? v + c.xi_db : 0.0); };
        const double dx_raw = c.k_xi * db(ex), dy_raw = c.k_xi * db(ey);
        dx_cmd = std::clamp(dx_raw, c.dx_min, c.dx_max);
        const double out_min = std::max(-c.dy_in_max, c.min_foot_sep - std::abs(ty_nom - fy_[support_]));
        dy_cmd = swing_sgn * std::clamp(swing_sgn * dy_raw, out_min, c.dy_out_max);
      }
      const double step = c.foot_rate * dt;
      xi_dx_ += std::clamp(dx_cmd - xi_dx_, -step, step);
      xi_dy_ += std::clamp(dy_cmd - xi_dy_, -step, step);
    }
  }

 private:
  void LatchNext(double vel) {
    const double target = V0FromVel(vel, base_);
    next_ = base_;
    next_.v0 = std::clamp(target, cur_.v0 - kMaxV0StepMps, cur_.v0 + kMaxV0StepMps);
    gn_ = Derive(next_);
  }

  // 지지발 sup의 SSP 시작 상태(월드).
  void SspStartWorld(int sup, const GaitParams& p, const GaitDerived& g, double* x, double* y,
                     double* vx, double* vy) const {
    const SspState r = SspRel(p, g, 0.0);
    const double sgn = sup == 0 ? 1.0 : -1.0;
    *x = fx_[sup] + p.zmp_dx + r.x;
    *y = fy_[sup] - sgn * r.d;
    *vx = r.vx;
    *vy = -sgn * r.vd;
  }

  void BeginInit() {
    phase_ = Phase::kInit;
    t_ = 0.0;
    support_ = 0;
    pa_x_ = com_x_;
    pa_y_ = com_y_;
    va_x_ = va_y_ = 0.0;
    SspStartWorld(support_, cur_, g_, &pb_x_, &pb_y_, &vb_x_, &vb_y_);
  }

  void BeginSsp() {
    phase_ = Phase::kSsp;
    const int swing = 1 - support_;
    sx_ = fx_[swing];
    sy_ = fy_[swing];
    tx_nom_ = fx_[support_] + g_.S;
    ty_nom_ = swing == 0 ? base_.y0 : -base_.y0;
    lock_dx_ = corr_.dx();
    lock_dy_ = corr_.dy();
  }

  void EndSspBeginDsp(double vel) {
    const int swing = 1 - support_;
    // 스윙발 착지 (보정이 잠금된 목표).
    fx_[swing] = tx_;
    fy_[swing] = ty_;

    // DSP 시작 상태 = 직전 SSP 끝 상태 (이전 지지발 기준).
    const SspState end = SspRel(cur_, g_, g_.Ts);
    const double sgn = support_ == 0 ? 1.0 : -1.0;
    pa_x_ = fx_[support_] + cur_.zmp_dx + end.x;
    pa_y_ = fy_[support_] - sgn * end.d;
    va_x_ = end.vx;
    va_y_ = -sgn * end.vd;

    LatchNext(vel);
    support_ = swing;
    SspStartWorld(support_, next_, gn_, &pb_x_, &pb_y_, &vb_x_, &vb_y_);
    phase_ = Phase::kDsp;
  }

  GaitParams base_;
  GaitParams cur_, next_;
  GaitDerived g_, gn_;
  FootCorrector corr_;

  double waist_yaw_ = 0.0;  // 허리 yaw 목표 (변화율 제한 상태)
  bool freeze_corrector_ = false;
  bool initialized_ = false;
  Phase phase_ = Phase::kInit;
  double t_ = 0.0;
  int support_ = 0;

  double fx_[2] = {0, 0}, fy_[2] = {0, 0};  // 발 월드 위치 (착지 위치)
  double sx_ = 0, sy_ = 0;                  // 스윙 시작
  double tx_nom_ = 0, ty_nom_ = 0;          // 공칭 착지 목표
  double tx_ = 0, ty_ = 0;                  // 보정 적용 목표
  double lock_dx_ = 0, lock_dy_ = 0;

  double pa_x_ = 0, va_x_ = 0, pb_x_ = 0, vb_x_ = 0;
  double pa_y_ = 0, va_y_ = 0, pb_y_ = 0, vb_y_ = 0;
  double com_x_ = 0, com_y_ = 0, com_vx_ = 0, com_vy_ = 0;

  // 외란 적응 상태
  bool have_meas_ = false;
  double prev_cx_ = 0, prev_cy_ = 0, meas_vx_ = 0, meas_vy_ = 0;
  double base_ex_ = 0, base_ey_ = 0;
  double urgency_ = 0, rate_ = 1.0;
  double xi_dx_ = 0, xi_dy_ = 0;
  double adapt_time_ = 0;
};

}  // namespace gait
}  // namespace walking_kjw
