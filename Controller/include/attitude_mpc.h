// 몸통 자세 MPC (슬라이드 "MPC": 질량 M, 관성 I인 몸통 + 질량 없는 다리, QP form -> OSQP).
//
// 상태 x = [θx, θ̇x, θy, θ̇y] (몸통 roll/pitch와 각속도), 입력 u = [Fx, Fy, Fz, τx, τy]
// (지면이 로봇에 가하는 접촉 wrench, 발 프레임 원점 기준, 월드 정렬).
// 접촉점이 COM 기준 (x,y,z) = COM - 접촉점 일 때 (슬라이드 연속식):
//   Iy θ̈y = x Fz - z Fx + τy
//   Ix θ̈x = z Fy - y Fz + τx
// 이산화(오일러): θ_{k+1} = θ_k + Δt θ̇_k, θ̇_{k+1} = θ̇_k + Δt/I (...)
// ※ 슬라이드 이산 행렬의 θ̇y 행 Fz 계수는 -x 로 적혀 있으나 위 연속식(와 물리)은 +x 라서 연속식을 따른다.
// 중력/코리올리는 모델에서 제외하고 힘 기준값 f_ref[k] = [Fx,Fy,Fz]에 중력과 계획 COM 가속도를 넣는다:
// Fx,Fy 기준 = m*COM가속도(LIPM 보행의 수평 지면반력), Fz 기준 = m*g(+높이 PD). 토크 기준은 0 (ZMP=발목).
// 비용은 u를 u_ref=[f_ref,0,0]로 당긴다 -> 계획 궤적을 따를 때 자세를 흔들지 않고, MPC는 자세 오차만 보정.
// (x,y,z)는 스텝마다 다르다(발 위치 계획) -> B_k 가 시변. 접촉점이 SSP/DSP 교대에 따라 바뀌는 것도
// r[k]로 표현 (호출 측이 접촉 비중으로 섞은 유효 접촉점을 넘긴다).
//
// 제약 (스텝마다): 0 <= Fz <= fz_max, |Fx|,|Fy| <= μ Fz (마찰 피라미드),
// -toe*Fz <= τy <= heel*Fz, |τx| <= wid*Fz (CoP가 발 안; 발목 기준이라 앞뒤가 비대칭).
#pragma once

#include <array>

#include <Eigen/Dense>

#include "data_warehouse.h"
#include "mpc_config.h"

namespace walking_kjw {

struct AttitudeMpcConfig {
  double dt = kMpcDtS;
  double inertia_xx = 1.0;  // 로봇 전체 관성 (COM 기준, 월드 정렬)
  double inertia_yy = 1.0;
  double friction_mu = 0.5;
  double cop_heel = 0.045;        // CoP 뒤쪽 한계 (발목 프레임 기준 m): τy <= heel*Fz
  double cop_toe = 0.115;         // CoP 앞쪽 한계: τy >= -toe*Fz   (τy = -x_cop*Fz)
  double cop_half_width = 0.025;  // τx 한계 (m) = CoP 좌우 허용
  double fz_max = 1000.0;
};

struct AttitudeMpcWeights {
  Eigen::Vector4d q = Eigen::Vector4d::Ones();  // 상태 가중 diag [θx, θ̇x, θy, θ̇y]
  Eigen::Matrix<double, 5, 1> r = Eigen::Matrix<double, 5, 1>::Ones();  // 입력(u-u_ref) 가중
  double terminal_scale = 5.0;                  // 마지막 상태 가중 = q * scale
};

class AttitudeMpc {
 public:
  explicit AttitudeMpc(const AttitudeMpcConfig& cfg);
  ~AttitudeMpc();
  AttitudeMpc(const AttitudeMpc&) = delete;
  AttitudeMpc& operator=(const AttitudeMpc&) = delete;

  // x0=[θx, θ̇x, θy, θ̇y]. r[k] = COM - 유효 접촉점 (k=0..N-1, k=0이 현재). f_ref[k]: 힘 기준 [Fx,Fy,Fz].
  // 성공 시 true, *u0 = 첫 스텝 입력. 실패 시 false (*u0 불변).
  bool Solve(const Eigen::Vector4d& x0, const std::array<Eigen::Vector3d, kMpcHorizon>& r,
             const std::array<Eigen::Vector3d, kMpcHorizon>& f_ref, const AttitudeMpcWeights& w,
             ContactWrench* u0);

  int last_iterations() const { return last_iter_; }

 private:
  struct Impl;
  Impl* impl_;
  int last_iter_ = 0;
};

}  // namespace walking_kjw
