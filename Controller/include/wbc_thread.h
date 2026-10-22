// WholeBodyControlThread (Th4), 1kHz.
// 역할: 자세 MPC(슬라이드 "MPC", attitude_mpc.h)로 접촉 wrench [Fx,Fy,Fz,τx,τy]를 계산해
// 접지 중인 발들에 접촉 비중대로 나눠 DataWarehouse에 쓴다. Kinematics(Th1)의 ID가 이를 J^T로 관절 토크로 변환.
// MPC는 kMpcSolvePeriodTicks(=1)틱마다(1kHz, 1ms) 풀고, 사이 틱은 직전 해를 유지한다.
// 설계/근거: Controller/docs/mpc_plan.md.
#pragma once

#include <atomic>
#include <cstdio>

#include "data_warehouse.h"

namespace walking_kjw {

// 로봇 전체 강체 파라미터 (main.cpp가 pinocchio로 계산).
struct WbcBodyParams {
  double mass_kg = 35.0;
  double inertia_xx = 1.5;  // COM 기준, 월드 정렬 관성
  double inertia_yy = 1.5;
};

// timing_log: 초당 1회 출력되는 LoopTimer 리포트를 쓸 파일 (Common/loop_timer.h 참고).
void RunWholeBodyControlThread(DataWarehouse& dw, const WbcBodyParams& body, std::atomic<bool>& stop,
                                std::FILE* timing_log = stderr);

}  // namespace walking_kjw
