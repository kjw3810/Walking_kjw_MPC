// MPC 예측 구간 설정. MotionGen(preview 생성)과 WBC(MPC 풀이)가 공유한다.
#pragma once

namespace walking_kjw {

inline constexpr int kMpcHorizon = 10;       // 예측 스텝 수 N
inline constexpr double kMpcDtS = 0.02;      // 예측 스텝 간격 (N*dt = 0.2 s)
inline constexpr int kMpcSolvePeriodTicks = 1;  // 1kHz 틱 기준 재계산 주기 (=1 kHz, 1 ms). WBC의 MPC 풀이와 MotionGen의 미리보기 갱신이 함께 쓴다

}  // namespace walking_kjw
