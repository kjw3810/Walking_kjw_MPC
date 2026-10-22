// MotionGenThread (Th2), 1kHz.
// 역할: LIPM(선형 역진자 모델) 기반 지지발(support-foot) 중심 COM 궤적 생성 +
// 사인파 형태의 스윙 발 궤적 + (현재는 끔) 포물선 형태 몸통 롤(roll).
// 설계 배경은 Controller/docs/architecture.md 3절 참고.
#pragma once

#include <atomic>
#include <cstdio>

#include "data_warehouse.h"

namespace walking_kjw {

// timing_log: 초당 1회 출력되는 LoopTimer 리포트를 쓸 파일 (Common/loop_timer.h
//   참고). stdin/stderr을 그대로 쓰지 않고 별도 파일을 넘기는 이유는,
//   CommandInputThread가 stdin으로 사용자 명령을 받는 대화형 프롬프트와 이
//   타이밍 로그가 같은 화면에 섞여 출력되면 명령어를 입력하기 어려워지기
//   때문이다.
void RunMotionGenThread(DataWarehouse& dw, std::atomic<bool>& stop, std::FILE* timing_log = stderr);

}  // namespace walking_kjw
