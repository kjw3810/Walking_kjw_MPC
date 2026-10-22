// KinematicsThread (Th1), 1kHz.
// 흐름: FK(순기구학) -> IK(역기구학) -> ID(역동역학) -> SendRefToSim(Simulate로 전송).
// 설계 배경은 Controller/docs/architecture.md 3절 참고.
#pragma once

#include <atomic>
#include <cstdio>

#include <pinocchio/multibody/model.hpp>

#include "data_warehouse.h"

namespace walking_kjw {

// *stop이 true가 될 때까지 계속 돈다. model은 반드시 LoadG1Model()이 만든
// free-flyer(플로팅 베이스) G1 모델이어야 한다. msgref_id/msgsen_id는
// Common/ipc_protocol.h에 정의된 System V 메시지 큐 id.
// timing_log: 초당 1회 출력되는 LoopTimer 리포트를 쓸 파일 (Common/loop_timer.h
//   참고). stdin/stderr을 그대로 쓰지 않고 별도 파일을 넘기는 이유는,
//   CommandInputThread가 stdin으로 사용자 명령을 받는 대화형 프롬프트와 이
//   타이밍 로그가 같은 화면에 섞여 출력되면 명령어를 입력하기 어려워지기
//   때문이다.
void RunKinematicsThread(DataWarehouse& dw, const pinocchio::Model& model,
                          int msgref_id, int msgsen_id, std::atomic<bool>& stop,
                          std::FILE* timing_log = stderr);

}  // namespace walking_kjw
