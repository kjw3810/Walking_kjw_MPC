#include "command_input_thread.h"
#include "data_warehouse.h"
#include "g1_model.h"
#include "ipc_protocol.h"
#include "kinematics_thread.h"
#include "logging_thread.h"
#include "motion_gen_thread.h"
#include "wbc_thread.h"

#include <pinocchio/algorithm/center-of-mass.hpp> //— 무게중심 계산 함수(computeTotalMass) 쓰려고
#include <pinocchio/algorithm/centroidal.hpp>      // ccrba: 전체 관성(COM 기준)
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <thread>

#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <sched.h>

namespace {

std::atomic<bool> g_stop{false};

void HandleSigint(int) {
  g_stop.store(true);
}

// 스레드를 코어 하나에 고정. 실측상 효과는 미미했지만 유지 (자세한 내용: 코드 해설 문서).
void PinThreadToCore(std::thread& t, unsigned core_id) {
  if (core_id >= std::thread::hardware_concurrency()) return;
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  int rc = pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);
  if (rc != 0) {
    std::fprintf(stderr, "main: pthread_setaffinity_np(core %u) failed: rc=%d\n", core_id, rc);
  }
}

// SCHED_FIFO로 승격 (CFS 스케줄링 지연으로 인한 스파이크 제거, 실측 근거: 코드 해설 문서).
// CAP_SYS_NICE 또는 /etc/security/limits.conf의 rtprio 설정 필요, 없으면 EPERM.
void SetThreadRealtime(std::thread& t, int priority) {
  sched_param param{};
  param.sched_priority = priority;
  int rc = pthread_setschedparam(t.native_handle(), SCHED_FIFO, &param);
  if (rc != 0) {
    std::fprintf(stderr,
                 "main: pthread_setschedparam(SCHED_FIFO, prio=%d) failed: %s -- thread stays "
                 "SCHED_OTHER. Needs CAP_SYS_NICE or an rtprio limit in "
                 "/etc/security/limits.conf (e.g. '%s - rtprio 50'), then a fresh login.\n",
                 priority, std::strerror(rc), std::getenv("USER") ? std::getenv("USER") : "<user>");
  }
}

}  // namespace

int main() {
  std::signal(SIGINT, HandleSigint);

  const pinocchio::Model g1_model = walking_kjw::LoadG1Model();
  const double total_mass_kg = pinocchio::computeTotalMass(g1_model);
  std::printf("Loaded G1 model: nq=%d nv=%d njoints=%d mass=%.2f kg\n", g1_model.nq, g1_model.nv,
              g1_model.njoints, total_mass_kg);

  // MPC용 전체 관성 (COM 기준, 월드 정렬; neutral 자세에서 centroidal composite inertia).
  walking_kjw::WbcBodyParams body;
  {
    pinocchio::Data data(g1_model);
    pinocchio::ccrba(g1_model, data, pinocchio::neutral(g1_model), Eigen::VectorXd::Zero(g1_model.nv));
    const Eigen::Matrix3d I = data.Ig.inertia().matrix();
    body.mass_kg = total_mass_kg;
    body.inertia_xx = I(0, 0);
    body.inertia_yy = I(1, 1);
    std::printf("Body inertia about COM: Ixx=%.3f Iyy=%.3f kg m^2\n", body.inertia_xx, body.inertia_yy);
  }

  int msgref_id = msgget(walking_kjw::MsgRefKey(), IPC_CREAT | 0666);
  int msgsen_id = msgget(walking_kjw::MsgSenKey(), IPC_CREAT | 0666);
  if (msgref_id < 0 || msgsen_id < 0) {
    std::perror("msgget");
    return 1;
  }

  // CommandInputThread의 대화형 프롬프트와 안 섞이도록 타이밍 로그는 별도 파일로.
  std::FILE* timing_log = std::fopen("controller_timing.log", "w");
  if (!timing_log) {
    std::perror("main: fopen(controller_timing.log) failed, using stderr instead");
    timing_log = stderr;
  } else {
    std::printf("Thread timing log: controller_timing.log\n");
  }

  walking_kjw::DataWarehouse dw;

  std::thread kinematics_thread([&] {
    walking_kjw::RunKinematicsThread(dw, g1_model, msgref_id, msgsen_id, g_stop, timing_log);
  });
  std::thread motion_gen_thread(
      [&] { walking_kjw::RunMotionGenThread(dw, g_stop, timing_log); });
  std::thread wbc_thread(
      [&] { walking_kjw::RunWholeBodyControlThread(dw, body, g_stop, timing_log); });
  std::thread command_input_thread([&] { walking_kjw::RunCommandInputThread(dw, g_stop); });
  std::thread logging_thread([&] { walking_kjw::RunLoggingThread(dw, g_stop); });

  // 1kHz 실시간 스레드 셋에만 코어 배정 (코어 0은 OS용으로 비움; 9,11,13은 서로 다른 물리 코어의 SMT 형제 스레드 중 한쪽).
  // WKJ_NO_PIN=1: 코어 고정/RT 승격 생략 (다른 컨트롤러 인스턴스와 동시에 돌리는 테스트용).
  if (!std::getenv("WKJ_NO_PIN")) {
    PinThreadToCore(kinematics_thread, 9);
    PinThreadToCore(motion_gen_thread, 11);
    PinThreadToCore(wbc_thread, 13);

    constexpr int kRtPriority = 50;
    SetThreadRealtime(kinematics_thread, kRtPriority);
    SetThreadRealtime(motion_gen_thread, kRtPriority);
    SetThreadRealtime(wbc_thread, kRtPriority);
  }

  std::printf("Controller running (5 threads). Ctrl+C also stops.\n");

  kinematics_thread.join();
  motion_gen_thread.join();
  wbc_thread.join();
  logging_thread.join();
  // stdin을 닫아 CommandInputThread의 블로킹 read를 강제로 깨움 (g_stop만으론 안 깨어남).
  std::fclose(stdin);
  command_input_thread.join();

  if (timing_log != stderr) {
    std::fclose(timing_log);
  }

  std::printf("Stopped.\n");
  return 0;
}
