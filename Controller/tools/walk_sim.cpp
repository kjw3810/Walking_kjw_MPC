// 헤드리스 폐루프 시험 하네스: MuJoCo(G1) 물리 + 실제 controller 바이너리를 별도 IPC 큐로 연결해
// GUI 없이 스크립트된 명령/외란으로 보행을 돌려 본다. 실시간(1 ms 스텝, 벽시계 동기)으로 돈다.
//
// 사용: walk_sim <duration_s> [옵션...]
//   cmd=<t>:<명령>             t(시뮬 초)에 controller stdin으로 명령 전송 (예: cmd=10:m w)
//   push=<t>:<fx>:<fy>:<dur>   pelvis에 외력(N, 월드) 인가
//   csv=<파일>                 10 ms 간격 로그
//   view=1                     MuJoCo 3D 창으로 보면서 실행 (푸시는 빨간 화살표, 로봇을 따라다니는 카메라)
//   tag=<이름>                 IPC 키 구분(동시 실행용, 기본 pid 기반)
// 종료 코드: 0 서있음, 2 넘어짐. 결과 요약을 stdout에 출력.
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#ifdef WALK_SIM_VIEWER
#include <GLFW/glfw3.h>
#endif

#include "pid_ipc.h"

namespace {
struct Cmd { double t; std::string text; bool sent = false; };
struct Push { double t, fx, fy, dur; bool announced = false; };
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: walk_sim <duration_s> [cmd=t:text] [push=t:fx:fy:dur] [csv=file]\n"); return 1; }
  const double duration = std::atof(argv[1]);
  std::vector<Cmd> cmds; std::vector<Push> pushes; std::string csv_path; bool view = false;
  long key = 0x5A000000 + (getpid() & 0xFFFF) * 4;
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a.rfind("cmd=", 0) == 0) { auto c = a.find(':', 4); cmds.push_back({std::atof(a.substr(4, c - 4).c_str()), a.substr(c + 1)}); }
    else if (a.rfind("push=", 0) == 0) { Push p; std::sscanf(a.c_str() + 5, "%lf:%lf:%lf:%lf", &p.t, &p.fx, &p.fy, &p.dur); pushes.push_back(p); }
    else if (a.rfind("csv=", 0) == 0) csv_path = a.substr(4);
    else if (a == "view=1") view = true;
  }
  setenv("WKJ_IPC_KEY", std::to_string(key).c_str(), 1);
  if (!std::getenv("WKJ_PIN")) setenv("WKJ_NO_PIN", "1", 1);  // WKJ_PIN=1: controller의 코어 고정/RT 승격을 켠 채 측정

  // ---- controller 자식 프로세스 ----
  int pipefd[2];
  if (pipe(pipefd) != 0) return 1;
  const pid_t child = fork();
  if (child == 0) {
    dup2(pipefd[0], STDIN_FILENO);
    close(pipefd[1]);
    int devnull = open("/dev/null", O_WRONLY);
    dup2(devnull, STDOUT_FILENO);
    dup2(devnull, STDERR_FILENO);
    execl(CONTROLLER_BIN, CONTROLLER_BIN, (char*)nullptr);
    _exit(127);
  }
  close(pipefd[0]);

  // ---- MuJoCo ----
  char err[1000] = "";
  mjModel* m = mj_loadXML(MUJOCO_G1_XML, nullptr, err, sizeof err);
  if (!m) { std::fprintf(stderr, "load failed: %s\n", err); return 1; }
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);
  mj_forward(m, d);
  mjcb_control = walking_kjw::PidControl;
  const int pelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");

  FILE* csv = csv_path.empty() ? nullptr : std::fopen(csv_path.c_str(), "w");
  if (csv) std::fprintf(csv, "t,px,py,pz,roll,pitch,yaw\n");

#ifdef WALK_SIM_VIEWER
  GLFWwindow* win = nullptr;
  mjvCamera cam; mjvOption vopt; mjvScene scn; mjrContext con;
  if (view) {
    if (!glfwInit()) { std::fprintf(stderr, "glfwInit failed (DISPLAY 확인)\n"); view = false; }
    else {
      win = glfwCreateWindow(1200, 800, "walk_sim", nullptr, nullptr);
      if (!win) { std::fprintf(stderr, "창 생성 실패\n"); view = false; }
      else {
        glfwMakeContextCurrent(win);
        glfwSwapInterval(0);
        mjv_defaultCamera(&cam); mjv_defaultOption(&vopt); mjv_defaultScene(&scn); mjr_defaultContext(&con);
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);
        cam.type = mjCAMERA_TRACKING; cam.trackbodyid = pelvis;
        cam.distance = 2.8; cam.azimuth = 140; cam.elevation = -12;
      }
    }
  }
  auto render = [&](double tnow, const std::vector<Push>& pushes_) {
    int w, h; glfwGetFramebufferSize(win, &w, &h);
    mjrRect vp = {0, 0, w, h};
    mjv_updateScene(m, d, &vopt, nullptr, &cam, mjCAT_ALL, &scn);
    char info[200] = "";
    for (const auto& p : pushes_) {
      if (tnow >= p.t && tnow < p.t + p.dur + 0.8 && scn.ngeom < scn.maxgeom) {
        const double mag = std::hypot(p.fx, p.fy);
        if (mag < 1e-9) continue;
        const double len = 0.004 * mag;  // 화살표 길이 ~ 힘 (80 N -> 0.32 m)
        mjtNum to[3] = {d->xpos[3 * pelvis + 0], d->xpos[3 * pelvis + 1], d->xpos[3 * pelvis + 2] + 0.1};
        mjtNum from[3] = {to[0] - p.fx / mag * len, to[1] - p.fy / mag * len, to[2]};
        mjv_connector(&scn.geoms[scn.ngeom], mjGEOM_ARROW, 0.02, from, to);
        scn.geoms[scn.ngeom].rgba[0] = 1; scn.geoms[scn.ngeom].rgba[1] = 0.1f; scn.geoms[scn.ngeom].rgba[2] = 0.1f; scn.geoms[scn.ngeom].rgba[3] = 1;
        ++scn.ngeom;
        std::snprintf(info, sizeof info, "PUSH fx=%.0f fy=%.0f N (%.1f s)%s", p.fx, p.fy, p.dur, tnow < p.t + p.dur ? "  <== 가하는 중" : "");
      }
    }
    mjr_render(vp, &scn, &con);
    char head[120]; std::snprintf(head, sizeof head, "t = %.2f s", tnow);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, head, info, &con);
    glfwSwapBuffers(win); glfwPollEvents();
  };
#endif
  const double dt = m->opt.timestep;
  const auto wall0 = std::chrono::steady_clock::now();
  bool fell = false; double fell_t = 0, max_tilt = 0, max_tilt_after = 0;
  double min_z = 1e9;
  long step = 0;
  while (d->time < duration) {
    for (auto& c : cmds) if (!c.sent && d->time >= c.t) {
      const std::string line = c.text + "\n";
      if (write(pipefd[1], line.c_str(), line.size()) < 0) {}
      c.sent = true;
      std::fprintf(stderr, "[walk_sim] t=%.2f cmd: %s\n", d->time, c.text.c_str());
    }
    d->xfrc_applied[6 * pelvis + 0] = d->xfrc_applied[6 * pelvis + 1] = 0;
    for (auto& p : pushes) {
      if (d->time >= p.t && d->time < p.t + p.dur) { d->xfrc_applied[6 * pelvis + 0] = p.fx; d->xfrc_applied[6 * pelvis + 1] = p.fy; }
      if (!p.announced && d->time >= p.t) {
        p.announced = true;
        std::fprintf(stderr, "[walk_sim] t=%.2f PUSH fx=%.0f N fy=%.0f N for %.2f s (pelvis 월드 힘)\n", d->time, p.fx, p.fy, p.dur);
      }
    }
    if (step % 1000 == 0) std::fprintf(stderr, "[walk_sim] t=%.0f / %.0f s  pelvis z=%.3f\n", d->time, duration, d->qpos[2]);
    mj_step(m, d);
    ++step;

    // 몸통 기울기 (pelvis quaternion -> roll/pitch)
    const double* q = d->qpos + 3;
    const double roll = std::atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    const double pitch = std::asin(std::clamp(2 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0));
    const double yaw = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    const double tilt = std::max(std::abs(roll), std::abs(pitch));
    if (d->time > 9.0) max_tilt_after = std::max(max_tilt_after, tilt);
    max_tilt = std::max(max_tilt, tilt);
    if (d->time > 9.0) min_z = std::min(min_z, d->qpos[2]);
    if (csv && step % 10 == 0) std::fprintf(csv, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", d->time, d->qpos[0], d->qpos[1], d->qpos[2], roll, pitch, yaw);
    if (d->time > 9.0 && (d->qpos[2] < 0.45 || tilt > 0.8)) { fell = true; fell_t = d->time; break; }
#ifdef WALK_SIM_VIEWER
    if (view && step % 16 == 0) {
      render(d->time, pushes);
      if (glfwWindowShouldClose(win)) break;
    }
#endif
    std::this_thread::sleep_until(wall0 + std::chrono::microseconds(static_cast<long>(step * dt * 1e6)));
  }
  std::fprintf(stderr, "[walk_sim] 끝: 넘어지지 않으면 설정 시간(%.0f s)까지 실시간으로 돈 뒤 결과를 출력합니다.\n", duration);
  std::printf("%s  t_end=%.2f  fell_at=%.2f  max_tilt(after 9s)=%.3f rad  min_z=%.3f  final pelvis=(%.3f %.3f %.3f)\n",
              fell ? "FELL" : "STOOD", d->time, fell ? fell_t : -1.0, max_tilt_after, min_z, d->qpos[0], d->qpos[1], d->qpos[2]);
#ifdef WALK_SIM_VIEWER
  if (view && win) {  // 끝난 뒤 3초 더 보여주고(닫기 가능) 정리
    const auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!glfwWindowShouldClose(win) && std::chrono::steady_clock::now() < t_end) {
      render(d->time, pushes);
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    mjr_freeContext(&con); mjv_freeScene(&scn); glfwDestroyWindow(win); glfwTerminate();
  }
#endif
  if (csv) std::fclose(csv);
  const std::string q = "q\n";
  if (write(pipefd[1], q.c_str(), q.size()) < 0) {}
  close(pipefd[1]);
  int st; waitpid(child, &st, 0);
  mj_deleteData(d); mj_deleteModel(m);
  return fell ? 2 : 0;
}
