#include "command_input_thread.h"

#include <cstdio>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include "quat_euler.h"

namespace walking_kjw {
namespace {

// Printed at startup and after every command, so the terminal always shows
// the full command syntax next to that field's *current* value right above
// the next input line -- lets the user see what to type and what state
// they're changing it from without scrolling back.
void PrintStatus(DataWarehouse& dw, bool diag = false) {
  const MotionMode mode = dw.GetMotionMode();
  const char* mode_str = mode == MotionMode::kRun          ? "run"
                          : mode == MotionMode::kUpAndDown  ? "updown"
                          : mode == MotionMode::kWalkReady  ? "walkready"
                                                             : "walk";
  const Pose3d com_cal = dw.GetComPosOriCal();
  double roll, pitch, yaw;
  EulerFromQuat(com_cal.quat.w(), com_cal.quat.x(), com_cal.quat.y(), com_cal.quat.z(), &roll, &pitch, &yaw);
  const double vel = dw.GetVelCmd();
  const bool wbc_active = dw.GetWbcFlag();
  const bool wbc_user = dw.GetWbcUserEnabled();
  // 사용자 설정(wb on/off)과 실제 적용(walk 모드에서만)을 같이 보여준다.
  const char* wbc_str = wbc_user ? (wbc_active ? "on (동작 중)" : "on (walk 모드에서 동작)") : "off";
  const Eigen::VectorXd kp_wbc = dw.GetKpWbc();
  const double kp_foot = dw.GetKpFoot(), kd_foot = dw.GetKdFoot();
  const double waist_k = dw.GetWaistK(), waist_max = dw.GetWaistMax();
  const double kp_joint = dw.GetKpJoint(), kd_joint = dw.GetKdJoint();
  const SaveFlag save = dw.GetSaveFlag();
  const char* save_str = save == SaveFlag::kStart ? "start(logging)" : save == SaveFlag::kEnd ? "end" : "idle";
  const bool drop = dw.GetDropRobot();
  const char* drop_str = drop                        ? "on(released)"
                          : dw.GetDropDescending()    ? "descending..."
                                                       : "off(held)";
  const char* control_mode_str =
      dw.GetControlMode() == ControlMode::kTaskSpace ? "task" : "joint";

  std::printf(
      "\n"
      "== 명령 (현재값) ==\n"
      " m  w|r|u|wr             motion walk/run/updown/walkready [%s]\n"
      " cp x y z                compos [%.3f %.3f %.3f]\n"
      " co r p y                comori rad [%.3f %.3f %.3f]\n"
      " v  <v>                  vel [%.3f]\n"
      " wb on|off               wbc [%s]\n"
      " sa on|off               stepadapt 외란 시 조기 착지/스텝 주기 단축 [%s]\n"
      " cm t|j                  controlmode task/joint [%s]\n"
      " g w a b c d e           gain wbc kp_r kd_r kp_p kd_p kp_h (앞4=MPC 상태가중^0.5) [%.0f %.0f %.0f %.0f %.0f]\n"
      " g f kp kd               gain foot [%.1f %.1f]\n"
      " g y k [max]             gain waist 상체 카운터로테이션 yaw=-k*(오른발x-왼발x) rad/m, 상한 rad [%.2f %.2f]\n"
      " g j kp kd               gain joint [%.1f %.1f]\n"
      " pu fx fy [dur]         push pelvis 외력 N (x앞 y좌), dur 기본 0.1s\n"
      " sv s|e                  save start/end [%s]\n"
      " d  on|off|a             drop, a=auto [%s]\n"
      " s [d]  상태(d=관절 진단)   q  종료\n",
      mode_str, com_cal.pos.x(), com_cal.pos.y(), com_cal.pos.z(), roll, pitch, yaw, vel,
      wbc_str, dw.GetStepAdapt() ? "on" : "off", control_mode_str, kp_wbc[0], kp_wbc[1], kp_wbc[2], kp_wbc[3], kp_wbc[4],
      kp_foot, kd_foot, waist_k, waist_max, kp_joint, kd_joint, save_str, drop_str);
  // 진단: 관절 목표/측정/토크 (관절이 안 움직일 때 어디서 끊기는지 확인용).
  if (diag) {
    const Eigen::VectorXd targ = dw.GetJointTargPos(), sen = dw.GetJointPosSen(),
                          torq = dw.GetJointTargTorq();
    std::printf(" idx  targ_pos   sen_pos   targ_torq\n");
    for (int i = 0; i < targ.size(); ++i) {
      std::printf(" %3d %9.4f %9.4f %10.3f\n", i, targ[i], sen[i], torq[i]);
    }
  }
  std::printf("> ");
  std::fflush(stdout);
}

// 짧은 명령/인자를 내부 파서가 쓰는 긴 이름으로 확장 (긴 이름도 그대로 허용).
std::string Expand(const std::string& tok, const std::map<std::string, std::string>& table) {
  auto it = table.find(tok);
  return it == table.end() ? tok : it->second;
}

std::string ExpandLine(const std::string& line) {
  static const std::map<std::string, std::string> cmds = {
      {"m", "motion"}, {"cp", "compos"}, {"co", "comori"}, {"v", "vel"},   {"wb", "wbc"}, {"sa", "stepadapt"},
      {"cm", "controlmode"}, {"pu", "push"}, {"g", "gain"}, {"sv", "save"}, {"d", "drop"}, {"s", "status"},
      {"q", "quit"}};
  static const std::map<std::string, std::map<std::string, std::string>> args = {
      {"motion", {{"w", "walk"}, {"r", "run"}, {"u", "updown"}, {"wr", "walkready"}}},
      {"controlmode", {{"t", "taskspace"}, {"j", "jointspace"}}},
      {"gain", {{"w", "wbc"}, {"f", "foot"}, {"y", "waist"}, {"j", "joint"}}},
      {"save", {{"s", "start"}, {"e", "end"}}},
      {"drop", {{"a", "auto"}}}};
  std::istringstream iss(line);
  std::string cmd, arg, rest;
  iss >> cmd >> arg;
  std::getline(iss, rest);
  cmd = Expand(cmd, cmds);
  auto it = args.find(cmd);
  if (it != args.end()) arg = Expand(arg, it->second);
  return cmd + " " + arg + rest;
}

}  // namespace

void RunCommandInputThread(DataWarehouse& dw, std::atomic<bool>& stop) {
  PrintStatus(dw);
  std::string line;
  while (!stop.load() && std::getline(std::cin, line)) {
    std::istringstream iss(ExpandLine(line));
    std::string cmd;
    iss >> cmd;
    if (cmd.empty()) continue;
    const bool diag = (line.find(" d") != std::string::npos) && cmd == "status";

    if (cmd == "motion") {
      std::string mode;
      iss >> mode;
      if (mode == "walk" || mode == "run" || mode == "updown" || mode == "walkready") {
        dw.SetMotionMode(mode == "run"        ? MotionMode::kRun
                          : mode == "updown"   ? MotionMode::kUpAndDown
                          : mode == "walkready" ? MotionMode::kWalkReady
                                                 : MotionMode::kWalk);
      } else {
        std::cerr << "cmd: 'motion' needs walk|run|updown|walkready, got '" << mode
                   << "' -- ignored\n";
      }
    } else if (cmd == "compos") {
      Pose3d p = dw.GetComPosOriCal();
      iss >> p.pos.x() >> p.pos.y() >> p.pos.z();
      if (iss.fail()) {
        std::cerr << "cmd: 'compos' needs 3 numbers <x> <y> <z> -- ignored\n";
      } else {
        dw.SetComPosOriCal(p);
      }
    } else if (cmd == "comori") {
      double roll = 0, pitch = 0, yaw = 0;
      iss >> roll >> pitch >> yaw;
      if (iss.fail()) {
        std::cerr << "cmd: 'comori' needs 3 numbers <roll> <pitch> <yaw> -- ignored\n";
      } else {
        Pose3d p = dw.GetComPosOriCal();
        double w, x, y, z;
        QuatFromEuler(roll, pitch, yaw, &w, &x, &y, &z);
        p.quat = Eigen::Quaterniond(w, x, y, z);
        dw.SetComPosOriCal(p);
      }
    } else if (cmd == "vel") {
      double v = 0.0;
      iss >> v;
      if (iss.fail()) {
        std::cerr << "cmd: 'vel' needs a number <v> -- ignored\n";
      } else {
        dw.SetVelCmd(v);
      }
    } else if (cmd == "wbc") {
      std::string onoff;
      iss >> onoff;
      if (onoff == "on" || onoff == "off") {
        dw.SetWbcUserEnabled(onoff == "on");
      } else {
        std::cerr << "cmd: 'wbc' needs on|off, got '" << onoff << "' -- ignored\n";
      }
    } else if (cmd == "stepadapt") {
      std::string onoff;
      iss >> onoff;
      if (onoff == "on" || onoff == "off") {
        dw.SetStepAdapt(onoff == "on");
      } else {
        std::cerr << "cmd: 'stepadapt' needs on|off, got '" << onoff << "' -- ignored\n";
      }
    } else if (cmd == "push") {
      double fx = 0.0, fy = 0.0, dur = 0.1;
      iss >> fx >> fy;
      if (iss.fail()) {
        std::cerr << "cmd: 'push' needs <fx> <fy> [<dur_s>] (N, x forward, y left) -- ignored\n";
      } else {
        double d = 0.0;
        if (iss >> d) dur = d;
        if (std::hypot(fx, fy) > 500.0 || dur <= 0.0 || dur > 2.0) {
          std::cerr << "cmd: 'push' needs |F| <= 500 N and 0 < dur <= 2 s -- ignored\n";
        } else {
          PushRequest req = dw.GetPushRequest();
          req.fx = fx;
          req.fy = fy;
          req.dur = dur;
          req.seq += 1;
          dw.SetPushRequest(req);
          std::printf("push: fx=%.0f fy=%.0f N for %.2f s\n", fx, fy, dur);
        }
      }
    } else if (cmd == "controlmode") {
      std::string m;
      iss >> m;
      if (m == "taskspace" || m == "jointspace") {
        dw.SetControlMode(m == "taskspace" ? ControlMode::kTaskSpace : ControlMode::kJointSpace);
      } else {
        std::cerr << "cmd: 'controlmode' needs taskspace|jointspace, got '" << m
                   << "' -- ignored\n";
      }
    } else if (cmd == "gain") {
      std::string group;
      iss >> group;
      if (group == "wbc") {
        Eigen::VectorXd kp(6);
        kp.setZero();
        iss >> kp[0] >> kp[1] >> kp[2] >> kp[3] >> kp[4];
        if (iss.fail()) {
          std::cerr << "cmd: 'gain wbc' needs 5 numbers <kp_roll> <kd_roll> <kp_pitch> "
                        "<kd_pitch> <kp_height> -- ignored\n";
        } else {
          dw.SetKpWbc(kp);
        }
      } else if (group == "foot") {
        double kp = 0.0, kd = 0.0;
        iss >> kp >> kd;
        if (iss.fail()) {
          std::cerr << "cmd: 'gain foot' needs 2 numbers <kp> <kd> -- ignored\n";
        } else {
          dw.SetKpFoot(kp);
          dw.SetKdFoot(kd);
        }
      } else if (group == "waist") {
        double k = 0.0, mx = dw.GetWaistMax();
        iss >> k;
        if (iss.fail() || k < 0.0) {
          std::cerr << "cmd: 'gain waist' needs <k >= 0> [<max_rad>] -- ignored\n";
        } else {
          double m2 = 0.0;
          if (iss >> m2) mx = m2;
          dw.SetWaistK(k);
          dw.SetWaistMax(std::clamp(mx, 0.0, 0.5));
        }
      } else if (group == "joint") {
        double kp = 0.0, kd = 0.0;
        iss >> kp >> kd;
        if (iss.fail()) {
          std::cerr << "cmd: 'gain joint' needs 2 numbers <kp> <kd> -- ignored\n";
        } else {
          dw.SetKpJoint(kp);
          dw.SetKdJoint(kd);
        }
      } else {
        std::cerr << "cmd: unknown gain group '" << group << "'\n";
      }
    } else if (cmd == "save") {
      std::string when;
      iss >> when;
      if (when == "start" || when == "end") {
        dw.SetSaveFlag(when == "start" ? SaveFlag::kStart : SaveFlag::kEnd);
      } else {
        std::cerr << "cmd: 'save' needs start|end, got '" << when << "' -- ignored\n";
      }
    } else if (cmd == "drop") {
      std::string onoff;
      iss >> onoff;
      if (onoff == "on" || onoff == "off") {
        // Cancels a *pending* auto sequence (not yet picked up by
        // KinematicsThread). Known gap: if a sequence is already actively
        // descending, this doesn't stop it -- KinematicsThread will still
        // release on touchdown afterward, possibly re-overriding this. Not
        // handled since it's an unusual thing to do (issue "drop on/off"
        // mid-sequence) rather than a normal flow.
        dw.SetDropRobot(onoff == "on");
        dw.SetDropSequenceRequested(false);
      } else if (onoff == "auto") {
        dw.SetDropSequenceRequested(true);
      } else {
        std::cerr << "cmd: 'drop' needs on|off|auto, got '" << onoff << "' -- ignored\n";
      }
    } else if (cmd == "status") {
      // no-op: PrintStatus() below reprints regardless of which command ran
    } else if (cmd == "quit") {
      stop.store(true);
      break;
    } else {
      std::cerr << "cmd: unknown command '" << cmd << "'\n";
    }
    PrintStatus(dw, diag);
  }
}

}  // namespace walking_kjw
