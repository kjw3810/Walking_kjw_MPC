#include "logging_thread.h"

#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>
#include <thread>

#include <Eigen/Dense>

#include "ipc_protocol.h"

namespace walking_kjw {
namespace {

constexpr auto kLogPeriod = std::chrono::milliseconds(10);  // 100Hz

std::string TimestampedFilename() {
  std::time_t now = std::time(nullptr);
  std::ostringstream oss;
  oss << "walking_kjw_log_" << now << ".csv";
  return oss.str();
}

void WriteHeader(std::ofstream& f) {
  f << "t_s";
  for (int i = 0; i < kNumJoints; ++i) f << ",joint_pos_sen_" << i;
  for (int i = 0; i < kNumJoints; ++i) f << ",joint_targ_pos_" << i;
  for (int i = 0; i < kNumJoints; ++i) f << ",joint_targ_torq_" << i;
  f << ",imu_roll,imu_pitch,imu_yaw,imu_wx,imu_wy,imu_wz";
  f << ",com_targ_x,com_targ_y,com_targ_z,wbc_enabled,control_mode";
  f << ",foot_targ_lx,foot_targ_ly,foot_targ_lz,foot_targ_rx,foot_targ_ry,foot_targ_rz";
  f << ",walk_phase,walk_support,foot_corr_dx,foot_corr_dy";
  for (const char* side : {"L", "R"})
    for (const char* c : {"fx", "fy", "fz", "tx", "ty"}) f << ",wbc_" << side << "_" << c;
  f << ",com_meas_x,com_meas_y,com_meas_z,pelvis_x,pelvis_y,pelvis_z,contact_L,contact_R,waist_yaw_targ\n";
}

}  // namespace

void RunLoggingThread(DataWarehouse& dw, std::atomic<bool>& stop) {
  std::ofstream file;
  bool logging_active = false;
  double t_s = 0.0;

  while (!stop.load()) {
    const auto tick_start = std::chrono::steady_clock::now();
    const SaveFlag flag = dw.GetSaveFlag();

    if (flag == SaveFlag::kStart) {
      if (!logging_active) {
        file.open(TimestampedFilename(), std::ios::out | std::ios::trunc);
        WriteHeader(file);
        logging_active = true;
        t_s = 0.0;
      }
      const Eigen::VectorXd pos_sen = dw.GetJointPosSen();
      const Eigen::VectorXd targ_pos = dw.GetJointTargPos();
      const Eigen::VectorXd targ_torq = dw.GetJointTargTorq();
      const Eigen::Vector3d imu_ang = dw.GetImuAng();
      const Eigen::Vector3d imu_angvel = dw.GetImuAngVel();
      const Eigen::Vector3d com_targ = dw.GetComTargWorld();
      const FootTargetWorld foot_targ = dw.GetFootTargWorld();
      const WalkDebug walk = dw.GetWalkDebug();
      const std::array<ContactWrench, 2> wbc = dw.GetWbcForceTorque();
      const Eigen::Vector3d com_meas = dw.GetComMeasWorld();
      const Pose3d pelvis = dw.GetTorsoPoseMeasWorld();
      const std::array<bool, 2> contact = dw.GetFootContact();

      file << t_s;
      for (int i = 0; i < kNumJoints; ++i) file << "," << pos_sen[i];
      for (int i = 0; i < kNumJoints; ++i) file << "," << targ_pos[i];
      for (int i = 0; i < kNumJoints; ++i) file << "," << targ_torq[i];
      file << "," << imu_ang[0] << "," << imu_ang[1] << "," << imu_ang[2] << "," << imu_angvel[0]
           << "," << imu_angvel[1] << "," << imu_angvel[2];
      file << "," << com_targ.x() << "," << com_targ.y() << "," << com_targ.z() << ","
           << dw.GetWbcFlag() << "," << static_cast<int>(dw.GetControlMode());
      file << "," << foot_targ.left.x() << "," << foot_targ.left.y() << "," << foot_targ.left.z() << ","
           << foot_targ.right.x() << "," << foot_targ.right.y() << "," << foot_targ.right.z();
      file << "," << walk.phase << "," << walk.support << "," << walk.dx << "," << walk.dy;
      for (const ContactWrench& w : wbc)
        file << "," << w.fx << "," << w.fy << "," << w.fz << "," << w.tx << "," << w.ty;
      file << "," << com_meas.x() << "," << com_meas.y() << "," << com_meas.z() << "," << pelvis.pos.x()
           << "," << pelvis.pos.y() << "," << pelvis.pos.z() << "," << contact[0] << "," << contact[1] << "," << dw.GetWaistYawTarg() << "\n";
      t_s += std::chrono::duration<double>(kLogPeriod).count();
    } else if (logging_active) {
      file.close();
      logging_active = false;
    }

    std::this_thread::sleep_until(tick_start + kLogPeriod);
  }

  if (logging_active) {
    file.close();
  }
}

}  // namespace walking_kjw
