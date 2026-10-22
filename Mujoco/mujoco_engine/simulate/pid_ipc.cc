#include "pid_ipc.h"

#include "g1_joints.h"
#include "ipc_protocol.h"
#include "loop_timer.h"
#include "quat_euler.h"

#include <cstdio>
#include <cstring>

namespace walking_kjw {
namespace {

LoopTimer g_physics_timer("MujocoPhysicsStep", 1.0);  // 실측 정상 주기 1ms 기준

int g_msgref_id = -1;
int g_msgsen_id = -1;
bool g_queues_initialized = false;

RefMsg g_latest_ref;

// 외력 푸시: msg_ref의 push_seq가 바뀌면 시작, 시뮬 시간 기준으로 push_dur_s 동안 pelvis에 월드 힘 인가.
bool g_ref_received = false;
long g_push_last_seq = 0;
bool g_push_active = false;
double g_push_end_time = 0.0;
double g_push_fx = 0.0, g_push_fy = 0.0;
int g_pelvis_body = -1;
int g_push_body = -1;  // 푸시를 가하는 바디: 상체가 합쳐진 waist_yaw_link (없으면 pelvis)
// 푸시 작용점: push 바디 원점(= pelvis 원점) 기준 로컬 +z 오프셋 [m].
// 서 있는 자세(pelvis 원점 kNominalStandHeightM)에서 지면 위 1.0 m가 되도록.
constexpr double kPushHeightM = 1.0;
constexpr double kPushPointLocalZ = kPushHeightM - kNominalStandHeightM;

const mjModel* g_resolved_model = nullptr;
bool g_joints_ok = false;
int g_qpos_adr[kNumJoints];
int g_dof_adr[kNumJoints];
double g_torque_min[kNumJoints];
double g_torque_max[kNumJoints];
int g_gyro_adr = -1;
int g_quat_adr = -1;
int g_pelvis_hold_eq = -1;
int g_pelvis_hold_eq_data_adr = -1;

// 큐가 없으면 생성, 있으면 그대로 붙는다(경쟁 상태 방지, 파괴 후 재생성 금지).
void EnsureQueues() {
  if (g_queues_initialized) {
    return;
  }
  g_msgref_id = msgget(MsgRefKey(), IPC_CREAT | 0666);
  g_msgsen_id = msgget(MsgSenKey(), IPC_CREAT | 0666);
  if (g_msgref_id < 0 || g_msgsen_id < 0) {
    std::fprintf(stderr, "pid_ipc: msgget failed (msgref=%d, msgsen=%d)\n",
                 g_msgref_id, g_msgsen_id);
    g_queues_initialized = true;
    return;
  }
  RefMsg stale_ref;
  while (msgrcv(g_msgref_id, &stale_ref, sizeof(RefMsg) - sizeof(long), 1, IPC_NOWAIT | MSG_NOERROR) > 0) {
  }
  SenMsg stale_sen;
  while (msgrcv(g_msgsen_id, &stale_sen, sizeof(SenMsg) - sizeof(long), 1, IPC_NOWAIT | MSG_NOERROR) > 0) {
  }
  g_queues_initialized = true;
}

// 모델이 바뀔 때만 관절/센서/이퀄리티 id 재계산. G1이 아니면 false.
bool ResolveModel(const mjModel* m) {
  if (m == g_resolved_model) {
    return g_joints_ok;
  }
  g_resolved_model = m;
  g_joints_ok = true;
  const auto& names = G1JointNames();
  for (int i = 0; i < kNumJoints; ++i) {
    int joint_id = mj_name2id(m, mjOBJ_JOINT, names[i]);
    if (joint_id < 0) {
      g_joints_ok = false;
      break;
    }
    g_qpos_adr[i] = m->jnt_qposadr[joint_id];
    g_dof_adr[i] = m->jnt_dofadr[joint_id];
    // qfrc_applied로 직접 토크를 가하므로(액추에이터 아님) 토크 한계는 직접 클리핑.
    g_torque_min[i] = m->jnt_actfrcrange[2 * joint_id];
    g_torque_max[i] = m->jnt_actfrcrange[2 * joint_id + 1];
  }

  int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "imu-pelvis-angular-velocity");
  g_gyro_adr = (gyro_id >= 0) ? m->sensor_adr[gyro_id] : -1;
  int quat_id = mj_name2id(m, mjOBJ_SENSOR, "imu-pelvis-orientation");
  g_quat_adr = (quat_id >= 0) ? m->sensor_adr[quat_id] : -1;
  g_pelvis_body = mj_name2id(m, mjOBJ_BODY, "pelvis");
  g_push_body = mj_name2id(m, mjOBJ_BODY, "waist_yaw_link");
  if (g_push_body < 0) {
    g_push_body = g_pelvis_body;
  }
  g_pelvis_hold_eq = mj_name2id(m, mjOBJ_EQUALITY, "pelvis_hold");
  g_pelvis_hold_eq_data_adr = (g_pelvis_hold_eq >= 0) ? g_pelvis_hold_eq * mjNEQDATA : -1;

  return g_joints_ok;
}

void DrainRefQueue() {
  if (g_msgref_id < 0) {
    return;
  }
  RefMsg msg;
  while (msgrcv(g_msgref_id, &msg, sizeof(RefMsg) - sizeof(long), 1, IPC_NOWAIT | MSG_NOERROR) > 0) {
    if (!g_ref_received) {  // 첫 수신: 이미 지난 푸시 요청은 무시하고 기준만 맞춘다
      g_ref_received = true;
      g_push_last_seq = msg.push_seq;
    }
    g_latest_ref = msg;
  }
}

void SendFeedback(const mjData* d, const double torque[kNumJoints]) {
  if (g_msgsen_id < 0) {
    return;
  }
  SenMsg fb;
  for (int i = 0; i < kNumJoints; ++i) {
    fb.joint_pos[i] = d->qpos[g_qpos_adr[i]];
    fb.joint_vel[i] = d->qvel[g_dof_adr[i]];
    fb.joint_torq[i] = torque[i];
  }
  if (g_gyro_adr >= 0) {
    fb.imu_angvel[0] = d->sensordata[g_gyro_adr + 0];
    fb.imu_angvel[1] = d->sensordata[g_gyro_adr + 1];
    fb.imu_angvel[2] = d->sensordata[g_gyro_adr + 2];
  }
  if (g_quat_adr >= 0) {
    double roll, pitch, yaw;
    EulerFromQuat(d->sensordata[g_quat_adr + 0], d->sensordata[g_quat_adr + 1],
                  d->sensordata[g_quat_adr + 2], d->sensordata[g_quat_adr + 3],
                  &roll, &pitch, &yaw);
    fb.imu_ang[0] = roll;
    fb.imu_ang[1] = pitch;
    fb.imu_ang[2] = yaw;
  }
  fb.sim_time_s = d->time;
  msgsnd(g_msgsen_id, &fb, sizeof(SenMsg) - sizeof(long), IPC_NOWAIT);
}

// pelvis_hold weld 제어. drop=true면 해제(자유낙하), false면 target height로 고정.
// eq_data는 mjModel 소유라 const_cast 필요 (물리 스레드 단독 접근이라 안전).
void ApplyPelvisHold(const mjModel* m, mjData* d, bool drop, double height_m) {
  if (g_pelvis_hold_eq < 0) {
    return;
  }
  d->eq_active[g_pelvis_hold_eq] = drop ? 0 : 1;
  if (!drop && g_pelvis_hold_eq_data_adr >= 0) {
    const_cast<mjModel*>(m)->eq_data[g_pelvis_hold_eq_data_adr + 2] = height_m;
  }
}

}  // namespace

void PidControl(const mjModel* m, mjData* d) {
  g_physics_timer.TickStart();
  EnsureQueues();
  DrainRefQueue();

  if (!ResolveModel(m)) {
    return;
  }

  ApplyPelvisHold(m, d, g_latest_ref.drop_robot_sim, g_latest_ref.pelvis_hold_height_m);

  // qfrc_applied는 이 콜백만 쓴다: 매 스텝 0에서 다시 채워 이전 스텝의 푸시가 남거나 누적되지 않게 한다.
  // (xfrc_applied에 += 하면 시뮬레이터가 렌더 프레임마다만 지워서, 프레임당 물리 스텝 수만큼 힘이 쌓였다.)
  mju_zero(d->qfrc_applied, m->nv);

  // 외력 푸시: 명치 작용점에 월드 힘 -> mj_applyFT가 일반화 힘(작용점 토크 포함)으로 바꿔 qfrc_applied에 더한다.
  if (g_push_body >= 0) {
    if (g_ref_received && g_latest_ref.push_seq != g_push_last_seq) {
      g_push_last_seq = g_latest_ref.push_seq;
      g_push_fx = g_latest_ref.push_fx;
      g_push_fy = g_latest_ref.push_fy;
      g_push_end_time = d->time + g_latest_ref.push_dur_s;
      g_push_active = g_latest_ref.push_dur_s > 0.0;
      std::fprintf(stderr, "[push] t=%.2f fx=%.0f fy=%.0f N for %.2f s at %.2f m\n", d->time, g_push_fx,
                   g_push_fy, g_latest_ref.push_dur_s, kPushHeightM);
    }
    if (g_push_active) {
      if (d->time < g_push_end_time) {
        const mjtNum* R = d->xmat + 9 * g_push_body;
        const mjtNum* org = d->xpos + 3 * g_push_body;
        mjtNum point[3] = {org[0] + R[2] * kPushPointLocalZ, org[1] + R[5] * kPushPointLocalZ,
                           org[2] + R[8] * kPushPointLocalZ};
        mjtNum force[3] = {g_push_fx, g_push_fy, 0.0};
        mjtNum no_torque[3] = {0.0, 0.0, 0.0};
        mj_applyFT(m, d, force, no_torque, point, g_push_body, d->qfrc_applied);
      } else {
        g_push_active = false;
      }
    }
  }

  double torque[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) {
    double pos = d->qpos[g_qpos_adr[i]];
    double vel = d->qvel[g_dof_adr[i]];
    double pos_err = g_latest_ref.joint_targ_pos[i] - pos;
    double vel_err = g_latest_ref.joint_targ_vel[i] - vel;
    torque[i] = g_latest_ref.kp_joint * pos_err + g_latest_ref.kd_joint * vel_err +
                g_latest_ref.joint_targ_torq[i];
    if (torque[i] < g_torque_min[i]) torque[i] = g_torque_min[i];
    if (torque[i] > g_torque_max[i]) torque[i] = g_torque_max[i];

    // 액추에이터(ctrl) 대신 qfrc_applied로 직접 인가 (Act Group 0 비활성 상태와 무관하게 항상 적용).
    // 위에서 0으로 지우고 푸시 몫을 더해 두었으므로 += (푸시가 없으면 = 와 같다).
    d->qfrc_applied[g_dof_adr[i]] += torque[i];
  }

  SendFeedback(d, torque);
  g_physics_timer.ComputeDone();
}

}  // namespace walking_kjw
