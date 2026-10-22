# Controller 아키텍처 (목표 설계)

이 문서는 손으로 그린 컨트롤러 구조 다이어그램(`20260921_121536.jpg`, 2026-09-18 작성)을 분석·구조화한 것이다.
**이것은 목표(target) 설계 문서이며, 현재 구현 상태를 기술한 것이 아니다.** 현재 코드가 얼마나
못 미치는지는 [6. 현재 구현 상태 vs 목표 (Gap)](#6-현재-구현-상태-vs-목표-gap)에 정리했다.

다이어그램에서 손글씨로 불확실하게 표시된 부분(예: "??")은 그대로 "확인 필요"로 남겨뒀다 — 임의로
답을 채우지 않았다.

## 1. 개요

두 개의 별도 프로세스가 System V 메시지 큐 2개로 통신한다. 둘 다 **블로킹** 큐다.

```
┌─────────────────────────────── Controller 프로세스 ────────────────────────────────┐
│                                                                                    │
│   CommandInputThread (Th3)            ──────►┐                                     │
│   MotionGenThread (Th2), 1kHz         ──────►┼──►  전역변수 DataWarehouse (class)  │
│   WholeBodyControlThread (Th4), 1kHz  ──────►┘        (블랙보드, 모든 스레드 공유) │
│                                                              │                     │
│                         KinematicsThread (Th1), 1kHz  ◄──────┘                     │
│                           FK → IK → ID → SendRefToSim                              │
│                                                                                    │
│   LoggingThread (Th5) ── (save 트리거) ─── DataWarehouse 스냅샷 기록               │
└─────────────────────────────────┬─────────────────────────────────┬─────────────────┘
                                  │ msg_ref (관절타겟+게인+Drop)    │ msg_sen (관절+IMU 피드백)
                                  ▼                                 │
                               ┌───────────────────────────────────┴─────────────────┐
                               │                 Simulate 프로세스                   │
                               │ mj_step 루프 안에서:                                │
                               │  - pos/kp_joint, vel/kd_joint 기반 PID 제어         │
                               │  - + torq offset(torq)                              │
                               │ * 실로봇에서는 이 프로세스 대신 모터드라이버가 담당 │
                               └─────────────────────────────────────────────────────┘
```

(Th1~Th5는 원본 손그림 다이어그램 번호 대응용 표기 — 코드에서 쓸 실제 이름은 왼쪽의
역할명(`KinematicsThread`, `MotionGenThread`, `CommandInputThread`, `WholeBodyControlThread`,
`LoggingThread`))

### 설계 전제: 로봇 자유도

이 아키텍처는 G1 모델을 **13자유도(13-DOF)** 로 가정한다. 구체적으로는:

- 좌측 다리: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll = 6
- 우측 다리: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll = 6
- 허리: waist_yaw = 1
- 총합: 6 + 6 + 1 = 13
- 그 외의 관절은 모두 물리적으로 고정되어 있는 것으로 가정함.

따라서 관절 상태 벡터(`joint_pos_*`, `joint_vel_*`, `joint_torq_*`)는 크기 13으로 정의하고,
`msg_ref` / `msg_sen`도 13개 관절 인덱스를 기준으로 구성한다. 이 가정은
`Controller/model/g1/g1.urdf`의 revolute 관절 순서를 기준으로 한다.


## 2. DataWarehouse (전역변수 class)

모든 스레드가 읽고 쓰는 공유 블랙보드. 다이어그램에 명시된 필드:

| 필드 | 타입/형식 | 비고 |
|---|---|---|
| 관절 sen (pos, Torq) | 관절 벡터 | Simulate로부터 msg_sen으로 수신 |
| Torso_quat | quaternion | IMU 기반 몸통 자세 |
| Imu (기울기/각속도) | vector | |
| Foot_contact_switch | bool (발별) | |
| 관절 타겟 포즈 / 타겟관절(pos, Torq) | 관절 벡터 | Kinematics(Th1) IK 결과 / SendRefToSim 입력 |
| Mpc command | Fx,Fy,Fz,Tx,Ty | WBC(Th4) 출력 |
| Target pose (직교, 왼/오른발) | Cartesian | MotionGen(Th2) 출력 |
| Motion / Com | 명령값 | command input 스레드가 씀 |
| AdjKp / Kd | 게인 | foot 보정용 |
| vel | | command input에서 씀 |
| `wbc_enabled_` | bool | **wbc on/off와 연동** |
| `control_mode_` | int | **작업공간(1) / 관절공간(2)** |
| 관절게인 (kp_joint, kd_joint) | | SendRefToSim → Simulate 전달용 |
| Drop_robot_sim | bool | SendRefToSim → Simulate 전달용 |

**네이밍 가이드 (구현용)**: 원본 손그림 다이어그램은 `wbc_flag.bool_data`, `Control_mode.int_data`처럼
"객체.타입_필드" 형식을 쓴 경우가 있지만, 실제 C++ 구현에서는 이를 그대로 쓰지 않는다. 구현에서는
`wbc_enabled_`, `control_mode_` 같은 명시적 이름을 사용한다. 이유는 다음과 같다.

- bool 변수는 `wbc_flag_`처럼 명사형보다 `wbc_enabled_`처럼 서술형이 읽기 쉽다.
- `int` 기반 모드는 raw magic number 대신 `enum class ControlMode { kTaskSpace = 1, kJointSpace = 2 };`처럼 표현한다.
- WBC/IK 경계의 `Fx,Fy,Fz,Tx,Ty`는 5개 숫자 배열보다 `ContactWrench` 구조체로 묶는 편이 안전하다.
- 다이어그램 원문 표기는 문맥 설명용으로만 남기고, 코드에서는 구현 표기(`wbc_enabled_`, `control_mode_`)를 우선한다.

**동기화(mutex) — 필드별 락으로 확정 (2026-09-21)**: DataWarehouse의 각 필드는 **private 멤버 변수**로
두고, 각 필드마다 자신의 mutex로 보호되는 **get/set(접근자) 함수**를 통해서만 읽고 쓴다(필드 자체를
public으로 노출하지 않음). Kinematics(Th1)/MotionGen(Th2)/WBC(Th4)가 1kHz로 동시에 접근하고,
CommandInput(Th3)는 비주기, Logging(Th5)는 `save` 트리거 시점에 여러 필드를 읽는다.

- 트레이드오프: 필드별 락이라 DataWarehouse 전체를 한 번에 잠그는 것보다 가볍지만, Logging(Th5)처럼
  **여러 필드를 한 번에** 읽는 경우 필드마다 get()을 개별 호출하게 되어 필드 간 스냅샷 시점이 완전히
  일치하지 않을 수 있다(한 필드는 t=999, 다음 필자는 t=1000 시점 값일 수 있음). 로깅 정밀도가 중요해지면
  이 부분은 추가 설계(예: 스냅샷 전용 lock)가 필요할 수 있음 — 지금은 필드별 mutex로 진행.

## 3. 스레드별 상세

### KinematicsThread (Th1) — run_kinematics, 1kHz

세 하위 단계로 구성 (다이어그램상 하나의 스레드/박스 안에 순서대로 그려짐):

1. **FK (관절 센서 기반)**: 관절 sen(pos,Torq) + Torso_quat → 실측 발 pose 계산 → 몸통 기준 직교 pos(Torso_quat)를 DataWarehouse에 씀
   - pelvis 월드 위치는 다리 기구학 오도메트리로 추정 (weld 유지 중엔 XY=0, Z=`hold_height_m`). 8절 7번 참고
2. **IK ("모델 로딩 / kinematics 생성기")**: DataWarehouse의 월드기준 타겟 com/발 포즈 → com 기준 타겟 포즈 계산 → IK → 몸통 기준 직교 타겟 Pos(Torso quat), 관절타겟(pos)을 DataWarehouse에 씀
3. **ID (Inverse Dynamics)**: MPC output(WBC(Th4)의 Fx,Fy,Fz,Tx,Ty) + IMU → 관절타겟토크 계산.
   - **hip은 IMU PD 제어**
   - **ankle pitch는 관절타겟 PD 제어** — 다이어그램에 "??"로 표시, **확인 필요**
   - **나머지 관절(knee, hip roll/yaw 등)은 WBC에서 오는 타겟 힘/토크(Fx,Fy,Fz,Tx,Ty) 기준으로 산출**
4. 최종 관절타겟(pos,Torq)을 **SendRefToSim**으로 전달 → `msg_ref`로 관절타겟(pos,Vel,Torq), 관절게인(kp_joint,kd_joint), Drop_robot_sim을 Simulate 프로세스에 전송

### MotionGenThread (Th2) — motion generation (보행 기준), 1kHz

- COM 궤적: **LIPM 해석해**(한발지지) + **3차 Hermite**(양발지지, 위치·속도 연속)
- 스윙발의 카티지안 타겟포즈: x,y **cosine**, z **1−cosine** (`s_land`=0.85에서 x 도착)
- 몸통 목표 자세: Identity 고정 (균형은 WBC)
- 발 위치 실시간 보정: 몸통 기울기·각속도(IMU)로 착지 위치 Δx,Δy (`gain foot kp kd`, 기본 0=꺼짐, 스윙 70% 이후 잠금)
- 구현: `include/lipm_gait.h`(header-only), 검증: `tools/gait_check`. 설계/수식: [walking_pattern_plan.md](walking_pattern_plan.md)
- 입력: 관절 sen(pos,vel), IMU(각도/각속도)
- 출력: Target pose(직교, 왼/오른발) → DataWarehouse

### CommandInputThread (Th3) — command input (비주기, 사용자 입력)

명령 종류:
- `motion`: walk, run
- `Com_pos/ori_Cal` — COM 위치를 command로 조절할 수 있는 기능
- `wbc [on/off]`
- `Gain[Kp(wbc), AdjKp(foot보정), Kpsim(관절)]`
- `Save [start, end]`
- `Drop [on, off]`
- `quit`

DataWarehouse에 Motion/Com/AdjKp,Kd/vel과 `wbc_enabled_`(wbc on/off)를 씀.

(참고: 다이어그램 원본에는 이 스레드에 번호가 없었다. 빈 채로 남아 있던 Th3 자리를
CommandInputThread(command input)에 배정하기로 정리함 — 2026-09-21)

### WholeBodyControlThread (Th4) — Whole Body Control (자세 MPC), 1kHz

- 슬라이드 "MPC"(몸통 관성 I + 질량 없는 다리) 모델의 QP를 OSQP로 풀어 **Fx, Fy, Fz, Tx, Ty** 산출, 200Hz 재계산(사이 틱은 유지). 설계/근거/추천: [mpc_plan.md](mpc_plan.md)
- 입력: IMU 각도/각속도(상태), 실측 pelvis·발 위치, 접촉 판정, MotionGen의 향후 발 위치/접촉 비중(`WalkPreview`, N=10, Δt=20ms)
- 지지발 교대는 양발지지의 정확히 중간(접촉 비중으로 표현), 출력 wrench는 접촉 비중대로 발별 분배
- 출력 → DataWarehouse. Kinematics(Th1)의 ID 단계가 이 출력을 받아 관절토크로 변환

### LoggingThread (Th5) — Logging

`save` 트리거를 받아 DataWarehouse 상태를 기록(파일 저장 추정, 세부 포맷은 다이어그램에 없음).
스냅샷을 읽는 시점에 다른 스레드의 쓰기와 겹치지 않도록 락이 필요함 (위 2절 동기화 참고).

## 4. Simulate 프로세스 (별도 프로세스, 이미 구현됨)

- `msg_ref`로 관절타겟(pos,vel,Torq) + 관절게인(kp_joint,kd_joint) + Drop_robot_sim 수신
- `mj_step` 루프 내부에서 pos/kp_joint, vel/kd_joint 기반 PID 제어 + torq offset(torq) 적용
- `msg_sen`으로 관절(pos,vel,Torq입력), IMU(ang/각속도) 회신
- **주석**: "실로봇에서는 모터드라이버가 이 역할"을 대신함 — 즉 Simulate 프로세스는 실기체 인터페이스의 스탠드인

## 5. IPC 프로토콜: 현재 vs 목표

현재 `Common/ipc_protocol.h`:

```cpp
constexpr key_t kMsgRefKey = 0x574B4A31;  // "WKJ1"
constexpr key_t kMsgSenKey = 0x574B4A32;  // "WKJ2"

struct RefMsg {
  long mtype = 1;
  double targ_angle_rad = 0.0;
  double velocity_rad_s = 0.0;
};

struct SenMsg {
  long mtype = 1;
  double angle_rad = 0.0;
  double velocity_rad_s = 0.0;
  double sim_time_s = 0.0;
};
```

| | 현재 (pendulum 데모) | 목표 (다이어그램) |
|---|---|---|
| msg_ref 내용 | 스칼라 각도 1개 + 속도 1개 | 관절타겟 벡터(pos,Vel,Torq) × N관절 + 관절게인(kp_joint,kd_joint, 전 관절 공통 스칼라 각 1개) + Drop_robot_sim |
| msg_sen 내용 | 스칼라 각도/속도/sim_time | 관절 벡터(pos,vel,Torq입력) × N관절 + IMU(ang,각속도) |
| 대상 모델 | pendulum(관절 1개) | G1(다리 12 + 허리 yaw 1 = 13관절, `Controller/model/g1/g1.urdf` 기준) |

## 6. 현재 구현 상태 vs 목표 (Gap)

| 항목 | 현재 | 목표 | 확인한 파일 |
|---|---|---|---|
| IPC 프로토콜 | pendulum 스칼라 전용 | 다관절 벡터 + 게인 + Drop 플래그 + IMU | `Common/ipc_protocol.h` |
| Controller 스레드 구조 | 싱글스레드, ramp 루프 1개 | Kinematics(Th1)/MotionGen(Th2)/CommandInput(Th3)/WBC(Th4)/Logging(Th5) 5개 스레드 + DataWarehouse | `Controller/src/main.cpp` |
| DataWarehouse | 없음 | 전역 블랙보드 class | — |
| 동기화(mutex) | 없음 | 필드별 mutex + private 변수 + 필드별 get/set 접근자 (위 2절 참고) | — |
| Pinocchio 모델 로드 | `LoadG1Model()`로 G1 URDF 로드는 되어 있음 | Kinematics(Th1)의 FK/IK/ID에서 실제로 사용 | `Controller/include/g1_model.h` |
| MPC/WBC | 없음 (OSQP는 CMakeLists에 링크만 됨) | WBC(Th4): 자세 MPC(OSQP QP)로 Fx,Fy,Fz,Tx,Ty 산출 | `Controller/CMakeLists.txt` |
| Motion generation | 없음 | MotionGen(Th2): LIPM 해석해 COM + DSP Hermite + cosine 스윙발 + 몸통 기울기 발 위치 보정 | — |
| Simulate 쪽 제어기 | `PidControl()` — `"hinge"` 조인트 이름 하드코딩, pendulum 전용 | 다관절 일반화 (관절명 리스트 기반 PID) | `Mujoco/mujoco_engine/simulate/pid_ipc.{h,cc}` |
| command input / Logging 스레드 | 없음 | 다이어그램대로 별도 스레드 | — |

## 7. 용어 정리 (다이어그램 표기 그대로)

| 용어 | 의미 |
|---|---|
| FK | Forward Kinematics — 관절 센서값 → 실측 자세 |
| IK | Inverse Kinematics — 타겟 자세 → 관절 목표값 |
| ID | Inverse Dynamics — 힘/모멘트 목표 → 관절 토크 목표 |
| LIPM | Linear Inverted Pendulum Model — 지지발 타겟 포즈 계산에 사용 |
| WBC | Whole Body Control — WholeBodyControlThread(Th4), MPC 기반 |
| kp_joint / kd_joint | Simulate 프로세스의 PID 게인 (msg_ref로 전달) |
| Kp(wbc) | WBC 쪽 게인 (command input에서 조정) |
| AdjKp | foot 보정용 게인 조정치 |
| Kpsim(관절) | command input에서 조정하는 관절별 kp_joint |
| `wbc_enabled_` | DataWarehouse 필드, wbc on/off |
| `control_mode_` | DataWarehouse 필드, 1=작업공간, 2=관절공간 |
| Drop_robot_sim | 기본 상태: 로봇을 지면에서 약 10cm 띄운 채 몸통을 6자유도로 고정(constraint)해서 붙잡고 있음. `Drop`=그 고정을 풀어 로봇을 놓아버림 (2026-09-21 확정) |

## 8. 확인이 필요한 미확정 사항

1. ~~**Th3의 역할** — 다이어그램에 정의 없음 (TBD)~~ → CommandInputThread(command input)로 배정 완료 (2026-09-21, 위 3절 참고)
2. ~~**Th1 ID 단계 나머지 관절의 토크 산출 방식**~~ → knee, hip roll/yaw 등 나머지 관절은 WBC 타겟
   힘/토크(Fx,Fy,Fz,Tx,Ty) 기준으로 확정 (2026-09-21). **hip은 Imu PD, ankle pitch는 관절타겟 PD** 부분만
   여전히 다이어그램 원문 "??" 그대로 확인 필요
3. ~~**Fx,Fy,Fz,Mx,My vs Fx,Fy,Fz,Tx,Ty** 표기 혼용~~ → **Tx,Ty로 통일** 확정, 문서 전체 반영 완료 (2026-09-21)
4. ~~**Drop_robot_sim의 정확한 의미/트리거 조건**~~ → 확정 (2026-09-21, 위 7절 참고): 기본 상태는 로봇을
   10cm 공중에 띄운 채 몸통 6자유도 고정, Drop 시 고정 해제
5. **Logging 스레드의 저장 포맷/주기** — 아직 미정, 일단 비워둠 ("save [start,end]"만 있고 파일 형식 등은
   명시 안 됨)
6. ~~**DataWarehouse 동기화 방식**~~ → **필드별 mutex** + private 변수 + 필드별 get/set 접근자로 확정
   (2026-09-21, 위 2절 참고). 단 Logging의 다중 필드 스냅샷 일관성은 residual 트레이드오프로 남음

7. **pelvis 월드 위치 추정 (구현됨, 2026-09-30, `LegOdometry` in `kinematics_thread.cpp`)** — 접촉/Z는 낮은 발 기준, XY는 지지발 고정 가정:
   - Z: `pelvis_z = -(R·p_foot_in_pelvis).z`, 두 발 중 더 낮은 쪽을 지면으로 간주 → 접촉 센서 불필요
   - XY: `pelvis_xy = foot_world_xy - (R·p_foot_in_pelvis).xy` (지지발은 월드 고정). 지지발 전환 시 새 지지발의 월드 위치를 현재 추정값으로 재설정
   - 한계: 미끄러짐/전환 오차 누적, yaw는 IMU 드리프트 → 초기값 대비 상대 위치로만 유효
   - 접촉 판정은 "가장 낮은 발 대비 `kFootContactHeightM` 미만"으로 변경 (가정 Z 순환 해소). 히스테리시스 없음
   - 후속 과제: WBC Fx/Fy를 오도메트리 XY 기반으로 활성화 검토, 실측 드리프트 검증

## 9. 변수명 정리 (제안, 2026-09-21)

실제 구현에서 쓸 C++ 이름을 미리 확정해서, DataWarehouse 클래스를 쓸 때 이름을 다시 고민하지
않게 한다. **네이밍 컨벤션**: private 멤버는 트레일링 언더스코어(`name_`), getter는 `GetXxx()`, setter는
`SetXxx()` (기존 코드의 `LoadG1Model`, `HandleSigint` 같은 PascalCase 함수명과 일관). 벡터/쿼터니언은
Pinocchio가 이미 쓰는 `Eigen` 타입을 그대로 사용. 관절 벡터 크기는 **13** (G1: 다리 hip_pitch/roll/yaw,
knee, ankle_pitch/roll ×2 + waist_yaw 1, `Controller/model/g1/g1.urdf`의 revolute 관절 순서 기준).

### 9.1 DataWarehouse 필드

| 그룹 | 멤버 변수명 | 타입 | getter | setter | 다이어그램 원문 |
|---|---|---|---|---|---|
| 실측(FK 입력) | `joint_pos_sen_` | `Eigen::VectorXd`(13) | `GetJointPosSen()` | `SetJointPosSen()` | 관절 sen(pos) |
| 실측(FK 입력) | `joint_vel_sen_` | `Eigen::VectorXd`(13) | `GetJointVelSen()` | `SetJointVelSen()` | 관절 sen(vel) |
| 실측(FK 입력) | `joint_torq_sen_` | `Eigen::VectorXd`(13) | `GetJointTorqSen()` | `SetJointTorqSen()` | 관절 sen(Torq) |
| 실측(FK 결과) | `torso_pose_meas_world_` | `Pose3d`(`Eigen::Vector3d` + `Eigen::Quaterniond`) | `GetTorsoPoseMeasWorld()` | `SetTorsoPoseMeasWorld()` | 몸통 기준 직교 pos(Torso_quat) |
| 실측(FK 입력) | `imu_ang_` | `Eigen::Vector3d` | `GetImuAng()` | `SetImuAng()` | Imu(기울기) |
| 실측(FK 입력) | `imu_angvel_` | `Eigen::Vector3d` | `GetImuAngVel()` | `SetImuAngVel()` | Imu(각속도) |
| 실측(FK 입력) | `foot_contact_` | `std::array<bool,2>` | `GetFootContact()` | `SetFootContact()` | Foot_contact_switch |
| IK — Kinematics(Th1) 결과 | `joint_targ_pos_` | `Eigen::VectorXd`(13) | `GetJointTargPos()` | `SetJointTargPos()` | 관절타겟(pos) |
| 모션생성 — MotionGen(Th2) 출력 | `torso_quat_targ_` | `Eigen::Quaterniond` | `GetTorsoQuatTarg()` | `SetTorsoQuatTarg()` | 몸통 목표 **자세**(방향만). 위치는 `com_targ_world_` 하나로만 전달(예전 `Pose3d`의 중복 위치 필드 제거) |
| ID — Kinematics(Th1) 결과 | `joint_targ_torq_` | `Eigen::VectorXd`(13) | `GetJointTargTorq()` | `SetJointTargTorq()` | 관절타겟(joint_torq_) |
| 모션생성 — MotionGen(Th2) 출력 | `foot_targ_world_` | `struct{Eigen::Vector3d left, right;}` | `GetFootTargWorld()` | `SetFootTargWorld()` | World 기준 Target pose(직교, 왼/오른발) |
| 모션생성 — MotionGen(Th2) 출력 | `com_targ_world_` | `Eigen::Vector3d` | `GetComTargWorld()` | `SetComTargWorld()` | 월드기준 타겟 com 포즈 |
| WBC(Th4) 출력 | `wbc_force_torque_` | `struct ContactWrench { double fx, fy, fz, tx, ty; }` (발별 좌/우 각각) | `GetWbcForceTorque()` | `SetWbcForceTorque()` | Mpc command: Fx,Fy,Fz,Tx,Ty |
| 커맨드 | `motion_mode_` | `enum class MotionMode{kWalk,kRun}` | `GetMotionMode()` | `SetMotionMode()` | motion: walk/run |
| 커맨드 | `com_pos_ori_cal_` | `Eigen::Vector3d` + `Eigen::Quaterniond` | `GetComPosOriCal()` | `SetComPosOriCal()` | Com_pos/ori_Cal |
| 커맨드 | `vel_cmd_` | `double` | `GetVelCmd()` | `SetVelCmd()` | vel |
| 모드/플래그 | `wbc_enabled_` | `bool` | `GetWbcFlag()` | `SetWbcFlag()` | `wbc_flag.bool_data` |
| 모드/플래그 | `control_mode_` | `int`(1=작업공간,2=관절공간) | `GetControlMode()` | `SetControlMode()` | `Control_mode.int_data` |
| 게인 | `kp_wbc_` | `Eigen::VectorXd`(6, double) | `GetKpWbc()` | `SetKpWbc()` | Gain[Kp(wbc)] |
| 게인 | `kp_foot_` / `kd_foot_` | `double` / `double` | `GetKpFoot()` / `GetKdFoot()` | `SetKpFoot()` / `SetKdFoot()` | kp_foot/kd_foot(foot보정) — 전 관절 공통 스칼라 1개씩 |
| 게인 | `kp_joint_` / `kd_joint_` | `double` / `double` | `GetKpJoint()` / `GetKdJoint()` | `SetKpJoint()` / `SetKdJoint()` | 관절게인(Kp_joint,Kd_joint) — 전 관절 공통 스칼라 1개씩 |
| 시뮬레이터 연동 | `drop_robot_` | `bool` | `GetDropRobot()` | `SetDropRobot()` | Drop_robot, Drop[on,off] |
| 로깅 | `save_flag_` | `enum class SaveFlag{kIdle,kStart,kEnd}` | `GetSaveFlag()` | `SetSaveFlag()` | Save[start,end] |


### 9.2 IPC 메시지 필드 (`Common/ipc_protocol.h` 확장안)

메시지 큐를 오가는 구조체는 프로세스 경계를 넘는 POD여야 하므로 DataWarehouse와 달리 **Eigen이
아닌 고정 크기 배열**을 쓴다 (관절 수 13 고정).

**RefMsg (Controller → Simulate, `msg_ref`)**:

| 필드명 | 타입 | 다이어그램 원문 |
|---|---|---|
| `joint_targ_pos[13]` | `double[13]` | 관절타겟 pos |
| `joint_targ_vel[13]` | `double[13]` | 관절타겟 Vel |
| `joint_targ_torq[13]` | `double[13]` | 관절타겟 Torq |
| `kp_joint` | `double` | 관절게인 kp_joint — 전 관절 공통 스칼라 1개 |
| `kd_joint` | `double` | 관절게인 kd_joint — 전 관절 공통 스칼라 1개 |
| `drop_robot_sim` | `bool` | Drop_robot_sim |
| `drop_robot_sim` | `bool` | Drop_robot_sim |

**SenMsg (Simulate → Controller, `msg_sen`)**:

| 필드명 | 타입 | 다이어그램 원문 |
|---|---|---|
| `joint_pos[13]` | `double[13]` | 관절(pos) |
| `joint_vel[13]` | `double[13]` | 관절(vel) |
| `joint_torq[13]` | `double[13]` | 관절(Torq입력) |
| `imu_ang[3]` | `double[3]` | Imu(ang) |
| `imu_angvel[3]` | `double[3]` | Imu(각속도) |
