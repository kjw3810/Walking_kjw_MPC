# 보행 패턴 생성 계획 (MotionGen) — LIPM 해석해 + 양발지지 다항식 + 발 위치 실시간 보정

> 승인 후 첫 작업: 이 문서를 `Controller/docs/walking_pattern_plan.md` 로 복사 (plan 모드에서는 이 파일만 쓸 수 있어 여기 작성함).
> 우선순위: **슬라이드(사진 5장) > 기존 `TickWalking` 코드**. 기존 보행 코드(오일러 적분 LIPM, 0.35/0.15 타이밍, 선형보간 스윙)는 전부 버리고 새로 작성한다.
> WBC는 이번 범위 밖 (`SetWbcFlag(mode==kWalk)` 기존 동작 유지). 보행 "패턴 생성"에만 집중.

---

## 1. 목표와 범위

| 항목 | 내용 |
|---|---|
| 하는 것 | (a) 슬라이드의 LIPM 해석해로 COM 궤적, (b) cosine 스윙발 궤적, (c) 양발지지(DSP)를 매끄러운 다항식으로 연결, (d) 몸통 기울기·기울기속도로 발 착지 위치를 실시간 보정 |
| 안 하는 것 | WBC 튜닝, 방향 전환(yaw), 정지 시퀀스(양발 모으기), 뒤꿈치/발끝 (heel strike / toe off, 슬라이드에서도 꺼둔 상태), Run |
| 바꾸는 파일 | `Controller/src/motion_gen_thread.cpp` (TickWalking 교체), `Controller/include/lipm_gait.h` (신규, 순수 수식), `Controller/tools/gait_check.cpp` (신규, 오프라인 검증), `CMakeLists.txt` (gait_check 추가), `Controller/docs/architecture.md` (MotionGen 절 갱신) |
| 안 바꾸는 인터페이스 | 출력은 그대로 `SetFootTargWorld` / `SetComTargWorld` / `SetTorsoQuatTarg`. Kinematics(IK)가 그대로 소비 (`kinematics_thread.cpp`: pelvis≈COM, 발목 프레임에 `kFootSoleOffsetM` 보정) |

기본 파라미터(사용자 확정): **T=1.0 s(처음 0.7 s에서 변경), ρ=0.2, y0=0.09 m, 스윙 높이 0.05 m, v0=0.2~0.3 m/s(시작 0.25)**, h=전신 COM 높이 `kNominalComHeightM`=0.664 m (처음 계획은 pelvis 프레임 0.743 m).

---

## 2. 슬라이드 해석 (수식의 근거)

### 2.1 LIPM (슬라이드 1)
- 슬라이드 빨간 메모대로 `sinθ`가 아니라 `tanθ`, `l`이 아니라 `h`: `ẍ = (g/h)·x = ω²x`, **ω = √(g/h) = 3.634 rad/s**
- 해석해 `x(t) = a·sinh(ωt) + b·cosh(ωt)`, a·b는 초기/최종 조건으로 결정.

### 2.2 좌표/기호
- 지지발 기준 상대좌표. 측면은 **d = 지지발에서 몸 중심선 쪽으로 잰 거리**(항상 > 0), `y_world = y_지지발 − sgn·d` (sgn: 왼발 +1, 오른발 −1. 월드 y는 왼쪽이 +, 기존 코드와 동일: 왼발 +y0, 오른발 −y0).
- 슬라이드 그림: 발 간격 `2·y0`, COM은 좌우 극단 사이를 왕복, 극단 위치는 지지발에서 `y1`, `y0 = y1 + y2`, **COM sway 폭 = 2·y2**.
- 슬라이드의 stage 1~4는 각각 T/2. **지지발은 COM이 중앙선을 지날 때 교대**(stage 1→2). 즉 한 발 지지 구간은 "중앙선 → 극단(속도 0) → 중앙선" 대칭 구조이고 길이는 슬라이드의 T (구현에서는 `Ts`).

### 2.3 측면(y) — 슬라이드 2, 3
- Stage 1: `d(t) = y1·cosh(ωt)`, `d(0)=y1, ḋ(0)=0`, `y0 = y1·cosh(ωT/2)` → `y1 = y0/cosh(ωT/2)`
- ⚠️ **슬라이드 3 (Stage 2) 식 불일치**: 슬라이드의 `a = −sinh²(ωT/2)/cosh(ωT/2)·y0` 는 F.C. `ẏ(T/2)=0` 을 만족하지 않는다. 속도 연속(stage 1 끝 속도 = `ω·y0·tanh(ωT/2)`)에서 유도하면 **`a = −y0·tanh(ωT/2)`**, 이때 `ẏ(T/2)=0` 자동 만족, `y(T/2) = y0/cosh(ωT/2) = y1` (지지발에서 극단까지 거리 y1 — 그림과 일치). 슬라이드의 `y0−y1` 도 그림상 `y2`이지 지지발 기준 거리가 아니다.
  → **구현은 유도한 식을 쓰고, `gait_check`에서 주기성으로 확인**한다(슬라이드 식은 검증 대상으로 남김).
- 대칭 형태로 쓰면 더 간단: **SSP 전체를 `d(t) = y1·cosh(ω(t − Ts/2))`, t∈[0,Ts]** 한 줄로 표현 (stage 4+1, 또는 2+3).

### 2.4 전진(x) — 슬라이드 4
- Stage 1: `x(t) = v0/(ω·cosh(ωT/2)) · sinh(ωt)`, `x(0)=0`(지지발 바로 위), `ẋ(T/2)=v0`
- 반 구간 이동량 `x0 = v0·tanh(ωT/2)/ω`, 슬라이드 결론 "보폭 = 2·x0".
- SSP 대칭 형태: **`x(t) = [v0/(ω cosh a)]·sinh(ω(t − Ts/2))`, a=ωTs/2**, x: −x0 → +x0, 양 끝 속도 v0.
- v0=0 이면 x0=0, 보폭 0 → **제자리 걸음(슬라이드 2, 3)이 그대로 특수해.**

---

## 3. 양발지지(DSP) 구간 — 사용자 요청 반영

슬라이드는 DSP가 없다(DSP=0인 특수 경우). 여기에 T_dsp를 넣는다.

- **시간 배분**: `T_step = T = 1.0 s`, `T_dsp = ρ·T` (ρ = 0.2 → 0.2 s), `Ts = T − T_dsp = 0.8 s` (슬라이드의 T 역할). ρ=0 이면 슬라이드 원형으로 환원.
- **DSP 동안 COM**: 한 발 지지 끝 상태 → 다음 한 발 지지 시작 상태로 **3차 Hermite 다항식**(위치+속도 양 끝 일치)으로 이동. 각 축(x, y) 독립.

  ```
  u = t / Td                                  (0→1)
  p(u) = (2u³−3u²+1)·pa + (u³−2u²+u)·Td·va + (−2u³+3u²)·pb + (u³−u²)·Td·vb
  ```
  - `pa, va` = 직전 SSP 끝의 COM 위치/속도, `pb, vb` = 다음 SSP 시작 상태(다음 지지발 기준 해석해의 t=0 값).
  - 양 끝 속도가 맞으므로 **SSP↔DSP 경계에서 속도 불연속(충격) 없음**. 식이 한 줄이라 이해하기 쉽다.
  - 왜 단순 선형이 아니라 Hermite인가: 속도가 v0인 상태로 이어지려면 "선형 보간"이면 충분하지만, 아래 §6의 발 위치 보정으로 끝점이 바뀌면 선형은 속도 불연속이 생긴다. Hermite는 끝점이 바뀌어도 항상 C¹.
- **공칭(보정 없을 때) 조건 — DSP가 정확히 직선(등속)이 되도록 y1, 보폭을 정함**:
  - 측면: 극단까지 `y1 = 2·y0 / (2·cosh a + ω·Td·sinh a)`, SSP 끝 거리 `y_s = y1·cosh a`, 끝 횡속도 `ẏ_e = ω·y1·sinh a`. (DSP 이동거리 `2(y0−y_s) = ẏ_e·Td` 를 만족)
  - 전진: `x0 = v0·tanh(a)/ω`, **보폭 `S = 2·x0 + v0·Td`**
  - 손계산(T=.7, ρ=.2, v0=.25): ω=3.634, a≈1.017, tanh≈0.77 → x0≈0.053 m, S≈0.141 m, y1≈0.048 m, y_s≈0.075 m, ẏ_e≈0.21 m/s, 발간격 대비 sway 폭 2·y2≈0.085 m (**근사값, gait_check로 확정**).
- **vel 명령 해석**: `vel` = 평균 보행 속도로 취급. `v0 = vel·T / (2·tanh(a)/ω + Td)` (S/T = vel 이 되도록). 한 스텝 변화폭 제한(`Δv0_max`, 예 0.05 m/s/step)으로 속도 명령 급변 방지.

---

## 4. 스윙발 궤적 — 슬라이드 5 (Sine 함수 사용)

스윙 구간 = SSP 전체(`Ts`), 진행률 `s = t/Ts ∈ [0,1]`.

| 축 | 식 |
|---|---|
| X (진행) | `s_x = min(s / s_land, 1)`, `x = x_start + (x_target − x_start)·(1 − cos(π·s_x))/2` — **X: cosine** |
| Z (높이) | `z = (h_step/2)·(1 − cos(2π·s))` — **Z: 1−cosine**, 이륙/착지 속도 0 |
| Y | X와 같은 cosine 보간 (발 간격 y0 유지 / 보정 반영) |
| 착지 근처 수직 하강 | `s_land = 0.85` — x가 s_land에서 먼저 도착하고 남은 15%는 z만 내려옴 |

- 5차 다항식/Bezier는 슬라이드에서 "Cf"(비교 참고)이므로 **채택하지 않음** (Bezier=착지 충격, 5차=바닥에 붙어 끌림).
- 이륙·착지 직전 z가 매우 작은 구간이 있어 끌림 위험 → 검증 시 `z(s=0.05, 0.95)` 값 확인, 필요 시 `s_land`/`h_step` 조정.
- Heel strike / Toe off: 슬라이드상 구현은 있으나 꺼둔 상태 → 이번엔 `kEnableHeelToe=false` 상수와 TODO 주석만 둠(구현 안 함).

---

## 5. 전체 상태머신 (한눈에)

```
Init(T_init=0.5s)           정지 자세 → 첫 지지(왼발) 중간 상태로 Hermite
   │   (COM: x_rel=0, d=y1, ḋ=0, ẋ=v0/cosh a  ← 슬라이드 stage 1 시작 조건)
   ▼
SSP_L(반쪽, Ts/2)          슬라이드 stage 1. 오른발 스윙 시작
   ▼
DSP (Td)                    COM: Hermite. 양발 지면
   ▼
SSP_R(전체, Ts)  ──DSP──►  SSP_L(전체, Ts) ──DSP──► …  (지지발 교대, 스윙발 = 반대발)
```
- 첫 지지발은 왼발(기존 `StartSwing(1,…)` 동작과 동일: 오른발이 먼저 스윙).
- 매 스텝 `Params` 래치: `vel`은 **DSP 시작 시점(및 Init 시작)에** 한 번 읽어 다음 스텝 v0로 확정 → SSP 내부에서 파라미터가 바뀌어 궤적이 끊기는 일 방지 (속도 변화는 한 스텝 뒤 반영).
- `walkready → walk` 전환 시 `WalkingState` 리셋 (현재는 리셋 안 됨 → 모드 진입 때 `phase_initialized=false`로 재시작하도록 수정).
- COM 높이 z = `kNominalStandHeightM` 고정, 몸통 목표 자세 = Identity (기존과 동일; 몸통 roll 포물선은 안 함).
- **지지발 월드 위치 앵커**: 각 SSP에서 `COM_world = 지지발_world(실제 착지 위치) + 상대해석해`. 착지 위치가 보정으로 바뀌어도 COM 기준이 새 발을 따라가고, 그 차이는 DSP Hermite가 매끄럽게 흡수.

---

## 6. 몸통 기울기 기반 발 위치 실시간 보정 (신규)

### 6.1 아이디어
몸이 앞으로 기울거나(뒤에서 밀림) 기울기 속도가 크면 COM이 지지발보다 앞서 나가고 있다는 뜻 → 스윙발을 그만큼 앞으로 더 멀리 딛는다. (capture point: `ξ = x + ẋ/ω`, 작은 각도에서 `x ≈ h·θ`, `ẋ ≈ h·θ̇`)

### 6.2 식
```
Δx =  Kθ · pitch + Kω · ω_y            (앞으로 기울면(pitch>0) 발을 더 앞으로)
Δy = −(Kθ · roll  + Kω · ω_x)          (오른쪽으로 기울면(roll>0) 오른쪽(−y)으로)
```
- 부호 근거: `quat_euler.h`는 x=roll, y=pitch, z=yaw(ZYX), x 앞, y 왼쪽, z 위 → +pitch = 앞으로 숙임, +roll = 오른쪽으로 기움.
- 이론 기본값: `Kθ = α·h ≈ 0.37 m/rad`, `Kω = α·h/ω ≈ 0.10 m·s/rad` (α=0.5). 실기(시뮬) 튜닝 대상.
- **게인은 기존 `DataWarehouse::kp_foot_ / kd_foot_` 재사용** (`gain foot <kp> <kd>` 명령 이미 존재, 현재 미사용, 기본 0 = 보정 꺼짐). Kθ=kp_foot, Kω=kd_foot. 새 명령/필드 추가 없음. x/y 별도 게인이 필요하면 그때 확장.

### 6.3 처리 파이프라인 (매 틱, 1 kHz)
1. 입력: `dw.GetImuAng()`(roll,pitch,yaw), `dw.GetImuAngVel()`. 목표 몸통 기울기는 0(Identity)이므로 기울기 오차 = 측정값.
2. 1차 저역통과: 각속도 fc≈15 Hz, 각도 fc≈30 Hz (노이즈, 보행 중 자연 흔들림 제거).
3. **데드밴드**: 각도 0.02 rad(≈1°), 각속도 0.1 rad/s 이하는 0 (평상 보행 시 보정이 계속 켜지지 않게).
4. 식 적용 → `Δx, Δy`.
5. **제한**: `Δx ∈ [−0.05, +0.12] m`, `Δy`는 스윙발 기준 바깥쪽 +0.06 m / 안쪽 −0.03 m, 그리고 좌우 발 간격 최소 0.12 m 유지(발 교차/충돌 방지).
6. **변화율 제한** ≤ 1 m/s (보정값이 튀어 발이 순간이동하지 않게).
7. 적용: 스윙 중 `x_target = x_nominal + Δx`. 스윙발 위치가 `start + w(s)·(target − start)` 형태라 Δ가 연속이면 발 궤적도 연속.
8. **잠금(freeze)**: `s > s_lock = 0.7` 이후에는 Δ를 고정 → 착지 직전 발이 흔들리지 않고, DSP 시작 시 착지 위치가 확정되어 Hermite 끝점 계산에 사용 가능.
9. DSP 중에는 발이 지면에 있으므로 보정 없음(다음 스텝 스윙부터 반영).

### 6.4 알려진 한계 (문서에 명시)
- COM 기준궤적은 착지한 새 발에 다시 앵커되는 방식(§5)이라 보정으로 넓게 딛으면 다음 SSP에서 기준 COM이 새 발 뒤쪽 −x0에서 다시 시작 → 기준 COM이 조금 앞으로 당겨짐. 그 이동량은 DSP Hermite로 부드럽게 흡수되며, 큰 보정은 §6.3-5 제한으로 막는다. 더 물리적인 방식(기준 COM 상태를 이어받아 일반해 사용, 큰 외란 시 다음 v0 감속)은 WBC 작업 이후 개선안으로 남김.
- 몸통 기울기 ≠ COM~발 각도이므로 `x≈hθ` 는 근사. 게인 튜닝으로 보완.
- IMU 각속도의 기준 좌표계(body/world)는 구현 전 확인 필요 (`Mujoco/simulate` 쪽 `imu_angvel` 채우는 부분 확인; 작은 각도에서는 차이 미미).

---

## 7. 구현 단계 (순서 = 검증 가능한 단위)

1. **`Controller/include/lipm_gait.h` (신규, header-only, 외부 의존 없음)**
   - `GaitParams{h, T, rho, y0, v0, h_step, s_land}` → 파생값 `{omega, Ts, Td, a, x0, y1, y_s, ydot_e, S}` 계산
   - `SspRel(t)` (지지발 기준 x, d, 속도), `Hermite(pa,va,pb,vb,Td,t)`, `SwingFoot(s, start, target, h_step, s_land)`, `V0FromVel(vel, params)`
   - Eigen/DataWarehouse 미사용 → 오프라인 도구와 스레드가 같은 코드를 쓴다.
2. **`Controller/tools/gait_check.cpp` (신규)** — 시뮬레이터 없이 실행, `ik_check`와 같은 방식으로 CMake 타깃 추가. 1 ms 틱으로 수 스텝 생성 후 다음을 자동 판정(PASS/FAIL 출력, CSV 덤프):
   - 주기성: 연속 SSP의 끝/시작 상태 일치 (특히 §2.3 슬라이드 식 vs 유도식 비교)
   - COM 위치/속도 연속(경계에서 |Δp|, |Δv| ≈ 0), 최대 가속도 크기 출력
   - 발 위치 연속, 착지 시 z=0, 스윙 최고 높이 = h_step, 착지 z속도 ≈ 0
   - v0=0 제자리 걸음, vel 변화 시 연속성
   - 발 보정: 인위적 pitch/pitch_rate 입력에 대한 Δx 부호·제한·잠금 동작
3. **`motion_gen_thread.cpp` 교체** — `WalkingState`/`TickWalking`을 위 상태머신으로 재작성. 기존 `kWalkLeftSupportS` 등 상수 삭제. 모드 진입 시 리셋. 보정 필터 상태(LPF, rate limiter)는 `WalkingState`에 포함. 나머지 모드(UpAndDown, WalkReady, Run)는 건드리지 않음.
4. **로깅**: `logging_thread.cpp`에 스윙발/지지발 목표, 보정 Δx/Δy, 위상 인덱스 컬럼 추가 검토 (현재는 COM 목표만 기록). `DataWarehouse`에 디버그 필드가 필요하면 최소한으로 추가.
5. **문서**: `architecture.md`의 MotionGen 절(“sine 곡선 / 포물선 몸통 roll” 표현)을 이번 설계로 갱신하고 `Controller/docs/walking_pattern_plan.md` 링크.

---

## 8. 검증 계획

1. **오프라인 (`gait_check`)** — 위 2단계 항목 전부 PASS. 손계산 표(§3)와 수치 비교.
2. **빌드**: `./build_controller.sh` (경고 없이), `gait_check` 실행.
3. **시뮬레이션**(`./run_mujoco.sh Mujoco/model/g1/g1.xml` + `./run_controller.sh`), 순서대로:
   1. `motion walkready` → `drop auto` → `motion walk`, `vel 0` (제자리 걸음): 좌우 sway·발 들기 확인, IK 실패/관절 한계 없는지
   2. `vel 0.1` → `0.2` → `0.25`: 보폭·착지 충격, `save start/end` 로그로 COM·발 목표 연속성 확인
   3. `gain foot 0.37 0.10` 후 시뮬 창에서 뒤에서 밀기(Ctrl+드래그): 기울면 발 목표가 앞으로 길어지는지(로그 Δx), 한계/잠금 동작
   4. 문제 시 `gain foot 0 0`(보정 끔), `vel 0`, DSP 비율 ρ=0 (슬라이드 원형)로 좁혀 원인 분리
4. 결과가 안정 보행이 아니어도 **이번 목표는 패턴 생성의 정확성** — 추종 성능은 WBC 작업에서 다룬다.

---

## 9. 결정 사항 / 가정 (이의 있으면 알려주세요)

- DSP 비율 ρ=0.2(0.14 s)를 기본값으로 둠 (기존 코드 30%보다 짧음. 슬라이드 원형은 0). 상수 하나로 변경 가능.
- DSP 보간은 3차 Hermite (선형 보간을 매끄럽게 확장한 형태, 위치+속도 연속).
- 슬라이드 3의 Stage 2 계수는 유도식(`−y0·tanh(ωT/2)`)을 채택, gait_check로 확인 후 문서에 근거 기록.
- 보정 게인은 기존 `kp_foot/kd_foot` 재사용, 기본 0(꺼짐)으로 시작해 시뮬에서 명령으로 켠다.
- `vel` = 평균 속도, 변경은 다음 스텝부터 반영.


---

## 10. 상체 카운터 로테이션 (허리 yaw) — 추가
- 모델에는 허리 roll이 없고 yaw(±2.618 rad) 하나뿐이다(상체 14.8 kg이 한 덩어리). 그래서 **허리 yaw**로 상체를 비튼다.
- 식: `허리 yaw = −k·(x_오른발 − x_왼발)`, `|yaw| ≤ waist_max`, 변화율 ≤ 1 rad/s. 오른발이 앞이면 상체는 반대(시계 방향, −). 보폭에 비례해 속도와 함께 커지고 제자리 걸음(S=0)에서는 0.
- 구현: `lipm_gait.h` `WalkOutput::waist_yaw` → MotionGen `WaistYawTarg` → Kinematics task-space IK의 허리 관절 목표. 명령 `gain waist <k> [max]`(기본 k=0.7 rad/m, max=0.15 rad, 0이면 꺼짐).
- 폐루프 시험(v=0.2 m/s, T=1.0 s, WBC on, 20~32 s의 pelvis yaw 진동): k=0 → RMS 0.30° / k=+0.35 → 0.27° / **+0.70 → 0.25°** / +1.0 → 0.23°(상한 0.15 rad에 자주 걸림) / +1.5 → 0.28° / **−0.35 → 0.30°(개선 없음)**. 부호는 +(카운터 로테이션)가 맞고, 기본값은 상한에 걸리지 않으면서 효과가 있는 0.7로 정했다. 걸음 거리(약 3.5 m)와 몸통 기울기(≤0.035 rad)는 대조와 같다. 허리 관절은 목표를 RMS 0.001 rad로 따라간다.
- 한계: 이 시뮬에서 pelvis yaw 흔들림은 원래 0.3° 수준으로 작아서 "흔들림 감소" 효과는 작다. 주목적은 **자연스러운 모양**이다. 팔 관절이 없어 팔 스윙은 없다.
