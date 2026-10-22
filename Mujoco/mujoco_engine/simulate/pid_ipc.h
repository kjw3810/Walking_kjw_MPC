// G1 13개 관절용 PID 서보. mjcb_control 콜백으로 mj_step마다 실행됨.
// G1이 아닌 모델이 로드되어 있으면 no-op. 자세한 설명: 코드 해설 문서 참고.
#pragma once

#include <mujoco/mujoco.h>

namespace walking_kjw {

void PidControl(const mjModel* m, mjData* d);

}  // namespace walking_kjw
