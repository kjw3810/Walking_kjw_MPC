// ID(J^T * wrench) 오프라인 검증. MuJoCo를 정답으로 사용. 사용: Controller/build/id_check
//
// 정적으로 서 있을 때 (양발 지면, 속도 0):
//   MuJoCo 평형:  tau_j + sum_feet Jc_j^T x_foot = qfrc_bias_j   (모든 dof, 베이스 tau=0)
//   베이스 6행으로 발 접촉 wrench x (지면이 로봇에 가하는 힘/토크, 월드, 발 프레임 원점) 를 구하고
//   관절에 필요한 토크 tau_req = qfrc_bias_j - sum Jc_j^T x  를 얻는다.
// 컨트롤러는 tau = J_pin^T * w (LOCAL_WORLD_ALIGNED) 를 쓴다. w = x (WBC의 fz>0은 지면이 로봇을 미는 힘)
// 로 넣었을 때 +J^T w 와 -J^T w 중 어느 쪽이 tau_req 와 맞는지(나머지는 발 아래 링크 자중) 확인한다.
#include <mujoco/mujoco.h>

#include <cstdio>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "g1_joints.h"
#include "g1_model.h"
#include "leg_ik.h"
#include "quat_euler.h"

using namespace walking_kjw;

int main() {
  const pinocchio::Model model = LoadG1Model();
  pinocchio::Data data(model);
  char err[1000] = "";
  mjModel* m = mj_loadXML(MUJOCO_G1_XML, nullptr, err, sizeof(err));
  if (!m) { std::printf("MuJoCo 로드 실패: %s\n", err); return 1; }
  mjData* d = mj_makeData(m);

  int qadr[kNumJoints], dadr[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) {
    const int j = mj_name2id(m, mjOBJ_JOINT, G1JointNames()[i]);
    qadr[i] = m->jnt_qposadr[j];
    dadr[i] = m->jnt_dofadr[j];
  }
  const int body[2] = {mj_name2id(m, mjOBJ_BODY, "left_ankle_roll_link"),
                       mj_name2id(m, mjOBJ_BODY, "right_ankle_roll_link")};
  const LegContext L{model.getFrameId(kLeftFootFrame), kLeftLegJoints};
  const LegContext R{model.getFrameId(kRightFootFrame), kRightLegJoints};
  const LegContext* legs[2] = {&L, &R};

  for (double pelvis_pitch : {0.0, 0.08}) {
    for (double px : {0.0, 0.03}) {
      // 서 있는 자세: IK로 발바닥을 지면(z=sole offset 보정) 양발 (0,+-0.09).
      Eigen::VectorXd q = pinocchio::neutral(model);
      double w, x, y, z;
      QuatFromEuler(0.0, pelvis_pitch, 0.0, &w, &x, &y, &z);
      q.segment<3>(0) = Eigen::Vector3d(px, 0, kNominalStandHeightM);
      q[3] = x; q[4] = y; q[5] = z; q[6] = w;
      const double ready[6] = {-0.3, 0, 0, 0.6, -0.3, 0};
      for (const LegContext* leg : legs)
        for (int k = 0; k < 6; ++k) q[model.idx_qs[JointId(model, leg->joints[k])]] = ready[k];
      const pinocchio::SE3 tl(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0.09, kFootSoleOffsetM));
      const pinocchio::SE3 tr(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, -0.09, kFootSoleOffsetM));
      for (int it = 0; it < 200; ++it) { SolveLegIk(model, data, L, tl, q); SolveLegIk(model, data, R, tr, q); }

      // MuJoCo 상태
      mju_zero(d->qpos, m->nq);
      mju_zero(d->qvel, m->nv);
      d->qpos[0] = q[0]; d->qpos[1] = q[1]; d->qpos[2] = q[2];
      d->qpos[3] = q[6]; d->qpos[4] = q[3]; d->qpos[5] = q[4]; d->qpos[6] = q[5];
      for (int i = 0; i < kNumJoints; ++i) d->qpos[qadr[i]] = q[model.idx_qs[JointId(model, i)]];
      mj_forward(m, d);

      // 발 접촉 Jacobian (월드, 발목 프레임 원점) 과 평형 방정식
      Eigen::MatrixXd Jc[2];
      for (int s = 0; s < 2; ++s) {
        Eigen::MatrixXd jp(3, m->nv), jr(3, m->nv);
        mj_jacBody(m, d, jp.data(), jr.data(), body[s]);  // row-major 아님 주의: 아래서 변환
        // mju는 row-major(3 x nv) 배열을 채운다.
        Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> P(jp.data(), 3, m->nv);
        Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> Rr(jr.data(), 3, m->nv);
        Jc[s].resize(6, m->nv);
        Jc[s] << P, Rr;
      }
      Eigen::VectorXd bias(m->nv);
      for (int i = 0; i < m->nv; ++i) bias[i] = d->qfrc_bias[i];
      Eigen::Matrix<double, 6, 12> A;
      A << Jc[0].leftCols(6).transpose(), Jc[1].leftCols(6).transpose();
      const Eigen::Matrix<double, 12, 1> xw = A.completeOrthogonalDecomposition().solve(bias.head(6));
      const Eigen::Matrix<double, 6, 1> xf[2] = {xw.head<6>(), xw.tail<6>()};

      // 필요한 관절 토크
      Eigen::VectorXd tau_req(kNumJoints);
      for (int i = 0; i < kNumJoints; ++i) {
        double t = bias[dadr[i]];
        for (int s = 0; s < 2; ++s) t -= Jc[s].col(dadr[i]).dot(xf[s]);
        tau_req[i] = t;
      }

      // 컨트롤러 방식: J_pin^T * w, w = [fx fy fz tx ty 0]
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      pinocchio::computeJointJacobians(model, data, q);
      Eigen::VectorXd tau_ctrl = Eigen::VectorXd::Zero(kNumJoints);
      for (int s = 0; s < 2; ++s) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::getFrameJacobian(model, data, legs[s]->foot_frame, pinocchio::LOCAL_WORLD_ALIGNED, J);
        Eigen::Matrix<double, 6, 1> wv;
        wv << xf[s][0], xf[s][1], xf[s][2], xf[s][3], xf[s][4], 0.0;
        for (int k = 0; k < 6; ++k) {
          const int ji = legs[s]->joints[k];
          tau_ctrl[ji] = J.col(model.idx_vs[JointId(model, ji)]).dot(wv);
        }
      }

      // 중력 보상 후보: -J^T w + g(q)  (g = Pinocchio 일반화 중력, 관절 성분)
      const Eigen::VectorXd g_pin = pinocchio::computeGeneralizedGravity(model, data, q);
      Eigen::VectorXd tau_full(kNumJoints);
      for (int i = 0; i < kNumJoints; ++i) tau_full[i] = -tau_ctrl[i] + g_pin[model.idx_vs[JointId(model, i)]];

      double fz_sum = xf[0][2] + xf[1][2], total_mass = 0;
      for (int b = 0; b < m->nbody; ++b) total_mass += m->body_mass[b];
      const Eigen::VectorXd r_plus = tau_req - tau_ctrl, r_minus = tau_req + tau_ctrl;
      std::printf("\n[pelvis pitch=%.2f, pelvis x=%.2f] 지면 반력 Fz 합=%.1f N (mg=%.1f N), tz(왼/오)=%.2f/%.2f Nm(WBC는 tz=0으로 무시)\n",
                  pelvis_pitch, px, fz_sum, total_mass * 9.81, xf[0][5], xf[1][5]);
      std::printf("  다리 관절 토크 [Nm]        tau_req(MuJoCo)   +J^T w(코드)   -J^T w\n");
      const char* names[6] = {"hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"};
      for (int k = 0; k < 6; ++k) {
        const int ji = kLeftLegJoints[k];
        std::printf("  L %-12s %16.3f %14.3f %10.3f\n", names[k], tau_req[ji], tau_ctrl[ji], -tau_ctrl[ji]);
      }
      std::printf("  L hip_pitch 중력항 g(q)=%.3f  L knee g=%.3f\n", g_pin[model.idx_vs[JointId(model, 0)]],
                  g_pin[model.idx_vs[JointId(model, 3)]]);
      std::printf("  ||tau_req - (-J^T w + g(q))|| = %.4f   (0에 가까우면 '-J^T w + g' 가 정확한 정적 토크)\n",
                  (tau_req - tau_full).norm());
      std::printf("  ||tau_req - (+J^T w)|| = %.3f   ||tau_req - (-J^T w)|| = %.3f   ||tau_req|| = %.3f\n",
                  r_plus.norm(), r_minus.norm(), tau_req.norm());
    }
  }
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
