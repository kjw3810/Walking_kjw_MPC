#include "leg_ik.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>

#include "g1_joints.h"

namespace walking_kjw {
namespace {

constexpr int kIkIterations = 4;
constexpr double kIkDampingMin = 1e-6;
// 오차 비례 감쇠(Levenberg-Marquardt류): 오차가 크거나 특이점 근처면 스텝을 줄여 발산 방지.
constexpr double kIkDampingErrGain = 1e-2;
// 반복당 관절 스텝 상한(rad). 특이점에서 야코비안 역이 폭주해도 한 번에 크게 못 움직이게 함.
constexpr double kIkMaxStepRad = 0.2;
constexpr double kIkConvergedNorm = 1e-4;

}  // namespace

pinocchio::JointIndex JointId(const pinocchio::Model& model, int joint_index) {
  return model.getJointId(G1JointNames()[joint_index]);
}

void SolveLegIk(const pinocchio::Model& model, pinocchio::Data& data, const LegContext& leg,
                 const pinocchio::SE3& target, Eigen::VectorXd& q) {
  for (int iter = 0; iter < kIkIterations; ++iter) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const pinocchio::SE3 iMd = data.oMf[leg.foot_frame].actInv(target);
    const Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();
    if (err.norm() < kIkConvergedNorm) break;

    pinocchio::computeJointJacobians(model, data, q);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, leg.foot_frame, pinocchio::LOCAL, J);

    Eigen::Matrix<double, 6, 6> j_leg;
    for (int k = 0; k < 6; ++k) {
      j_leg.col(k) = J.col(model.idx_vs[JointId(model, leg.joints[k])]);
    }
    const double damping = kIkDampingMin + kIkDampingErrGain * err.squaredNorm();
    const Eigen::Matrix<double, 6, 6> jtj =
        j_leg.transpose() * j_leg + damping * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 1> dq = jtj.ldlt().solve(j_leg.transpose() * err);
    const double max_dq = dq.cwiseAbs().maxCoeff();
    if (max_dq > kIkMaxStepRad) dq *= kIkMaxStepRad / max_dq;

    for (int k = 0; k < 6; ++k) {
      const int jid = JointId(model, leg.joints[k]);
      const int qi = model.idx_qs[jid];
      const double lo = model.lowerPositionLimit[qi], hi = model.upperPositionLimit[qi];
      q[qi] = std::clamp(q[qi] + dq[k], lo, hi);  // 관절 가동범위 안에 유지
    }
  }
}

}  // namespace walking_kjw
