#include "attitude_mpc.h"

#include <osqp/osqp.h>

#include <Eigen/Sparse>
#include <cstdio>
#include <vector>

namespace walking_kjw {
namespace {

constexpr int kN = kMpcHorizon;
constexpr int kNx = 4;
constexpr int kNu = 5;
constexpr int kNz = kN * (kNx + kNu);  // z = [x_1..x_N, u_0..u_{N-1}]
constexpr int kDynRows = kN * kNx;
constexpr int kIneqPerStep = 9;
constexpr int kRows = kDynRows + kN * kIneqPerStep;

int XCol(int k, int j) { return (k - 1) * kNx + j; }  // k = 1..N
int UCol(int k, int j) { return kN * kNx + k * kNu + j; }

}  // namespace

struct AttitudeMpc::Impl {
  AttitudeMpcConfig cfg;
  OSQPWorkspace* work = nullptr;

  std::vector<c_float> Ax, Px, q, l, u, ax_new;
  std::vector<c_int> Ai, Ap, Pi, Pp, ax_idx;

  Eigen::Vector4d last_q = Eigen::Vector4d::Constant(-1.0);
  Eigen::Matrix<double, 5, 1> last_r = Eigen::Matrix<double, 5, 1>::Constant(-1.0);
  double last_term = -1.0;
};

AttitudeMpc::AttitudeMpc(const AttitudeMpcConfig& cfg) : impl_(new Impl) {
  Impl& m = *impl_;
  m.cfg = cfg;
  const double dt = cfg.dt;

  // ---- 제약 행렬 A (희소 구조 고정, B_k 값만 매 풀이마다 갱신) ----
  std::vector<Eigen::Triplet<double>> t;
  auto add = [&](int r, int c, double v) { t.emplace_back(r, c, v); };
  for (int k = 0; k < kN; ++k) {
    for (int i = 0; i < kNx; ++i) add(4 * k + i, XCol(k + 1, i), 1.0);
    if (k >= 1) {  // -A x_k
      add(4 * k + 0, XCol(k, 0), -1.0);
      add(4 * k + 0, XCol(k, 1), -dt);
      add(4 * k + 1, XCol(k, 1), -1.0);
      add(4 * k + 2, XCol(k, 2), -1.0);
      add(4 * k + 2, XCol(k, 3), -dt);
      add(4 * k + 3, XCol(k, 3), -1.0);
    }
    for (int j : {1, 2, 3}) add(4 * k + 1, UCol(k, j), 1.0);  // -B_k 자리표시 (θ̇x 행)
    for (int j : {0, 2, 4}) add(4 * k + 3, UCol(k, j), 1.0);  // θ̇y 행

    const int b = kDynRows + kIneqPerStep * k;
    const double mu = cfg.friction_mu, heel = cfg.cop_heel, toe = cfg.cop_toe, wy = cfg.cop_half_width;
    add(b + 0, UCol(k, 2), 1.0);  // 0 <= Fz <= fz_max
    add(b + 1, UCol(k, 0), 1.0); add(b + 1, UCol(k, 2), -mu);  // Fx - μFz <= 0
    add(b + 2, UCol(k, 0), 1.0); add(b + 2, UCol(k, 2), mu);   // Fx + μFz >= 0
    add(b + 3, UCol(k, 1), 1.0); add(b + 3, UCol(k, 2), -mu);
    add(b + 4, UCol(k, 1), 1.0); add(b + 4, UCol(k, 2), mu);
    add(b + 5, UCol(k, 4), 1.0); add(b + 5, UCol(k, 2), -heel);  // τy - heel Fz <= 0
    add(b + 6, UCol(k, 4), 1.0); add(b + 6, UCol(k, 2), toe);    // τy + toe Fz >= 0
    add(b + 7, UCol(k, 3), 1.0); add(b + 7, UCol(k, 2), -wy);
    add(b + 8, UCol(k, 3), 1.0); add(b + 8, UCol(k, 2), wy);
  }
  Eigen::SparseMatrix<double> A(kRows, kNz);
  A.setFromTriplets(t.begin(), t.end());
  A.makeCompressed();

  // B_k 항목의 값 배열 내 위치 (osqp_update_A용). 갱신 순서는 Solve()의 채움 순서와 같아야 한다.
  for (int k = 0; k < kN; ++k) {
    for (int j : {1, 2, 3}) m.ax_idx.push_back(&A.coeffRef(4 * k + 1, UCol(k, j)) - A.valuePtr());
    for (int j : {0, 2, 4}) m.ax_idx.push_back(&A.coeffRef(4 * k + 3, UCol(k, j)) - A.valuePtr());
  }
  m.ax_new.assign(m.ax_idx.size(), 0.0);

  m.Ax.assign(A.valuePtr(), A.valuePtr() + A.nonZeros());
  m.Ai.assign(A.innerIndexPtr(), A.innerIndexPtr() + A.nonZeros());
  m.Ap.assign(A.outerIndexPtr(), A.outerIndexPtr() + kNz + 1);

  // ---- P: 대각 (상삼각 CSC) ----
  m.Px.assign(kNz, 1.0);
  for (int i = 0; i < kNz; ++i) {
    m.Pi.push_back(i);
    m.Pp.push_back(i);
  }
  m.Pp.push_back(kNz);

  m.q.assign(kNz, 0.0);
  m.l.assign(kRows, 0.0);
  m.u.assign(kRows, 0.0);
  for (int k = 0; k < kN; ++k) {
    const int b = kDynRows + kIneqPerStep * k;
    m.l[b + 0] = 0.0;       m.u[b + 0] = cfg.fz_max;
    m.l[b + 1] = -OSQP_INFTY; m.u[b + 1] = 0.0;
    m.l[b + 2] = 0.0;       m.u[b + 2] = OSQP_INFTY;
    m.l[b + 3] = -OSQP_INFTY; m.u[b + 3] = 0.0;
    m.l[b + 4] = 0.0;       m.u[b + 4] = OSQP_INFTY;
    m.l[b + 5] = -OSQP_INFTY; m.u[b + 5] = 0.0;
    m.l[b + 6] = 0.0;       m.u[b + 6] = OSQP_INFTY;
    m.l[b + 7] = -OSQP_INFTY; m.u[b + 7] = 0.0;
    m.l[b + 8] = 0.0;       m.u[b + 8] = OSQP_INFTY;
  }

  csc* P = csc_matrix(kNz, kNz, kNz, m.Px.data(), m.Pi.data(), m.Pp.data());
  csc* Amat = csc_matrix(kRows, kNz, static_cast<c_int>(m.Ax.size()), m.Ax.data(), m.Ai.data(), m.Ap.data());
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.warm_start = 1;
  settings.polish = 0;
  settings.max_iter = 400;
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-4;
  OSQPData data;
  data.n = kNz;
  data.m = kRows;
  data.P = P;
  data.A = Amat;
  data.q = m.q.data();
  data.l = m.l.data();
  data.u = m.u.data();
  if (osqp_setup(&m.work, &data, &settings) != 0) {
    std::fprintf(stderr, "AttitudeMpc: osqp_setup failed\n");
    m.work = nullptr;
  }
  // osqp_setup이 P/A를 복사하므로 래퍼 구조체만 해제 (배열은 std::vector 소유라 csc_spfree 금지).
  c_free(P);
  c_free(Amat);
}

AttitudeMpc::~AttitudeMpc() {
  if (impl_->work) osqp_cleanup(impl_->work);
  delete impl_;
}

bool AttitudeMpc::Solve(const Eigen::Vector4d& x0, const std::array<Eigen::Vector3d, kMpcHorizon>& r,
                        const std::array<Eigen::Vector3d, kMpcHorizon>& f_ref,
                        const AttitudeMpcWeights& w, ContactWrench* u0) {
  Impl& m = *impl_;
  if (!m.work) return false;
  const double dt = m.cfg.dt;
  const double sx = dt / m.cfg.inertia_xx, sy = dt / m.cfg.inertia_yy;

  // B_k (θ̇x 행: [0, z, -y, 1, 0]*dt/Ix, θ̇y 행: [-z, 0, x, 0, 1]*dt/Iy), A에는 -B.
  size_t n = 0;
  for (int k = 0; k < kN; ++k) {
    const double x = r[k].x(), y = r[k].y(), z = r[k].z();
    m.ax_new[n++] = -z * sx;
    m.ax_new[n++] = y * sx;
    m.ax_new[n++] = -sx;
    m.ax_new[n++] = z * sy;
    m.ax_new[n++] = -x * sy;
    m.ax_new[n++] = -sy;
  }
  osqp_update_A(m.work, m.ax_new.data(), m.ax_idx.data(), static_cast<c_int>(m.ax_new.size()));

  // 가중치 (바뀐 경우만 P 갱신)
  if (w.q != m.last_q || w.r != m.last_r || w.terminal_scale != m.last_term) {
    for (int k = 1; k <= kN; ++k) {
      const double s = (k == kN) ? w.terminal_scale : 1.0;
      for (int j = 0; j < kNx; ++j) m.Px[XCol(k, j)] = s * w.q[j];
    }
    for (int k = 0; k < kN; ++k)
      for (int j = 0; j < kNu; ++j) m.Px[UCol(k, j)] = w.r[j];
    osqp_update_P(m.work, m.Px.data(), nullptr, 0);
    m.last_q = w.q;
    m.last_r = w.r;
    m.last_term = w.terminal_scale;
  }

  // 선형항: 입력을 u_ref=[f_ref,0,0]로 당김 (상태 기준은 0)
  for (int k = 0; k < kN; ++k)
    for (int j = 0; j < 3; ++j) m.q[UCol(k, j)] = -w.r[j] * f_ref[k][j];
  osqp_update_lin_cost(m.work, m.q.data());

  // 초기 상태 등식: x_1 - B_0 u_0 = A x_0
  m.l[0] = m.u[0] = x0[0] + dt * x0[1];
  m.l[1] = m.u[1] = x0[1];
  m.l[2] = m.u[2] = x0[2] + dt * x0[3];
  m.l[3] = m.u[3] = x0[3];
  osqp_update_bounds(m.work, m.l.data(), m.u.data());

  osqp_solve(m.work);
  last_iter_ = static_cast<int>(m.work->info->iter);
  const c_int st = m.work->info->status_val;
  if (st != OSQP_SOLVED && st != OSQP_SOLVED_INACCURATE) return false;

  const c_float* z = m.work->solution->x;
  *u0 = ContactWrench{z[UCol(0, 0)], z[UCol(0, 1)], z[UCol(0, 2)], z[UCol(0, 3)], z[UCol(0, 4)]};
  return true;
}

}  // namespace walking_kjw
