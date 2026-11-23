
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace miau
{

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

inline Vec3 add(const Vec3 & a, const Vec3 & b) {return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};}
inline Vec3 sub(const Vec3 & a, const Vec3 & b) {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}
inline Vec3 scale(double s, const Vec3 & a) {return {s * a[0], s * a[1], s * a[2]};}
inline double norm(const Vec3 & a) {return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);}
inline double maxAbs(const Vec3 & a)
{
  return std::max({std::fabs(a[0]), std::fabs(a[1]), std::fabs(a[2])});
}

inline Vec3 mul(const Mat3 & A, const Vec3 & x)
{
  Vec3 y{0.0, 0.0, 0.0};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {y[i] += A[i][j] * x[j];}
  }
  return y;
}

inline Mat3 transpose(const Mat3 & A)
{
  Mat3 T{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {T[i][j] = A[j][i];}
  }
  return T;
}

inline Mat3 matmul(const Mat3 & A, const Mat3 & B)
{
  Mat3 C{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) {C[i][j] += A[i][k] * B[k][j];}
    }
  }
  return C;
}

inline double det3(const Mat3 & A)
{
  return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
         A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
         A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
}

// Inversa por matriz adjunta. Devuelve false si det ~ 0.
inline bool inv3(const Mat3 & A, Mat3 & Ai, double & det)
{
  det = det3(A);
  if (std::fabs(det) < 1e-300) {return false;}
  Ai[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
  Ai[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
  Ai[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
  Ai[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
  Ai[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
  Ai[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det;
  Ai[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
  Ai[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det;
  Ai[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;
  return true;
}

// Cinemática del robot: posición del efector y Jacobiano lineal 3x3.
class KinematicModel
{
public:
  virtual ~KinematicModel() = default;
  virtual Vec3 fk(const Vec3 & q) const = 0;
  virtual Mat3 jacobian(const Vec3 & q) const = 0;
};


inline Vec3 applyInverseJacobian(
  const Mat3 & J, const Vec3 & v, double det_min, double lambda,
  double & det, bool & singular)
{
  Mat3 Ji{};
  double d = 0.0;
  if (inv3(J, Ji, d) && std::fabs(d) >= det_min) {
    det = d;
    singular = false;
    return mul(Ji, v);
  }
  det = det3(J);
  singular = true;
  Mat3 JT = transpose(J);
  Mat3 A = matmul(J, JT);
  for (int i = 0; i < 3; ++i) {A[i][i] += lambda * lambda;}
  Mat3 Ai{};
  double d2 = 0.0;
  if (!inv3(A, Ai, d2)) {return Vec3{0.0, 0.0, 0.0};}
  return mul(JT, mul(Ai, v));
}

inline bool solveIkNewton(
  const KinematicModel & m, const Vec3 & target, const Vec3 & seed, Vec3 & q,
  double det_min = 1e-9, double lambda = 0.02, int max_iter = 500, double tol = 1e-9)
{
  q = seed;
  for (int it = 0; it < max_iter; ++it) {
    Vec3 e = sub(target, m.fk(q));
    if (norm(e) < tol) {return true;}
    double det = 0.0;
    bool sing = false;
    Vec3 dq = applyInverseJacobian(m.jacobian(q), e, det_min, lambda, det, sing);
    const double mx = maxAbs(dq);
    if (mx > 0.3) {dq = scale(0.3 / mx, dq);}   // paso máximo 0.3 rad por iteración
    q = add(q, dq);
  }
  return norm(sub(target, m.fk(q))) < 1e-6;
}

struct PlannerParams
{
  double max_joint_vel{1.0};   
  double cart_speed{0.10};     
  double cart_accel{0.30};    
  double kp{5.0};              
  double dwell{0.5};           
  double dt{0.01};           
  double det_min{1e-5};      
  double damping{0.02};       
};

enum class Phase {Move, Dwell};

struct Sample
{
  double t{0.0};                // instante del paso k
  int segment{0};
  Phase phase{Phase::Move};
  Vec3 q_cmd{};                 // q(t_k+1): consigna de posición a enviar
  Vec3 qd_cmd{};                // q_dot(t_k): velocidad articular calculada
  Vec3 p_ref{};                 // punto de referencia sobre el segmento
  Vec3 p_fk{};                  // FK(q(t_k)) (efector del modelo)
  double scale{1.0};            // factor de recorte por el límite de velocidad (1 = sin recorte)
  double det{0.0};              // det J
  bool singular{false};
};

class DiffIkPlanner
{
public:
  DiffIkPlanner(const KinematicModel & model, const PlannerParams & prm)
  : model_(model), prm_(prm) {}

  void setWaypoints(const std::vector<Vec3> & wps) {wps_ = wps;}

  // q0 debe ser la solución de IK del primer waypoint.
  void reset(const Vec3 & q0)
  {
    q_ = q0;
    t_ = 0.0;
    seg_ = -1;
    dwell_left_ = 0.0;
    done_ = false;
    advanceSegment();
  }

  bool finished() const {return done_;}
  const Vec3 & q() const {return q_;}

  bool step(Sample & out)
  {
    if (done_) {return false;}
    out.t = t_;
    out.segment = seg_;

    if (dwell_left_ > 0.0) {
      out.phase = Phase::Dwell;
      out.q_cmd = q_;
      out.qd_cmd = Vec3{0.0, 0.0, 0.0};
      out.p_ref = B_;
      out.p_fk = model_.fk(q_);
      out.scale = 1.0;
      out.singular = false;
      out.det = det3(model_.jacobian(q_));
      dwell_left_ -= prm_.dt;
      t_ += prm_.dt;
      if (dwell_left_ <= 1e-9) {
        dwell_left_ = 0.0;
        advanceSegment();
      }
      return true;
    }

    const double v_min = std::min(0.005, prm_.cart_speed);
    const double v = std::min(
      {prm_.cart_speed,
        std::sqrt(2.0 * prm_.cart_accel * d_ + v_min * v_min),
        std::sqrt(2.0 * prm_.cart_accel * (L_ - d_) + v_min * v_min)});

    const Vec3 p_ref = add(A_, scale(d_, u_));
    const Vec3 p_now = model_.fk(q_);
    const Vec3 e = sub(p_ref, p_now);
    const Vec3 v_e = add(scale(v, u_), scale(prm_.kp, e));     // v_e = v_ref + Kp e

    double det = 0.0;
    bool sing = false;
    Vec3 qd = applyInverseJacobian(                            // q_dot = J^-1(q) v_e
      model_.jacobian(q_), v_e, prm_.det_min, prm_.damping, det, sing);

    const double m = maxAbs(qd);
    const double k = (m > prm_.max_joint_vel) ? prm_.max_joint_vel / m : 1.0;
    qd = scale(k, qd);                                         // límite de velocidad articular

    q_ = add(q_, scale(prm_.dt, qd));                          // Euler
    d_ += v * k * prm_.dt;

    out.phase = Phase::Move;
    out.q_cmd = q_;
    out.qd_cmd = qd;
    out.p_ref = p_ref;
    out.p_fk = p_now;
    out.scale = k;
    out.det = det;
    out.singular = sing;
    t_ += prm_.dt;

    if (d_ >= L_) {
      if (prm_.dwell > 0.0) {
        dwell_left_ = prm_.dwell;
      } else {
        advanceSegment();
      }
    }
    return true;
  }

private:
  void advanceSegment()
  {
    ++seg_;
    while (seg_ + 1 < static_cast<int>(wps_.size())) {
      A_ = wps_[seg_];
      B_ = wps_[seg_ + 1];
      L_ = norm(sub(B_, A_));
      if (L_ > 1e-9) {
        u_ = scale(1.0 / L_, sub(B_, A_));
        d_ = 0.0;
        return;
      }
      ++seg_;   // segmento de longitud cero: se salta
    }
    done_ = true;
  }

  const KinematicModel & model_;
  PlannerParams prm_;
  std::vector<Vec3> wps_;
  Vec3 q_{};
  Vec3 A_{}, B_{}, u_{};
  double L_{0.0}, d_{0.0}, t_{0.0}, dwell_left_{0.0};
  int seg_{-1};
  bool done_{true};
};

}  
