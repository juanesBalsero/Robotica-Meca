#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "miau/diff_ik_planner.hpp"

using miau::Vec3;


namespace smooth
{
using A3 = std::array<double, 3>;

struct Peaks {double v{0.0}, a{0.0}, j{0.0};};

struct Config
{
  double dt{0.01};
  double window{0.30};       // ancho inicial del filtro [s]
  double window_max{1.50};   // ancho máximo permitido [s]
  double vmax{1.0}, amax{2.0}, jmax{10.0};
  double margin{0.95};       // se apunta a este % de cada límite
};

struct Result
{
  std::vector<A3> qd;        // velocidades finales (última muestra = 0)
  std::vector<A3> qd_raw;    // velocidades originales del planificador (alineadas)
  Peaks peaks, peaks_raw;
  double window_used{0.0};
  double stretch{1.0};
  double end_corr{0.0};      // máx |corrección| de posición final [rad]
};

inline double maxAbs3(const A3 & v)
{
  return std::max({std::fabs(v[0]), std::fabs(v[1]), std::fabs(v[2])});
}

// Picos de |vel|, |acel| y |jerk| (diferencias finitas, partiendo y terminando en reposo)
inline Peaks peaks(const std::vector<A3> & qd, double dt)
{
  Peaks p;
  A3 prev{0.0, 0.0, 0.0}, prev_a{0.0, 0.0, 0.0};
  for (size_t i = 0; i <= qd.size(); ++i) {
    const A3 cur = (i < qd.size()) ? qd[i] : A3{0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) {
      const double a = (cur[k] - prev[k]) / dt;
      const double j = (a - prev_a[k]) / dt;
      p.v = std::max(p.v, std::fabs(cur[k]));
      p.a = std::max(p.a, std::fabs(a));
      p.j = std::max(p.j, std::fabs(j));
      prev_a[k] = a;
    }
    prev = cur;
  }
  return p;
}

// Núcleo de Hann normalizado (2H+1 puntos, extremos = 0, suma = 1)
inline std::vector<double> hann(int H)
{
  std::vector<double> w(static_cast<size_t>(2 * H + 1));
  double sum = 0.0;
  for (int i = 0; i <= 2 * H; ++i) {
    w[static_cast<size_t>(i)] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (2.0 * H)));
    sum += w[static_cast<size_t>(i)];
  }
  for (auto & v : w) {v /= sum;}
  return w;
}

// Convolución completa (longitud n+2H). Conserva exactamente la suma, o sea el desplazamiento.
inline std::vector<A3> convolve(const std::vector<A3> & x, const std::vector<double> & w)
{
  std::vector<A3> y(x.size() + w.size() - 1, A3{0.0, 0.0, 0.0});
  for (size_t i = 0; i < x.size(); ++i) {
    for (size_t j = 0; j < w.size(); ++j) {
      for (int k = 0; k < 3; ++k) {y[i + j][k] += x[i][k] * w[j];}
    }
  }
  return y;
}

// Alarga el tiempo de la señal por k (Catmull-Rom) conservando la integral
inline std::vector<A3> stretch(const std::vector<A3> & x, double k)
{
  const long n = static_cast<long>(x.size());
  const long m = static_cast<long>(std::ceil(static_cast<double>(n) * k));
  auto at = [&](long i) -> A3 {
      return (i >= 0 && i < n) ? x[static_cast<size_t>(i)] : A3{0.0, 0.0, 0.0};
    };
  std::vector<A3> y(static_cast<size_t>(m));
  for (long j = 0; j < m; ++j) {
    const double u = static_cast<double>(j) / k;
    const long i1 = static_cast<long>(std::floor(u));
    const double f = u - static_cast<double>(i1);
    const A3 p0 = at(i1 - 1), p1 = at(i1), p2 = at(i1 + 1), p3 = at(i1 + 2);
    for (int c = 0; c < 3; ++c) {
      y[static_cast<size_t>(j)][c] = 0.5 * (2.0 * p1[c] + (-p0[c] + p2[c]) * f +
        (2.0 * p0[c] - 5.0 * p1[c] + 4.0 * p2[c] - p3[c]) * f * f +
        (-p0[c] + 3.0 * p1[c] - 3.0 * p2[c] + p3[c]) * f * f * f) / k;
    }
  }
  return y;
}

// Reparte (con una campana suave) el pequeño error de desplazamiento para que la
// trayectoria termine EXACTAMENTE en q_B. Devuelve el máx |corrección| [rad].
inline double fixEndpoint(std::vector<A3> & qd, double dt, const A3 & dq_target)
{
  A3 e{0.0, 0.0, 0.0};
  for (const auto & v : qd) {for (int k = 0; k < 3; ++k) {e[k] += v[k] * dt;}}
  for (int k = 0; k < 3; ++k) {e[k] = dq_target[k] - e[k];}
  const size_t n = qd.size();
  std::vector<double> b(n);
  double S = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double s = std::sin(M_PI * (static_cast<double>(i) + 0.5) / static_cast<double>(n));
    b[i] = s * s * s * s;
    S += b[i];
  }
  for (size_t i = 0; i < n; ++i) {
    for (int k = 0; k < 3; ++k) {qd[i][k] += e[k] * b[i] / (S * dt);}
  }
  return maxAbs3(e);
}

// raw: velocidades crudas del planificador -> perfil suave que respeta vel/acel/jerk
inline Result build(const std::vector<A3> & raw, const A3 & dq_target, const Config & c)
{
  Result r;
  const A3 zero{0.0, 0.0, 0.0};
  size_t i0 = 0, i1 = raw.size();
  while (i0 < i1 && maxAbs3(raw[i0]) < 1e-4) {++i0;}
  while (i1 > i0 && maxAbs3(raw[i1 - 1]) < 1e-4) {--i1;}
  if (i0 >= i1) {                       // trayectoria nula (mismo punto)
    r.qd.assign(1, zero);
    r.qd_raw = r.qd;
    return r;
  }
  const std::vector<A3> x(raw.begin() + static_cast<long>(i0), raw.begin() + static_cast<long>(i1));
  r.peaks_raw = peaks(x, c.dt);

  // 1) filtro de Hann; se ensancha hasta cumplir acel y jerk (o llegar al máximo)
  double W = c.window;
  int H = 1;
  std::vector<A3> y;
  for (;;) {
    H = std::max(1, static_cast<int>(std::lround(W / (2.0 * c.dt))));
    y = convolve(x, hann(H));
    const Peaks p = peaks(y, c.dt);
    if ((p.a <= c.amax * c.margin && p.j <= c.jmax * c.margin) || W >= c.window_max) {break;}
    W = std::min(W * 1.25, c.window_max);
  }
  r.window_used = 2.0 * H * c.dt;

  // 2) Terminar EXACTAMENTE en q_B y respetar vel/acel/jerk sobre la trayectoria FINAL
  //    (la corrección de posición también cuenta). Si algo se pasa, se alarga el tiempo
  //    (vel/k, acel/k², jerk/k³) y se vuelve a comprobar.
  double k_total = 1.0;
  for (int it = 0; it < 6; ++it) {
    const double e = fixEndpoint(y, c.dt, dq_target);
    if (it == 0) {r.end_corr = e;}
    const Peaks p = peaks(y, c.dt);
    double k = 1.0;
    if (p.v > 0.0) {k = std::max(k, p.v / (c.vmax * c.margin));}
    if (p.a > 0.0) {k = std::max(k, std::sqrt(p.a / (c.amax * c.margin)));}
    if (p.j > 0.0) {k = std::max(k, std::cbrt(p.j / (c.jmax * c.margin)));}
    if (k <= 1.001) {break;}
    y = stretch(y, k);
    k_total *= k;
  }
  r.stretch = k_total;

  y.push_back(zero);
  r.qd = y;
  r.peaks = peaks(r.qd, c.dt);

  r.qd_raw.assign(r.qd.size(), zero);
  for (size_t n = 0; n < x.size(); ++n) {
    if (static_cast<size_t>(H) + n < r.qd_raw.size()) {r.qd_raw[static_cast<size_t>(H) + n] = x[n];}
  }
  return r;
}
}  // namespace smooth

static const char * const kPlotScript = R"PYEOF(#!/usr/bin/env python3
"""Genera una gráfica por trayectoria a partir de <carpeta>/trayectorias/*.csv
Uso:  python3 plot_trayectorias.py <carpeta_de_salida>
"""
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
tdir = os.path.join(root, "trayectorias")
gdir = os.path.join(root, "graficas")
os.makedirs(gdir, exist_ok=True)


def plot_one(path):
    d = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    names = d.dtype.names
    joints = [n[:-4] for n in names if n.endswith("_pos")]
    t = d["t_sec"]
    stem = os.path.splitext(os.path.basename(path))[0]
    title = stem.replace("_a_", "  ->  ")

    def peak(suffix):
        return max(np.max(np.abs(d[j + suffix])) for j in joints)

    fig, ax = plt.subplots(2, 3, figsize=(17, 9))
    fig.suptitle(
        f"Trayectoria {title}   |   T = {t[-1]:.2f} s   |   "
        f"|vel| max = {peak('_vel'):.3f} rad/s   |acel| max = {peak('_acc'):.2f} rad/s²   "
        f"|jerk| max = {peak('_jerk'):.1f} rad/s³",
        fontsize=13, fontweight="bold")

    for j in joints:
        ax[0, 0].plot(t, d[j + "_pos"], label=j)
    ax[0, 0].set(title="Posición articular (referencia)", xlabel="t [s]", ylabel="rad")

    for i, j in enumerate(joints):
        c = f"C{i}"
        if j + "_vel_raw" in names:
            ax[0, 1].plot(t, d[j + "_vel_raw"], "--", color=c, alpha=0.35, lw=1)
        ax[0, 1].plot(t, d[j + "_vel"], color=c, lw=1.8, label=j)
    ax[0, 1].set(title="Velocidad inyectada (sólido) vs. cruda del planificador (punteado)",
                 xlabel="t [s]", ylabel="rad/s")

    for j in joints:
        ax[0, 2].plot(t, d[j + "_acc"], label=j)
    ax[0, 2].set(title="Aceleración", xlabel="t [s]", ylabel="rad/s²")

    for j in joints:
        ax[1, 0].plot(t, d[j + "_jerk"], label=j)
    ax[1, 0].set(title="Jerk", xlabel="t [s]", ylabel="rad/s³")

    for k, lab in enumerate(("x", "y", "z")):
        ax[1, 1].plot(t, d[lab + "_m"] * 1000.0, label=lab)
    ax2 = ax[1, 1].twinx()
    ax2.plot(t, d["dev_linea_mm"], "k:", lw=1.2, label="desv. recta")
    ax2.set_ylabel("desviación de la recta A-B [mm]")
    ax[1, 1].set(title="Posición cartesiana del TCP", xlabel="t [s]", ylabel="mm")

    fig.delaxes(ax[1, 2])
    a3 = fig.add_subplot(2, 3, 6, projection="3d")
    a3.plot(d["x_m"], d["y_m"], d["z_m"], lw=2)
    a3.scatter(d["x_m"][0], d["y_m"][0], d["z_m"][0], c="g", s=50, label="inicio")
    a3.scatter(d["x_m"][-1], d["y_m"][-1], d["z_m"][-1], c="r", s=50, label="fin")
    a3.set(title="Trayectoria del TCP [m]", xlabel="x", ylabel="y", zlabel="z")
    a3.legend()

    for a in (ax[0, 0], ax[0, 1], ax[0, 2], ax[1, 0]):
        a.grid(alpha=0.3)
        a.legend(loc="best")
    ax[1, 1].grid(alpha=0.3)
    ax[1, 1].legend(loc="upper left")
    ax2.legend(loc="upper right")

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = os.path.join(gdir, stem + ".png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


files = sorted(glob.glob(os.path.join(tdir, "*.csv")))
if not files:
    print("No hay CSV en", tdir)
for f in files:
    try:
        print("OK ", plot_one(f))
    except Exception as e:  # una gráfica mala no detiene las demás
        print("ERR", f, e)
)PYEOF";

// =====================================================================================
class KdlModel : public miau::KinematicModel
{
public:
  explicit KdlModel(const KDL::Chain & chain)
  : chain_(chain),
    n_(chain.getNrOfJoints()),
    fk_(chain),
    jac_(chain),
    id_(chain, KDL::Vector(0.0, 0.0, -9.81))
  {}

  Vec3 fk(const Vec3 & q) const override
  {
    KDL::JntArray qa = toJnt(q);
    KDL::Frame f;
    fk_.JntToCart(qa, f);
    return {f.p(0), f.p(1), f.p(2)};
  }

  miau::Mat3 jacobian(const Vec3 & q) const override
  {
    KDL::JntArray qa = toJnt(q);
    KDL::Jacobian J(n_);
    jac_.JntToJac(qa, J);
    miau::Mat3 out{};
    for (unsigned i = 0; i < 3; ++i) {          // filas 0..2 = velocidad lineal
      for (unsigned j = 0; j < 3; ++j) {out[i][j] = J(i, j);}
    }
    return out;
  }

  Vec3 inverseDynamics(const Vec3 & q, const Vec3 & qd, const Vec3 & qdd) const
  {
    KDL::JntArray a = toJnt(q), b = toJnt(qd), c = toJnt(qdd), tau(n_);
    std::vector<KDL::Wrench> f_ext(chain_.getNrOfSegments(), KDL::Wrench::Zero());
    id_.CartToJnt(a, b, c, f_ext, tau);
    return {tau(0), tau(1), tau(2)};
  }

private:
  KDL::JntArray toJnt(const Vec3 & v) const
  {
    KDL::JntArray a(n_);
    for (unsigned i = 0; i < 3; ++i) {a(i) = v[i];}
    return a;
  }

  const KDL::Chain & chain_;
  unsigned int n_;
  mutable KDL::ChainFkSolverPos_recursive fk_;
  mutable KDL::ChainJntToJacSolver jac_;
  mutable KDL::ChainIdSolver_RNE id_;
};

// Una trayectoria completa A -> B, ya planeada y suavizada, lista para reproducirse.
// Muestra i corresponde al instante t = i*dt:  se inyecta qd[i] y se espera estar en q[i].
struct Traj
{
  using A3 = smooth::A3;
  std::string from, to;
  std::vector<A3> q, qd, qdd, jerk, p, qd_raw;
  std::vector<double> dev_mm;       // distancia del TCP a la recta A-B
  double duration{0.0};
  double max_dev_mm{0.0};
  double min_det{std::numeric_limits<double>::max()};
  long n_sing{0};
  smooth::Result info;
};

class TrajectoryJacobianNode : public rclcpp::Node
{
  enum class State {WAIT_STATE, HOMING, TRACKING, DONE};
  using A3 = smooth::A3;

public:
  explicit TrajectoryJacobianNode(const rclcpp::NodeOptions & options)
  : Node("trajectory_jacobian_node", options)
  {
    // ---------------- Parámetros ----------------
    const std::string pkg = param<std::string>("package_name", "miau");
    const std::string urdf_rel = param<std::string>("urdf_relative_path", "urdf/Completo.urdf");
    const std::string base_link = param<std::string>("base_link", "base_link");
    const std::string tip_link = param<std::string>("tip_link", "tcp");
    joint_names_ = param<std::vector<std::string>>("joint_names", {"J1", "J2", "J3"});
    // AHORA se publican VELOCIDADES articulares [rad/s] (JointGroupVelocityController)
    const std::string commands_topic =
      param<std::string>("commands_topic", "/velocity_controller/commands");
    const std::string js_topic = param<std::string>("joint_states_topic", "/joint_states");

    prm_.max_joint_vel = getDouble("max_joint_vel", 1.0);
    prm_.cart_speed = getDouble("cart_speed", 0.10);
    prm_.cart_accel = getDouble("cart_accel", 0.30);
    prm_.kp = getDouble("kp_task", 5.0);
    prm_.dwell = getDouble("dwell_time", 0.5);
    prm_.det_min = getDouble("det_min", 1e-5);
    prm_.damping = getDouble("damping", 0.02);
    const double rate = getDouble("control_rate_hz", 100.0);
    prm_.dt = 1.0 / rate;
    home_vel_ = std::min(getDouble("home_joint_vel", 0.5), prm_.max_joint_vel);
    tail_time_ = getDouble("log_tail_time", 1.0);

    // --- suavizado de velocidades ---
    max_acc_ = getDouble("max_joint_acc", 2.0);          // rad/s^2
    max_jerk_ = getDouble("max_joint_jerk", 10.0);       // rad/s^3
    smooth_time_ = getDouble("smooth_time", 0.30);       // s, ancho inicial del filtro
    smooth_time_max_ = getDouble("smooth_time_max", 1.50);  // s, ancho máximo del filtro
    max_path_dev_mm_ = getDouble("max_path_dev_mm", 5.0);   // aviso si se aleja de la recta
    // Realimentación de posición articular sobre la velocidad (0 = lazo abierto puro)
    kp_joint_ = getDouble("kp_joint", 2.0);
    auto_plot_ = param<bool>("auto_plot", true);

    const std::vector<double> seed = getDoubleArray("q_seed", {0.0, -1.5, 0.3});
    if (seed.size() != 3) {throw std::runtime_error("q_seed debe tener 3 valores");}

    const std::string log_dir = param<std::string>("log_dir", "");
    const std::string log_prefix = param<std::string>("log_prefix", "trayectoria");

    // ---------------- Cadena KDL desde el URDF ----------------
    const std::string urdf_path =
      ament_index_cpp::get_package_share_directory(pkg) + "/" + urdf_rel;
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
      RCLCPP_FATAL(get_logger(), "No pude parsear el URDF: %s", urdf_path.c_str());
      throw std::runtime_error("URDF parse failed");
    }
    if (!tree.getChain(base_link, tip_link, chain_)) {
      RCLCPP_FATAL(get_logger(), "No existe cadena entre '%s' y '%s'",
        base_link.c_str(), tip_link.c_str());
      throw std::runtime_error("Chain not found");
    }
    if (chain_.getNrOfJoints() != 3 || joint_names_.size() != 3) {
      RCLCPP_FATAL(get_logger(),
        "Se esperaban 3 juntas (Jacobiano cuadrado 3x3); la cadena tiene %u y joint_names %zu",
        chain_.getNrOfJoints(), joint_names_.size());
      throw std::runtime_error("Se requiere Jacobiano cuadrado");
    }
    model_ = std::make_unique<KdlModel>(chain_);

    // ---------------- Puntos y secuencia (desde parámetros) ----------------
    std::map<std::string, std::vector<double>> pts;
    try {
      this->get_parameters("points", pts);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(),
        "Error leyendo 'points' (¿coordenadas sin punto decimal?): %s", e.what());
      throw;
    }
    sequence_ = param<std::vector<std::string>>(
      "sequence", {"PH", "PA", "PH", "P1", "PH", "PB", "PH", "P2", "PH"});
    if (sequence_.size() < 2) {throw std::runtime_error("'sequence' necesita al menos 2 puntos");}

    RCLCPP_INFO(get_logger(), "Puntos cargados (%zu):", pts.size());
    for (const auto & kv : pts) {
      if (kv.second.size() != 3) {
        RCLCPP_FATAL(get_logger(), "El punto '%s' debe tener 3 valores [x, y, z]", kv.first.c_str());
        throw std::runtime_error("punto inválido");
      }
      RCLCPP_INFO(get_logger(), "  %-4s = (%9.6f, %9.6f, %9.6f) m", kv.first.c_str(),
        kv.second[0], kv.second[1], kv.second[2]);
    }
    for (const auto & name : sequence_) {
      auto it = pts.find(name);
      if (it == pts.end()) {
        RCLCPP_FATAL(get_logger(), "El punto '%s' de 'sequence' no existe en 'points'", name.c_str());
        throw std::runtime_error("punto no definido");
      }
      point_xyz_[name] = Vec3{it->second[0], it->second[1], it->second[2]};
    }
    {
      std::ostringstream ss;
      for (size_t i = 0; i < sequence_.size(); ++i) {ss << (i ? " -> " : "") << sequence_[i];}
      RCLCPP_INFO(get_logger(), "Secuencia: %s", ss.str().c_str());
    }

    // ---------------- Configuración articular q de cada punto distinto ----------------
    // Solo el primer punto se resuelve por IK (con q_seed). Los demás se obtienen SIGUIENDO
    // CONTINUAMENTE la recta desde el punto anterior con el mismo planificador (igual que la
    // ejecución real), así todos quedan en la MISMA rama del brazo (sin saltos de 2*pi ni cambios
    // de codo). Luego cada trayectoria A->B empieza y termina exactamente en esas q.
    {
      const std::string & first = sequence_.front();
      Vec3 sol{};
      if (!miau::solveIkNewton(*model_, point_xyz_.at(first), Vec3{seed[0], seed[1], seed[2]}, sol,
          prm_.det_min, prm_.damping))
      {
        RCLCPP_FATAL(get_logger(), "La IK de '%s' no convergió (¿fuera del espacio de trabajo?)",
          first.c_str());
        throw std::runtime_error("IK no convergió");
      }
      q_ik_[first] = toArr(sol);
      RCLCPP_INFO(get_logger(), "q %-4s = [%.4f, %.4f, %.4f] rad (IK con q_seed)", first.c_str(),
        sol[0], sol[1], sol[2]);
    }
    for (size_t i = 0; i + 1 < sequence_.size(); ++i) {
      const std::string & a = sequence_[i], & b = sequence_[i + 1];
      if (q_ik_.count(b)) {continue;}
      miau::DiffIkPlanner sim(*model_, prm_);
      sim.setWaypoints(std::vector<Vec3>{point_xyz_.at(a), point_xyz_.at(b)});
      sim.reset(toVec(q_ik_.at(a)));
      miau::Sample s;
      const size_t max_steps = static_cast<size_t>(600.0 / prm_.dt);
      Vec3 q_end = toVec(q_ik_.at(a));
      size_t n = 0;
      while (n++ < max_steps && sim.step(s)) {q_end = s.q_cmd;}
      q_ik_[b] = toArr(q_end);
      RCLCPP_INFO(get_logger(), "q %-4s = [%.4f, %.4f, %.4f] rad (siguiendo %s -> %s)", b.c_str(),
        q_end[0], q_end[1], q_end[2], a.c_str(), b.c_str());
    }
    q_start_ = q_ik_.at(sequence_.front());

    // ---------------- Carpeta de salida y planeación de TODAS las trayectorias ----------------
    createOutputDirs(log_dir, log_prefix);
    buildLibrary();
    exportLibrary();

    // ---------------- ROS: publicador, suscriptor, temporizador ----------------
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(commands_topic, 10);
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      js_topic, 50,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {onJointState(msg);});
    timer_ = this->create_timer(
      std::chrono::duration<double>(prm_.dt), [this]() {controlLoop();});

    openLogs();
    RCLCPP_INFO(get_logger(), "Publicando VELOCIDADES articulares en %s", commands_topic.c_str());
    RCLCPP_INFO(get_logger(), "Esperando /joint_states para comenzar...");
  }

  ~TrajectoryJacobianNode() override {closeLogs();}

private:
  // ------------------------------------------------------------------ utilidades
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!this->has_parameter(name)) {this->declare_parameter<T>(name, def);}
    return this->get_parameter(name).get_value<T>();
  }

  // Acepta enteros o reales en el YAML (p. ej. "1" o "1.0")
  double getDouble(const std::string & name, double def)
  {
    if (!this->has_parameter(name)) {this->declare_parameter<double>(name, def);}
    const rclcpp::Parameter p = this->get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(p.as_int());
    }
    return p.as_double();
  }

  std::vector<double> getDoubleArray(const std::string & name, const std::vector<double> & def)
  {
    if (!this->has_parameter(name)) {this->declare_parameter<std::vector<double>>(name, def);}
    const rclcpp::Parameter p = this->get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      std::vector<double> out;
      for (auto v : p.as_integer_array()) {out.push_back(static_cast<double>(v));}
      return out;
    }
    return p.as_double_array();
  }

  static Vec3 toVec(const A3 & a) {return Vec3{a[0], a[1], a[2]};}
  static A3 toArr(const Vec3 & v) {return A3{v[0], v[1], v[2]};}

  static std::string timestamp()
  {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
  }

  const char * stateName() const
  {
    switch (state_) {
      case State::WAIT_STATE: return "espera";
      case State::HOMING: return "homing";
      case State::TRACKING: return "trayectoria";
      case State::DONE: return "fin";
    }
    return "?";
  }

  static double distToSegment(const Vec3 & p, const Vec3 & a, const Vec3 & b)
  {
    double ab[3], ap[3], L2 = 0.0, dot = 0.0;
    for (int i = 0; i < 3; ++i) {
      ab[i] = b[i] - a[i];
      ap[i] = p[i] - a[i];
      L2 += ab[i] * ab[i];
      dot += ap[i] * ab[i];
    }
    const double u = (L2 > 1e-12) ? std::min(1.0, std::max(0.0, dot / L2)) : 0.0;
    double d2 = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double e = ap[i] - u * ab[i];
      d2 += e * e;
    }
    return std::sqrt(d2);
  }

  // ------------------------------------------------------------------ carpetas de salida
  void createOutputDirs(const std::string & dir, const std::string & prefix)
  {
    namespace fs = std::filesystem;
    const fs::path base = dir.empty() ? fs::current_path() : fs::path(dir);
    root_dir_ = base / (prefix + "_" + timestamp());
    traj_dir_ = root_dir_ / "trayectorias";   // un CSV por trayectoria A->B
    plot_dir_ = root_dir_ / "graficas";       // un PNG por trayectoria A->B
    run_dir_ = root_dir_ / "ejecucion";       // CSV planeado vs. real de esta corrida
    std::error_code ec;
    for (const auto & d : {traj_dir_, plot_dir_, run_dir_}) {
      fs::create_directories(d, ec);
      if (ec) {
        RCLCPP_ERROR(get_logger(), "No pude crear la carpeta %s: %s", d.c_str(),
          ec.message().c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "Carpeta de salida: %s", root_dir_.c_str());
  }

  // ------------------------------------------------------------------ planeación
  Traj buildTrajectory(const std::string & from, const std::string & to)
  {
    Traj tr;
    tr.from = from;
    tr.to = to;
    const double dt = prm_.dt;
    const Vec3 pa = point_xyz_.at(from), pb = point_xyz_.at(to);
    const A3 qa = q_ik_.at(from), qb = q_ik_.at(to);

    // 1) Planificador cartesiano + Jacobiano (velocidades "crudas")
    miau::DiffIkPlanner sim(*model_, prm_);
    sim.setWaypoints(std::vector<Vec3>{pa, pb});
    sim.reset(toVec(qa));
    // La velocidad cruda se saca de las DIFERENCIAS de q_cmd (lo que el planificador realmente
    // hace con las articulaciones), no de qd_cmd; así la trayectoria es consistente con q_cmd.
    std::vector<A3> raw;
    miau::Sample s;
    const size_t max_steps = static_cast<size_t>(600.0 / dt);
    A3 q_prev = qa, q_int = qa;       // q_int: integral de qd_cmd (solo para diagnóstico)
    while (raw.size() < max_steps && sim.step(s)) {
      const A3 qc = toArr(s.q_cmd);
      A3 v{};
      for (int k = 0; k < 3; ++k) {
        v[k] = (qc[k] - q_prev[k]) / dt;
        q_int[k] += s.qd_cmd[k] * dt;
      }
      raw.push_back(v);
      q_prev = qc;
      tr.min_det = std::min(tr.min_det, std::fabs(s.det));
      if (s.singular) {++tr.n_sing;}
    }
    {
      double mism = 0.0;
      for (int k = 0; k < 3; ++k) {mism = std::max(mism, std::fabs(q_int[k] - q_prev[k]));}
      if (mism > 0.02) {
        RCLCPP_WARN(get_logger(),
          "  %s_a_%s: en el planificador, integral(qd_cmd) y q_cmd difieren %.3f rad; se usa q_cmd",
          from.c_str(), to.c_str(), mism);
      }
    }

    // 2) Suavizado: vel/acel/jerk acotados, arranque y llegada en reposo, termina en q_B
    smooth::Config cfg;
    cfg.dt = dt;
    cfg.window = smooth_time_;
    cfg.window_max = smooth_time_max_;
    cfg.vmax = prm_.max_joint_vel;
    cfg.amax = max_acc_;
    cfg.jmax = max_jerk_;
    A3 dq_target{};
    for (int i = 0; i < 3; ++i) {dq_target[i] = qb[i] - qa[i];}
    tr.info = smooth::build(raw, dq_target, cfg);
    tr.qd = tr.info.qd;
    tr.qd_raw = tr.info.qd_raw;

    // 3) Posición de referencia (integral de la velocidad), acel., jerk y TCP
    const size_t M = tr.qd.size();
    tr.q.resize(M);
    tr.qdd.resize(M);
    tr.jerk.resize(M);
    tr.p.resize(M);
    tr.dev_mm.resize(M);
    A3 q = qa, prev_v{0.0, 0.0, 0.0}, prev_a{0.0, 0.0, 0.0};
    for (size_t i = 0; i < M; ++i) {
      tr.q[i] = q;
      for (int k = 0; k < 3; ++k) {
        tr.qdd[i][k] = (tr.qd[i][k] - prev_v[k]) / dt;
        tr.jerk[i][k] = (tr.qdd[i][k] - prev_a[k]) / dt;
      }
      const Vec3 p = model_->fk(toVec(q));
      tr.p[i] = toArr(p);
      tr.dev_mm[i] = 1000.0 * distToSegment(p, pa, pb);
      tr.max_dev_mm = std::max(tr.max_dev_mm, tr.dev_mm[i]);
      prev_v = tr.qd[i];
      prev_a = tr.qdd[i];
      for (int k = 0; k < 3; ++k) {q[k] += tr.qd[i][k] * dt;}
    }
    tr.duration = static_cast<double>(M - 1) * dt;
    return tr;
  }

  void buildLibrary()
  {
    RCLCPP_INFO(get_logger(), "Planeando trayectorias (cada par A->B distinto una sola vez)...");
    double total = 0.0;
    for (size_t i = 0; i + 1 < sequence_.size(); ++i) {
      const std::string & a = sequence_[i], & b = sequence_[i + 1];
      const std::string key = a + "_a_" + b;
      if (library_.find(key) == library_.end()) {
        Traj tr = buildTrajectory(a, b);
        RCLCPP_INFO(get_logger(),
          "  %-9s T=%5.2f s | vel %.2f->%.2f rad/s | acel %6.1f->%5.2f | jerk %8.0f->%6.1f | "
          "filtro %.2f s x%.2f | desv. recta %.2f mm",
          key.c_str(), tr.duration, tr.info.peaks_raw.v, tr.info.peaks.v, tr.info.peaks_raw.a,
          tr.info.peaks.a, tr.info.peaks_raw.j, tr.info.peaks.j, tr.info.window_used,
          tr.info.stretch, tr.max_dev_mm);
        if (tr.max_dev_mm > max_path_dev_mm_) {
          RCLCPP_WARN(get_logger(),
            "  %s se aleja %.1f mm de la recta (> %.1f); baja smooth_time/smooth_time_max "
            "o sube max_joint_acc / max_joint_jerk", key.c_str(), tr.max_dev_mm,
            max_path_dev_mm_);
        }
        if (tr.info.end_corr > 0.1) {
          RCLCPP_FATAL(get_logger(),
            "  %s: el planificador termina a %.3f rad de la q de '%s'. Trayectoria inconsistente "
            "(¿otra rama del brazo o singularidad?); no se ejecuta. Revisa los puntos / q_seed.",
            key.c_str(), tr.info.end_corr, b.c_str());
          throw std::runtime_error("trayectoria inconsistente");
        }
        if (tr.info.end_corr > 0.02) {
          RCLCPP_WARN(get_logger(),
            "  %s: el planificador terminó a %.3f rad de la IK de '%s' (se corrigió suavemente)",
            key.c_str(), tr.info.end_corr, b.c_str());
        }
        if (tr.n_sing > 0) {
          RCLCPP_WARN(get_logger(), "  %s: %ld pasos cerca de singularidad (min|detJ|=%.2e)",
            key.c_str(), tr.n_sing, tr.min_det);
        }
        for (int k = 0; k < 3; ++k) {
          double mn = 1e9, mx = -1e9;
          for (const auto & qq : tr.q) {mn = std::min(mn, qq[k]); mx = std::max(mx, qq[k]);}
          if (mn < -M_PI || mx > M_PI) {
            RCLCPP_WARN(get_logger(), "  %s: %s sale de [-pi, pi] ([%.3f, %.3f])", key.c_str(),
              joint_names_[static_cast<size_t>(k)].c_str(), mn, mx);
          }
        }
        library_.emplace(key, std::move(tr));
        lib_order_.push_back(key);
      }
      queue_.push_back(&library_.at(key));
      total += library_.at(key).duration + prm_.dwell;
    }
    RCLCPP_INFO(get_logger(), "%zu trayectorias distintas, %zu tramos en la secuencia, ~%.1f s",
      library_.size(), queue_.size(), total);
  }

  // ------------------------------------------------------------------ exportar CSV / gráficas
  void exportLibrary()
  {
    for (const auto & key : lib_order_) {writeTrajCsv(library_.at(key));}
    writeSummary();

    const std::filesystem::path script = root_dir_ / "plot_trayectorias.py";
    {
      std::ofstream f(script.string());
      f << kPlotScript;
    }
    if (auto_plot_) {
      // En segundo plano para no retrasar el arranque; log en graficas.log
      const std::string cmd = "python3 \"" + script.string() + "\" \"" + root_dir_.string() +
        "\" > \"" + (root_dir_ / "graficas.log").string() + "\" 2>&1 &";
      if (std::system(cmd.c_str()) != 0) {
        RCLCPP_WARN(get_logger(),
          "No pude lanzar python3 para las gráficas. Ejecútalo a mano:  python3 %s %s",
          script.c_str(), root_dir_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Generando gráficas en %s (ver graficas.log)", plot_dir_.c_str());
      }
    } else {
      RCLCPP_INFO(get_logger(), "auto_plot=false. Para graficar:  python3 %s %s",
        script.c_str(), root_dir_.c_str());
    }
  }

  void writeTrajCsv(const Traj & tr)
  {
    const std::string path = (traj_dir_ / (tr.from + "_a_" + tr.to + ".csv")).string();
    std::ofstream f(path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "No pude abrir %s", path.c_str());
      return;
    }
    f << std::fixed << std::setprecision(9);
    f << "t_sec";
    for (const char * suf : {"_pos", "_vel", "_acc", "_jerk"}) {
      for (const auto & j : joint_names_) {f << "," << j << suf;}
    }
    f << ",x_m,y_m,z_m,dev_linea_mm";
    for (const auto & j : joint_names_) {f << "," << j << "_vel_raw";}
    f << "\n";
    for (size_t i = 0; i < tr.qd.size(); ++i) {
      f << static_cast<double>(i) * prm_.dt;
      for (int k = 0; k < 3; ++k) {f << "," << tr.q[i][k];}
      for (int k = 0; k < 3; ++k) {f << "," << tr.qd[i][k];}      // <- velocidad que se inyecta
      for (int k = 0; k < 3; ++k) {f << "," << tr.qdd[i][k];}
      for (int k = 0; k < 3; ++k) {f << "," << tr.jerk[i][k];}
      for (int k = 0; k < 3; ++k) {f << "," << tr.p[i][k];}
      f << "," << tr.dev_mm[i];
      for (int k = 0; k < 3; ++k) {f << "," << tr.qd_raw[i][k];}
      f << "\n";
    }
  }

  void writeSummary()
  {
    std::ofstream f((root_dir_ / "resumen_trayectorias.csv").string());
    f << std::fixed << std::setprecision(6);
    f << "trayectoria,desde,hasta,duracion_s,n_muestras,vel_max_rad_s,acel_max_rad_s2,"
      "jerk_max_rad_s3,vel_max_cruda,acel_max_cruda,jerk_max_crudo,ventana_filtro_s,"
      "factor_estiramiento,desv_max_recta_mm,correccion_final_rad,min_abs_detJ,pasos_singulares\n";
    for (const auto & key : lib_order_) {
      const Traj & t = library_.at(key);
      f << key << "," << t.from << "," << t.to << "," << t.duration << "," << t.qd.size() << ","
        << t.info.peaks.v << "," << t.info.peaks.a << "," << t.info.peaks.j << ","
        << t.info.peaks_raw.v << "," << t.info.peaks_raw.a << "," << t.info.peaks_raw.j << ","
        << t.info.window_used << "," << t.info.stretch << "," << t.max_dev_mm << ","
        << t.info.end_corr << "," << t.min_det << "," << t.n_sing << "\n";
    }
  }

  // ------------------------------------------------------------------ logs de la ejecución
  void openLogs()
  {
    plan_path_ = (run_dir_ / "plan.csv").string();
    real_path_ = (run_dir_ / "real.csv").string();
    plan_log_.open(plan_path_);
    real_log_.open(real_path_);
    if (!plan_log_.is_open() || !real_log_.is_open()) {
      RCLCPP_ERROR(get_logger(), "No pude abrir los CSV en %s", run_dir_.c_str());
      return;
    }
    plan_log_ << std::fixed << std::setprecision(9);
    real_log_ << std::fixed << std::setprecision(9);

    plan_log_ << "t_sec,fase,segmento";
    for (const auto & j : joint_names_) {plan_log_ << "," << j << "_pos_ref";}
    for (const auto & j : joint_names_) {plan_log_ << "," << j << "_vel_ff";}
    for (const auto & j : joint_names_) {plan_log_ << "," << j << "_vel_cmd";}
    for (const auto & j : joint_names_) {plan_log_ << "," << j << "_acc_ref";}
    for (const auto & j : joint_names_) {plan_log_ << "," << j << "_torque_rne";}
    plan_log_ << ",x_ref,y_ref,z_ref,det_J\n";

    real_log_ << "t_sec,fase,segmento";
    for (const auto & j : joint_names_) {
      real_log_ << "," << j << "_pos," << j << "_vel," << j << "_torque";
    }
    real_log_ << ",x_real,y_real,z_real\n";

    RCLCPP_INFO(get_logger(), "CSV planeado: %s", plan_path_.c_str());
    RCLCPP_INFO(get_logger(), "CSV real    : %s", real_path_.c_str());
  }

  void closeLogs()
  {
    if (plan_log_.is_open()) {plan_log_.flush(); plan_log_.close();}
    if (real_log_.is_open()) {real_log_.flush(); real_log_.close();}
  }

  void writePlanRow(
    const A3 & q, const A3 & qd_ff, const A3 & qd_cmd, const A3 & qdd)
  {
    if (!plan_log_.is_open()) {return;}
    const Vec3 qv = toVec(q);
    const Vec3 tau = model_->inverseDynamics(qv, toVec(qd_ff), toVec(qdd));
    const Vec3 p = model_->fk(qv);
    plan_log_ << (this->now() - t0_).seconds() << "," << stateName() << "," << segment_;
    for (int i = 0; i < 3; ++i) {plan_log_ << "," << q[i];}
    for (int i = 0; i < 3; ++i) {plan_log_ << "," << qd_ff[i];}
    for (int i = 0; i < 3; ++i) {plan_log_ << "," << qd_cmd[i];}
    for (int i = 0; i < 3; ++i) {plan_log_ << "," << qdd[i];}
    for (int i = 0; i < 3; ++i) {plan_log_ << "," << tau[i];}
    for (int i = 0; i < 3; ++i) {plan_log_ << "," << p[i];}
    plan_log_ << "," << miau::det3(model_->jacobian(qv)) << "\n";
    if (++plan_rows_ % 100 == 0) {plan_log_.flush();}
  }

  // ------------------------------------------------------------------ /joint_states
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    A3 q{}, qd{}, eff{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (size_t k = 0; k < 3; ++k) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[k]);
      if (it == msg->name.end()) {return;}                       // faltan juntas en el mensaje
      const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
      q[k] = idx < msg->position.size() ? msg->position[idx] : nan;
      qd[k] = idx < msg->velocity.size() ? msg->velocity[idx] : nan;
      eff[k] = idx < msg->effort.size() ? msg->effort[idx] : nan;
    }
    q_meas_ = q;

    if (state_ == State::WAIT_STATE) {
      t0_ = this->now();
      startHoming();
    }

    if (real_log_.is_open()) {
      if (state_ == State::DONE && (this->now() - t_done_).seconds() > tail_time_) {
        closeLogs();
        RCLCPP_INFO(get_logger(), "CSV cerrados. Puedes detener el nodo con Ctrl+C.");
        return;
      }
      const Vec3 p = model_->fk(toVec(q));
      real_log_ << (this->now() - t0_).seconds() << "," << stateName() << "," << segment_;
      for (int i = 0; i < 3; ++i) {real_log_ << "," << q[i] << "," << qd[i] << "," << eff[i];}
      real_log_ << "," << p[0] << "," << p[1] << "," << p[2] << "\n";
      if (++real_rows_ % 100 == 0) {real_log_.flush();}
    }
  }

  // ------------------------------------------------------------------ control
  void publishVel(const A3 & v)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {v[0], v[1], v[2]};
    cmd_pub_->publish(msg);
  }

  // Velocidad enviada = velocidad planeada + kp_joint * (q_ref - q_medida), saturada.
  void command(const A3 & q_ref, const A3 & qd_ff, const A3 & qdd_ref)
  {
    A3 v{};
    for (int i = 0; i < 3; ++i) {
      double e = q_ref[i] - q_meas_[i];
      if (!std::isfinite(e)) {e = 0.0;}
      v[i] = qd_ff[i] + kp_joint_ * e;
      v[i] = std::max(-prm_.max_joint_vel, std::min(prm_.max_joint_vel, v[i]));
    }
    publishVel(v);
    writePlanRow(q_ref, qd_ff, v, qdd_ref);
  }

  void startHoming()
  {
    double m = 0.0;
    for (int i = 0; i < 3; ++i) {
      q_home0_[i] = q_meas_[i];
      dq_home_[i] = q_start_[i] - q_home0_[i];
      m = std::max(m, std::fabs(dq_home_[i]));
    }
    // Mínimo jerk: vel. pico = 1.875*dq/T ; acel. pico = 5.7735*dq/T^2
    T_home_ = std::max({0.5, 1.875 * m / home_vel_, std::sqrt(5.7735 * m / max_acc_)});
    t_phase_ = 0.0;
    segment_ = -1;
    state_ = State::HOMING;
    RCLCPP_INFO(get_logger(),
      "HOMING: de [%.3f, %.3f, %.3f] a [%.3f, %.3f, %.3f] rad en %.1f s (vel. pico %.2f rad/s)",
      q_home0_[0], q_home0_[1], q_home0_[2], q_start_[0], q_start_[1], q_start_[2], T_home_,
      1.875 * m / T_home_);
  }

  void controlLoop()
  {
    switch (state_) {
      case State::HOMING: homingStep(); break;
      case State::TRACKING: trackingStep(); break;
      default: break;
    }
  }

  void homingStep()
  {
    t_phase_ += prm_.dt;
    const double tau = std::min(1.0, t_phase_ / T_home_);
    const double s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
    const double sd = 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / T_home_;
    const double sdd = 60.0 * tau * (1.0 - 3.0 * tau + 2.0 * tau * tau) / (T_home_ * T_home_);
    A3 q{}, qd{}, qdd{};
    for (int i = 0; i < 3; ++i) {
      q[i] = q_home0_[i] + s * dq_home_[i];
      qd[i] = sd * dq_home_[i];
      qdd[i] = sdd * dq_home_[i];
    }
    command(q, qd, qdd);

    if (tau >= 1.0) {
      seg_idx_ = 0;
      tick_ = 0;
      dwell_left_ = 0;
      hold_q_ = q_start_;
      segment_ = 0;
      state_ = State::TRACKING;
      announceSegment();
      RCLCPP_INFO(get_logger(), "Homing completo. Iniciando trayectoria.");
    }
  }

  void announceSegment()
  {
    const Traj & t = *queue_[seg_idx_];
    segment_ = static_cast<int>(seg_idx_);
    RCLCPP_INFO(get_logger(), "Segmento %zu/%zu: %s -> %s (%.2f s)", seg_idx_ + 1, queue_.size(),
      t.from.c_str(), t.to.c_str(), t.duration);
  }

  void trackingStep()
  {
    const A3 zero{0.0, 0.0, 0.0};
    if (dwell_left_ > 0) {                         // pausa entre trayectorias (reposo)
      --dwell_left_;
      command(hold_q_, zero, zero);
      return;
    }
    const Traj * tr = queue_[seg_idx_];
    if (tick_ >= tr->qd.size()) {                  // el tramo actual terminó
      hold_q_ = tr->q.back();
      if (seg_idx_ + 1 >= queue_.size()) {finish(); return;}
      ++seg_idx_;
      tick_ = 0;
      announceSegment();
      dwell_left_ = static_cast<int>(std::lround(prm_.dwell / prm_.dt));
      if (dwell_left_ > 0) {
        --dwell_left_;
        command(hold_q_, zero, zero);
        return;
      }
      tr = queue_[seg_idx_];
    }
    command(tr->q[tick_], tr->qd[tick_], tr->qdd[tick_]);
    ++tick_;
  }

  void finish()
  {
    publishVel(A3{0.0, 0.0, 0.0});
    state_ = State::DONE;
    t_done_ = this->now();
    timer_->cancel();
    if (plan_log_.is_open()) {plan_log_.flush();}
    RCLCPP_INFO(get_logger(), "Trayectoria completa (%.1f s desde el arranque).",
      (t_done_ - t0_).seconds());
  }

  // ------------------------------------------------------------------ miembros
  KDL::Chain chain_;
  std::unique_ptr<KdlModel> model_;
  miau::PlannerParams prm_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> sequence_;
  std::map<std::string, Vec3> point_xyz_;          // punto -> [x,y,z]
  std::map<std::string, A3> q_ik_;                 // punto -> q (IK)
  std::map<std::string, Traj> library_;            // "A_a_B" -> trayectoria (todas distintas)
  std::vector<std::string> lib_order_;
  std::vector<const Traj *> queue_;                // orden de ejecución (según 'sequence')

  A3 q_start_{}, q_meas_{}, q_home0_{}, dq_home_{}, hold_q_{};
  double home_vel_{0.5}, T_home_{1.0}, t_phase_{0.0}, tail_time_{1.0};
  double max_acc_{2.0}, max_jerk_{10.0}, smooth_time_{0.3}, smooth_time_max_{1.5};
  double max_path_dev_mm_{5.0}, kp_joint_{2.0};
  bool auto_plot_{true};

  State state_{State::WAIT_STATE};
  int segment_{0};
  size_t seg_idx_{0}, tick_{0};
  int dwell_left_{0};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t_done_{0, 0, RCL_ROS_TIME};

  std::filesystem::path root_dir_, traj_dir_, plot_dir_, run_dir_;
  std::string plan_path_, real_path_;
  std::ofstream plan_log_, real_log_;
  long plan_rows_{0}, real_rows_{0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  try {
    auto node = std::make_shared<TrajectoryJacobianNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_jacobian_node"), "Error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
