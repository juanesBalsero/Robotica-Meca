#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Perfil trapezoidal de velocidad (tipo 2: rampa - meseta - rampa) para una
// sola junta, sincronizado a un tiempo total T y un tiempo de aceleración ta
// impuestos externamente (definidos por la junta "líder", la de mayor
// recorrido). Esto hace que TODAS las juntas arranquen y terminen a la vez.
// ---------------------------------------------------------------------------
struct JointTrapProfile
{
  double q0{0.0};
  double qf{0.0};
  double d{0.0};      // recorrido con signo (qf-q0)
  double vmax{0.0};   // velocidad máxima ESCALADA para esta junta
  double amax{0.0};   // aceleración máxima ESCALADA para esta junta

  // Evalúa posición y velocidad en el instante t (0 <= t <= T)
  void eval(double t, double ta, double T, double & pos, double & vel) const
  {
    double sign = (d >= 0.0) ? 1.0 : -1.0;
    double av = std::fabs(vmax);
    double aa = std::fabs(amax);

    if (t <= ta) {
      // Rampa de aceleración
      pos = q0 + sign * 0.5 * aa * t * t;
      vel = sign * aa * t;
    } else if (t <= (T - ta)) {
      // Meseta de velocidad constante
      pos = q0 + sign * (av * (t - ta / 2.0));
      vel = sign * av;
    } else if (t <= T) {
      // Rampa de desaceleración
      double tr = T - t;
      pos = qf - sign * 0.5 * aa * tr * tr;
      vel = sign * aa * tr;
    } else {
      pos = qf;
      vel = 0.0;
    }
  }
};

class TrajectoryIkNode : public rclcpp::Node
{
public:
  TrajectoryIkNode() : Node("trajectory_ik_node")
  {
    // Instante de referencia (t=0) para los logs: se resta a cada muestra
    // para loggear tiempo transcurrido en segundos, en vez del timestamp
    // absoluto de época Unix.
    t0_ = this->now();

    // ---------------- Parámetros ----------------
    this->declare_parameter<std::string>("package_name", "miau");
    this->declare_parameter<std::string>("urdf_relative_path", "urdf/Completo.urdf");
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("tip_link", "tcp");
    this->declare_parameter<double>("max_vel", 1.0);     // rad/s, junta líder
    this->declare_parameter<double>("max_accel", 2.0);   // rad/s^2, junta líder (ta = 0.5 s)
    this->declare_parameter<double>("control_rate_hz", 100.0);
    this->declare_parameter<std::string>("planned_log_path", "trajectory_planned.csv");
    this->declare_parameter<std::string>("real_log_path", "trajectory_real.csv");
    this->declare_parameter<std::string>("commands_topic", "/position_controller/commands");

    std::string pkg       = this->get_parameter("package_name").as_string();
    std::string urdf_rel  = this->get_parameter("urdf_relative_path").as_string();
    std::string base_link = this->get_parameter("base_link").as_string();
    std::string tip_link  = this->get_parameter("tip_link").as_string();
    max_vel_       = this->get_parameter("max_vel").as_double();
    max_accel_     = this->get_parameter("max_accel").as_double();
    control_rate_  = this->get_parameter("control_rate_hz").as_double();
    planned_log_path_ = this->get_parameter("planned_log_path").as_string();
    real_log_path_    = this->get_parameter("real_log_path").as_string();
    std::string commands_topic = this->get_parameter("commands_topic").as_string();

    std::string urdf_path = ament_index_cpp::get_package_share_directory(pkg) + "/" + urdf_rel;

    // ---------------- Cargar cadena cinemática desde el URDF (KDL) ----------------
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
      RCLCPP_FATAL(this->get_logger(), "No pude parsear el URDF en: %s", urdf_path.c_str());
      throw std::runtime_error("URDF parse failed");
    }
    if (!tree.getChain(base_link, tip_link, chain_)) {
      RCLCPP_FATAL(this->get_logger(), "No existe cadena entre '%s' y '%s'",
                   base_link.c_str(), tip_link.c_str());
      throw std::runtime_error("Chain not found");
    }
    n_joints_ = chain_.getNrOfJoints();
    RCLCPP_INFO(this->get_logger(), "Cadena KDL cargada con %u juntas activas", n_joints_);

    // ---------------- Puntos cartesianos: Home -> Caja A ----------------
    KDL::Vector home (-0.073269, 0.149745, 0.147251);
    KDL::Vector caja_a(0.045943,  0.28315,  0.0779937);

    // ---------------- IK numérica (LMA), solo restringe posición x,y,z ----------------
    // Brazo de 3 GDL sin muñeca: no hay orientación independiente que fijar,
    // por eso los pesos de orientación (rx,ry,rz) van en 0.
    Eigen::Matrix<double, 6, 1> L;
    L(0) = 1.0; L(1) = 1.0; L(2) = 1.0;
    L(3) = 0.0; L(4) = 0.0; L(5) = 0.0;
    KDL::ChainIkSolverPos_LMA ik_solver(chain_, L);

    KDL::JntArray q_seed(n_joints_);
    q_seed.data.setZero();

    KDL::JntArray q_home(n_joints_), q_target(n_joints_);
    {
      KDL::Frame target(KDL::Rotation::Identity(), home);
      int rc = ik_solver.CartToJnt(q_seed, target, q_home);
      if (rc < 0) {
        RCLCPP_WARN(this->get_logger(),
          "IK no convergió del todo para Home (rc=%d); se usa la mejor aproximación", rc);
      }
    }
    q_seed = q_home;
    {
      KDL::Frame target(KDL::Rotation::Identity(), caja_a);
      int rc = ik_solver.CartToJnt(q_seed, target, q_target);
      if (rc < 0) {
        RCLCPP_WARN(this->get_logger(),
          "IK no convergió del todo para Caja A (rc=%d); se usa la mejor aproximación", rc);
      }
    }
    RCLCPP_INFO(this->get_logger(), "IK calculada para Home y Caja A");

    // ---------------- Perfil trapezoidal sincronizado ----------------
    buildTrapezoidalProfile(q_home, q_target);

    // ---------------- Publicador de comandos de posición ----------------
    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(commands_topic, 10);

    // ---------------- Log de la trayectoria PLANEADA (analítica) ----------------
    planned_log_.open(planned_log_path_);
    planned_log_ << std::fixed << std::setprecision(9);
    planned_log_ << "t_sec,joint,position_cmd,velocity_cmd\n";

    // ---------------- Log de /joint_states (trayectoria REAL en Gazebo) ----------------
    real_log_.open(real_log_path_);
    real_log_ << std::fixed << std::setprecision(9);
    real_log_ << "t_sec,joint,position,velocity,effort\n";
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { this->logJointState(msg); });

    // ---------------- Temporizador de control a control_rate_hz ----------------
    t_traj_ = 0.0;
    double dt = 1.0 / control_rate_;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      [this, dt]() { this->timerCallback(dt); });

    RCLCPP_INFO(this->get_logger(),
      "Trayectoria trapezoidal iniciada: T=%.3f s, ta=%.3f s, vmax_lider=%.3f rad/s, amax_lider=%.3f rad/s^2",
      T_, ta_, max_vel_, max_accel_);
  }

  ~TrajectoryIkNode() override
  {
    if (planned_log_.is_open()) planned_log_.close();
    if (real_log_.is_open()) real_log_.close();
  }

private:
  // Calcula, para cada junta, el perfil trapezoidal escalado de forma que
  // todas compartan el mismo T (tiempo total) y ta (tiempo de aceleración),
  // definidos por la junta de mayor recorrido angular ("líder") usando
  // max_vel_ / max_accel_.
  void buildTrapezoidalProfile(const KDL::JntArray & q_home, const KDL::JntArray & q_target)
  {
    profiles_.resize(n_joints_);
    double d_max = 0.0;
    for (unsigned int j = 0; j < n_joints_; ++j) {
      profiles_[j].q0 = q_home(j);
      profiles_[j].qf = q_target(j);
      profiles_[j].d  = q_target(j) - q_home(j);
      d_max = std::max(d_max, std::fabs(profiles_[j].d));
    }

    if (d_max < 1e-9) {
      // Sin movimiento apreciable: perfil trivial.
      T_ = 0.0;
      ta_ = 0.0;
      for (auto & p : profiles_) { p.vmax = 0.0; p.amax = 0.0; }
      RCLCPP_WARN(this->get_logger(), "Home y Caja A coinciden en el espacio articular; T=0");
      return;
    }

    // Perfil de la junta líder con vmax = max_vel_, amax = max_accel_.
    double ta_lead = max_vel_ / max_accel_;
    if (d_max >= (max_vel_ * max_vel_) / max_accel_) {
      // Caso trapezoidal: hay meseta de velocidad constante.
      ta_ = ta_lead;
      double tv = (d_max - max_vel_ * ta_) / max_vel_;
      T_ = 2.0 * ta_ + tv;
    } else {
      // Caso triangular: no alcanza a llegar a max_vel_, se recorta.
      ta_ = std::sqrt(d_max / max_accel_);
      T_  = 2.0 * ta_;
    }

    // Escalamos vmax/amax de cada junta manteniendo el MISMO ta_ y T_,
    // de modo que el área bajo la curva de velocidad (= recorrido) sea
    // proporcional a d_j / d_max. Esto sincroniza inicio y fin de todas
    // las juntas sin cambiar la forma del perfil.
    for (auto & p : profiles_) {
      double ratio = std::fabs(p.d) / d_max;
      p.vmax = ratio * max_vel_;
      p.amax = (ta_ > 1e-9) ? (p.vmax / ta_) : 0.0;
    }
  }

  void publishAt(double t)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(n_joints_);
    double t_sec = (this->now() - t0_).seconds();
    for (unsigned int j = 0; j < n_joints_; ++j) {
      double pos, vel;
      profiles_[j].eval(t, ta_, T_, pos, vel);
      msg.data[j] = pos;
      planned_log_ << t_sec << "," << "joint" << j << "," << pos << "," << vel << "\n";
    }
    planned_log_.flush();
    cmd_pub_->publish(msg);
  }

  void timerCallback(double dt)
  {
    if (t_traj_ > T_) {
      if (!finished_) {
        // Última muestra exacta en T_ para terminar limpio en el objetivo.
        publishAt(T_);
        timer_->cancel();
        finished_ = true;
        RCLCPP_INFO(this->get_logger(), "Trayectoria completa.");
      }
      return;
    }
    publishAt(t_traj_);
    t_traj_ += dt;
  }

  void logJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Tiempo transcurrido desde el arranque del nodo (t0_), no timestamp absoluto.
    double t = (this->now() - t0_).seconds();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      double pos = i < msg->position.size() ? msg->position[i] : std::nan("");
      double vel = i < msg->velocity.size() ? msg->velocity[i] : std::nan("");
      // effort solo tendrá datos reales si el URDF/ros2_control declara
      // <state_interface name="effort"/> para cada junta.
      double eff = i < msg->effort.size() ? msg->effort[i] : std::nan("");
      real_log_ << t << "," << msg->name[i] << "," << pos << "," << vel << "," << eff << "\n";
    }
    real_log_.flush();
  }

  rclcpp::Time t0_;
  KDL::Chain chain_;
  unsigned int n_joints_{0};

  std::vector<JointTrapProfile> profiles_;
  double max_vel_{1.0};
  double max_accel_{2.0};
  double T_{0.0};    // tiempo total de la trayectoria (todas las juntas)
  double ta_{0.0};   // tiempo de aceleración (todas las juntas)
  double t_traj_{0.0};
  bool finished_{false};

  double control_rate_{100.0};
  std::string planned_log_path_;
  std::string real_log_path_;
  std::ofstream planned_log_;
  std::ofstream real_log_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryIkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}