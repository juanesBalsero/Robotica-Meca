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

using namespace std::chrono_literals;

class TrajectoryIkNode : public rclcpp::Node
{
public:
  TrajectoryIkNode() : Node("trajectory_ik_node")
  {
    // Instante de referencia (t=0) para el log: se resta a cada muestra
    // para loggear tiempo transcurrido en segundos, en vez del timestamp
    // absoluto de época Unix (que perdía precisión al truncarse a 6 cifras
    // significativas y hacía que todas las muestras parecieran tener el
    // mismo tiempo).
    t0_ = this->now();

    // ---------------- Parámetros ----------------
    this->declare_parameter<std::string>("package_name", "miau");
    this->declare_parameter<std::string>("urdf_relative_path", "urdf/Completo.urdf");
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("tip_link", "tcp");
    this->declare_parameter<double>("total_time", 3.661);   // segundos, distancia/velocidad
    this->declare_parameter<int>("num_divisions", 500);
    this->declare_parameter<std::string>("log_path", "trajectory_log.csv");
    this->declare_parameter<std::string>("commands_topic", "/position_controller/commands");

    std::string pkg       = this->get_parameter("package_name").as_string();
    std::string urdf_rel  = this->get_parameter("urdf_relative_path").as_string();
    std::string base_link = this->get_parameter("base_link").as_string();
    std::string tip_link  = this->get_parameter("tip_link").as_string();
    total_time_    = this->get_parameter("total_time").as_double();
    num_divisions_ = this->get_parameter("num_divisions").as_int();
    log_path_      = this->get_parameter("log_path").as_string();
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

    // ---------------- Waypoints cartesianos: Home -> Medicamento A ----------------
    KDL::Vector home (0.2043, 0.1135, 0.3847);
    KDL::Vector med_a(0.3541, 0.3657, 0.1656);
    for (int i = 0; i <= num_divisions_; ++i) {
      double s = static_cast<double>(i) / static_cast<double>(num_divisions_);
      waypoints_.push_back(home + s * (med_a - home));
    }

    // ---------------- IK numérica (LMA), solo restringe posición x,y,z ----------------
    // Brazo de 3 GDL sin muñeca: no hay orientación independiente que fijar,
    // por eso los pesos de orientación (rx,ry,rz) van en 0.
    Eigen::Matrix<double, 6, 1> L;
    L(0) = 1.0; L(1) = 1.0; L(2) = 1.0;
    L(3) = 0.0; L(4) = 0.0; L(5) = 0.0;
    KDL::ChainIkSolverPos_LMA ik_solver(chain_, L);

    KDL::JntArray q_seed(n_joints_);
    q_seed.data.setZero();

    for (const auto & p : waypoints_) {
      KDL::Frame target(KDL::Rotation::Identity(), p);
      KDL::JntArray q_out(n_joints_);
      int rc = ik_solver.CartToJnt(q_seed, target, q_out);
      if (rc < 0) {
        RCLCPP_WARN(this->get_logger(),
          "IK no convergió del todo en un waypoint (rc=%d); se usa la mejor aproximación", rc);
      }
      joint_solutions_.push_back(q_out);
      q_seed = q_out;  // siguiente punto arranca desde la solución anterior (continuidad)
    }
    RCLCPP_INFO(this->get_logger(), "IK calculada para %zu waypoints", joint_solutions_.size());

    // ---------------- Publicador de comandos de posición ----------------
    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(commands_topic, 10);

    // ---------------- Log de /joint_states (posición, velocidad, esfuerzo si existe) ----------------
    log_file_.open(log_path_);
    log_file_ << std::fixed << std::setprecision(9);
    log_file_ << "t_sec,joint,position,velocity,effort\n";
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { this->logJointState(msg); });

    // ---------------- Avance de waypoints cada (total_time / num_divisions) s ----------------
    double seg_time = total_time_ / static_cast<double>(num_divisions_);
    current_idx_ = 0;
    sendWaypoint(current_idx_);   // manda Home de entrada
    current_idx_ = 1;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(seg_time),
      [this]() { this->timerCallback(); });

    RCLCPP_INFO(this->get_logger(),
      "Trayectoria iniciada: %d segmentos de %.3f s (total %.3f s)",
      num_divisions_, seg_time, total_time_);
  }

  ~TrajectoryIkNode() override
  {
    if (log_file_.is_open()) log_file_.close();
  }

private:
  void sendWaypoint(size_t idx)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(n_joints_);
    for (unsigned int j = 0; j < n_joints_; ++j) {
      msg.data[j] = joint_solutions_[idx](j);
    }
    cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Waypoint %zu -> J1=%.4f J2=%.4f J3=%.4f",
                idx, msg.data[0], msg.data[1], msg.data[2]);
  }

  void timerCallback()
  {
    if (current_idx_ > static_cast<size_t>(num_divisions_)) {
      timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Trayectoria completa.");
      return;
    }
    sendWaypoint(current_idx_);
    current_idx_++;
  }

  void logJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Tiempo transcurrido desde el arranque del nodo (t0_), no timestamp absoluto.
    double t = (this->now() - t0_).seconds();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      double pos = i < msg->position.size() ? msg->position[i] : std::nan("");
      double vel = i < msg->velocity.size() ? msg->velocity[i] : std::nan("");
      // effort solo tendrá datos reales si se agrega <state_interface name="effort"/> en el URDF
      double eff = i < msg->effort.size() ? msg->effort[i] : std::nan("");
      log_file_ << t << "," << msg->name[i] << "," << pos << "," << vel << "," << eff << "\n";
    }
    log_file_.flush();
  }

  rclcpp::Time t0_;
  KDL::Chain chain_;
  unsigned int n_joints_{0};
  std::vector<KDL::Vector> waypoints_;
  std::vector<KDL::JntArray> joint_solutions_;
  double total_time_;
  int num_divisions_;
  size_t current_idx_{0};
  std::string log_path_;
  std::ofstream log_file_;

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