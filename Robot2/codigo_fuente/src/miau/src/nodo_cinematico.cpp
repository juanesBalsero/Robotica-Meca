#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "miau/diff_ik_planner.hpp"
#include "miau/realimentacion.hpp"

using miau::Vec3;

struct Trayectoria
{
  std::string desde, hasta;
  std::vector<Vec3> q, qd;
};

class NodoCinematico : public rclcpp::Node
{
  enum class Estado {ESPERA, HOMING, LIBRE, EJECUTANDO, PAUSA};

public:
  explicit NodoCinematico(const rclcpp::NodeOptions & options)
  : Node("nodo_cinematico", options)
  {
    juntas_ = param<std::vector<std::string>>("joint_names", {"J1", "J2", "J3"});
    if (juntas_.size() != 3) {throw std::runtime_error("joint_names debe tener 3 juntas");}

    const std::string topic_cmd =
      param<std::string>("commands_topic", "/velocity_controller/commands");
    const std::string fuente = param<std::string>("fuente_realimentacion", "gazebo");
    const std::string topic_fb = param<std::string>(
      "topic_realimentacion", fuente == "gazebo" ? "/joint_states" : "/robot/posiciones");
    const std::string topic_orden = param<std::string>("topic_comando", "/comando_trayectoria");
    const std::string topic_estado = param<std::string>("topic_estado", "/estado_trayectoria");

    ts_ = getDouble("ts", 0.1);
    kp_ = getDouble("kp_joint", 2.0);
    qd_max_ = getDouble("max_joint_vel", 1.0);
    acc_max_ = getDouble("max_joint_acc", 2.0);
    home_vel_ = std::min(getDouble("home_joint_vel", 0.5), qd_max_);
    pausa_ = getDouble("dwell_time", 0.5);
    tol_inicio_ = getDouble("tol_inicio", 0.1);
    err_max_ = getDouble("error_maximo", 0.3);

    const std::string dir =
      expandir(param<std::string>("directorio_trayectorias", "~/miau_trayectorias"));
    cargarDirectorio(dir);

    punto_inicial_ = param<std::string>("punto_inicial", "PH");
    if (!punto_inicial_.empty() && puntos_.find(punto_inicial_) == puntos_.end()) {
      RCLCPP_WARN(get_logger(), "punto_inicial '%s' no existe en los archivos; se ignora",
        punto_inicial_.c_str());
      punto_inicial_.clear();
    }

    fb_ = miau::crearRealimentacion(*this, fuente, topic_fb, juntas_);
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_cmd, 10);
    pub_estado_ = create_publisher<std_msgs::msg::String>(topic_estado, 10);
    sub_ = create_subscription<std_msgs::msg::String>(
      topic_orden, 10, [this](std_msgs::msg::String::SharedPtr msg) {onOrden(msg->data);});
    timer_ = create_timer(std::chrono::duration<double>(ts_), [this]() {paso();});

    RCLCPP_INFO(get_logger(), "Ts=%.3f s | fuente=%s | realimentacion=%s | comandos=%s",
      ts_, fuente.c_str(), topic_fb.c_str(), topic_cmd.c_str());
    RCLCPP_INFO(get_logger(),
      "Ordenes en %s, ejemplo: ros2 topic pub --once %s std_msgs/msg/String \"{data: 'PH P1'}\"",
      topic_orden.c_str(), topic_orden.c_str());
  }

private:
  template<typename T>
  T param(const std::string & nombre, const T & def)
  {
    if (!has_parameter(nombre)) {declare_parameter<T>(nombre, def);}
    return get_parameter(nombre).get_value<T>();
  }

  double getDouble(const std::string & nombre, double def)
  {
    if (!has_parameter(nombre)) {declare_parameter<double>(nombre, def);}
    const rclcpp::Parameter p = get_parameter(nombre);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(p.as_int());
    }
    return p.as_double();
  }

  static std::string expandir(const std::string & ruta)
  {
    if (!ruta.empty() && ruta[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home != nullptr) {return std::string(home) + ruta.substr(1);}
    }
    return ruta;
  }

  static std::string mayusculas(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(),
      [](unsigned char c) {return static_cast<char>(std::toupper(c));});
    return s;
  }

  static std::vector<std::string> dividir(const std::string & s, char sep)
  {
    std::vector<std::string> out;
    std::string item;
    std::istringstream ss(s);
    while (std::getline(ss, item, sep)) {out.push_back(item);}
    return out;
  }

  static std::vector<std::string> separar(std::string s)
  {
    for (const std::string & sep :
      {std::string("->"), std::string("_a_"), std::string(","), std::string(";")})
    {
      size_t pos = 0;
      while ((pos = s.find(sep, pos)) != std::string::npos) {
        s.replace(pos, sep.size(), " ");
        pos += 1;
      }
    }
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string t;
    while (ss >> t) {
      if (t != "a" && t != "A") {out.push_back(t);}
    }
    return out;
  }

  bool cargarCsv(const std::string & ruta, Trayectoria & tr) const
  {
    std::ifstream f(ruta);
    if (!f.is_open()) {return false;}
    std::string linea;
    if (!std::getline(f, linea)) {return false;}
    const std::vector<std::string> cols = dividir(linea, ',');
    auto indice = [&cols](const std::string & nombre) {
        const auto it = std::find(cols.begin(), cols.end(), nombre);
        return it == cols.end() ? -1 : static_cast<int>(std::distance(cols.begin(), it));
      };
    const int it_t = indice("t_sec");
    int ip[3], iv[3];
    for (size_t k = 0; k < 3; ++k) {
      ip[k] = indice(juntas_[k] + "_pos");
      iv[k] = indice(juntas_[k] + "_vel");
    }
    if (it_t < 0 || ip[0] < 0 || ip[1] < 0 || ip[2] < 0 || iv[0] < 0 || iv[1] < 0 || iv[2] < 0) {
      return false;
    }
    std::vector<double> tiempos;
    try {
      while (std::getline(f, linea)) {
        if (linea.empty()) {continue;}
        const std::vector<std::string> c = dividir(linea, ',');
        tiempos.push_back(std::stod(c.at(static_cast<size_t>(it_t))));
        Vec3 q{}, qd{};
        for (size_t k = 0; k < 3; ++k) {
          q[k] = std::stod(c.at(static_cast<size_t>(ip[k])));
          qd[k] = std::stod(c.at(static_cast<size_t>(iv[k])));
        }
        tr.q.push_back(q);
        tr.qd.push_back(qd);
      }
    } catch (const std::exception &) {
      return false;
    }
    if (tr.q.size() < 2) {return false;}
    const double dt = tiempos[1] - tiempos[0];
    if (std::fabs(dt - ts_) > 1e-4) {
      RCLCPP_WARN(get_logger(), "%s: el CSV tiene dt=%.4f s y ts=%.4f s; se descarta",
        ruta.c_str(), dt, ts_);
      return false;
    }
    return true;
  }

  void cargarDirectorio(const std::string & dir)
  {
    namespace fs = std::filesystem;
    if (!fs::is_directory(dir)) {
      throw std::runtime_error("No existe el directorio de trayectorias: " + dir);
    }
    std::vector<fs::path> archivos;
    for (const auto & e : fs::directory_iterator(dir)) {
      if (e.is_regular_file() && e.path().extension() == ".csv") {archivos.push_back(e.path());}
    }
    std::sort(archivos.begin(), archivos.end());
    for (const auto & p : archivos) {
      const std::string nombre = p.stem().string();
      const size_t pos = nombre.find("_a_");
      if (pos == std::string::npos) {continue;}
      Trayectoria tr;
      tr.desde = nombre.substr(0, pos);
      tr.hasta = nombre.substr(pos + 3);
      if (!cargarCsv(p.string(), tr)) {
        RCLCPP_WARN(get_logger(), "No pude leer %s", p.c_str());
        continue;
      }
      trayectorias_[nombre] = std::move(tr);
    }
    if (trayectorias_.empty()) {
      throw std::runtime_error("No hay trayectorias validas en " + dir);
    }
    for (const auto & kv : trayectorias_) {
      if (puntos_.find(kv.second.hasta) == puntos_.end()) {
        puntos_[kv.second.hasta] = kv.second.q.back();
      }
    }
    for (const auto & kv : trayectorias_) {
      const Vec3 & q0 = kv.second.q.front();
      const auto it = puntos_.find(kv.second.desde);
      if (it != puntos_.end() && miau::maxAbs(miau::sub(it->second, q0)) > 0.05) {
        RCLCPP_WARN(get_logger(), "El punto '%s' no tiene la misma q en todos los archivos",
          kv.second.desde.c_str());
      }
      puntos_[kv.second.desde] = q0;
    }
    std::string lista;
    for (const auto & kv : trayectorias_) {lista += " " + kv.first;}
    RCLCPP_INFO(get_logger(), "%zu trayectorias cargadas de %s:%s",
      trayectorias_.size(), dir.c_str(), lista.c_str());
  }

  void estado(const std::string & texto)
  {
    std_msgs::msg::String msg;
    msg.data = texto;
    pub_estado_->publish(msg);
  }

  void publicar(const Vec3 & v)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {v[0], v[1], v[2]};
    pub_->publish(msg);
  }

  void comandar(const Vec3 & q_ref, const Vec3 & qd_ref, const Vec3 & q)
  {
    Vec3 v{};
    for (size_t i = 0; i < 3; ++i) {
      v[i] = qd_ref[i] + kp_ * (q_ref[i] - q[i]);
      v[i] = std::max(-qd_max_, std::min(qd_max_, v[i]));
    }
    publicar(v);
  }

  void mantener(const Vec3 & q)
  {
    comandar(hold_q_, Vec3{0.0, 0.0, 0.0}, q);
  }

  void abortar(const Vec3 & q, const std::string & motivo)
  {
    RCLCPP_ERROR(get_logger(), "%s", motivo.c_str());
    cola_.clear();
    hold_q_ = q;
    estado_ = Estado::LIBRE;
    estado("error: " + motivo);
    mantener(q);
  }

  void onOrden(const std::string & texto)
  {
    const std::vector<std::string> tok = separar(texto);
    if (tok.empty()) {return;}
    const std::string clave = mayusculas(tok[0]);

    if (clave == "PARAR" || clave == "STOP") {
      cola_.clear();
      Vec3 q{};
      if (fb_->leer(q)) {
        hold_q_ = q;
        estado_ = Estado::LIBRE;
        mantener(q);
      }
      RCLCPP_WARN(get_logger(), "PARAR: cola vaciada, manteniendo posicion");
      estado("parado");
      return;
    }

    if (clave == "LISTAR") {
      std::string lista;
      for (const auto & kv : trayectorias_) {lista += " " + kv.first;}
      RCLCPP_INFO(get_logger(), "Trayectorias disponibles:%s", lista.c_str());
      return;
    }

    if (clave == "IR") {
      if (tok.size() != 2 || puntos_.find(tok[1]) == puntos_.end()) {
        RCLCPP_ERROR(get_logger(), "Uso: ir <punto> (usa 'listar' para ver las trayectorias)");
        return;
      }
      Vec3 q{};
      if (estado_ != Estado::LIBRE || !fb_->leer(q)) {
        RCLCPP_ERROR(get_logger(), "No puedo ir a '%s' ahora (ocupado o sin realimentacion)",
          tok[1].c_str());
        return;
      }
      iniciarHoming(q, puntos_.at(tok[1]), tok[1]);
      return;
    }

    if (tok.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Dame al menos origen y destino, por ejemplo: PH P1");
      return;
    }
    std::vector<std::string> nuevas;
    for (size_t i = 0; i + 1 < tok.size(); ++i) {
      const std::string nombre = tok[i] + "_a_" + tok[i + 1];
      if (trayectorias_.find(nombre) == trayectorias_.end()) {
        RCLCPP_ERROR(get_logger(), "No existe la trayectoria %s (usa 'listar')", nombre.c_str());
        return;
      }
      nuevas.push_back(nombre);
    }
    for (const auto & n : nuevas) {cola_.push_back(n);}
    RCLCPP_INFO(get_logger(), "En cola: %zu trayectoria(s)", nuevas.size());
  }

  void iniciarHoming(const Vec3 & q, const Vec3 & destino, const std::string & nombre)
  {
    q_home0_ = q;
    q_home1_ = destino;
    double m = 0.0;
    for (size_t i = 0; i < 3; ++i) {
      dq_home_[i] = destino[i] - q[i];
      m = std::max(m, std::fabs(dq_home_[i]));
    }
    T_home_ = std::max({0.5, 1.875 * m / home_vel_, std::sqrt(5.7735 * m / acc_max_)});
    t_ = 0.0;
    estado_ = Estado::HOMING;
    RCLCPP_INFO(get_logger(), "HOMING a %s: de [%.3f, %.3f, %.3f] a [%.3f, %.3f, %.3f] en %.1f s",
      nombre.c_str(), q[0], q[1], q[2], destino[0], destino[1], destino[2], T_home_);
    estado("homing " + nombre);
  }

  void pasoHoming(const Vec3 & q)
  {
    t_ += ts_;
    const double tau = std::min(1.0, t_ / T_home_);
    const double s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
    const double sd = 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / T_home_;
    Vec3 q_ref{}, qd_ref{};
    for (size_t i = 0; i < 3; ++i) {
      q_ref[i] = q_home0_[i] + s * dq_home_[i];
      qd_ref[i] = sd * dq_home_[i];
    }
    comandar(q_ref, qd_ref, q);
    if (tau >= 1.0) {
      hold_q_ = q_home1_;
      estado_ = Estado::LIBRE;
      estado("libre");
    }
  }

  void iniciarTrayectoria(const Vec3 & q)
  {
    const std::string nombre = cola_.front();
    cola_.pop_front();
    actual_ = &trayectorias_.at(nombre);
    nombre_actual_ = nombre;
    const double d = miau::maxAbs(miau::sub(actual_->q.front(), q));
    if (d > tol_inicio_) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "el robot no esta en '%s' (diferencia %.3f rad). Usa: ir %s",
        actual_->desde.c_str(), d, actual_->desde.c_str());
      abortar(q, buf);
      return;
    }
    tick_ = 0;
    estado_ = Estado::EJECUTANDO;
    RCLCPP_INFO(get_logger(), "Ejecutando %s (%zu muestras, %.1f s)", nombre.c_str(),
      actual_->q.size(), static_cast<double>(actual_->q.size()) * ts_);
    estado("ejecutando " + nombre);
  }

  void paso()
  {
    Vec3 q{};
    if (!fb_->leer(q)) {return;}

    if (estado_ == Estado::ESPERA) {
      hold_q_ = q;
      estado_ = Estado::LIBRE;
      if (!punto_inicial_.empty()) {
        iniciarHoming(q, puntos_.at(punto_inicial_), punto_inicial_);
      } else {
        estado("libre");
      }
    }

    if (estado_ == Estado::HOMING) {
      pasoHoming(q);
      return;
    }

    if (estado_ == Estado::PAUSA) {
      if (pausa_left_ > 0) {
        --pausa_left_;
        mantener(q);
        return;
      }
      estado_ = Estado::LIBRE;
      if (cola_.empty()) {estado("libre");}
    }

    if (estado_ == Estado::LIBRE) {
      if (cola_.empty()) {
        mantener(q);
        return;
      }
      iniciarTrayectoria(q);
      if (estado_ != Estado::EJECUTANDO) {return;}
    }

    if (estado_ == Estado::EJECUTANDO) {
      const Trayectoria & tr = *actual_;
      if (tick_ >= tr.q.size()) {
        hold_q_ = tr.q.back();
        estado("fin " + nombre_actual_);
        RCLCPP_INFO(get_logger(), "Fin de %s", nombre_actual_.c_str());
        estado_ = Estado::PAUSA;
        pausa_left_ = static_cast<int>(std::lround(pausa_ / ts_));
        mantener(q);
        return;
      }
      const double err = miau::maxAbs(miau::sub(tr.q[tick_], q));
      if (err > err_max_) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
          "%s: el robot se alejo %.3f rad de la referencia. Deteniendo.",
          nombre_actual_.c_str(), err);
        abortar(q, buf);
        return;
      }
      comandar(tr.q[tick_], tr.qd[tick_], q);
      ++tick_;
    }
  }

  std::unique_ptr<miau::Realimentacion> fb_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_estado_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> juntas_;
  std::map<std::string, Trayectoria> trayectorias_;
  std::map<std::string, Vec3> puntos_;
  std::deque<std::string> cola_;
  const Trayectoria * actual_{nullptr};
  std::string nombre_actual_, punto_inicial_;

  Vec3 hold_q_{}, q_home0_{}, q_home1_{}, dq_home_{};
  Estado estado_{Estado::ESPERA};
  size_t tick_{0};
  int pausa_left_{0};
  double t_{0.0}, T_home_{1.0};
  double ts_{0.1}, kp_{2.0}, qd_max_{1.0}, acc_max_{2.0}, home_vel_{0.5}, pausa_{0.5};
  double tol_inicio_{0.1}, err_max_{0.3};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  try {
    auto nodo = std::make_shared<NodoCinematico>(options);
    rclcpp::spin(nodo);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("nodo_cinematico"), "Error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
