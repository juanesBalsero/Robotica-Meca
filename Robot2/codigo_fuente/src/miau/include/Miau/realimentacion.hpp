#pragma once
#include <algorithm>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "miau/diff_ik_planner.hpp"

namespace miau
{

class Realimentacion
{
public:
  virtual ~Realimentacion() = default;
  virtual bool leer(Vec3 & q) const = 0;
};

class RealimentacionGazebo : public Realimentacion
{
public:
  RealimentacionGazebo(
    rclcpp::Node & nodo, const std::string & topic, const std::vector<std::string> & juntas)
  : juntas_(juntas)
  {
    sub_ = nodo.create_subscription<sensor_msgs::msg::JointState>(
      topic, 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        Vec3 q{};
        for (size_t k = 0; k < 3; ++k) {
          const auto it = std::find(msg->name.begin(), msg->name.end(), juntas_[k]);
          if (it == msg->name.end()) {return;}
          const size_t i = static_cast<size_t>(std::distance(msg->name.begin(), it));
          if (i >= msg->position.size()) {return;}
          q[k] = msg->position[i];
        }
        q_ = q;
        valido_ = true;
      });
  }

  bool leer(Vec3 & q) const override
  {
    if (!valido_) {return false;}
    q = q_;
    return true;
  }

private:
  std::vector<std::string> juntas_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  Vec3 q_{};
  bool valido_{false};
};

class RealimentacionRobot : public Realimentacion
{
public:
  RealimentacionRobot(rclcpp::Node & nodo, const std::string & topic)
  {
    sub_ = nodo.create_subscription<std_msgs::msg::Float64MultiArray>(
      topic, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) {return;}
        q_ = Vec3{msg->data[0], msg->data[1], msg->data[2]};
        valido_ = true;
      });
  }

  bool leer(Vec3 & q) const override
  {
    if (!valido_) {return false;}
    q = q_;
    return true;
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  Vec3 q_{};
  bool valido_{false};
};

inline std::unique_ptr<Realimentacion> crearRealimentacion(
  rclcpp::Node & nodo, const std::string & fuente, const std::string & topic,
  const std::vector<std::string> & juntas)
{
  if (fuente == "gazebo") {
    return std::make_unique<RealimentacionGazebo>(nodo, topic, juntas);
  }
  if (fuente == "robot") {
    return std::make_unique<RealimentacionRobot>(nodo, topic);
  }
  throw std::runtime_error("fuente_realimentacion debe ser 'gazebo' o 'robot'");
}

}  // namespace miau
