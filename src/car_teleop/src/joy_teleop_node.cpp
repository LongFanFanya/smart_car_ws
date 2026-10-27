#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
  stop_requested = 1;
}
}

class JoyTeleopNode : public rclcpp::Node
{
public:
  using Clock = std::chrono::steady_clock;

  JoyTeleopNode()
  : Node("joy_teleop_node")
  {
    // game_controller_node 标准映射。
    axis_speed_ = declare_parameter<int>(
      "axis_speed", 1);

    axis_steering_ = declare_parameter<int>(
      "axis_steering", 0);

    enable_button_ = declare_parameter<int>(
      "enable_button", 9);

    stop_button_ = declare_parameter<int>(
      "stop_button", 1);

    max_speed_ = declare_parameter<double>(
      "max_speed", 0.28);

    max_alpha_ = declare_parameter<double>(
      "max_alpha", 0.32);

    // 摇杆中心死区，过滤轻微漂移。
    deadzone_ = declare_parameter<double>(
      "deadzone", 0.10);

    // 超过该时间没有收到有效 /joy，停车。
    joy_timeout_ = declare_parameter<double>(
      "joy_timeout", 0.30);

    // 方向反了时，通过参数改成 -1.0。
    speed_sign_ = declare_parameter<double>(
      "speed_sign", 1.0);

    // 默认保持刚才键盘版的转向映射：
    // 摇杆向左产生正 alpha_stm32，
    // 前进时换算为负 ROS wz。
    steering_sign_ = declare_parameter<double>(
      "steering_sign", 1.0);

    validateParameters();

    publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(1));

    // 只保留最新的手柄状态。
    auto joy_qos = rclcpp::SensorDataQoS();
    joy_qos.keep_last(1);

    subscription_ =
      create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        joy_qos,
        [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
          joyCallback(*msg);
        });

    timer_ = create_wall_timer(
      50ms,
      [this]() {
        timerCallback();
      });

    RCLCPP_INFO(
      get_logger(),
      "Joy teleop ready: speed_axis=%d, steering_axis=%d, "
      "enable_button=%d, stop_button=%d",
      axis_speed_,
      axis_steering_,
      enable_button_,
      stop_button_);

    RCLCPP_INFO(
      get_logger(),
      "max_speed=%.3f m/s, max_alpha=%.3f rad, "
      "joy_timeout=%.2f s",
      max_speed_,
      max_alpha_,
      joy_timeout_);

    RCLCPP_INFO(
      get_logger(),
      "Release LB once, then hold LB to drive. "
      "Do not run keyboard teleop at the same time.");
  }

  void stopAndFlush()
  {
    timer_->cancel();
    disarm();

    // 在 ROS shutdown 前发送停车指令。
    for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
      publishCommand();
      std::this_thread::sleep_for(20ms);
    }
  }

private:
  void validateParameters()
  {
    for (double value : {
        max_speed_, max_alpha_,
        deadzone_, joy_timeout_,
        speed_sign_, steering_sign_})
    {
      if (!std::isfinite(value)) {
        throw std::invalid_argument(
          "All numeric parameters must be finite");
      }
    }

    if (max_speed_ <= 0.0 ||
        max_alpha_ <= 0.0 ||
        max_alpha_ > 0.32 ||
        deadzone_ < 0.0 ||
        deadzone_ >= 1.0 ||
        joy_timeout_ < 0.1)
    {
      throw std::invalid_argument(
        "Invalid limits: max_speed > 0; "
        "0 < max_alpha <= 0.32; "
        "0 <= deadzone < 1; joy_timeout >= 0.1");
    }

    if ((speed_sign_ != 1.0 && speed_sign_ != -1.0) ||
        (steering_sign_ != 1.0 && steering_sign_ != -1.0))
    {
      throw std::invalid_argument(
        "speed_sign and steering_sign must be +1.0 or -1.0");
    }

    if (axis_speed_ < 0 ||
        axis_steering_ < 0 ||
        axis_speed_ == axis_steering_ ||
        enable_button_ < 0 ||
        stop_button_ < -1 ||
        stop_button_ == enable_button_)
    {
      throw std::invalid_argument(
        "Invalid axis/button indices; "
        "stop_button=-1 disables the stop button");
    }
  }

  bool validMessage(const sensor_msgs::msg::Joy & msg) const
  {
    if (static_cast<std::size_t>(axis_speed_) >=
          msg.axes.size() ||
        static_cast<std::size_t>(axis_steering_) >=
          msg.axes.size() ||
        static_cast<std::size_t>(enable_button_) >=
          msg.buttons.size())
    {
      return false;
    }

    if (stop_button_ >= 0 &&
        static_cast<std::size_t>(stop_button_) >=
          msg.buttons.size())
    {
      return false;
    }

    return
      std::isfinite(msg.axes[axis_speed_]) &&
      std::isfinite(msg.axes[axis_steering_]);
  }

  double applyDeadzone(double value) const
  {
    value = std::clamp(value, -1.0, 1.0);

    const double magnitude = std::abs(value);

    if (magnitude <= deadzone_) {
      return 0.0;
    }

    // 将死区之外的范围重新映射到 [0, 1]。
    const double scaled =
      (magnitude - deadzone_) / (1.0 - deadzone_);

    return std::copysign(scaled, value);
  }

  void disarm()
  {
    enabled_ = false;

    // 启动、超时、异常或按 B 后，
    // 必须先松开使能键，再重新按住。
    need_release_ = true;

    vx_ = 0.0;
    alpha_ = 0.0;
  }

  void joyCallback(const sensor_msgs::msg::Joy & msg)
  {
    if (stop_requested) {
      disarm();
      return;
    }

    if (!validMessage(msg)) {
      have_joy_ = false;
      disarm();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Invalid /joy message: axes=%zu, buttons=%zu. "
        "Check axis/button parameters.",
        msg.axes.size(),
        msg.buttons.size());

      return;
    }

    const auto now = Clock::now();

    // 即使定时器尚未执行，也检查接收间隔。
    if (have_joy_) {
      const double gap =
        std::chrono::duration<double>(
          now - last_joy_time_).count();

      if (gap > joy_timeout_) {
        disarm();
      }
    }

    last_joy_time_ = now;
    have_joy_ = true;

    // B 按钮停车。
    if (stop_button_ >= 0 &&
        msg.buttons[stop_button_] != 0)
    {
      disarm();
      return;
    }

    const bool held =
      msg.buttons[enable_button_] == 1;

    // 松开 LB：停车，并允许下一次按下使能。
    if (!held) {
      enabled_ = false;
      need_release_ = false;

      vx_ = 0.0;
      alpha_ = 0.0;
      return;
    }

    // 防止启动或断连恢复时，
    // 因使能键一直按住而自动恢复运动。
    if (need_release_) {
      enabled_ = false;
      vx_ = 0.0;
      alpha_ = 0.0;
      return;
    }

    enabled_ = true;

    const double speed_input =
      applyDeadzone(
        speed_sign_ * msg.axes[axis_speed_]);

    const double steering_input =
      applyDeadzone(
        steering_sign_ * msg.axes[axis_steering_]);

    // 速度和转角独立设置。
    vx_ = max_speed_ * speed_input;

    // STM32 协议定义的右前轮转角。
    alpha_ = max_alpha_ * steering_input;
  }

  double calculateWz() const
  {
    if (vx_ == 0.0 || alpha_ == 0.0) {
      return 0.0;
    }

    const double tangent = std::tan(alpha_);

    // 与键盘版相同的右前轮转角模型。
    const double wz_stm32 =
      vx_ * tangent /
      (wheelbase_ + 0.5 * track_width_ * tangent);

    // STM32 与 ROS 的角速度方向转换。
    return -wz_stm32;
  }

  void publishCommand()
  {
    geometry_msgs::msg::Twist command;

    if (enabled_) {
      command.linear.x = vx_;
      command.angular.z = calculateWz();
    }

    // 未使能时发送默认的全零消息。
    publisher_->publish(command);
  }

  void timerCallback()
  {
    if (stop_requested) {
      disarm();
    }

    if (have_joy_) {
      const double elapsed =
        std::chrono::duration<double>(
          Clock::now() - last_joy_time_).count();

      if (elapsed > joy_timeout_) {
        have_joy_ = false;
        disarm();

        RCLCPP_WARN(
          get_logger(),
          "/joy timeout: stopped. "
          "Release and press LB again after reconnecting.");
      }
    }

    // 每 50 ms 发布，即 20 Hz。
    publishCommand();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "JOY: enabled=%s, vx=%+.3f m/s, "
      "alpha_stm32=%+.3f rad, wz_ros=%+.3f rad/s",
      enabled_ ? "yes" : "no",
      vx_,
      alpha_,
      enabled_ ? calculateWz() : 0.0);
  }

  static constexpr double wheelbase_ = 0.144;
  static constexpr double track_width_ = 0.162;

  int axis_speed_{1};
  int axis_steering_{0};

  int enable_button_{9};
  int stop_button_{1};

  double max_speed_{0.28};
  double max_alpha_{0.32};

  double deadzone_{0.10};
  double joy_timeout_{0.30};

  double speed_sign_{1.0};
  double steering_sign_{1.0};

  double vx_{0.0};
  double alpha_{0.0};

  bool enabled_{false};
  bool need_release_{true};
  bool have_joy_{false};

  Clock::time_point last_joy_time_{};

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::Subscription<
    sensor_msgs::msg::Joy>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(
    argc,
    argv,
    rclcpp::InitOptions(),
    rclcpp::SignalHandlerOptions::None);

  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  std::signal(SIGHUP, requestStop);
  std::signal(SIGTSTP, requestStop);

  int result = 0;

  std::shared_ptr<JoyTeleopNode> node;

  try {
    node = std::make_shared<JoyTeleopNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok() && !stop_requested) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }

    executor.remove_node(node);

  } catch (const std::exception & error) {
    std::cerr
      << "joy_teleop_node error: "
      << error.what()
      << '\n';

    result = 1;
  }

  if (node && rclcpp::ok()) {
    try {
      node->stopAndFlush();
    } catch (const std::exception & error) {
      std::cerr
        << "Stop command failed: "
        << error.what()
        << '\n';

      result = 1;
    }
  }

  node.reset();
  rclcpp::shutdown();

  return result;
}