#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cerrno>
#include <cstring>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// ============================================================
// 退出信号
// ============================================================

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
  stop_requested = 1;
}
}

// ============================================================
// 终端管理
// ============================================================

class Terminal
{
public:
  Terminal()
  {
    if (!::isatty(STDIN_FILENO) ||
        ::tcgetattr(STDIN_FILENO, &saved_) < 0)
    {
      throw std::runtime_error(
        "Please run this node directly in an interactive terminal");
    }

    termios mode = saved_;

    // 关闭整行输入和回显，保留 Ctrl+C 信号。
    mode.c_lflag &= static_cast<tcflag_t>(
      ~(ICANON | ECHO | IEXTEN));

    // 关闭 Ctrl+S / Ctrl+Q 软件流控。
    mode.c_iflag &= static_cast<tcflag_t>(
      ~(IXON | IXOFF));

    mode.c_cc[VMIN] = 0;
    mode.c_cc[VTIME] = 0;

    if (::tcsetattr(STDIN_FILENO, TCSANOW, &mode) < 0) {
      fail("tcsetattr");
    }

    active_ = true;

    // 清除启动前的残留输入。
    ::tcflush(STDIN_FILENO, TCIFLUSH);
  }

  ~Terminal()
  {
    if (active_) {
      ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
  }

  Terminal(const Terminal &) = delete;
  Terminal & operator=(const Terminal &) = delete;

  std::string readKeys()
  {
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};

    const int result = ::poll(&descriptor, 1, 0);

    if (result < 0) {
      if (errno == EINTR) {
        return {};
      }

      fail("poll");
    }

    if (result == 0) {
      return {};
    }

    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error(
        "Keyboard terminal disconnected");
    }

    if (!(descriptor.revents & POLLIN)) {
      return {};
    }

    char buffer[128];

    const auto count =
      ::read(STDIN_FILENO, buffer, sizeof(buffer));

    if (count < 0) {
      if (errno == EINTR ||
          errno == EAGAIN ||
          errno == EWOULDBLOCK)
      {
        return {};
      }

      fail("read");
    }

    if (count == 0) {
      throw std::runtime_error(
        "Keyboard terminal closed");
    }

    return std::string(
      buffer, static_cast<std::size_t>(count));
  }

private:
  [[noreturn]] static void fail(const char * operation)
  {
    throw std::runtime_error(
      std::string(operation) + ": " + std::strerror(errno));
  }

  termios saved_{};
  bool active_{false};
};

// ============================================================
// 按键处理结果
// ============================================================

enum class KeyResult
{
  ignored,
  changed,
  stop,
  quit
};

// ============================================================
// 键盘控制逻辑
//
// 保存：
//   vx：目标线速度，m/s
//   alpha：STM32 协议定义的右前轮转角，rad
//
// 发布前计算：
//   wz_stm32 = vx * tan(alpha)
//              / (L + B/2 * tan(alpha))
//
//   wz_ros = -wz_stm32
// ============================================================

class KeyboardLogic
{
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;

  KeyboardLogic(
    double max_speed,
    double speed_step,
    double max_alpha,
    double alpha_step,
    double input_timeout)
  : max_speed_(max_speed),
    speed_step_(speed_step),
    max_alpha_(max_alpha),
    alpha_step_(alpha_step),
    input_timeout_(input_timeout)
  {
    for (double value : {
        max_speed, speed_step,
        max_alpha, alpha_step, input_timeout})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(
          "All parameters must be finite and positive");
      }
    }

    if (speed_step > max_speed ||
        alpha_step > max_alpha ||
        max_alpha > 0.32 ||
        input_timeout < 0.2)
    {
      throw std::invalid_argument(
        "Steps must not exceed limits; "
        "max_alpha <= 0.32; input_timeout >= 0.2");
    }
  }

  KeyResult processKey(unsigned char key, Time now)
  {
    // 停车、退出优先处理。
    if (key == ' ' || key == 'x' || key == 'X' ||
        key == 'q' || key == 'Q' ||
        key == 0x03 || key == 0x04)
    {
      stop();
      escape_state_ = 0;

      return (key == ' ' || key == 'x' || key == 'X')
        ? KeyResult::stop
        : KeyResult::quit;
    }

    // 忽略方向键等终端转义序列。
    if (escape_state_ == 1) {
      escape_state_ =
        key == '[' ? 2 : (key == 'O' ? 3 : 0);

      return KeyResult::ignored;
    }

    if (escape_state_ == 2) {
      if (key >= 0x40 && key <= 0x7e) {
        escape_state_ = 0;
      }

      return KeyResult::ignored;
    }

    if (escape_state_ == 3) {
      escape_state_ = 0;
      return KeyResult::ignored;
    }

    if (key == 0x1b) {
      escape_state_ = 1;
      return KeyResult::ignored;
    }

    // 大小写均可。
    if (key >= 'A' && key <= 'Z') {
      key = static_cast<unsigned char>(
        key - 'A' + 'a');
    }

    switch (key) {
      // W/S 只改变速度，不改变目标转角。
      case 'w':
        vx_ = std::clamp(
          vx_ + speed_step_,
          -max_speed_,
          max_speed_);
        break;

      case 's':
        vx_ = std::clamp(
          vx_ - speed_step_,
          -max_speed_,
          max_speed_);
        break;

      // 保留之前实测后的 A/D 映射：
      // 前进时，A 对应负 ROS wz，D 对应正 ROS wz。
      //
      // alpha 使用 STM32 协议的符号，
      // 因而 A 增加 alpha，D 减小 alpha。
      case 'a':
        alpha_ = std::clamp(
          alpha_ + alpha_step_,
          -max_alpha_,
          max_alpha_);
        break;

      case 'd':
        alpha_ = std::clamp(
          alpha_ - alpha_step_,
          -max_alpha_,
          max_alpha_);
        break;

      // 回正，速度不变。
      case 'r':
        alpha_ = 0.0;
        break;

      default:
        return KeyResult::ignored;
    }

    // 清除浮点加减产生的极小残差。
    if (std::abs(vx_) < 1e-9) {
      vx_ = 0.0;
    }

    if (std::abs(alpha_) < 1e-9) {
      alpha_ = 0.0;
    }

    last_input_ = now;
    have_input_ = true;

    return KeyResult::changed;
  }

  bool checkTimeout(Time now)
  {
    if (!have_input_) {
      return false;
    }

    const double elapsed =
      std::chrono::duration<double>(
        now - last_input_).count();

    if (elapsed < input_timeout_) {
      return false;
    }

    const bool changed =
      vx_ != 0.0 || alpha_ != 0.0;

    stop();

    return changed;
  }

  void stop()
  {
    vx_ = 0.0;
    alpha_ = 0.0;
    have_input_ = false;
  }

  double vx() const
  {
    return vx_;
  }

  double alpha() const
  {
    return alpha_;
  }

  double wz() const
  {
    if (vx_ == 0.0 || alpha_ == 0.0) {
      return 0.0;
    }

    const double tangent = std::tan(alpha_);

    const double wz_stm32 =
      vx_ * tangent /
      (wheelbase_ + 0.5 * track_width_ * tangent);

    // STM32 右转为正，ROS 左转为正。
    return -wz_stm32;
  }

private:
  // 底盘尺寸，单位 m。
  static constexpr double wheelbase_ = 0.144;
  static constexpr double track_width_ = 0.162;

  double max_speed_;
  double speed_step_;

  double max_alpha_;
  double alpha_step_;

  double input_timeout_;

  double vx_{0.0};
  double alpha_{0.0};

  Time last_input_{};
  bool have_input_{false};

  int escape_state_{0};
};

// ============================================================
// ROS 2 键盘节点
// ============================================================

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  explicit KeyboardTeleopNode(Terminal & terminal)
  : Node("keyboard_teleop_node"),
    terminal_(terminal),
    logic_(
      declare_parameter<double>("max_speed", 0.28),
      declare_parameter<double>("speed_step", 0.05),
      declare_parameter<double>("max_alpha", 0.32),
      declare_parameter<double>("alpha_step", 0.04),
      declare_parameter<double>("input_timeout", 2.0))
  {
    publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(1));

    // 每 50 ms 发布一次，即 20 Hz。
    timer_ = create_wall_timer(
      50ms,
      [this]() {
        timerCallback();
      });

    std::cout
      << "\n========== Keyboard Teleop ==========\n"
      << "W: increase vx | S: decrease vx\n"
      << "A: increase alpha | D: decrease alpha\n"
      << "R: reset alpha to zero, keep vx\n"
      << "SPACE / X: stop and reset alpha\n"
      << "Q / Ctrl+C: stop and quit\n"
      << "------------------------------------\n"
      << "max_speed: "
      << get_parameter("max_speed").as_double()
      << " m/s\n"
      << "speed_step: "
      << get_parameter("speed_step").as_double()
      << " m/s\n"
      << "max_alpha: "
      << get_parameter("max_alpha").as_double()
      << " rad\n"
      << "alpha_step: "
      << get_parameter("alpha_step").as_double()
      << " rad\n"
      << "input_timeout: "
      << get_parameter("input_timeout").as_double()
      << " s\n"
      << "------------------------------------\n"
      << "alpha: right-front angle in STM32 convention.\n"
      << "wz: calculated ROS angular velocity.\n"
      << "Releasing a key does NOT immediately stop.\n"
      << "Keep this terminal focused.\n"
      << "Publishing /cmd_vel at 20 Hz.\n"
      << "====================================\n\n"
      << std::flush;

    showState("READY");
  }

  bool finished() const
  {
    return finished_;
  }

  void stopAndFlush()
  {
    timer_->cancel();
    logic_.stop();

    // 在关闭 ROS 前发送停车指令。
    for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
      publishCommand();
      std::this_thread::sleep_for(20ms);
    }

    showState("STOP / EXIT");
  }

private:
  void timerCallback()
  {
    if (stop_requested || finished_) {
      logic_.stop();
      publishCommand();
      return;
    }

    const auto now = KeyboardLogic::Clock::now();

    // 先清除过期状态，再处理新按键。
    const bool expired = logic_.checkTimeout(now);

    bool changed = false;
    bool immediate_stop = false;

    const std::string keys = terminal_.readKeys();

    for (const unsigned char key : keys) {
      const auto action =
        logic_.processKey(key, now);

      if (action == KeyResult::quit) {
        finished_ = true;
        break;
      }

      if (action == KeyResult::stop) {
        immediate_stop = true;
        break;
      }

      changed =
        changed || action == KeyResult::changed;
    }

    publishCommand();

    if (immediate_stop) {
      showState("STOP");
    } else if (changed) {
      showState("CMD");
    } else if (expired) {
      showState("INPUT TIMEOUT / STOP");
    }
  }

  void publishCommand()
  {
    geometry_msgs::msg::Twist command;

    command.linear.x = logic_.vx();

    // 每次根据当前速度和目标转角重新计算角速度。
    command.angular.z = logic_.wz();

    publisher_->publish(command);
  }

  void showState(const char * reason)
  {
    RCLCPP_INFO(
      get_logger(),
      "%s: vx=%+.3f m/s, alpha_stm32=%+.3f rad, "
      "wz_ros=%+.3f rad/s",
      reason,
      logic_.vx(),
      logic_.alpha(),
      logic_.wz());
  }

  Terminal & terminal_;
  KeyboardLogic logic_;

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool finished_{false};
};

// ============================================================
// main
// ============================================================

int main(int argc, char ** argv)
{
  // 自行处理退出信号，先停车，再关闭 ROS。
  rclcpp::init(
    argc,
    argv,
    rclcpp::InitOptions(),
    rclcpp::SignalHandlerOptions::None);

  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  std::signal(SIGHUP, requestStop);

  // Ctrl+Z 改为停车退出。
  std::signal(SIGTSTP, requestStop);

  int result = 0;

  std::shared_ptr<KeyboardTeleopNode> node;

  try {
    Terminal terminal;

    try {
      node =
        std::make_shared<KeyboardTeleopNode>(terminal);

      rclcpp::executors::SingleThreadedExecutor executor;

      executor.add_node(node);

      while (rclcpp::ok() &&
             !stop_requested &&
             !node->finished())
      {
        executor.spin_some();

        std::this_thread::sleep_for(5ms);
      }

      node->stopAndFlush();

      executor.remove_node(node);
      node.reset();

    } catch (...) {
      if (node && rclcpp::ok()) {
        try {
          node->stopAndFlush();
        } catch (...) {
        }
      }

      node.reset();
      throw;
    }

  } catch (const std::exception & error) {
    std::cerr
      << "keyboard_teleop_node error: "
      << error.what()
      << '\n';

    result = 1;
  }

  rclcpp::shutdown();

  return result;
}