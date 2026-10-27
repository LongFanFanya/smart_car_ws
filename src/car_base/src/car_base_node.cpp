#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2_ros/transform_broadcaster.hpp"

using namespace std::chrono_literals;


// ============================================================
// 1. Linux串口
// ============================================================

class SerialPort
{
public:
    explicit SerialPort(const std::string &port)
    {
        fd_ = ::open(
            port.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

        if (fd_ < 0) {
            throw std::runtime_error(
                "Open serial port failed: " + port +
                ": " + std::strerror(errno));
        }

        termios options{};

        if (::tcgetattr(fd_, &options) != 0) {
            configError("tcgetattr");
        }

        ::cfmakeraw(&options);

        if (::cfsetispeed(&options, B115200) != 0 ||
            ::cfsetospeed(&options, B115200) != 0)
        {
            configError("Set baud rate");
        }

        // 115200、8N1、无流控
        options.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
        options.c_cflag |= CS8 | CLOCAL | CREAD;

        options.c_cflag &= ~CRTSCTS;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
            configError("tcsetattr");
        }
    }

    ~SerialPort()
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    // wz使用固件约定：右转为正
    void sendVelocity(double vx, double wz)
    {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());

        stream << std::fixed << std::setprecision(3)
               << "$WZ," << vx << "," << wz << "#";

        writeAll(stream.str());
    }

    // 没有数据时立即返回0
    std::size_t readSome(char *buffer, std::size_t capacity)
    {
        pollfd descriptor{};
        descriptor.fd = fd_;
        descriptor.events = POLLIN;

        const int result = ::poll(&descriptor, 1, 0);

        if (result < 0) {
            if (errno == EINTR) {
                return 0;
            }

            throw std::runtime_error(
                std::string("Serial read poll failed: ") +
                std::strerror(errno));
        }

        if (result == 0) {
            return 0;
        }

        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            throw std::runtime_error("Serial port disconnected");
        }

        if (!(descriptor.revents & POLLIN)) {
            return 0;
        }

        const ssize_t count = ::read(fd_, buffer, capacity);

        if (count >= 0) {
            return static_cast<std::size_t>(count);
        }

        if (errno == EAGAIN ||
            errno == EWOULDBLOCK ||
            errno == EINTR)
        {
            return 0;
        }

        throw std::runtime_error(
            std::string("Serial read failed: ") +
            std::strerror(errno));
    }

private:
    [[noreturn]] void configError(const std::string &operation)
    {
        const int saved_errno = errno;

        ::close(fd_);
        fd_ = -1;

        throw std::runtime_error(
            operation + " failed: " +
            std::strerror(saved_errno));
    }

    void writeAll(const std::string &frame)
    {
        std::size_t sent = 0;

        const auto deadline =
            std::chrono::steady_clock::now() + 20ms;

        while (sent < frame.size()) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error("Serial write timeout");
            }

            const ssize_t count = ::write(
                fd_,
                frame.data() + sent,
                frame.size() - sent);

            if (count > 0) {
                sent += static_cast<std::size_t>(count);
                continue;
            }

            if (count < 0 && errno == EINTR) {
                continue;
            }

            if (count < 0 &&
                errno != EAGAIN &&
                errno != EWOULDBLOCK)
            {
                throw std::runtime_error(
                    std::string("Serial write failed: ") +
                    std::strerror(errno));
            }

            pollfd descriptor{};
            descriptor.fd = fd_;
            descriptor.events = POLLOUT;

            const int result = ::poll(&descriptor, 1, 5);

            if (result < 0) {
                if (errno == EINTR) {
                    continue;
                }

                throw std::runtime_error(
                    std::string("Serial write poll failed: ") +
                    std::strerror(errno));
            }

            if (result > 0 &&
                (descriptor.revents &
                (POLLERR | POLLHUP | POLLNVAL)))
            {
                throw std::runtime_error("Serial port disconnected");
            }
        }
    }

    int fd_{-1};
};


// ============================================================
// 2. STM32原始状态
// $DATA,VL,VR,VX,WZ,ALPHA#
// WZ和ALPHA仍保持固件的右转为正约定
// ============================================================

struct ChassisState
{
    double vl;
    double vr;
    double vx;
    double wz;
    double alpha;
};


// ============================================================
// 3. DATA组帧与解析
// ============================================================

class DataFrameParser
{
public:
    std::vector<ChassisState> feed(
        const char *data,
        std::size_t size)
    {
        std::vector<ChassisState> frames;

        for (std::size_t i = 0; i < size; ++i) {
            const char ch = data[i];

            if (ch == '$') {
                body_.clear();
                collecting_ = true;
            } else if (!collecting_) {
                continue;
            } else if (ch == '#') {
                ChassisState state{};

                if (parseBody(state)) {
                    frames.push_back(state);
                }

                body_.clear();
                collecting_ = false;
            } else if (ch == '\r' || ch == '\n') {
                continue;
            } else if (body_.size() >= 128) {
                body_.clear();
                collecting_ = false;
            } else {
                body_ += ch;
            }
        }

        return frames;
    }

private:
    bool parseBody(ChassisState &state) const
    {
        if (std::count(body_.begin(), body_.end(), ',') != 5) {
            return false;
        }

        std::istringstream fields(body_);
        std::string token;

        if (!std::getline(fields, token, ',') ||
            token != "DATA")
        {
            return false;
        }

        std::array<double, 5> values{};

        for (double &value : values) {
            if (!std::getline(fields, token, ',')) {
                return false;
            }

            std::istringstream number(token);
            number.imbue(std::locale::classic());

            if (!(number >> value) || !std::isfinite(value)) {
                return false;
            }

            number >> std::ws;

            if (!number.eof()) {
                return false;
            }
        }

        state = {
            values[0],
            values[1],
            values[2],
            values[3],
            values[4]
        };

        return true;
    }

    std::string body_;
    bool collecting_{false};
};


// ============================================================
// 4. 轮式里程计数学模型
//
// ROS约定：
//   x向前、y向左、yaw左转为正
//
// 与你纸上的推导等价：
//   本步局部圆弧位移 -> 按旧航向旋转 -> 累加位置
// ============================================================

class WheelOdometry
{
public:
    void update(double vx, double wz, double dt)
    {
        const double delta_yaw = wz * dt;

        double dx_body;
        double dy_body;

        if (std::abs(delta_yaw) < 1e-6) {
            // 同一圆弧公式的小角度极限
            // wz=0时自然得到：dx=vx*dt，dy=0
            dx_body = vx * dt;
            dy_body = 0.5 * vx * dt * delta_yaw;
        } else {
            const double radius = vx / wz;

            dx_body = radius * std::sin(delta_yaw);

            // 等价于 radius * (1 - cos(delta_yaw))
            // 使用半角形式减小接近零时的相减误差
            const double half_sin =
                std::sin(delta_yaw * 0.5);

            dy_body =
                2.0 * radius * half_sin * half_sin;
        }

        // 必须使用更新前的yaw
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        x += c * dx_body - s * dy_body;
        y += s * dx_body + c * dy_body;

        // 将航向归一化到[-pi, pi]
        yaw = std::atan2(
            std::sin(yaw + delta_yaw),
            std::cos(yaw + delta_yaw));

        // 前进、倒车都增加累计路程
        distance += std::abs(vx) * dt;
    }

    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double distance{0.0};
};


// ============================================================
// 5. ROS 2底盘节点
// ============================================================

class CarBaseNode : public rclcpp::Node
{
public:
    CarBaseNode()
        : Node("car_base_node")
    {
        const std::string port =
            this->declare_parameter<std::string>(
                "serial_port", "/dev/ttyUSB0");

        serial_ = std::make_unique<SerialPort>(port);

        // 启动时先停车
        serial_->sendVelocity(0.0, 0.0);

        cmd_vel_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                rclcpp::QoS(1),
                std::bind(
                    &CarBaseNode::cmdVelCallback,
                    this,
                    std::placeholders::_1));

        velocity_pub_ =
            this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/car/velocity", 10);

        left_wheel_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/car/left_wheel_speed", 10);

        right_wheel_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/car/right_wheel_speed", 10);

        steering_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/car/steering_angle", 10);

        distance_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/car/distance", 10);

        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "/odom", 10);

        // 使用节点引用，适配新的tf2_ros节点接口
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        send_timer_ = this->create_wall_timer(
            50ms,
            std::bind(&CarBaseNode::sendTimerCallback, this));

        receive_timer_ = this->create_wall_timer(
            10ms,
            std::bind(&CarBaseNode::receiveTimerCallback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Started: port=%s, baud=115200, "
            "TX=20 Hz, odom frame=odom, base frame=base_link",
            port.c_str());
    }

    ~CarBaseNode() override
    {
        if (serial_) {
            try {
                serial_->sendVelocity(0.0, 0.0);
            } catch (...) {
                // 析构时不向外抛异常
            }
        }
    }

private:
    // --------------------------------------------------------
    // 上层速度指令
    // --------------------------------------------------------

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double vx_ros = msg->linear.x;
        const double wz_ros = -(msg->angular.z);

        if (!std::isfinite(vx_ros) ||
            !std::isfinite(wz_ros))
        {
            command_valid_ = false;
            target_vx_ = 0.0;
            target_wz_ = 0.0;

            RCLCPP_WARN(
                this->get_logger(),
                "Invalid velocity received; stopping.");

            return;
        }

        // 保留现有联调限速
        target_vx_ = std::clamp(vx_ros, -0.8, 0.8);

        // ROS左转为正 -> 固件右转为正
        target_wz_ = -wz_ros;

        if (std::abs(target_vx_) < 0.001) {
            target_vx_ = 0.0;
            target_wz_ = 0.0;
        } else {
            // 当前固件右前轮±0.32 rad所对应的曲率边界
            const double curvature_min =
                1.0 / (0.144 / std::tan(-0.32) + 0.081);

            const double curvature_max =
                1.0 / (0.144 / std::tan(0.32) + 0.081);

            const double curvature = std::clamp(
                target_wz_ / target_vx_,
                curvature_min,
                curvature_max);

            target_wz_ = curvature * target_vx_;
        }

        last_cmd_time_ = std::chrono::steady_clock::now();
        command_valid_ = true;
    }

    // --------------------------------------------------------
    // 每50 ms下发一次命令
    // --------------------------------------------------------

    void sendTimerCallback()
    {
        double vx = 0.0;
        double wz = 0.0;

        const auto now = std::chrono::steady_clock::now();

        if (command_valid_) {
            if (now - last_cmd_time_ <= 300ms) {
                vx = target_vx_;
                wz = target_wz_;
            } else {
                command_valid_ = false;

                RCLCPP_WARN(
                    this->get_logger(),
                    "/cmd_vel timeout; sending zero velocity.");
            }
        }

        serial_->sendVelocity(vx, wz);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "TX: vx=%.3f, wz_stm32=%.3f",
            vx,
            wz);
    }

    // --------------------------------------------------------
    // 每10 ms检查串口
    //
    // 当前协议没有采样时间戳。
    // 同一批收到多帧时取最新状态，使用实际接收间隔积分一次。
    // 避免把逐帧解析的微小时间间隔当成采样周期。
    // --------------------------------------------------------

    void receiveTimerCallback()
    {
        char buffer[256];

        ChassisState latest_state{};
        bool received_state = false;

        for (int i = 0; i < 8; ++i) {
            const std::size_t count =
                serial_->readSome(buffer, sizeof(buffer));

            if (count == 0) {
                break;
            }

            const auto frames =
                frame_parser_.feed(buffer, count);

            if (!frames.empty()) {
                latest_state = frames.back();
                received_state = true;
            }
        }

        if (received_state) {
            handleState(latest_state);
        }
    }

    // --------------------------------------------------------
    // 有效反馈到达时更新里程计
    // --------------------------------------------------------

    void handleState(const ChassisState &state)
    {
        const auto steady_now =
            std::chrono::steady_clock::now();

        const rclcpp::Time stamp = this->now();

        // 使用反馈速度，不使用目标速度
        const double vx_ros = state.vx;
        const double wz_ros = -state.wz;

        if (feedback_initialized_) {
            const double dt =
                std::chrono::duration<double>(
                    steady_now - last_feedback_time_).count();

            if (dt > 0.0 && dt <= 0.3) {
                odometry_.update(vx_ros, wz_ros, dt);
            } else {
                // 反馈长时间中断后，不拿一帧速度补猜整段运动
                RCLCPP_WARN(
                    this->get_logger(),
                    "Feedback gap %.3f s: skipped this integration "
                    "interval; motion during the gap is unknown.",
                    dt);
            }
        } else {
            // 第一帧只建立时间基准，位姿从零开始
            feedback_initialized_ = true;
        }

        last_feedback_time_ = steady_now;

        publishFeedback(state, stamp);
        publishOdometry(vx_ros, wz_ros, stamp);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "ODOM: x=%.3f m, y=%.3f m, yaw=%.3f rad, "
            "distance=%.3f m",
            odometry_.x,
            odometry_.y,
            odometry_.yaw,
            odometry_.distance);
    }

    // --------------------------------------------------------
    // 保留原来的四个反馈话题
    // --------------------------------------------------------

    void publishFeedback(
        const ChassisState &state,
        const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::TwistStamped velocity_msg;

        velocity_msg.header.stamp = stamp;
        velocity_msg.header.frame_id = "base_link";

        velocity_msg.twist.linear.x = state.vx;
        velocity_msg.twist.angular.z = -state.wz;

        velocity_pub_->publish(velocity_msg);

        std_msgs::msg::Float64 left_msg;
        left_msg.data = state.vl;
        left_wheel_pub_->publish(left_msg);

        std_msgs::msg::Float64 right_msg;
        right_msg.data = state.vr;
        right_wheel_pub_->publish(right_msg);

        std_msgs::msg::Float64 steering_msg;

        // 右前轮指令角，转换为左偏为正
        steering_msg.data = -state.alpha;
        steering_pub_->publish(steering_msg);
    }

    // --------------------------------------------------------
    // 发布/odom、TF和累计路程
    // --------------------------------------------------------

    void publishOdometry(
        double vx,
        double wz,
        const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = odometry_.x;
        odom_msg.pose.pose.position.y = odometry_.y;
        odom_msg.pose.pose.position.z = 0.0;

        // 平面yaw转换为四元数
        // roll=pitch=0，所以qx=qy=0
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z =
            std::sin(odometry_.yaw * 0.5);
        odom_msg.pose.pose.orientation.w =
            std::cos(odometry_.yaw * 0.5);

        // 速度在base_link坐标系下表示
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = wz;

        // 协方差顺序：x、y、z、roll、pitch、yaw
        //
        // 以下只是联调占位值，不是实测标定结果，
        // 也没有实现随时间传播的协方差估计。
        odom_msg.pose.covariance[0] = 0.05 * 0.05;
        odom_msg.pose.covariance[7] = 0.05 * 0.05;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = 0.10 * 0.10;

        odom_msg.twist.covariance[0] = 0.02 * 0.02;
        odom_msg.twist.covariance[7] = 0.05 * 0.05;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 0.05 * 0.05;

        odom_pub_->publish(odom_msg);

        // TF与/odom使用相同的时间戳、位置和四元数
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = odometry_.x;
        transform.transform.translation.y = odometry_.y;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation =
            odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);

        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = odometry_.distance;

        distance_pub_->publish(distance_msg);
    }

    // 串口、解析器与数学模型
    std::unique_ptr<SerialPort> serial_;
    DataFrameParser frame_parser_;
    WheelOdometry odometry_;

    // 订阅器
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_sub_;

    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        left_wheel_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        right_wheel_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        steering_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        distance_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    // 定时器
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr receive_timer_;

    // 指令状态：角速度使用固件右转为正约定
    double target_vx_{0.0};
    double target_wz_{0.0};
    bool command_valid_{false};

    std::chrono::steady_clock::time_point last_cmd_time_{};

    // 反馈计时
    bool feedback_initialized_{false};

    std::chrono::steady_clock::time_point last_feedback_time_{};
};


// ============================================================
// 6. 程序入口
// ============================================================

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    int result = 0;

    try {
        auto node = std::make_shared<CarBaseNode>();
        rclcpp::spin(node);
    } catch (const std::exception &error) {
        std::cerr
            << "car_base_node error: "
            << error.what()
            << std::endl;

        result = 1;
    }

    rclcpp::shutdown();

    return result;
}