#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class SignalPublisher : public rclcpp::Node {
public:
    SignalPublisher() : Node("signal_publisher"), count_(0) {
        sin_pub_ = this->create_publisher<std_msgs::msg::Float64>("sin_signal", 10);
        square_pub_ = this->create_publisher<std_msgs::msg::Float64>("square_signal", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), // 500Hz
            std::bind(&SignalPublisher::publishSignals, this)
        );
    }

private:
    void publishSignals() {
        auto sin_msg = std_msgs::msg::Float64();
        auto square_msg = std_msgs::msg::Float64();

        double t = count_ * 0.002;  // 时间步长
        sin_msg.data = std::sin(2 * M_PI * 10.0 * t); // 10Hz 正弦
        square_msg.data = (std::fmod(t, 1.0) < 0.5) ? 1.0 : -1.0; // 1Hz 方波

        sin_pub_->publish(sin_msg);
        square_pub_->publish(square_msg);

        // 打印日志
        RCLCPP_INFO(this->get_logger(), "Sin: %.3f, Square: %.3f", sin_msg.data, square_msg.data);

        count_++;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SignalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
