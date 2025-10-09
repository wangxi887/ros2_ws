#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class SignalSubscriber : public rclcpp::Node {
public:
    SignalSubscriber() : Node("signal_subscriber") {
        sin_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "sin_signal", 10, std::bind(&SignalSubscriber::sinCallback, this, std::placeholders::_1));

        square_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "square_signal", 10, std::bind(&SignalSubscriber::squareCallback, this, std::placeholders::_1));
    }

private:
    void sinCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        last_sin_ = msg->data;
        processSignals();
    }

    void squareCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        last_square_ = msg->data;
        processSignals();
    }

    void processSignals() {
        if (!std::isnan(last_sin_) && !std::isnan(last_square_)) {
            double output = ((last_sin_ >= 0 && last_square_ >= 0) ||
                             (last_sin_ < 0 && last_square_ < 0)) ? last_sin_ : 0.0;
            RCLCPP_INFO(this->get_logger(), "Output: %.3f", output);
        }
    }

    double last_sin_ = NAN;
    double last_square_ = NAN;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sin_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
