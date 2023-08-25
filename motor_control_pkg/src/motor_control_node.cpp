#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/Int32.hpp>
#include <wiringPi.h>
#include <softPwm.h>

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() : Node("motor_control_node") {
        pwmPins_ = {0, 1, 2, 3}; // Replace with actual GPIO pin numbers
        numControllers_ = pwmPins_.size();

        initializePWMControllers();

        motor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/motor_control", 10,
            std::bind(&MotorControlNode::motorSpeedCallback, this, std::placeholders::_1)
        );
    }

private:
    void initializePWMControllers() {
        for (int pin : pwmPins_) {
            softPwmCreate(pin, 0, 100);
        }
    }

    void setSpeed(int controllerIndex, int speedPercent) {
        if (controllerIndex >= 0 && controllerIndex < numControllers_) {
            softPwmWrite(pwmPins_[controllerIndex], speedPercent);
        }
    }

    void motorSpeedCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Assuming msg->data is the speed percentage (0-100)
        // Here, we map the range to 0-100 since softPwmWrite uses 0-100 range
        int speed = msg->data;
        for (int i = 0; i < numControllers_; ++i) {
            setSpeed(i, speed);
        }
    }

    std::vector<int> pwmPins_;
    size_t numControllers_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motor_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
