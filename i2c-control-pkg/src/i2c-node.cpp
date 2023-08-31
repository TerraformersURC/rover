#include <wiringPiI2C.h>
// Other necessary includes

class I2CCommunicatorNode : public rclcpp::Node
{
public:
    I2CCommunicatorNode()
        : Node("i2c_communicator_node")
    {
        i2c_fd = wiringPiI2CSetup(ARDUINO_ADDRESS); // ARDUINO_ADDRESS is the I2C address of the Arduino

        // Initialize publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Set up a timer to publish Twist messages
        auto publish_timer_callback = std::bind(&I2CCommunicatorNode::publishTwistMessage, this);
        publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), publish_timer_callback);
    }

private:
    void publishTwistMessage()
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        // Set linear x value based on your requirements
        // ...

        publisher_->publish(std::move(twist_msg));

        // Send linear x value as an integer via I2C
        int linear_x_int = static_cast<int>(twist_msg->linear.x * 1000);  // Convert to milli-units
        wiringPiI2CWriteReg16(i2c_fd, LINEAR_X_REGISTER, linear_x_int);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    int i2c_fd;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2CCommunicatorNode>());
    rclcpp::shutdown();
    return 0;
}
