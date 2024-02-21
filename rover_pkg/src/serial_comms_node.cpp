#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

int serialPort;

// Construct the ROS2 node
class HelloWorldPublisher : public rclcpp::Node {
public:
    HelloWorldPublisher(): Node("hello_world_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&HelloWorldPublisher::timer_callback, this));   
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        char formattedData[50]; // Define a character array to hold the formatted string
        
        // note that this line here is for sending a string
        std::sprintf(formattedData, "<%s>", message.data.c_str());

        // while this one can be used for sending some formatted integers, as an example
        //std::sprintf(formattedData, "<%d, %d, %d, %d>", aVel, bVel, cVel, dVel);

        const char* data = formattedData; // Assign the formatted string to the data variable
        int bytesWritten = write(serialPort, data, strlen(data));
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char * argv[]) {
    // ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorldPublisher>();

    // Arduino
    const char* portName = "/dev/ttyACM0"; // Replace with the appropriate serial port
    serialPort = open(portName, O_WRONLY | O_NOCTTY);

    if (serialPort == -1) {
        RCLCPP_ERROR(node->get_logger(), "Error opening serial port");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serialPort, &tty) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Error getting serial port attributes");
        close(serialPort);
        return 1;
    }

    cfsetospeed(&tty, B115200); // Set baud rate to match your Arduino configuration
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Error setting serial port attributes");
        close(serialPort);
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    close(serialPort);
    RCLCPP_INFO(node->get_logger(), "Node exiting gracefully!");
    return 0;
}
