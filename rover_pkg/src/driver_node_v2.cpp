#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rover_interfaces/msg/motor_control.hpp>

using std::placeholders::_1;

int serialPort;

int pwmRange(float ds4_speed){
    float pwm = (((ds4_speed+100) * (1900-1100)/200)+1100);
    return int(pwm);
}

// Construct the ROS2 node
class MotorDataSubscriber : public rclcpp::Node{

    public:
    MotorDataSubscriber(): Node("motor_data_subscriber"){
        subscription_ = this->create_subscription<rover_interfaces::msg::MotorControl>(
            "/motor_control", 10, std::bind(&MotorDataSubscriber::motor_callback, this, _1));
    }

    private:
    void motor_callback(const rover_interfaces::msg::MotorControl & msg) const{
        // Retrieve each motor's speeds here
        int aVel = pwmRange(msg.fl);
        int bVel = pwmRange(msg.fr);
        int cVel = pwmRange(msg.bl);
        int dVel = pwmRange(msg.br);

        RCLCPP_INFO(this->get_logger(), "Sending data...");
        RCLCPP_INFO(this->get_logger(), "MotorA: %d", aVel);
        RCLCPP_INFO(this->get_logger(), "MotorB: %d", bVel);
        RCLCPP_INFO(this->get_logger(), "MotorC: %d", cVel);
        RCLCPP_INFO(this->get_logger(), "MotorD: %d", dVel);
        
        char formattedData[50]; // Define a character array to hold the formatted string
        std::sprintf(formattedData, "<%d, %d, %d, %d>", aVel, bVel, cVel, dVel);
        const char* data = formattedData; // Assign the formatted string to the data variable
        int bytesWritten = write(serialPort, data, strlen(data));
        sleep(0.75);
    
        // if (bytesWritten == -1) {
            // RCLCPP_ERROR(this->get_logger(), "Error writing to serial port")
            // close(serialPort);
            // return 1;
        // }

    }
    rclcpp::Subscription<rover_interfaces::msg::MotorControl>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    // Arduino
    const char* portName = "/dev/ttyACM0"; // Replace with the appropriate serial port
    serialPort = open(portName, O_WRONLY | O_NOCTTY);

    if (serialPort == -1) {
        // RCLCPP_ERROR(rclcpp::get_logger(), "Error opening serial port");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serialPort, &tty) != 0) {
        // RCLCPP_ERROR(rclcpp::get_logger(), "Error getting serial port attributes");
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
        // RCLCPP_ERROR(rclcpp::get_logger(), "Error getting serial port attributes");
        close(serialPort);
        return 1;
    }

    // ROS
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDataSubscriber>());
    rclcpp::shutdown();

    close(serialPort);
    // RCLCPP_INFO(rclcpp::get_logger(), "Data sent successfully!");
    return 0;
}