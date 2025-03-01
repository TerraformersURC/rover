#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <comms_interfaces/msg/motor_control.hpp>
#include <std_msgs/msg/bool.hpp>
#include <deque>

#define MAX_SPEED 2000
#define MIN_SPEED 1000
#define PADDING 100

const int MID_SPEED = (MAX_SPEED + MIN_SPEED) / 2;
const int HALF_RANGE = ((MAX_SPEED - MIN_SPEED) / 2) - PADDING;

const char* SUBSCRIBER_NAME = "motor_data_subscriber";
const char* MOTOR_CONTROL_TOPIC = "motor_control";
const char* STATUS_TOPIC = "connection_status/rover";

using std::placeholders::_1;

int serial_port;

int pwm_range(float ds4_speed){
    float pwm = (ds4_speed * HALF_RANGE) + MID_SPEED;
    return int(pwm);
}

// Construct the ROS2 node
class MotorDataSubscriber : public rclcpp::Node{

    public:
    MotorDataSubscriber(): Node(SUBSCRIBER_NAME){
        subscription_ = this->create_subscription<comms_interfaces::msg::MotorControl>(
            MOTOR_CONTROL_TOPIC, 10, std::bind(&MotorDataSubscriber::motor_callback, this, _1));
        status_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            STATUS_TOPIC, 5, std::bind(&MotorDataSubscriber::status_callback, this, _1));
    }

    private:
    void motor_callback(const comms_interfaces::msg::MotorControl & msg) const{
        // Retrieve each motor's speeds here
        int fl_vel = pwm_range(msg.fl);
        int fr_vel = pwm_range(msg.fr);
        int bl_vel = pwm_range(msg.bl);
        int br_vel = pwm_range(msg.br);

        RCLCPP_INFO(this->get_logger(), "Sending data...");
        RCLCPP_INFO(this->get_logger(), "%04d %04d", fl_vel, fr_vel);
        RCLCPP_INFO(this->get_logger(), "%04d %04d", bl_vel, br_vel);
        
        // int motor_speeds[4] = {fl_vel, fr_vel, bl_vel, br_vel};
        // write(serial_port, motor_speeds, sizeof(motor_speeds));

        char formattedData[50]; // Define a character array to hold the formatted string
        std::sprintf(formattedData, "<%d, %d, %d, %d>", 
            fl_vel, fr_vel, bl_vel, br_vel);
        const char* data = formattedData; // Assign the formatted string to the data variable
        int bytesWritten = write(serial_port, data, strlen(data));
        
        sleep(0.25);
    
        if (bytesWritten == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
            close(serial_port);
            return;
        }

    }

    void status_callback(const std_msgs::msg::Bool::SharedPtr msg) const{
        if (!msg->data)
            return;
        RCLCPP_ERROR(this->get_logger(), "Connection to station lost");
        char formattedData[50];
        std::sprintf(formattedData, "<%d, %d, %d, %d>", 
            MID_SPEED, MID_SPEED, MID_SPEED, MID_SPEED);
        write(serial_port, 
            (char *) formattedData, strlen(formattedData));
        sleep(0.25);
    }

    rclcpp::Subscription<comms_interfaces::msg::MotorControl>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_subscription_;
    std::deque<std::tuple<std::chrono::system_clock::time_point,int>> velocity_buffer_;
};

int main(int argc, char * argv[]) {
    // temporary solution to get arduino port, use udev / linux sysfs in future
    const char* arduino_port_format = "/dev/ttyACM%d";
    int arduino_port_id = 0;
    char port_name[32];
    std::sprintf(port_name, arduino_port_format, arduino_port_id);

    while (arduino_port_id < 100 
            && (serial_port = open(port_name, O_WRONLY | O_NOCTTY)) == -1) {
        arduino_port_id++;
        std::sprintf(port_name, arduino_port_format, arduino_port_id);
    }

    if (serial_port == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
            "could not find arduino port. is it connected?"
        );
        return 2;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
            "error getting serial port attributes.");
        close(serial_port);
        return 2;
    }

    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "error getting serial port attributes");
        close(serial_port);
        return 2;
    }

    // ROS
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDataSubscriber>());
    rclcpp::shutdown();

    close(serial_port);
    return 0;
}