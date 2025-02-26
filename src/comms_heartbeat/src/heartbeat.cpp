#include <rclcpp/rclcpp.hpp>
#include <comms_interfaces/msg/heartbeat.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define TTL_MSEC 500
#define RATE_MSEC 200
#define SEC_TO_MSEC 1000
#define NSEC_TO_MSEC 0.000001

const auto ttl_duration = std::chrono::milliseconds(RATE_MSEC);

class HeartbeatNode : public rclcpp::Node {
  public:
	HeartbeatNode() : Node("heartbeat_node") {
		this->declare_parameter<std::string>("write");
		this->declare_parameter<std::string>("read");
		this->declare_parameter<std::string>("broadcast", "emergency_broadcast");

		write_ = this->get_parameter("write").as_string();
		read_ = this->get_parameter("read").as_string();
		broadcast_ = this->get_parameter("broadcast").as_string();

		publisher_ = this->create_publisher<comms_interfaces::msg::Heartbeat>(write_, 5);
		subscription_ = this->create_subscription<comms_interfaces::msg::Heartbeat>(
			read_, 5, std::bind(&HeartbeatNode::heartbeat_callback, this, _1)
		);
		broadcast_publisher_ = this->create_publisher<std_msgs::msg::Bool>(broadcast_, 5);

		timer = this->create_wall_timer(ttl_duration, std::bind(&HeartbeatNode::timer_callback, this));
		rclcpp::Time now = this->get_clock()->now();
		last_received_time_.sec = 0;
		last_received_time_.nanosec = 0;
	}

	private:

	void heartbeat_callback(const comms_interfaces::msg::Heartbeat::SharedPtr msg) {
		// RCLCPP_INFO(this->get_logger(), "Received timestamp: sec=%8d, nanosec=%8d", 
		// 			 msg->timestamp.sec, msg->timestamp.nanosec);
		std::lock_guard<std::mutex> lock(time_mutex_);
		last_received_time_.set__sec(msg->timestamp.sec);
		last_received_time_.set__nanosec(msg->timestamp.nanosec);
		// Unlock the mutex
		return;
	}

	void timer_callback() {
		// publish heartbeat
		auto message = comms_interfaces::msg::Heartbeat();
		
		auto now = this->get_clock()->now();
		message.timestamp.sec = now.seconds();
		message.timestamp.nanosec = now.nanoseconds() % 1000000000UL;
		publisher_->publish(message);

		// Check for timeout
		std::lock_guard<std::mutex> lock(time_mutex_);
		auto elapsed = now - last_received_time_;
		auto msecs = elapsed.seconds() * SEC_TO_MSEC 
				+ elapsed.nanoseconds() * NSEC_TO_MSEC;

		if (msecs > TTL_MSEC) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
				"Heartbeat timeout detected! Broadcasting emergency signal. Time since last heartbeat: %.2f ms", msecs);
		}
		auto broadcast_message = std_msgs::msg::Bool();
		broadcast_message.data = msecs > TTL_MSEC;
		broadcast_publisher_->publish(broadcast_message);

		return;
	}

	rclcpp::Publisher<comms_interfaces::msg::Heartbeat>::SharedPtr publisher_;
	rclcpp::Subscription<comms_interfaces::msg::Heartbeat>::SharedPtr subscription_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr broadcast_publisher_;
	rclcpp::TimerBase::SharedPtr timer;

	std::string write_, read_, broadcast_;

	builtin_interfaces::msg::Time last_received_time_;
	std::mutex time_mutex_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HeartbeatNode>());
	rclcpp::shutdown();
	return 0;
}