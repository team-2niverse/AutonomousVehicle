#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "can_msgs/msg/frame.hpp"

#include "ssv_interfaces/msg/motion_stamped.hpp"
#include "planner_manager/mode.hpp"

class SsvControlNode : public rclcpp::Node {
public:
	SsvControlNode() : 
		Node("ssv_control_node"),
		mode(Mode::MANUAL) {
		control_pub_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 10);
		motion_pub_ = this->create_publisher<ssv_interfaces::msg::MotionStamped>("manual_motion", 10);

		mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
				"vehicle_mode", 10,
				std::bind(&SsvControlNode::mode_callback, this, std::placeholders::_1)
				);
		joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
				"joy", 10,
				std::bind(&SsvControlNode::joy_callback, this, std::placeholders::_1)
				);
		motion_sub_ = this->create_subscription<ssv_interfaces::msg::MotionStamped>(
				"planner/motion", 10,
				std::bind(&SsvControlNode::motion_callback, this, std::placeholders::_1)
				);
	}

private:
	void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
		mode = static_cast<Mode>(msg->data);
	}

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
		if (mode != Mode::MANUAL)
			return;

		if (msg->axes.size() < 4)
			return;

		publish_pwm(msg->axes[1], msg->axes[2]);

		ssv_interfaces::msg::MotionStamped motion_msg;
		motion_msg.motion.speed = msg->axes[1];
		motion_msg.motion.steering = msg->axes[2];
		motion_msg.header.frame_id = "base_link";
		motion_msg.header.stamp = this->now();

		motion_pub_->publish(motion_msg);
	}

	void motion_callback(const ssv_interfaces::msg::MotionStamped::SharedPtr msg) {
		if (mode == Mode::MANUAL)
			return;

		publish_pwm(msg->motion.speed, msg->motion.steering);
	}

	void publish_pwm(float speed, float steering) {
		int speed_scaled = static_cast<int>(255 * speed);
		int diff = static_cast<int>(speed_scaled * steering);

		int pwm_left = speed_scaled - diff;
		int pwm_right = speed_scaled + diff;

		can_msgs::msg::Frame can_msg{};

		if (pwm_left > 255) pwm_left = 255;
		else if (pwm_left < -255) pwm_left = -255;

		if (pwm_left >= 0) {
			can_msg.data[0] = 1;
			can_msg.data[1] = static_cast<uint8_t>(pwm_left);
		} else {
			can_msg.data[0] = 0;
			can_msg.data[1] = static_cast<uint8_t>(-pwm_left);
		}

		if (pwm_right> 255) pwm_right = 255;
		else if (pwm_right < -255) pwm_right = -255;

		if (pwm_right >= 0) {
			can_msg.data[2] = 1;
			can_msg.data[3] = static_cast<uint8_t>(pwm_right);
		} else {
			can_msg.data[2] = 0;
			can_msg.data[3] = static_cast<uint8_t>(-pwm_right);
		}

		can_msg.id = 0x100;
		can_msg.is_rtr = false;
		can_msg.is_extended = false;
		can_msg.is_error = false;
		can_msg.dlc = 4;
		can_msg.header.frame_id = "base_link";
		can_msg.header.stamp = this->now();

		control_pub_->publish(can_msg);
	}

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr control_pub_;
	rclcpp::Publisher<ssv_interfaces::msg::MotionStamped>::SharedPtr motion_pub_;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::Subscription<ssv_interfaces::msg::MotionStamped>::SharedPtr motion_sub_;
	Mode mode;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SsvControlNode>());
	rclcpp::shutdown();
	return 0;
}

