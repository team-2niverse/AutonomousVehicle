#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "planner_manager/mode.hpp"

#define TOGGLE_BTN_IDX 0

class PlannerManagerNode : public rclcpp::Node {
public:
	PlannerManagerNode() : 
		Node("planner_manager_node"),
		mode(Mode::MANUAL), toggle_button_prev(0) {
		mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("vehicle_mode", 10);

		joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
				"joy", 10,
				std::bind(&PlannerManagerNode::joy_callback, this, std::placeholders::_1)
				);
	}

private:
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
		if (msg->buttons.size() <= TOGGLE_BTN_IDX) return;

		if (toggle_button_prev == 0 && msg->buttons[TOGGLE_BTN_IDX] == 1) {
			std_msgs::msg::UInt8 mode_msg;
			if (mode == Mode::MANUAL) {
				mode = Mode::CRUISE;
				mode_msg.data = static_cast<uint8_t>(Mode::CRUISE);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: CRUISE");
			} else {
				mode = Mode::MANUAL;
				mode_msg.data = static_cast<uint8_t>(Mode::MANUAL);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: MANUAL");
			}
		}

		toggle_button_prev = msg->buttons[TOGGLE_BTN_IDX];
	}

	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	Mode mode;
	int toggle_button_prev;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PlannerManagerNode>());
	rclcpp::shutdown();
	return 0;
}

