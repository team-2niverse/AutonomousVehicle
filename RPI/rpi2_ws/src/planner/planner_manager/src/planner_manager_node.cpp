#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"

#include "planner_manager/mode.hpp"

#define TOGGLE_BTN_IDX 0

class PlannerManagerNode : public rclcpp::Node {
public:
	PlannerManagerNode() : 
		Node("planner_manager_node"),
		mode(Mode::MANUAL), button_prev{0, } {

		zero_cnt_ = 0;
		signs_flag = false;

		mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("vehicle_mode", 10);

		// controller 입력
		joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
				"joy", 10,
				std::bind(&PlannerManagerNode::joy_callback, this, std::placeholders::_1)
				);
		// DODGE, SWITCH, TURN 모드 종료
		mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
				"return_mode", 10,
				std::bind(&PlannerManagerNode::mode_callback, this, std::placeholders::_1)
				);
		// SWITCH 모드로 변경 (표지판 감지)
		change_sub_ = this->create_subscription<std_msgs::msg::Empty>(
				"changeFlag", 10,
				std::bind(&PlannerManagerNode::change_callback, this, std::placeholders::_1)
				);
		// TURN 모드로 변경 (표지판 감지)
		turn_sub_ = this->create_subscription<std_msgs::msg::Empty>(
				"turnFlag", 10,
				std::bind(&PlannerManagerNode::turn_callback, this, std::placeholders::_1)
				);
		// speed count → 0이 3초 이상 지속되면 DODGE 모드로 변경 (장애물 회피)
		speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/planner/speed", 10,
            std::bind(&PlannerManagerNode::speed_callback, this, std::placeholders::_1)
        );
	}

private:
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
		if (msg->buttons.size() <= TOGGLE_BTN_IDX) return;

		if (button_prev[0] == 0 && msg->buttons[TOGGLE_BTN_IDX] == 1) {
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
		} else if (button_prev[1] == 0 && msg->buttons[1] == 1) {
			std_msgs::msg::UInt8 mode_msg;
			if (mode == Mode::MANUAL) {
				mode = Mode::DODGE;
				mode_msg.data = static_cast<uint8_t>(Mode::DODGE);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: DODGE");
			} else {
				mode = Mode::MANUAL;
				mode_msg.data = static_cast<uint8_t>(Mode::MANUAL);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: MANUAL");
			}
		} else if (button_prev[2] == 0 && msg->buttons[2] == 1) {
			std_msgs::msg::UInt8 mode_msg;
			if (mode == Mode::MANUAL) {
				mode = Mode::SWITCH;
				mode_msg.data = static_cast<uint8_t>(Mode::SWITCH);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: SWITCH");
			} else {
				mode = Mode::MANUAL;
				mode_msg.data = static_cast<uint8_t>(Mode::MANUAL);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: MANUAL");
			}
		} else if (button_prev[3] == 0 && msg->buttons[3] == 1) {
			std_msgs::msg::UInt8 mode_msg;
			if (mode == Mode::MANUAL) {
				mode = Mode::TURN;
				mode_msg.data = static_cast<uint8_t>(Mode::TURN);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: TURN");
			} else {
				mode = Mode::MANUAL;
				mode_msg.data = static_cast<uint8_t>(Mode::MANUAL);
				mode_pub_->publish(mode_msg);
				RCLCPP_INFO(this->get_logger(), "Mode Changed: MANUAL");
			}
		}

		button_prev[0] = msg->buttons[0];
		button_prev[1] = msg->buttons[1];
		button_prev[2] = msg->buttons[2];
		button_prev[3] = msg->buttons[3];
	}

	void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
		if (mode == Mode::SWITCH && signs_flag) signs_flag = false;
		// 차선 변경 후 flag 초기화

		mode = static_cast<Mode>(msg->data);
		std_msgs::msg::UInt8 mode_msg;

		// DODGE 종료 후 SWITCH 발행 + 표지판을 감지하지 못한 경우 (표지판을 감지하지 못한 경우 무조건 CRUISE로 복귀)
		if (!signs_flag){
			mode = Mode::CRUISE;
			mode_msg.data = static_cast<uint8_t>(Mode::CRUISE);
			mode_pub_->publish(mode_msg);
			RCLCPP_INFO(this->get_logger(), "Mode Changed: CRUISE");
		}
		else{
			mode_msg.data = msg->data;
			mode_pub_->publish(mode_msg);
			RCLCPP_INFO(this->get_logger(), "Mode Changed: %d", msg->data);
		}
	}

	void change_callback(const std_msgs::msg::Empty::SharedPtr msg) {
//		mode = Mode::SWITCH;
//		std_msgs::msg::UInt8 mode_msg;
//		mode_msg.data = static_cast<uint8_t>(Mode::SWITCH);
//		mode_pub_->publish(mode_msg);
//		RCLCPP_INFO(this->get_logger(), "Mode Changed: SWITCH");

		signs_flag = true;
	}

	void turn_callback(const std_msgs::msg::Empty::SharedPtr msg) {
		mode = Mode::TURN;
		std_msgs::msg::UInt8 mode_msg;
		mode_msg.data = static_cast<uint8_t>(Mode::TURN);
		mode_pub_->publish(mode_msg);
		RCLCPP_INFO(this->get_logger(), "Mode Changed: TURN");
	}

	void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
		float speed_now = msg->data;

		// ACC에서만 count
		if (mode != Mode::CRUISE){
			zero_cnt_ = 0;
			return;
		}

		if (speed_now < 0.01f) zero_cnt_++;
		else zero_cnt_ = 0;

		// ACC에서 3초 동안 정지할 경우 모드 변경 → DODGE
		if (zero_cnt_ == 30){
			mode = Mode::DODGE;
			std_msgs::msg::UInt8 mode_msg;
			mode_msg.data = static_cast<uint8_t>(Mode::DODGE);
			mode_pub_->publish(mode_msg);
			RCLCPP_INFO(this->get_logger(), "Mode Changed: DODGE");
		}
	}


	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr change_sub_;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr turn_sub_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;

	Mode mode;
	int zero_cnt_;
	int button_prev[4];
	bool signs_flag;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PlannerManagerNode>());
	rclcpp::shutdown();
	return 0;
}

