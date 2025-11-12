#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/joy.hpp"
#include "can_msgs/msg/frame.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

class Tf2CanNode : public rclcpp::Node {
public:
	Tf2CanNode() : Node("tf2can_node"), buttons_prev({0, }), stopped(1), dirA(1), dirB(1) {
		pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
		sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
				"/regression/output", 10,
				std::bind(&Tf2CanNode::tf_callback, this, std::placeholders::_1)
				);
	}
private:
	uint8_t buttons_prev[8];
	int stopped;
	uint8_t dirA, dirB;
	void tf_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		if (msg->data.size() > 4)
			return;

		can_msgs::msg::Frame can_msg;
		can_msg.is_rtr = false;
		can_msg.is_extended = false;
		can_msg.is_error = false;
		can_msg.header.frame_id = "base_link";

		int send = 0;
		for (int i = 0; i < 8; i++)
			can_msg.data[i] = 0;

		if (std::fabs(msg->data[1]) < 1e-4f) {
			if (stopped)
				return;
			else
				stopped = 1;
		} else
			stopped = 0;

		int OFFSET = 64;
		float drive = (255-OFFSET) * msg->data[1];
		if (drive > 0) {
			drive += OFFSET;
		} else if (drive < 0) {
			drive -= OFFSET;
		}
		float diff = drive * msg->data[0];
		
		int pwmA = static_cast<int>(std::round(drive - diff));
		int pwmB = static_cast<int>(std::round(drive + diff));
		//RCLCPP_INFO(this->get_logger(), " msg->data[1]: %.2f , msg->data[0]:%.2f", msg->data[1], msg->data[0]);
		RCLCPP_INFO(this->get_logger(), "drive: %.2f, diff: %.2f", drive, diff);

		if (pwmA > 255)
			pwmA = 255;
		else if (pwmA < -255)
			pwmA = -255;
		if (pwmB > 255)
			pwmB = 255;
		else if (pwmB < -255)
			pwmB = -255;

		if (pwmA < 0) {
			can_msg.data[0] = 0;
			can_msg.data[1] = static_cast<uint8_t>(-pwmA);
			dirA = 0;
		} else if (pwmA > 0) {
			can_msg.data[0] = 1;
			can_msg.data[1] = static_cast<uint8_t>(pwmA);
			dirA = 1;
		} else {
			can_msg.data[0] = dirA;
			can_msg.data[1] = 0;
		}

		if (pwmB < 0) {
			can_msg.data[2] = 0;
			can_msg.data[3] = static_cast<uint8_t>(-pwmB);
			dirB = 0;
		} else if (pwmB > 0) {
			can_msg.data[2] = 1;
			can_msg.data[3] = static_cast<uint8_t>(pwmB);
			dirB = 1;
		} else {
			can_msg.data[2] = dirB;
			can_msg.data[3] = 0;
		}

		for (int i = 4; i < 8; i++)
			can_msg.data[i] = 0;

		can_msg.id = 0x100;
		can_msg.dlc = 4;
		can_msg.header.stamp = this->now();

		pub_->publish(can_msg);
	}

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Tf2CanNode>());
	rclcpp::shutdown();
	return 0;
}

