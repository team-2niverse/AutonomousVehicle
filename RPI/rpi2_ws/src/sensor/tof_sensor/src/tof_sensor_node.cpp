// 전방의 ToF 센서 값을 읽어옴
// 1차 프로젝트의 frame_to_topic.cpp 리팩토링

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp" // tof 값을 UInt32 메시지 타입으로 발행 (단위: mm)
#include "can_msgs/msg/frame.hpp" // can 프레임 메시지

class TofSensorNode : public rclcpp::Node {
public:
    /* 생성자 (객체를 만들 때 자동 호출 + 초기설정) */
    // node: read_tof_node
	TofSensorNode() : Node("tof_sensor_node") {
        /* publisher */
        // topic: tof_distance_mm
        // type: UInt32 (queue size: 10)
		tof_pub_ = this->create_publisher<std_msgs::msg::UInt32>("/tof_distance_mm", 10);

        /* subscriber */
        // topic: from_can_bus (CAN 버스로부터 오는 모든 메시지를 받음)
        // type: can frame
		sub_ = this->create_subscription<can_msgs::msg::Frame>(
				"/from_can_bus", 10,
				std::bind(&TofSensorNode::can_callback, this, std::placeholders::_1)
				);
		
		RCLCPP_INFO(this->get_logger(), "TOF Sensor Node (ID: 522) ready"); // ROS2의 printf
	}
private:
    /* callback function */
	void can_callback(const can_msgs::msg::Frame::SharedPtr msg) {
		
		// tof can id = 0x200
		if (msg->id == 0x200) {
			unsigned int tofVal = (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0]; // 3바이트를 조합해서 센싱 값을 mm 값으로 변환

			// ****************************************************디버깅용 로그
			//RCLCPP_INFO(this->get_logger(), "ToF received (ID 522): %u mm", tofVal);
			// ****************************************************디버깅용 로그

			auto tof_msg = std_msgs::msg::UInt32();
			tof_msg.data = tofVal;

			tof_pub_->publish(tof_msg); // topic /tof_distance_mm로 발행
		}
	}

	/* 멤버 변수 선언 */
	rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr tof_pub_; // ToF publisher
	rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_; // CAN subscriber
};

/* main */
int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TofSensorNode>()); // TofSensorNode 클래스로 생성하고 실행
	rclcpp::shutdown();
	return 0;
}
