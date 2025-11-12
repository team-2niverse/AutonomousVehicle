// 저속도 주행을 수행하는 노드 생성
// MODE - DODGE, SWITCH, TURN

#include <memory>
#include <chrono> // 타이머를 사용하기 위한 라이브러리

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"  // 모드 구독용
#include "std_msgs/msg/float32.hpp"  // speed 발행용 (0.0-1.0)

#include "planner_manager/mode.hpp"

using namespace std::chrono_literals;

class CreepNode : public rclcpp::Node {
public:
    /* 생성자 */
    // node: creep_node
    CreepNode() : Node("creep_node"), current_mode_(Mode::MANUAL) {
        // manual 모드로 초기화

        /* publisher */
        // topic: /target_speed
        // type: Float32 (speed 0.0~1.0)
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/planner/speed", 10);

        /* subscriber */
        // topic: /vehicle_mode
        // type: UInt8
        mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/vehicle_mode", 10,
            std::bind(&CreepNode::mode_callback, this, std::placeholders::_1)
        );

        /* Timer */
        // 100ms 주기마다 timer_callback
        timer_ = this->create_wall_timer(
            100ms, // ToF 센서의 주기와 동일 (10Hz)
            std::bind(&CreepNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Creep Node ready");
    }

private:
    /* callback function 1 */
    void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg){
        this->current_mode_ = static_cast<Mode>(msg->data);
    }

    /* callback function 2 */
    void timer_callback(){
        // 현재 저장된 모드가 normal일 경우 즉시 종료 (skip)
        if (current_mode_ == Mode::MANUAL || current_mode_ == Mode::CRUISE) {
            return;
        }

        const float creep_speed_output = 0.32f;

        //********************************************* 디버깅용 로그
        //RCLCPP_INFO(this->get_logger(), "Creep Mode (%d) - Speed: %.2f", static_cast<int>(current_mode_), creep_speed_output);
        //********************************************* 디버깅용 로그

        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = creep_speed_output;
        speed_pub_->publish(speed_msg);
    }

    /* 멤버 변수 선언 */
    // Timer
    rclcpp::TimerBase::SharedPtr timer_; // 타이머 객체
    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_; // speed 발행용
    // Subscriber
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_; // 모드 구독용
    // Variable
    Mode current_mode_; // 현재 모드를 저장하기 위한 변수
};

/* main */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CreepNode>()); // CreepNode로 변경
    rclcpp::shutdown();
    return 0;
}
