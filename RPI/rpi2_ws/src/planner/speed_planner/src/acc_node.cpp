// ACC 기능을 수행하는 노드 생성
// MODE - CRUISE

#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"  // 모드 구독용
#include "std_msgs/msg/u_int32.hpp" // ToF 구독용 (단위: mm)
#include "std_msgs/msg/float32.hpp" // speed 발행용 (0.0 ~ 1.0)

#include "ssv_interfaces/msg/motion_stamped.hpp"
#include "planner_manager/mode.hpp"

const float DETECT_RANGE_MULTIPLIER = 1.5f;
const float MAX_INTEGRAL = 2.0f;
const float DT = 0.1f;

class AccNode : public rclcpp::Node {
public:
    /* 생성자 */
    // node: acc_node
    AccNode() : Node("acc_node"), current_mode_(Mode::MANUAL) {
        // manual 모드로 초기화

        this->declare_parameter<double>("kp", 0.005);
        this->declare_parameter<double>("ki", 0.001);
        this->declare_parameter<double>("cruise_speed", 0.4);
        this->declare_parameter<double>("target_dist", 200.0);
        this->declare_parameter<double>("panic_dist", 50.0);
        this->declare_parameter<double>("braking_factor", 200.0);

        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        target_dist_ = this->get_parameter("target_dist").as_double();
        panic_dist_ = this->get_parameter("panic_dist").as_double();
        braking_factor_ = this->get_parameter("braking_factor").as_double();

        braking_dist_ = (braking_factor_ * cruise_speed_) + panic_dist_;
        detect_threshold_ = target_dist_ * DETECT_RANGE_MULTIPLIER;

        RCLCPP_INFO(this->get_logger(), "--- ACC Parameters Loaded (mm units) ---");
        RCLCPP_INFO(this->get_logger(), " - kp: %f, ki: %f", kp_, ki_);
        RCLCPP_INFO(this->get_logger(), " - dist_target: %.1f mm, dist_panic: %.1f mm", target_dist_, panic_dist_);
        RCLCPP_INFO(this->get_logger(), " - cruise_speed (Max): %.2f", cruise_speed_);
        RCLCPP_INFO(this->get_logger(), " - braking_factor: %.1f mm", braking_factor_);

        error_integral_ = 0.0f;

        /* publisher */
        // topic: /planner/speed
        // type: Float32 (pwm 0.0~1.0)
	speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/planner/speed", 10);
        // speed_pub_ = this->create_publisher<ssv_interfaces::msg::MotionStamped>("/planner/motion", 10);

        /* subscriber 1 */
        // topic: /tof_distance_mm
        // type: UInt32
        tof_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
            "/tof_distance_mm", 10,
            std::bind(&AccNode::tof_callback, this, std::placeholders::_1)
        );

        /* subscriber 2 */
        // topic: /vehicle_mode
        // type: UInt8
        mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/vehicle_mode", 10,
            std::bind(&AccNode::mode_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ACC Node ready");
    }
private:
    /* callback function 1 */
    void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg){
        this->current_mode_ = static_cast<Mode>(msg->data);
    }

    /* callback function 2 */
    void tof_callback(const std_msgs::msg::UInt32::SharedPtr msg){
        
        // 현재 저장된 모드가 Mode::CRUISE 아닐 경우 skip (즉시 종료)
        if (current_mode_ != Mode::CRUISE){
            return;
        }

        float current_distance = static_cast<float>(msg->data);
        float speed_new = 0.0f;

        // case 1: predictive braking (AEB)
        if (current_distance < braking_dist_) {
            speed_new = 0.0f;
            error_integral_ = 0.0f;

            RCLCPP_WARN_ONCE(this->get_logger(), "PREDICTIVE BRAKING! Dist: %.1f < Needed: %.1f",
                current_distance, braking_dist_);
        }
        // case 2: cruise
        else if (current_distance > detect_threshold_) {
            speed_new = cruise_speed_;
            error_integral_ = 0.0f;
        }
        // case 3: PI control
        else {
            float error_dist = current_distance - target_dist_;
            error_integral_ += error_dist * DT;
            // anti-windup
            error_integral_ = std::max(std::min(error_integral_, MAX_INTEGRAL), -MAX_INTEGRAL);

            speed_new = kp_ * error_dist + ki_ * error_integral_;
        }

        // post processing for safty
        speed_new = std::max(0.0f, std::min(speed_new, static_cast<float>(cruise_speed_)));

        //********************************************* 디버깅용 로그
        RCLCPP_INFO(this->get_logger(), "ACC Control - Dist: %.1f mm, Speed: %.3f", current_distance, speed_new);
        //********************************************* 디버깅용 로그

        std_msgs::msg::Float32 speed_msg;
        speed_msg.data = speed_new;
        speed_pub_->publish(speed_msg);
        // ssv_interfaces::msg::MotionStamped speed_msg;
        // speed_msg.motion.speed = speed_new;
        // speed_msg.motion.steering = 0.0f;
        // speed_msg.header.stamp = this->get_clock()->now();
        // speed_pub_->publish(speed_msg);
    }

    /* 멤버 변수 선언 */
    // publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_; // speed 발행용
    // rclcpp::Publisher<ssv_interfaces::msg::MotionStamped>::SharedPtr speed_pub_;
    // subscribers
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr tof_sub_;  // ToF 구독용
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_; // 모드 구독용
    // variable
    Mode current_mode_; // 현재 모드를 저장하기 위한 변수

    double kp_, ki_, cruise_speed_, target_dist_, panic_dist_, braking_factor_;
    double braking_dist_, detect_threshold_;
    float error_integral_;
};

/* main */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccNode>());
    rclcpp::shutdown();
    return 0;
}
