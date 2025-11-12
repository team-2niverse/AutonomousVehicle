// 장애물 회피 및 차선 변경 노드
// 장애물 회피 (오른쪽 → 왼쪽) : MODE - DODGE
// 차선 변경 (왼쪽 → 오른쪽) : MODE - SWITCH
// 우회전 (오른쪽으로 쭉 들어감) : MODE - TURN

#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp" 
#include "std_msgs/msg/float32.hpp"

#include "ssv_interfaces/msg/motion_stamped.hpp"
#include "planner_manager/mode.hpp"

class LineChangeNode : public rclcpp::Node{
public:
    LineChangeNode() : Node("line_change_node"), current_mode_(Mode::MANUAL){
        // manual mode로 초기화

        // DODGE : 좌회전 → 우회전 → 직진
        this->declare_parameter<int>("do_left_time", 2);
        this->declare_parameter<int>("do_right_time", 10);
        this->declare_parameter<int>("do_straight_time", 23);
        this->declare_parameter<int>("do_finish_time", 34);
        this->declare_parameter<double>("do_left_steering", 1.0);
        this->declare_parameter<double>("do_right_steering", 0.0);
        this->declare_parameter<double>("do_straight_steering", -1.0);

        // SWITCH : 우회전 → 좌회전 → 직진
        this->declare_parameter<int>("sw_right_time", 2);
        this->declare_parameter<int>("sw_left_time", 10);
        this->declare_parameter<int>("sw_straight_time", 23);
        this->declare_parameter<int>("sw_finish_time", 34);
        this->declare_parameter<double>("sw_right_steering", -1.0);
        this->declare_parameter<double>("sw_left_steering", 0.0);
        this->declare_parameter<double>("sw_straight_steering", 1.0);

        // TURN : 우회전1 → 우회전2 → 직진
        this->declare_parameter<int>("tu_right_time1", 2);
        this->declare_parameter<int>("tu_right_time2", 10);
        this->declare_parameter<int>("tu_straight_time", 23);
        this->declare_parameter<int>("tu_finish_time", 34);
        this->declare_parameter<double>("tu_right_steering1", -1.0);
        this->declare_parameter<double>("tu_right_steering2", -0.5);
        this->declare_parameter<double>("tu_straight_steering", 0.0);

        do_cnt_ = 0;
        sw_cnt_ = 0;
        tu_cnt_ = 0;

        do_left_time_ = this->get_parameter("do_left_time").as_int();
        do_right_time_ = this->get_parameter("do_right_time").as_int();
        do_straight_time_ = this->get_parameter("do_straight_time").as_int();
        do_finish_time_ = this->get_parameter("do_finish_time").as_int();
        do_left_steering_ = this->get_parameter("do_left_steering").as_double();
        do_right_steering_ = this->get_parameter("do_right_steering").as_double();
        do_straight_steering_ = this->get_parameter("do_straight_steering").as_double();

        sw_right_time_ = this->get_parameter("sw_right_time").as_int();
        sw_left_time_ = this->get_parameter("sw_left_time").as_int();
        sw_straight_time_ = this->get_parameter("sw_straight_time").as_int();
        sw_finish_time_ = this->get_parameter("sw_finish_time").as_int();
        sw_right_steering_ = this->get_parameter("sw_right_steering").as_double();
        sw_left_steering_ = this->get_parameter("sw_left_steering").as_double();
        sw_straight_steering_ = this->get_parameter("sw_straight_steering").as_double();

        tu_right_time1_ = this->get_parameter("tu_right_time1").as_int();
        tu_right_time2_ = this->get_parameter("tu_right_time2").as_int();
        tu_straight_time_ = this->get_parameter("tu_straight_time").as_int();
        tu_finish_time_ = this->get_parameter("tu_finish_time").as_int();
        tu_right_steering1_ = this->get_parameter("tu_right_steering1").as_double();
        tu_right_steering2_ = this->get_parameter("tu_right_steering2").as_double();
        tu_straight_steering_ = this->get_parameter("tu_straight_steering").as_double();

        RCLCPP_INFO(this->get_logger(), "--- Line Change Parameters Loaded ---");        
        RCLCPP_INFO(this->get_logger(), "[DODGE (do) Parameters]");
        RCLCPP_INFO(this->get_logger(), " - Times (steps): Left=%d, Right=%d, Straight=%d, Finish=%d", do_left_time_, do_right_time_, do_straight_time_, do_finish_time_);
        RCLCPP_INFO(this->get_logger(), " - Steer Values: Left=%.2f, Right=%.2f, Straight=%.2f", do_left_steering_, do_right_steering_, do_straight_steering_);
        RCLCPP_INFO(this->get_logger(), "[SWITCH (sw) Parameters]");
        RCLCPP_INFO(this->get_logger(), " - Times (steps): Right=%d, Left=%d, Straight=%d, Finish=%d", sw_right_time_, sw_left_time_, sw_straight_time_, sw_finish_time_);
        RCLCPP_INFO(this->get_logger(), " - Steer Values: Right=%.2f, Left=%.2f, Straight=%.2f", sw_right_steering_, sw_left_steering_, sw_straight_steering_);
        RCLCPP_INFO(this->get_logger(), "[TURN (tu) Parameters]");
        RCLCPP_INFO(this->get_logger(), " - Times (steps): Right1=%d, Right2=%d, Straight=%d, Finish=%d", tu_right_time1_, tu_right_time2_, tu_straight_time_, tu_finish_time_);
        RCLCPP_INFO(this->get_logger(), " - Steer Values: Right1=%.2f, Right2=%.2f, Straight=%.2f", tu_right_steering1_, tu_right_steering2_, tu_straight_steering_);
        
        
        /* publisher */
        motion_pub_ = this->create_publisher<ssv_interfaces::msg::MotionStamped>("/planner/motion", 10);
        flag_pub_ = this->create_publisher<std_msgs::msg::UInt8>("return_mode", 10);

        /* subscriber */
        mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/vehicle_mode", 10,
            std::bind(&LineChangeNode::mode_callback, this, std::placeholders::_1)
        );
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/planner/speed", 10,
            std::bind(&LineChangeNode::speed_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Line Change Node ready");
    }

private:
    /* callback */
    void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg){
        this->current_mode_ = static_cast<Mode>(msg->data);
    }

    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg){
        float speed_now = msg->data; // creep mode speed
        // creep mode는 10Hz 주기로 pub (100ms 주기로 speed_callback)

        if ((current_mode_ != Mode::DODGE) && (current_mode_ != Mode::SWITCH) && (current_mode_ != Mode::TURN)){
            return;
        }

        bool pub_flag = false;
        bool finish_flag = false;
        float steering_new = 0.0f;

        bool dodge_flag = false;

        // DODGE Mode (장애물 회피)
        // DODGE : 좌회전 → 우회전 → 직진
        if (current_mode_ == Mode::DODGE){
            do_cnt_++;
            if (do_cnt_ == do_left_time_){
                steering_new = (float)do_left_steering_;
                pub_flag = true;
            } 
            else if (do_cnt_ == do_right_time_){
                steering_new = 0.0f;
		        speed_now = 0.0f;
                pub_flag = true;
            } // 300ms 동안 정지
	        else if (do_cnt_ == do_right_time_ + 3){
                steering_new = (float)do_right_steering_;
		        pub_flag = true;
	        }
            else if (do_cnt_ == do_straight_time_){
                steering_new = 0.0f;
		        speed_now = 0.0f;
                pub_flag = true;
            } // 300ms 동안 정지
	        else if (do_cnt_ == do_straight_time_ + 3){
		        steering_new = (float)do_straight_steering_;
		        pub_flag = true;
	        }
            else if (do_cnt_ == do_finish_time_){
                steering_new = 0.0f;
	        speed_now = 0.3f;
	        finish_flag = true;
                do_cnt_ = 0;
                dodge_flag = true;


                ssv_interfaces::msg::MotionStamped motion_msg;
                motion_msg.motion.speed = speed_now;
                motion_msg.motion.steering = steering_new;
                motion_msg.header.stamp = this->get_clock()->now();
                motion_pub_->publish(motion_msg);

            }

        }
        // SWTICH Mode (표지판 감지로 차선 변경)
        // SWITCH : 우회전 → 좌회전 → 직진
        else if (current_mode_ == Mode::SWITCH){
            sw_cnt_++;
            if (sw_cnt_ == sw_right_time_){
                steering_new = (float)sw_right_steering_;
                pub_flag = true;
            } 
            else if (sw_cnt_ == sw_left_time_){
                steering_new = 0.0f;
		        speed_now = 0.0f;
                pub_flag = true;
            } // 300ms 동안 정지
	        else if (sw_cnt_ == sw_left_time_ + 3){
                steering_new = (float)sw_left_steering_;
		        pub_flag = true;
	        }
            else if (sw_cnt_ == sw_straight_time_){
                steering_new = 0.0f;
		        speed_now = 0.0f;
                pub_flag = true;
            } // 300ms 동안 정지
	        else if (sw_cnt_ == sw_straight_time_ + 3){
		        steering_new = (float)sw_straight_steering_;
		        pub_flag = true;
	        }
            else if (sw_cnt_ == sw_finish_time_){
                finish_flag = true;
                sw_cnt_ = 0;
            }
        }
        // TURN Mode (표지판 감지로 우회전)
        // TURN : 우회전1 → 우회전2 → 직진
        else if (current_mode_ == Mode::TURN){
            tu_cnt_++;
            if (tu_cnt_ == tu_right_time1_) {
                steering_new = 0.0f;
		speed_now = 0.0f;
                pub_flag = true;
            }
            else if (tu_cnt_ == tu_right_time1_+3) {
                steering_new = tu_right_steering1_;
                pub_flag = true;
            }
	        else if (tu_cnt_ == tu_right_time2_) {
                steering_new = 0.0f;
		        speed_now = 0.0f;
		        pub_flag = true;
	        }
            else if (tu_cnt_ == tu_right_time2_+3) {
                finish_flag = true;
                tu_cnt_ = 0;
            }
        }

        // Publish
        if (pub_flag){
            ssv_interfaces::msg::MotionStamped motion_msg;
            motion_msg.motion.speed = speed_now;
            motion_msg.motion.steering = steering_new;
            motion_msg.header.stamp = this->get_clock()->now();
            motion_pub_->publish(motion_msg);
        }
        // Mode 종료
        if (finish_flag){
            do_cnt_ = 0;
            sw_cnt_ = 0;
            tu_cnt_ = 0;

            std_msgs::msg::UInt8 finish_flag_msg;

            if (dodge_flag) finish_flag_msg.data = static_cast<uint8_t>(Mode::SWITCH);
            else finish_flag_msg.data = static_cast<uint8_t>(Mode::CRUISE);

            flag_pub_->publish(finish_flag_msg);
        }
    }

    /* variable */
    rclcpp::Publisher<ssv_interfaces::msg::MotionStamped>::SharedPtr motion_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr flag_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    Mode current_mode_;
    // DODGE Mode
    int do_cnt_, do_right_time_, do_left_time_, do_straight_time_, do_finish_time_;
    double do_right_steering_, do_left_steering_, do_straight_steering_;
    // SWITCH Mode
    int sw_cnt_, sw_right_time_, sw_left_time_, sw_straight_time_, sw_finish_time_;
    double sw_right_steering_, sw_left_steering_, sw_straight_steering_;
    // TURN Mode
    int tu_cnt_, tu_right_time1_, tu_right_time2_, tu_straight_time_, tu_finish_time_;
    double tu_right_steering1_, tu_right_steering2_, tu_straight_steering_;
};

/* main */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineChangeNode>());
    rclcpp::shutdown();
    return 0;
}
