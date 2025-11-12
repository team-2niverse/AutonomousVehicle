#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "can_msgs/msg/frame.hpp"

#include "ssv_interfaces/msg/motion_stamped.hpp"
#include "planner_manager/mode.hpp"
#include <cmath>

const double DEG_TO_RAD = M_PI / 180.0;

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

		// publish_pwm(msg->axes[1], msg->axes[3]);
		convert_velocity_to_pwm_and_send(msg->axes[1], msg->axes[3]);

		ssv_interfaces::msg::MotionStamped motion_msg;
		motion_msg.motion.speed = msg->axes[1];
		motion_msg.motion.steering = msg->axes[3];
		motion_msg.header.frame_id = "base_link";
		motion_msg.header.stamp = this->now();

		motion_pub_->publish(motion_msg);
	}

	void motion_callback(const ssv_interfaces::msg::MotionStamped::SharedPtr msg) {
		if (mode == Mode::MANUAL)
			return;
		else if (mode == Mode::DODGE)
			publish_pwm(msg->motion.speed, msg->motion.steering);
		else
			convert_velocity_to_pwm_and_send(msg->motion.speed, msg->motion.steering);
	}

    void convert_velocity_to_pwm_and_send(double v, double w)
    {
        // 파라미터 가져오기
        const double MAX_PWM = 255.0;
        const double TRACK_WIDTH = 0.13;
        const double WHEEL_RADIUS = 0.033;
        const double MAX_LINEAR_VEL_LIMIT = 0.88;
        // YAML에서 deg/s 단위의 파라미터를 읽어옴
        const double MAX_ANGULAR_VEL_LIMIT_DEGS = 860.0;
        // 즉시 rad/s 단위로 변환하여 사용
        const double MAX_ANGULAR_VEL_LIMIT_RADS = MAX_ANGULAR_VEL_LIMIT_DEGS * DEG_TO_RAD;

		const double MAX_STEERING_ANGLE_DEG = 65.5;
        const double MAX_STEERING_ANGLE_RAD = MAX_STEERING_ANGLE_DEG * (M_PI / 180.0);

		double steering_angle = w * MAX_STEERING_ANGLE_RAD;
        
        v *= MAX_LINEAR_VEL_LIMIT;
        w = v * std::tan(steering_angle) / 0.13;		

	// 안전을 위해 들어온 목표 속도를 제한 (이제 단위가 일치함)
        v = std::max(-MAX_LINEAR_VEL_LIMIT, std::min(v, MAX_LINEAR_VEL_LIMIT));
        w = std::max(-MAX_ANGULAR_VEL_LIMIT_RADS, std::min(w, MAX_ANGULAR_VEL_LIMIT_RADS));

        // Inverse Kinematics: (v, w) -> (v_L, v_R)
        double v_left = v - (w * TRACK_WIDTH / 2.0);
        double v_right = v + (w * TRACK_WIDTH / 2.0);

		// RCLCPP_INFO(this->get_logger(), "v_left: %f, v_right: %f", v_left, v_right);

        // 바퀴 선속도(m/s) -> 바퀴 각속도(rad/s) 변환
        double w_left = v_left / WHEEL_RADIUS;
        double w_right = v_right / WHEEL_RADIUS;

        // 모터가 낼 수 있는 최대 각속도 계산
        double max_wheel_angular_vel = MAX_LINEAR_VEL_LIMIT / WHEEL_RADIUS;

        // 바퀴 각속도를 -1.0 ~ 1.0 비율로 정규화
        double left_ratio = (max_wheel_angular_vel > 1e-6) ? (w_left / max_wheel_angular_vel) : 0.0;
        double right_ratio = (max_wheel_angular_vel > 1e-6) ? (w_right / max_wheel_angular_vel) : 0.0;

        // 출력 정규화
        double max_abs_ratio = std::max(std::abs(left_ratio), std::abs(right_ratio));
        if (max_abs_ratio > 1.0) {
            left_ratio /= max_abs_ratio;
            right_ratio /= max_abs_ratio;
        }

        // PWM 값으로 최종 변환
        int left_pwm = static_cast<int>(left_ratio * MAX_PWM);
        int right_pwm = static_cast<int>(right_ratio * MAX_PWM);

	// RCLCPP_INFO(this->get_logger(), "left_pwm: %d, right_pwm: %d", left_pwm, right_pwm);

	send_pwm_command(left_pwm, right_pwm);
    }

    void send_pwm_command(int left_pwm, int right_pwm)
    {
        auto frame = can_msgs::msg::Frame();
        frame.id = 0x100;
        frame.dlc = 4;

        frame.data[0] = (left_pwm >= 0) ? 1 : 0;
        frame.data[1] = static_cast<uint8_t>(std::abs(left_pwm));
        frame.data[2] = (right_pwm >= 0) ? 1 : 0;
        frame.data[3] = static_cast<uint8_t>(std::abs(right_pwm));

        control_pub_->publish(frame);
    }

	void publish_pwm(float speed, float steering) {
		int speed_scaled = static_cast<int>(255 * speed);
		int diff = static_cast<int>(speed_scaled * steering);

		int pwm_left = speed_scaled - diff;
		int pwm_right = speed_scaled + diff;

		can_msgs::msg::Frame can_msg{};

		if (pwm_left > 255) pwm_left = 255;
		else if (pwm_left < -255) pwm_left = -255;

		if (pwm_left > 0) {
			can_msg.data[0] = 1;
			can_msg.data[1] = static_cast<uint8_t>(pwm_left);
		} else if (pwm_left < 0) {
			can_msg.data[0] = 0;
			can_msg.data[1] = static_cast<uint8_t>(-pwm_left);
		} else {
			can_msg.data[0] = 0;
			can_msg.data[1] = static_cast<uint8_t>(5);
		}

		if (pwm_right> 255) pwm_right = 255;
		else if (pwm_right < -255) pwm_right = -255;

		if (pwm_right > 0) {
			can_msg.data[2] = 1;
			can_msg.data[3] = static_cast<uint8_t>(pwm_right);
		} else if (pwm_left < 0) {
			can_msg.data[2] = 0;
			can_msg.data[3] = static_cast<uint8_t>(-pwm_right);
		} else {
			can_msg.data[2] = 0;
			can_msg.data[3] = static_cast<uint8_t>(5);
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

