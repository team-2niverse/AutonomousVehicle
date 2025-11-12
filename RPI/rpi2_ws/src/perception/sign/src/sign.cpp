#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
//#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

/*
 * 참고: "빨간색" 픽셀 정의
 * * 이 코드는 "빨간색"을 BGR 값 기준으로 (B < 100, G < 100, R > 200)로 가정합니다.
 * 이 값은 조명이나 카메라 조건에 따라 크게 달라질 수 있습니다.
 * * 더 정확한 감지를 위해서는 RGB 대신 HSV 색 공간을 사용하는 것이 좋습니다.
 * 1. cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
 * 2. cv::inRange(hsv_image, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1); // 낮은 H 범위
 * 3. cv::inRange(hsv_image, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2); // 높은 H 범위
 * 4. cv::Mat red_mask = mask1 + mask2;
 * 5. 이후 red_mask의 ROI에서 cv::countNonZero()를 사용해 픽셀 수를 계산할 수 있습니다.
 */

class SignDetector : public rclcpp::Node
{
public:
    SignDetector() : Node("sign_detector"), change_(3), turn_(3)
    {
        // QoS 설정
        auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();
	auto local_qos = rclcpp::QoS(1).transient_local();

        // 퍼블리셔 생성
        change_pub_ = this->create_publisher<std_msgs::msg::Empty>("/changeFlag", local_qos);
        turn_pub_ = this->create_publisher<std_msgs::msg::Empty>("/turnFlag", local_qos);

        // 서브스크라이버 생성
        // QOS는 카메라 드라이버와 맞추는 것이 좋습니다 (예: best_effort 또는 reliable)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/sign/camera/image_raw", qos,
            std::bind(&SignDetector::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Red Pixel Detector Node가 시작되었습니다.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            // ROS 메시지를 OpenCV 이미지(BGR8)로 변환
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 이미지 크기가 ROI를 처리하기에 충분한지 확인
        if (cv_ptr->image.rows <= 72 || cv_ptr->image.cols <= 112)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "이미지 크기가 너무 작아 ROI를 처리할 수 없습니다.");
            return;
        }


	// --- 1. BGR 이미지를 HSV 이미지로 변환 ---
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // --- 2. 빨간색 HSV 범위 정의 ---
        // OpenCV H 범위: 0-179
        // 낮은 범위 (0-10)
        cv::Scalar lower_red1(0, 120, 70);   // S(채도), V(명도) 값은 조절 필요
        cv::Scalar upper_red1(10, 255, 255);
        // 높은 범위 (170-179)
        cv::Scalar lower_red2(170, 120, 70);
        cv::Scalar upper_red2(179, 255, 255);

        // --- 3. 마스크 생성 및 결합 ---
        cv::Mat mask1, mask2;
        cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
        cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
        cv::Mat red_mask = mask1 + mask2; // 두 범위의 마스크를 합침
					  
     	// --- 4. 'change' 로직 (ROI: 48~60, 80~96) ---
        if (change_ > 0)
        {
            // ROI 영역 정의 (x, y, width, height)
            // x: 80, y: 48
            // width: 96 - 80 + 1 = 17
            // height: 60 - 48 + 1 = 13
            cv::Rect roi_change(80, 48, 17, 13);

            // ROI 영역의 마스크에서 0이 아닌 픽셀(빨간색) 수 계산
            int red_count_change = cv::countNonZero(red_mask(roi_change));

            if (red_count_change > 35)
            {
                change_--;
                RCLCPP_INFO(this->get_logger(), "/changeFlag 조건 충족. change 카운터: %d (픽셀 수: %d)", change_, red_count_change);

                if (change_ == 0)
                {
                    auto pub_msg = std_msgs::msg::Empty();
                    change_pub_->publish(pub_msg);
                    RCLCPP_INFO(this->get_logger(), "*** /changeFlag 토픽을 발행합니다! ***");
                }
            }
	    else{
		change_ = 3;
	    }
        }

        // --- 5. 'turn' 로직 (ROI: 48~72, 90~112) ---
        if (turn_ > 0)
        {
            // ROI 영역 정의 (x, y, width, height)
            // x: 90, y: 48
            // width: 112 - 90 + 1 = 23
            // height: 72 - 48 + 1 = 25
            cv::Rect roi_turn(90, 48, 23, 25);
            
            // ROI 영역의 마스크에서 0이 아닌 픽셀(빨간색) 수 계산
            int red_count_turn = cv::countNonZero(red_mask(roi_turn));

            if (red_count_turn > 300)
            {
                turn_--;
                RCLCPP_INFO(this->get_logger(), "/turnFlag 조건 충족. turn 카운터: %d (픽셀 수: %d)", turn_, red_count_turn);

                if (turn_ == 0)
                {
                    auto pub_msg = std_msgs::msg::Empty();
                    turn_pub_->publish(pub_msg);
                    RCLCPP_INFO(this->get_logger(), "*** /turnFlag 토픽을 발행합니다! ***");
                }
            }
	    else{
		turn_ = 3;
	    }
        }
    }

    // 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr change_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr turn_pub_;
    int change_;
    int turn_;
};

// main 함수
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignDetector>());
    rclcpp::shutdown();
    return 0;
}
