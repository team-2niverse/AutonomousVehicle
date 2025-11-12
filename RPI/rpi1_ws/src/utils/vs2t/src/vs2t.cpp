#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace std::chrono_literals;

// 동기화 정책 정의: CompressedImage와 Joy 메시지를 동기화
// 큐 크기 10, 약 50ms 이내의 시간차 허용 (조정 가능)
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Joy
> SyncPolicy;



class SyncRepublisherNode : public rclcpp::Node
{
public:
    SyncRepublisherNode() : Node("sync_republisher_node")
    {
        RCLCPP_INFO(this->get_logger(), "SyncRepublisherNode 초기화 중...");

        // 1. Subscriber 정의
        image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "/camera/image_raw");
        joy_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Joy>>(
            this, "/joy");

        // 2. Publisher 정의
        // 동기화된 이미지와 추출된 조이 스틱 값을 발행할 토픽
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/image_synced", 10);
        joy_output_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
            "~/joy_synced", 10);

        // 3. Time Synchronizer 설정
	sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *image_sub_, *joy_sub_);

        
        // 4. 콜백 등록
	sync_->registerCallback(std::bind(&SyncRepublisherNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));


        RCLCPP_INFO(this->get_logger(), "SyncRepublisherNode 초기화 완료.");
    }

private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Joy>> joy_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_output_pub_;

    /**
     * @brief 동기화된 메시지가 도착했을 때 실행되는 콜백 함수
     * @param image_msg 동기화된 CompressedImage 메시지
     * @param joy_msg 동기화된 Joy 메시지
     */
    void sync_callback(
   	 const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
   	 const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg)
    {
        // 1. Joy 데이터의 axes 1번과 3번 값 추출 및 확인
        double axis1_val = 0.0;
        double axis3_val = 0.0;
        
        if (joy_msg->axes.size() > 1) {
            axis1_val = joy_msg->axes[1]; // axes의 1번 값
        }
        if (joy_msg->axes.size() > 2) {
            axis3_val = joy_msg->axes[3]; // axes의 2번 값
        }

        RCLCPP_INFO(this->get_logger(), 
            "Synced! Image TS: %f, Joy TS: %f | Axes[1]: %f, Axes[3]: %f",
            rclcpp::Time(image_msg->header.stamp).seconds(), 
            rclcpp::Time(joy_msg->header.stamp).seconds(), 
            axis1_val, axis3_val);
        
        // 2. 동기화된 CompressedImage 메시지 재발행
        // (타임스탬프는 이미 동기화된 메시지의 것을 사용)
        image_pub_->publish(*image_msg);

        // 3. 추출한 axes 값만 담은 Joy 메시지 생성 및 발행 (동기화된 타임스탬프 사용)
        // Note: 기존 Joy 메시지에서 axes[1]과 axes[2]만 남긴 새 메시지 발행
        auto new_joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
        new_joy_msg->header.stamp = image_msg->header.stamp; // 이미지와 동기화된 타임스탬프 사용
        new_joy_msg->header.frame_id = joy_msg->header.frame_id;
        
        // axes 1번, 2번 값만 새 Joy 메시지의 axes[0]와 axes[1]에 담아서 발행
        new_joy_msg->axes.push_back(axis1_val); 
        new_joy_msg->axes.push_back(axis3_val); 

        joy_output_pub_->publish(std::move(new_joy_msg));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncRepublisherNode>());
    rclcpp::shutdown();
    return 0;
}
