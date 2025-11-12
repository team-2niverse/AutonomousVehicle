/*
 * lkas_node.cpp
 *
 * 이 노드는 /camera/image_raw 토픽에서 이미지를 구독하고,
 * OpenCV를 사용해 리사이즈 및 전처리를 수행한 뒤,
 * /tmp/mobilenet_v1_1.0_224.tflite 모델로 추론을 실행합니다.
 * 추론 결과(분류 확률)는 /classification/output 토픽으로 발행합니다.
 */

#include <memory>
#include <string>
#include <vector>
#include <chrono>

// ROS 2 관련 헤더
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include "ssv_interfaces/msg/motion_stamped.hpp"

#include "planner_manager/mode.hpp"

// OpenCV 및 cv_bridge 관련 헤더 (이미지 변환 및 리사이즈용)
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

// TensorFlow Lite 관련 헤더
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>

using std::placeholders::_1;

class LkasTestNode : public rclcpp::Node
{
public:
  LkasTestNode()
  : Node("lkas_test_node"), latest_speed_(0.0f), mode_(Mode::MANUAL)
  {
    RCLCPP_INFO(this->get_logger(), "TFLite 분류 노드를 시작합니다...");

    // TFLite 모델 로드
    std::string model_path = "/root/mypilot.tflite";
    model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    if (!model_) {
      RCLCPP_FATAL(this->get_logger(), "TFLite 모델 로드 실패: %s", model_path.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "모델 로드 성공: %s", model_path.c_str());

    // TFLite 인터프리터 빌드
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
    if (!interpreter_) {
      RCLCPP_FATAL(this->get_logger(), "TFLite 인터프리터 생성 실패");
      rclcpp::shutdown();
      return;
    }

    // TFLite 텐서 할당
    if (interpreter_->AllocateTensors() != kTfLiteOk) {
      RCLCPP_FATAL(this->get_logger(), "TFLite 텐서 할당 실패");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "TFLite 텐서 할당 성공");

    // 입력 텐서 정보 가져오기
    
    if (interpreter_->inputs().size() != 2) {
        RCLCPP_FATAL(this->get_logger(), "모델 입력이 2개가 아닙니다.");
        rclcpp::shutdown();
        return;
    }
    //shape info function
    auto get_shape_string = [](TfLiteIntArray* dims) -> std::string {
        std::string shape_str = "[";
        if (dims) {
            for (int i = 0; i < dims->size; ++i) {
                shape_str += std::to_string(dims->data[i]);
                if (i < dims->size - 1) shape_str += ", ";
            }
        }
        shape_str += "]";
        return shape_str;
    };
    //input (velocity)
    vel_input_tensor_ = interpreter_->input_tensor(0); // 1번 인덱스
    if (vel_input_tensor_ == nullptr || vel_input_tensor_->dims->data[1] != 1) {
        RCLCPP_FATAL(this->get_logger(), "입력 텐서 1 (실수)이 단일 값이 아니거나 가져올 수 없습니다.");
        rclcpp::shutdown();
        return;
    }
    vel_input_type_ = vel_input_tensor_->type;
    std::string input_0_shape = get_shape_string(vel_input_tensor_->dims);
    RCLCPP_INFO(this->get_logger(), "모델 입력 0번: 모양=%s, 타입=%s",
                input_0_shape.c_str(),
                (vel_input_type_ == kTfLiteFloat32 ? "Float32" : (vel_input_type_ == kTfLiteUInt8 ? "UInt8" : "Unknown")));
    
    //input (image)
    input_tensor_ = interpreter_->input_tensor(1);
    if (input_tensor_ == nullptr) {
        RCLCPP_FATAL(this->get_logger(), "입력 텐서 1 (이미지)을 가져올 수 없습니다.");
        rclcpp::shutdown();
        return;
    }
    TfLiteIntArray* input_dims = input_tensor_->dims;
    input_height_ = input_dims->data[1]; //120
    input_width_ = input_dims->data[2]; //160
    input_channels_ = input_dims->data[3]; //3
    input_type_ = input_tensor_->type;

    RCLCPP_INFO(this->get_logger(), "모델 입력 텐서 1 크기: [H=%d, W=%d, C=%d]", input_height_, input_width_, input_channels_);
    RCLCPP_INFO(this->get_logger(), "모델 입력 텐서1 타입: %s", (input_type_ == kTfLiteFloat32 ? "Float32" : (input_type_ == kTfLiteUInt8 ? "UInt8" : "Unknown")));

    // 출력 텐서 정보 가져오기 (MobileNet 보통 [1, 1001] 형태)
    output_tensor_ = interpreter_->output_tensor(0);
     if (output_tensor_ == nullptr || interpreter_->outputs().size() != 1) {
        RCLCPP_FATAL(this->get_logger(), "출력 텐서를 가져올 수 없습니다.");
        rclcpp::shutdown();
        return;
    }
    TfLiteIntArray* output_dims = output_tensor_->dims;
    output_size_ = (output_dims->size > 1) ? output_dims->data[1] : output_dims->data[0];
    output_type_ = output_tensor_->type;

    RCLCPP_INFO(this->get_logger(), "모델 출력 크기 (클래스 개수): %d", output_size_);
    RCLCPP_INFO(this->get_logger(), "모델 출력 타입: %s", (output_type_ == kTfLiteFloat32 ? "Float32" : (output_type_ == kTfLiteUInt8 ? "UInt8" : "Unknown")));


    // ROS 2 퍼블리셔 및 서브스크라이버 초기화
    //publisher_ = this->create_publisher<std_msgs::msg::Float32>("/steering", 10);
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/regression/output", 10);
    motion_pub_ = this->create_publisher<ssv_interfaces::msg::MotionStamped>("/planner/motion", 10);
    // QoS 설정 (센서 데이터에 적합하게)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/vehicle_mode", 10, std::bind(&LkasTestNode::mode_callback, this, _1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/planner/speed", 10, std::bind(&LkasTestNode::speed_callback, this, _1));

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", qos, std::bind(&LkasTestNode::image_callback, this, _1));
    
    //subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    //  "/camera/image_raw/compressed", qos, std::bind(&LkasTestNode::image_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "노드 초기화 완료. /camera/image_raw 토픽을 구독합니다.");
  }

private:
  void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    mode_ = static_cast<Mode>(msg->data);
  }
  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    latest_speed_ = msg->data;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  //void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    if (mode_ != Mode::CRUISE)
      return;

    // 1. ROS 이미지를 OpenCV 이미지로 변환
    cv_bridge::CvImagePtr cv_ptr;
    try {
      // MobileNet은 RGB를 입력으로 받으므로, bgr8로 받은 뒤 RGB로 변환합니다.
      // 만약 카메라가 이미 RGB8을 제공한다면 "rgb8"을 사용해도 됩니다.
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외 발생: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Mat resized_image, preprocessed_image;

    // 2. 모델 입력 크기에 맞게 리사이즈
    cv::resize(image, preprocessed_image, cv::Size(input_width_, input_height_));

    // 3. 모델 입력 형식에 맞게 전처리 (BGR -> RGB)
    // cv::cvtColor(resized_image, preprocessed_image, cv::COLOR_BGR2RGB);
    
    // 4. TFLite 입력 텐서 채우기
    // 모델의 입력 타입에 따라 분기 (UInt8 또는 Float32)
    if (input_type_ == kTfLiteUInt8) {
      // --- UInt8 (양자화) 모델 ---
      uint8_t* input_data_ptr = interpreter_->typed_input_tensor<uint8_t>(1);
      if (input_data_ptr == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "입력 텐서 (UInt8) 포인터를 가져올 수 없습니다.");
        return;
      }
      // 리사이즈된 이미지 데이터를 텐서 메모리로 복사
      memcpy(input_data_ptr, preprocessed_image.data, 
             preprocessed_image.total() * preprocessed_image.elemSize());

    } else if (input_type_ == kTfLiteFloat32) {
      // --- Float32 (비양자화) 모델 ---
      // (참고: MobileNet v1 float 모델은 보통 -1.0 ~ 1.0 범위로 정규화 필요)
      float* input_data_ptr = interpreter_->typed_input_tensor<float>(1);
      if (input_data_ptr == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "입력 텐서 (Float32) 포인터를 가져올 수 없습니다.");
        return;
      }

      int i = 0;
      for (int y = 0; y < input_height_; ++y) {
        for (int x = 0; x < input_width_; ++x) {
          for (int c = 0; c < input_channels_; ++c) {
            // [0, 255] -> [0, 1.0] 정규화
            //input_data_ptr[i++] = (preprocessed_image.at<cv::Vec3b>(y, x)[c] - 127.5f) / 127.5f;
	    input_data_ptr[i++] = (preprocessed_image.at<cv::Vec3b>(y,x)[c])/225.0f;
	    //input_data_ptr[i++] = static_cast<float>(preprocessed_image.at<cv::Vec3b>(y, x)[c]);
          }
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "지원하지 않는 모델 입력 타입입니다.");
      return;
    }
    float dummy_float_input = latest_speed_;
    if (vel_input_type_ == kTfLiteFloat32) {
        float* vel_input_data_ptr = interpreter_->typed_input_tensor<float>(0);
        if (vel_input_data_ptr == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "두 번째 입력 텐서 (Float) 포인터를 가져올 수 없습니다.");
            return;
        }
        vel_input_data_ptr[0] = dummy_float_input;
    } else if (vel_input_type_ == kTfLiteUInt8) {
        uint8_t* vel_input_data_ptr = interpreter_->typed_input_tensor<uint8_t>(0);
        if (vel_input_data_ptr == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "두 번째 입력 텐서 (UInt8) 포인터를 가져올 수 없습니다.");
            return;
        }
        // [0, 255] 범위로 양자화 (0.0f -> 128)
        vel_input_data_ptr[0] = static_cast<uint8_t>(dummy_float_input * 128.0f + 128.0f);
    } else {
         RCLCPP_ERROR(this->get_logger(), "지원하지 않는 모델 입력 1 (실수) 타입입니다.");
         return;
    }


    // 5. TFLite 추론 실행
    if (interpreter_->Invoke() != kTfLiteOk) {
      RCLCPP_ERROR(this->get_logger(), "TFLite 추론(Invoke) 실패");
      return;
    }

    // 6. TFLite 출력 텐서 읽기
    // (출력은 보통 Float32, 양자화 모델은 UInt8일 수 있음)
    //auto output_msg = std::make_unique<std_msgs::msg::Float32>();
    auto output_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
    // (선택 사항) 레이아웃 설정: 메시지 내용을 설명합니다.
    output_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    output_msg->layout.dim[0].label = "model_output, model_input_float";
    output_msg->layout.dim[0].size = 2;
    output_msg->layout.dim[0].stride = 2;
    output_msg->layout.data_offset = 0;

    // 데이터 배열 크기를 2로 설정
    output_msg->data.resize(2);

    if (output_type_ == kTfLiteFloat32) {
      float* output_data_ptr = interpreter_->typed_output_tensor<float>(0);
      if (output_data_ptr == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "출력 텐서 (Float32) 포인터를 가져올 수 없습니다.");
        return;
      }
      //memcpy(output_msg->data.data(), output_data_ptr, output_size_ * sizeof(float));
      output_msg->data[0] = output_data_ptr[0];

    } else if (output_type_ == kTfLiteUInt8) {
      // (출력이 UInt8인 경우, Float32로 정규화하여 발행)
      uint8_t* output_data_ptr = interpreter_->typed_output_tensor<uint8_t>(0);
       if (output_data_ptr == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "출력 텐서 (UInt8) 포인터를 가져올 수 없습니다.");
        return;
      }
      output_msg->data[0] = (static_cast<float>(output_data_ptr[0]) - 128.0f) / 128.0f;
    } else {
        RCLCPP_ERROR(this->get_logger(), "지원하지 않는 모델 출력 타입입니다.");
        return;
    }
    output_msg->data[1] = dummy_float_input;   // [1]: 모델 입력 실수값


    // 7. 결과 발행
    // publisher_->publish(std::move(output_msg));
    ssv_interfaces::msg::MotionStamped motion_msg;
    motion_msg.motion.speed = output_msg->data[1];
    motion_msg.motion.steering = output_msg->data[0];
    motion_msg.header.stamp = this->now();

    motion_pub_->publish(motion_msg);
  }

  // TFLite 관련 멤버 변수
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  TfLiteTensor* input_tensor_ = nullptr;
  TfLiteTensor* vel_input_tensor_ = nullptr;
  TfLiteTensor* output_tensor_ = nullptr;
  int input_height_ = 0;
  int input_width_ = 0;
  int input_channels_ = 0;
  int output_size_ = 0;
  TfLiteType input_type_;
  TfLiteType vel_input_type_;
  TfLiteType output_type_;

  Mode mode_;
  float latest_speed_;

  // ROS 2 관련 멤버 변수
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  //rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<ssv_interfaces::msg::MotionStamped>::SharedPtr motion_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LkasTestNode>());
  rclcpp::shutdown();
  return 0;
}


