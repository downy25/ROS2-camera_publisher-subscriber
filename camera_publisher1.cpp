#include <functional>
#include <memory>
#include <cstdio>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "camera_ros2/camera.hpp"

using namespace std::chrono_literals; //namespace 사용 -->추후에 500ms와 1s같이 가식성을 높인 문자로 표현하기 위해

class CameraPublisher1 : public rclcpp::Node //rclcpp의 Node클래스를 상속받는 CameraPublisher1 클래스 작성
{
public:
  CameraPublisher1() : Node("camera_publisher1"), count_(1), width_(160), height_(120) //생성자 정의 및 멤버변수 초기화
  {
    size_t depth = rmw_qos_profile_default.depth; //depth를 default값으로 저장 QoS 프로파일 사용
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; //reliability의 Best_effort 사용
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history; //history_policy값인 KEEP_LAST저장
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth));//qos구성
    qos_profile.reliability(reliability_policy); //best_effort로 지정

    camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image1", qos_profile); //토픽명을 image1으로 하는 camera_publisher_ 및
                                                                                                //qos_profile 퍼블리셔 초기화
    timer_ = this->create_wall_timer(30ms, std::bind(&CameraPublisher1::publish_image, this)); //30ms 주기마다 publish_image함수 호출

    // pi camera
    cap_.open(src_, cv::CAP_GSTREAMER);//GSTREAMER를 이용하여 pi camera 열기
    
    if (!cap_.isOpened()) {  //오류처리문장
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");        
    }

  }

private:
  void publish_image() //멤버 함수
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>(); //image 타입의 스마트 포인터
    msg->is_bigendian = false; //인수 지정
    cap_ >> frame_; //캠으로 부터 영상 받아오기
    if (!frame_.empty()) { //frame이 있으면 
        // Convert to a ROS image
        // cv::flip(frame, frame, 1); // Flip the frame if needed
        //convert_frame_to_message(frame_, count_, *msg);
        convert_frame_to_message(frame_, *msg); //프레임을 메시지로 변환
        
        //cv::Mat cvframe = frame_;
        //cv::imshow("camimage", cvframe);
        //cv::waitKey(1);
        
        // Publish the image message and increment the frame_id.
        RCLCPP_INFO(this->get_logger(), "Publishing image #%zd", count_++); //터미널에 띄우기 cout과 같은역할
        camera_publisher_->publish(std::move(msg));  //변환된 msg값     
    }
    else {
        RCLCPP_INFO(this->get_logger(), "frame empty"); //비어있으면 frame empty띄우기   
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
  size_t count_;
  cv::VideoCapture cap_;
  std::string src_ = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)160, height=(int)120, format=(string)NV12, framerate=(fraction)30/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)160, height=(int)120, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  cv::Mat frame_;
  size_t width_;
  size_t height_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //초기화 함수
  auto node = std::make_shared<CameraPublisher1>(); //작성 한 클래스(스마트 포인터->할당 메모리 자동 해제) 노드 변수에 생성
  rclcpp::spin(node); //노드 spin시켜 콜백 함수 실행 유지
  rclcpp::shutdown(); //종료(Ctrl +c)와 같은 인터럽트 시그널 예외 상황 ->노드 소멸 및 프로세스 종료
  return 0;
}
