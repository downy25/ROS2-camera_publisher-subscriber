#include <functional>
#include <memory>
#include <cstdio>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "camera_ros2/camera.hpp"

using std::placeholders::_1; //bind 함수 호출시 인자를 사용해야 하는데 이는 bind 함수가 호출될 때로 초기화->변경 불가
//std:placeholdse ->인자값을 새로운 함수의 인자로 받을 수 있도록 해준다

class CameraSubscriber1 : public rclcpp::Node
{ //메인 클래스로 rclcpp의 Node클래스를 상속해 사용
public:
  CameraSubscriber1() : Node("Camera_subscriber1")//클래스 생성자의 정의로 부모 클래스(Node)생성자 호출 및 노드 이름 초기화
  {
    //qos설정 --> publisher설정과 동일
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth));
    reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.reliability(reliability_policy);

    camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image1", qos_profile,
                              std::bind(&CameraSubscriber1::show_image, this, _1)); //(_1) ->함수의 인자값을 계속 변경해서 받도록 하겠다는 의미
    cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE); //창 생성
  }

private:
  void show_image(const sensor_msgs::msg::Image::SharedPtr msg) const
  {  //const -> 객체 내의 어떠한 멤버 변수도 바꿀 수 없음

    RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str()); //정보 전달    
    
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char*>(msg->data.data()), msg->step);  //msg의 정보를 받아 frame객체 생성
    if (msg->encoding == "rgb8") {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR); //RGB -> BGR 형태로 변환
    }

    cv::Mat cvframe = frame;
    //cv::threshold(cvframe, cvframe, 128, 255, cv::THRESH_BINARY);
    cv::imshow("showimage", cvframe);
    cv::waitKey(1);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_; //스마트 포인터 이용한 멤버 변수 생성
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //초기화 함수
  auto node = std::make_shared<CameraSubscriber1>(); //작성 한 클래스(스마트 포인터->할당 메모리 자동 해제)노드 변수에 생성
  rclcpp::spin(node); //노드 spin시켜 콜백 함수 실행 유지
  rclcpp::shutdown(); //종료와 같은 인터럽트 시그널 예외 상황 -> 노드 소멸 및 프로세스 종료
  return 0;
}
