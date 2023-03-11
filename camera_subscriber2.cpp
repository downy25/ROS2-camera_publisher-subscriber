// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <cstdio>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "camera_ros2/camera.hpp"

using std::placeholders::_1;

class CameraSubscriber2 : public rclcpp::Node
{
public:
  CameraSubscriber2() : Node("Camera_subscriber2")
  {
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth));
    reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.reliability(reliability_policy);

    camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image2", qos_profile,
                                    std::bind(&CameraSubscriber2::show_image, this, _1));
    cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
  }

private:
  void show_image(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str());     
    
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char*>(msg->data.data()), msg->step);
    if (msg->encoding == "rgb8") {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    cv::Mat cvframe = frame;
    //cv::threshold(cvframe, cvframe, 128, 255, cv::THRESH_BINARY);
    cv::imshow("showimage", cvframe);
    cv::waitKey(1);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraSubscriber2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
