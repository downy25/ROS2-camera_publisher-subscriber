#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

std::string mat_type2encoding(int mat_type);
//void convert_frame_to_message(const cv::Mat& frame, size_t frame_id, sensor_msgs::msg::Image& msg);
void convert_frame_to_message(const cv::Mat& frame, sensor_msgs::msg::Image& msg);
int encoding2mat_type(const std::string& encoding);

#endif  // CAMERA_HPP_
