// Copyright 2020
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
//
// Authors: Iñaki Lorente

#ifndef RASTREATOR_CAMERA_VIDEO_STREAMER_HPP_
#define RASTREATOR_CAMERA_VIDEO_STREAMER_HPP_

#include <stdio.h>
#include <iostream>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.h"
#include "camera_info_manager/camera_info_manager.h"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"


//OPENCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace camera
{

class Streamer : public rclcpp::Node
{
 public:
  explicit Streamer();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat frame;
  cv::Mat flipped_frame;
  cv::VideoCapture cap;

  typedef struct
  {
    int camera_id;
    std::string camera_name;
    std::string frame_id;
    int width;
    int height;
    double framerate;
    int flip_mode;
    std::string camera_calibration_file;

  } configuration;

  configuration configuration_;

  std::chrono::steady_clock::time_point last_frame_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
  image_transport::CameraPublisher camera_transport_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  
  
  std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
    
  std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(cv::Mat & frame);

  void ImageCallback();

};
} // namespace camera
#endif // RASTREATOR_CAMERA_VIDEO_STREAMER_HPP_