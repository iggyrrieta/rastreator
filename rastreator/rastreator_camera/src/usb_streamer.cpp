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

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rastreator_camera/usb_streamer.hpp"

using namespace camera;
using namespace std::chrono_literals;


Streamer::Streamer() : Node("asgasdga")
{

  RCLCPP_INFO(this->get_logger(), "Video streamer ON");

  this->declare_parameter("configuration.camera_id");
  this->declare_parameter("configuration.fame_id");
  this->declare_parameter("configuration.camera_name");
  this->declare_parameter("configuration.width");
  this->declare_parameter("configuration.height");
  this->declare_parameter("configuration.framerate");
  this->declare_parameter("configuration.flip_mode");
  this->declare_parameter("configuration.camera_calibration_file");

  this->get_parameter_or<int>("configuration.camera_id", configuration_.camera_id, 0);
  this->get_parameter_or<std::string>("configuration.frame_id", configuration_.frame_id, "camera");
  this->get_parameter_or<std::string>("configuration.camera_name", configuration_.camera_name, "usb_cam");
  this->get_parameter_or<int>("configuration.width", configuration_.width, 640);
  this->get_parameter_or<int>("configuration.height", configuration_.height, 480);
  this->get_parameter_or<int>("configuration.framerate", configuration_.framerate, 30);
  this->get_parameter_or<int>("configuration.flip_mode", configuration_.flip_mode, 2);
  this->get_parameter_or<std::string>("configuration.camera_calibration_file", configuration_.camera_calibration_file, "package://ros2_camera/config/usb_calibration.yaml");
  
	// Camera publisher image
  camera_transport_pub_ = image_transport::create_camera_publisher(this, "/image_raw");

  // Camera calibration file
  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, configuration_.camera_name, configuration_.camera_calibration_file);
  
  cap.open(configuration_.camera_id);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, configuration_.width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, configuration_.height);

	// Timer to imageCallback
  last_frame_ = std::chrono::steady_clock::now();
  timer_ = this->create_wall_timer(1ms, std::bind(&Streamer::ImageCallback, this));
}

void Streamer::ImageCallback()
{
    cap >> frame;

    auto now = std::chrono::steady_clock::now();

    if (!frame.empty() &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count() > 1/configuration_.framerate*1000)
    {
        last_frame_ = now;

        // Convert to a ROS2 image
        if (configuration_.flip_mode == 0)
        {
            // Flip the frame if needed
            cv::flip(frame, flipped_frame, 1);
        }

        std_msgs::msg::Header header_;
        sensor_msgs::msg::Image ros_image;

        // Make sure output in the size the user wants even if it is not native
        if(frame.rows != configuration_.width || frame.cols != configuration_.height){
            cv::resize(frame, frame, cv::Size(configuration_.width, configuration_.height));
        }

        /* To remove CV-bridge and boost-python3 dependencies, this is pretty much a copy of the toImageMsg method in cv_bridge. */
        ros_image.header = header_;
        ros_image.height = frame.rows;
        ros_image.width = frame.cols;
        ros_image.encoding = "bgr8";
        ros_image.is_bigendian = false;
        ros_image.step = frame.cols * frame.elemSize();
        size_t size = ros_image.step * frame.rows;
        ros_image.data.resize(size);

        if (frame.isContinuous())
        {
            memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
        }
        else
        {
            // Copy by row by row
            uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
            uchar *cv_data_ptr = frame.data;
            for (int i = 0; i < frame.rows; ++i)
            {
                memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
                ros_data_ptr += ros_image.step;
                cv_data_ptr += frame.step;
            }
        }

        image_msg_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
            new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));

        rclcpp::Time timestamp = this->get_clock()->now();

        image_msg_->header.stamp = timestamp;
        image_msg_->header.frame_id = configuration_.frame_id;

        camera_info_msg_->header.stamp = timestamp;
        camera_info_msg_->header.frame_id = configuration_.frame_id;

        camera_transport_pub_.publish(image_msg_, camera_info_msg_);

        //image_pub_->publish(std::move(ros_image));
    }
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera::Streamer>());
  rclcpp::shutdown();

  return 0;
}