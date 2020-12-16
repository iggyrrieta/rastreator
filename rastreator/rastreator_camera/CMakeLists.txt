cmake_minimum_required(VERSION 3.5)
project(rastreator_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#=====================================
#             Find dependencies
#=====================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV REQUIRED)

#=====================================
#          Build and Add executables
#=====================================
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/picam_streamer.cpp"
  "src/usb_streamer.cpp"  
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib 
                          rclcpp
                          rclcpp_components
                          sensor_msgs
                          std_msgs
                          camera_info_manager
                          image_transport
                          camera_calibration_parsers
                          OpenCV
                        )

add_executable(usb_streamer src/usb_streamer.cpp)
add_executable(picam_streamer src/picam_streamer.cpp)
target_link_libraries(usb_streamer ${PROJECT_NAME}_lib)
target_link_libraries(picam_streamer ${PROJECT_NAME}_lib)
ament_target_dependencies(usb_streamer
                          rclcpp
                          rclcpp_components
                          sensor_msgs
                          std_msgs
                          camera_info_manager
                          image_transport
                          camera_calibration_parsers
                          OpenCV
                        )
ament_target_dependencies(picam_streamer
                          rclcpp
                          rclcpp_components
                          sensor_msgs
                          std_msgs
                          camera_info_manager
                          image_transport
                          camera_calibration_parsers
                          OpenCV
                        )

#=====================================
#             Install
#=====================================
install(TARGETS
  usb_streamer
  picam_streamer
  DESTINATION lib/${PROJECT_NAME})   

install(DIRECTORY
  param
  bash
  launch
  DESTINATION share/${PROJECT_NAME}
)                   

ament_package()