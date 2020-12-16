#=====================================
#             CMake
#=====================================
cmake_minimum_required(VERSION 3.5)
project(rastreator_wheelmotor)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamixel_sdk REQUIRED)


#=====================================
#          Build and Add executables
#=====================================
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/wheel_js.cpp"
  "src/wheel_odometry.cpp"
  "src/wheelmotor.cpp"
  "src/dynamixel_sdk_wrapper.cpp" 
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib 
                          rclcpp 
                          dynamixel_sdk
                          geometry_msgs
                          sensor_msgs
                          nav_msgs
                          tf2
                          tf2_ros
                          std_msgs
                          std_srvs
                        )

add_executable(${PROJECT_NAME} src/wheelmotor.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp 
                          dynamixel_sdk
                          geometry_msgs
                          sensor_msgs
                          nav_msgs
                          tf2
                          tf2_ros
                          std_msgs
                          std_srvs
                        )


#=====================================
#             Install
#=====================================
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  param
  launch
  DESTINATION share/${PROJECT_NAME}
)    

ament_package()
