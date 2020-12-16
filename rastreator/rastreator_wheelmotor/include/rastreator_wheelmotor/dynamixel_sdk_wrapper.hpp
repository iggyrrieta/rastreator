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

#ifndef RASTREATOR_WHEELMOTOR_DYNAMIXEL_SDK_WRAPPER_HPP_
#define RASTREATOR_WHEELMOTOR_DYNAMIXEL_SDK_WRAPPER_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rcutils/logging_macros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#define LOG_INFO RCUTILS_LOG_INFO_NAMED
#define LOG_WARN RCUTILS_LOG_WARN_NAMED
#define LOG_ERROR RCUTILS_LOG_ERROR_NAMED
#define LOG_DEBUG RCUTILS_LOG_DEBUG_NAMED

namespace rastreator
{

class DynamixelSDKWrapper
{
 public:
  typedef struct
  {
    std::string usb_port;
    uint8_t id_right;
    uint8_t id_left;
    uint32_t baud_rate;
    float protocol_version;
  } Device;

  explicit DynamixelSDKWrapper(const Device & device);
  virtual ~DynamixelSDKWrapper();

  // Function that writes 1 single register to 1 single motor
  bool write_single_register(uint8_t id, uint16_t addr, uint16_t data_length);
  // Function that writes 1 single register to both motors (sync)
  bool write_single_register_sync(uint16_t addr, uint16_t data_length);
  // Function that adds id, start_address, data_length to the Sync Read list
  bool create_storage(uint8_t start_addr, uint8_t data_length);
  // Function that updates velocity register using specific id (1 or 2 motors)
  bool update_storage_velocity(
    uint8_t start_addr,
    uint8_t data_length,
    int32_t sp_vel_linear,
    int32_t sp_vel_angular
    );
  // Function that gets the data which might be received by
  // GroupSyncRead::rxPacket or GroupSyncRead::txRxPacket
  uint32_t get_data(uint8_t id, uint16_t start_addr, uint16_t data_length);
  
 
 private:
  bool init_dynamixel_sdk_handlers();

  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;

  Device device_;

  uint8_t dxl_error = 0;                            // Dynamixel error
  uint8_t sp_vel_right[4];    
  uint8_t sp_vel_left[4];   

  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result
  int32_t get_data_result = 0;                      // Present position
  
};
} // rastreator
#endif // RASTREATOR_WHEELMOTOR_DYNAMIXEL_SDK_WRAPPER_HPP_