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

#include "rastreator_wheelmotor/dynamixel_sdk_wrapper.hpp"
#include <stdio.h>
#include <stdlib.h>

using namespace rastreator;

DynamixelSDKWrapper::DynamixelSDKWrapper(const Device & device) : device_(device)
{
  if (init_dynamixel_sdk_handlers() == false)
  {
    LOG_ERROR("WHEEL-MOTORS", "Motors NOT detected!");
    return;
  }
  else
  {
    LOG_DEBUG("WHEEL-MOTORS", "Motors detected!");
    return;
  }
}

DynamixelSDKWrapper::~DynamixelSDKWrapper()
{
  portHandler_->closePort();
}


bool DynamixelSDKWrapper::init_dynamixel_sdk_handlers()
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_.usb_port.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler((int)device_.protocol_version);
   
  if (portHandler_->openPort())
  {
    LOG_INFO("WHEEL-MOTORS", "Succeeded to open the port(%s)!", device_.usb_port.c_str());
  }
  else
  {
    LOG_ERROR("WHEEL-MOTORS", "Failed to open the port(%s)!", device_.usb_port.c_str());
    return false;
  }

  if (portHandler_->setBaudRate((int)device_.baud_rate))
  {
    LOG_INFO("WHEEL-MOTORS", "Succeeded to change the baudrate!");
  }
  else
  {
    LOG_ERROR("WHEEL-MOTORS", "Failed to change the baudrate(%d)!", device_.baud_rate);
    return false;
  }

  return true;
}

bool DynamixelSDKWrapper::write_single_register(uint8_t id, uint16_t addr, uint16_t data_length)
{
  // Motor#x
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, data_length, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    LOG_ERROR("WHEEL-MOTORS", "%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    LOG_ERROR("WHEEL-MOTORS", "Error writing single register at motor %d", id);
    return false;
  }
  else
  {
    LOG_INFO("WHEEL-MOTORS", "Motor#%d has been successfully updated (1 single register) \n", id);
  }

  return true;
}


bool DynamixelSDKWrapper::write_single_register_sync(uint16_t addr, uint16_t data_length)
{

  // Motor#1 (right)
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, device_.id_right, addr, data_length, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    LOG_ERROR("WHEEL-MOTORS", "%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    LOG_ERROR("WHEEL-MOTORS", "Error writing single register at motor %d", device_.id_right);
    return false;
  }
  else
  {
    LOG_INFO("WHEEL-MOTORS", "Motor#%d has been successfully updated (1 single register) \n", device_.id_right);
  }
  // Motor#2 (left)
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, device_.id_left, addr, data_length, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    LOG_ERROR("WHEEL-MOTORS", "%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    LOG_ERROR("WHEEL-MOTORS", "Error writing single register at motor %d", device_.id_left);
    return false;
  }
  else
  {
    LOG_INFO("WHEEL-MOTORS", "Motor#%d has been successfully updated (1 single register) \n", device_.id_left);
  }

  return true;

}

// Function that adds id, start_address, data_length to the Sync Read list
bool DynamixelSDKWrapper::create_storage(uint8_t start_addr, uint8_t data_length)
{
  dynamixel::GroupSyncRead groupSyncRead_(portHandler_, packetHandler_, start_addr, data_length);

  // Add parameter storage for MOTOR#1
  dxl_addparam_result = groupSyncRead_.addParam(device_.id_right);
  if (dxl_addparam_result != true)
  {
    LOG_ERROR("WHEEL-MOTORS", "[ID:%03d] groupSyncRead addparam failed", device_.id_right);
    return false;
  }
  // Add parameter storage for MOTOR#2
  dxl_addparam_result = groupSyncRead_.addParam(device_.id_left);
  if (dxl_addparam_result != true)
  {
    LOG_ERROR("WHEEL-MOTORS", "[ID:%03d] groupSyncRead addparam failed", device_.id_left);
    return false;
  }

  return true;
}

uint32_t DynamixelSDKWrapper::get_data(uint8_t id, uint16_t start_addr, uint16_t data_length)
{
  dynamixel::GroupSyncRead groupSyncRead_(portHandler_, packetHandler_, start_addr, data_length);

  // Add parameter storage for MOTOR#1
  dxl_addparam_result = groupSyncRead_.addParam(device_.id_right);
  if (dxl_addparam_result != true)
  {
    LOG_ERROR("WHEEL-MOTORS", "[ID:%03d] groupSyncRead addparam failed", device_.id_right);
    return false;
  }
  // Add parameter storage for MOTOR#2
  dxl_addparam_result = groupSyncRead_.addParam(device_.id_left);
  if (dxl_addparam_result != true)
  {
    LOG_ERROR("WHEEL-MOTORS", "[ID:%03d] groupSyncRead addparam failed", device_.id_left);
    return false;
  }

  // Syncwrite data
  dxl_comm_result = groupSyncRead_.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) LOG_INFO("WHEEL-MOTORS", "%s\n", packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupsyncread data of Dynamixel#1 is available
  dxl_getdata_result = groupSyncRead_.isAvailable(id, start_addr, data_length);
  if (dxl_getdata_result != true)
  {
    LOG_ERROR("WHEEL-MOTORS", "[ID:%03d] groupSyncRead getdata failed", id);
    return 0;
  }

  // Get data value
  get_data_result = groupSyncRead_.getData(id, start_addr, data_length);
 
  //LOG_INFO("WHEEL-MOTORS", "[ID:%03d] PresPos:%03d\n", id, get_data_result);

  return get_data_result;

}
  
bool DynamixelSDKWrapper::update_storage_velocity(
  uint8_t start_addr,
  uint8_t data_length,
  int32_t sp_vel_linear,
  int32_t sp_vel_angular
  )
{
  
  dynamixel::GroupSyncWrite groupSyncWrite_(portHandler_, packetHandler_, start_addr, data_length);

  // Dynamixel motors range [-256:256]
  if (((sp_vel_linear >= -265) and (sp_vel_linear<=256)) &
     ((sp_vel_angular >= -265) and (sp_vel_angular<=256))) 
  {
    // Allocate goal velocity value into byte array
    sp_vel_right[0] = DXL_LOBYTE(DXL_LOWORD(sp_vel_linear + sp_vel_angular));
    sp_vel_right[1] = DXL_HIBYTE(DXL_LOWORD(sp_vel_linear + sp_vel_angular));
    sp_vel_right[2] = DXL_LOBYTE(DXL_HIWORD(sp_vel_linear + sp_vel_angular));
    sp_vel_right[3] = DXL_HIBYTE(DXL_HIWORD(sp_vel_linear + sp_vel_angular));

    sp_vel_left[0] = DXL_LOBYTE(DXL_LOWORD(sp_vel_linear - sp_vel_angular));
    sp_vel_left[1] = DXL_HIBYTE(DXL_LOWORD(sp_vel_linear - sp_vel_angular));
    sp_vel_left[2] = DXL_LOBYTE(DXL_HIWORD(sp_vel_linear - sp_vel_angular));
    sp_vel_left[3] = DXL_HIBYTE(DXL_HIWORD(sp_vel_linear - sp_vel_angular));
  }
  else
  {
    LOG_ERROR("WHEEL-MOTORS", "Velocity linear out of range! : %d\n", sp_vel_linear);
    sp_vel_right[0] = 0;
    sp_vel_left[0] = 0;
  }
  
  // Add MOTOR#1 goal velocity value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite_.addParam(device_.id_right, sp_vel_right);
  if (dxl_addparam_result != true)
  {
    LOG_ERROR("WHEEL-MOTORS", "[ID:%03d] groupSyncWrite addparam failed", device_.id_right);
    return false;
  }  
  // Add MOTOR#2 goal velocity value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWrite_.addParam(device_.id_left, sp_vel_left);
  if (dxl_addparam_result != true)
  {
    LOG_ERROR("WHEEL-MOTORS", "[ID:%03d] groupSyncWrite addparam failed", device_.id_left);
    return false;
  }

  // Syncwrite goal velocity
  dxl_comm_result = groupSyncWrite_.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    LOG_INFO("WHEEL-MOTORS", "%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
  }

  // Clear syncwrite parameter storage
  groupSyncWrite_.clearParam();

  return true;
}  