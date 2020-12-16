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


#ifndef RASTREATOR_WHEELMOTOR_CONTROL_TABLE_HPP_
#define RASTREATOR_WHEELMOTOR_CONTROL_TABLE_HPP_

#include <stdlib.h>

namespace rastreator
{
constexpr uint8_t EEPROM = 1;
constexpr uint8_t RAM = 2;

constexpr uint8_t READ = 1;
constexpr uint8_t READ_WRITE = 3;

typedef struct
{
  uint16_t addr;
  uint8_t memory;
  uint16_t length;
  uint8_t rw;
} ControlItem;

typedef struct
{
  // MOTORS Dynamixel XL430-250
  ControlItem model_number = {0, EEPROM, 2, READ};
  ControlItem model_information = {2, EEPROM, 4, READ};
  ControlItem firmware_version = {6, EEPROM, 1, READ};
  ControlItem id= {7, EEPROM, 1, READ};
  ControlItem baud_rate = {8, EEPROM, 1, READ};

  //right motor(id=1) = 1 (reverse) | left motor(id=2) = 0 (normal)
  ControlItem drive_mode = {10, RAM, 4, READ}; 
  ControlItem check_device_status = {123, RAM, 1, READ};

  ControlItem present_current = {144, RAM, 4, READ};
  ControlItem present_velocity = {128, RAM, 4, READ};
  ControlItem present_position = {132, RAM, 4, READ};

  ControlItem motor_torque_enable = {64, RAM, 1, READ_WRITE};
  ControlItem profile_acceleration = {108, RAM, 4, READ_WRITE};

  ControlItem goal_velocity = {104, RAM, 4, READ_WRITE};
  ControlItem min_velocity_value = {150, RAM, 4, READ_WRITE};
  ControlItem max_velocity_value = {154, RAM, 4, READ_WRITE};

} ControlTable;

const ControlTable extern_control_table;
} // rastreator

#endif // RASTREATOR_WHEELMOTOR_CONTROL_TABLE_HPP_