/* Authors: Iñaki Lorente */

//
// *********     Sync Read and Sync Write Example for Velocity     *********
//
// Modification of Ryu Woon Jung code for ROBOTIS
// Using Dynamixel XL430-250 motors connected only using U2D2 without opencm or opencr
// Terminal ask for velocity, both motors run using that velocity

#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Dynamixel XL430-250
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_VELOCITY          104
#define ADDR_PRO_PRESENT_VELOCITY       128

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRO_GOAL_VELOCITY           4
#define LEN_PRO_PRESENT_VELOCITY        4

// Protocol version
#define PROTOCOL_VERSION                2.0              

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1 - RIGHT MOTOR
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2 - LEFT MOTOR
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_VELOCITY_VALUE      0                   // Dynamixel MIN velocity values
#define DXL_MAXIMUM_VELOCITY_VALUE      256                // Dynamixel MAX velocity values
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, LEN_PRO_GOAL_VELOCITY);

  // Initialize Groupsyncread instance for Present Velocity
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY);

  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result
  int dxl_goal_velocity[2] = {DXL_MINIMUM_VELOCITY_VALUE, DXL_MAXIMUM_VELOCITY_VALUE};  

  uint8_t dxl_error = 0;                            // Dynamixel error
  uint8_t param_goal_velocity[4];
  int32_t dxl1_present_velocity = 0, dxl2_present_velocity = 0;                       

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    return 0;
  }

  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
  }

  // Add parameter storage for Dynamixel#1 present velocity value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    return 0;
  }

  // Add parameter storage for Dynamixel#2 present velocity value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
    return 0;
  }

  // ASK for SET POINT VELOCITY
  int sp_velocity;

  while(1)
  {
    printf("WRITE GOAL VELOCITY:");
    
    std::cin >> sp_velocity;

    if ((sp_velocity >= dxl_goal_velocity[0]) and (sp_velocity<=dxl_goal_velocity[1]))
    {
      // Allocate goal velocity value into byte array
      param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(sp_velocity));
      param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(sp_velocity));
      param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(sp_velocity));
      param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(sp_velocity));

    }
    else
    {
      printf("ERROR. Velocity out of range!\n");
      param_goal_velocity[0] = 0;
    }
    
    // Add Dynamixel#1 goal velocity value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_velocity);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
      return 0;
    }    

    // Add Dynamixel#2 goal velocity value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_velocity);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
      return 0;
    }

    // Syncwrite goal velocity
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

  }

  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
