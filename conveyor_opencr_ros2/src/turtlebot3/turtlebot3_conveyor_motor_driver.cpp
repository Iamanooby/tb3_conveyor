/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

// #include "turtlebot3_conveyor_motor_driver.h"

#include "../../include/turtlebot3/turtlebot3_conveyor_motor_driver.h"


Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  closeDynamixel();
}

bool Turtlebot3MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setMotorTorque(MotorLocation::WHEEL_L_R, true);
  setMotorTorque(MotorLocation::WHEEL_R_R, true);
  setMotorTorque(MotorLocation::WHEEL_L_F, true);
  setMotorTorque(MotorLocation::WHEEL_R_F, true);

  setMotorTorque(MotorLocation::JOINT_L_R, true);
  setMotorTorque(MotorLocation::JOINT_R_R, true);
  setMotorTorque(MotorLocation::JOINT_L_F, true);
  setMotorTorque(MotorLocation::JOINT_R_F, true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);

  Serial.println("SUCESSS");

  return true;
}



void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  closeDynamixel();
}

void Turtlebot3MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setMotorTorque(MotorLocation::WHEEL_L_R, false);
  setMotorTorque(MotorLocation::WHEEL_R_R, false);
  setMotorTorque(MotorLocation::WHEEL_L_F, false);
  setMotorTorque(MotorLocation::WHEEL_R_F, false);

  setMotorTorque(MotorLocation::JOINT_L_R, false);
  setMotorTorque(MotorLocation::JOINT_R_R, false);
  setMotorTorque(MotorLocation::JOINT_L_F, false);
  setMotorTorque(MotorLocation::JOINT_R_F, false);

  // Close port
  portHandler_->closePort();
}

bool Turtlebot3MotorDriver::motor_is_connected(uint8_t id)
{
  uint8_t dxl_error = 0;  
  int dxl_comm_result = COMM_TX_FAIL;
  dxl_comm_result = packetHandler_->ping(portHandler_, id, &dxl_error);


  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
    return false;
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
    return false;
  }

  return true;
}

bool Turtlebot3MotorDriver::is_connected()
{
  bool connected = true;
  connected = connected && motor_is_connected(MotorLocation::WHEEL_L_R);
  connected = connected && motor_is_connected(MotorLocation::WHEEL_R_R);
  connected = connected && motor_is_connected(MotorLocation::WHEEL_L_F);
  connected = connected && motor_is_connected(MotorLocation::WHEEL_R_F);
  connected = connected && motor_is_connected(MotorLocation::JOINT_L_R);
  connected = connected && motor_is_connected(MotorLocation::JOINT_R_R);
  connected = connected && motor_is_connected(MotorLocation::JOINT_L_F);
  connected = connected && motor_is_connected(MotorLocation::JOINT_R_F);
  
  return connected;

}

bool Turtlebot3MotorDriver::set_torque(bool onoff)
{

  bool set_torque = true;
  set_torque = set_torque && setMotorTorque(MotorLocation::WHEEL_L_R, onoff);
  set_torque = set_torque && setMotorTorque(MotorLocation::WHEEL_R_R, onoff);
  set_torque = set_torque && setMotorTorque(MotorLocation::WHEEL_L_F, onoff);
  set_torque = set_torque && setMotorTorque(MotorLocation::WHEEL_R_F, onoff);

  set_torque = set_torque && setMotorTorque(MotorLocation::JOINT_L_R, onoff);
  set_torque = set_torque && setMotorTorque(MotorLocation::JOINT_R_R, onoff);
  set_torque = set_torque && setMotorTorque(MotorLocation::JOINT_L_F, onoff);
  set_torque = set_torque && setMotorTorque(MotorLocation::JOINT_R_F, onoff);

  return set_torque;

}

bool Turtlebot3MotorDriver::setMotorTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
    return false;
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
    return false;
  }

  return true;
}

bool Turtlebot3MotorDriver::get_torque()
{
  bool get_torque = true;
  get_torque = get_torque && getMotorTorque(MotorLocation::WHEEL_L_R);
  get_torque = get_torque && getMotorTorque(MotorLocation::WHEEL_R_R);
  get_torque = get_torque && getMotorTorque(MotorLocation::WHEEL_L_F);
  get_torque = get_torque && getMotorTorque(MotorLocation::WHEEL_R_F);

  get_torque = get_torque && getMotorTorque(MotorLocation::JOINT_L_R);
  get_torque = get_torque && getMotorTorque(MotorLocation::JOINT_R_R);
  get_torque = get_torque && getMotorTorque(MotorLocation::JOINT_L_F);
  get_torque = get_torque && getMotorTorque(MotorLocation::JOINT_R_F);

  return get_torque;

}

bool Turtlebot3MotorDriver::getMotorTorque(uint8_t id)
{
  uint8_t onoff;
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, &onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
    return false;
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
    return false;
  }

  if (onoff==0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool Turtlebot3MotorDriver::read_present_position(int32_t positions[MotorLocation::MOTOR_NUM_MAX])
{

  for(int i=0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    uint32_t position;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, i, ADDR_X_PRESENT_POSITION, &position, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler_->getTxRxResult(dxl_comm_result);
      return false;
    }
    else if(dxl_error != 0)
    {
      packetHandler_->getRxPacketError(dxl_error);
      return false;
    }

    positions[i] = static_cast<int32_t>(position);

  }

  return true;
}

bool Turtlebot3MotorDriver::read_present_velocity(int32_t velocities[MotorLocation::MOTOR_NUM_MAX])
{

  for(int i=0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    uint32_t velocity;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, i, ADDR_X_PRESENT_VELOCITY, &velocity, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler_->getTxRxResult(dxl_comm_result);
      return false;
    }
    else if(dxl_error != 0)
    {
      packetHandler_->getRxPacketError(dxl_error);
      return false;
    }

    velocities[i] = static_cast<int32_t>(velocity);

  }

  return true;
}

bool Turtlebot3MotorDriver::read_present_current(int16_t currents[MotorLocation::MOTOR_NUM_MAX])
{

  for(int i=0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    uint16_t current;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, i, ADDR_X_PRESENT_CURRENT, &current, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler_->getTxRxResult(dxl_comm_result);
      return false;
    }
    else if(dxl_error != 0)
    {
      packetHandler_->getRxPacketError(dxl_error);
      return false;
    }

    currents[i] = static_cast<int32_t>(current);

  }

  return true;
}

bool Turtlebot3MotorDriver::read_profile_acceleration(uint32_t profiles[MotorLocation::MOTOR_NUM_MAX])
{

  for(int i=0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, i, ADDR_X_PROFILE_ACCELERATION, &profiles[i], &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler_->getTxRxResult(dxl_comm_result);
      return false;
    }
    else if(dxl_error != 0)
    {
      packetHandler_->getRxPacketError(dxl_error);
      return false;
    }


  }

  return true;
}

bool Turtlebot3MotorDriver::write_profile_acceleration(uint32_t profiles[MotorLocation::MOTOR_NUM_MAX])
{

  for(int i=0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, i, ADDR_X_PROFILE_ACCELERATION, profiles[i], &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler_->getTxRxResult(dxl_comm_result);
      return false;
    }
    else if(dxl_error != 0)
    {
      packetHandler_->getRxPacketError(dxl_error);
      return false;
    }


  }

  return true;
}

bool Turtlebot3MotorDriver::controlJoints(int32_t *value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(MotorLocation::JOINT_L_R, (uint8_t*)&value[MotorLocation::JOINT_L_R-4]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(MotorLocation::JOINT_R_R, (uint8_t*)&value[MotorLocation::JOINT_R_R-4]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(MotorLocation::JOINT_L_F, (uint8_t*)&value[MotorLocation::JOINT_L_F-4]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(MotorLocation::JOINT_R_F, (uint8_t*)&value[MotorLocation::JOINT_R_F-4]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}

bool Turtlebot3MotorDriver::controlWheels(int32_t *value)
{

  if (!value) {
  Serial.print("Invalid velocity input: nullptr");
    return false;
  }

  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(MotorLocation::WHEEL_L_R, (uint8_t*)&value[MotorLocation::WHEEL_L_R]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(MotorLocation::WHEEL_R_R, (uint8_t*)&value[MotorLocation::WHEEL_R_R]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(MotorLocation::WHEEL_L_F, (uint8_t*)&value[MotorLocation::WHEEL_L_F]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(MotorLocation::WHEEL_R_F, (uint8_t*)&value[MotorLocation::WHEEL_R_F]);
  if (dxl_addparam_result_ != true)
    return false;



  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS) {
    return false;
  }

  groupSyncWriteVelocity_->clearParam();

  
  return true;
}
