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

const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);



Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: torque_(false)
{
}



Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}


bool Turtlebot3MotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);



  sync_write_param.id_count = MotorLocation::MOTOR_NUM_MAX;

  for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    sync_write_param.xel[i].id = i;
  }

  sync_read_param.addr = ADDR_X_PRESENT_POSITION;
  sync_read_param.length = LEN_X_PRESENT_POSITION;
  sync_read_param.id_count = MotorLocation::MOTOR_NUM_MAX;

  for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    sync_read_param.xel[i].id = i;
  }

  // Enable Dynamixel Torque
  set_torque(true);

  return true;
}

Dynamixel2Arduino& Turtlebot3MotorDriver::getDxl()
{
  return dxl;
}


void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}



bool Turtlebot3MotorDriver::is_connected()
{
  bool connected = true;

  for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    connected = connected && dxl.ping(i);
  }

  return connected;
}



bool Turtlebot3MotorDriver::set_torque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = ADDR_X_TORQUE_ENABLE;
  sync_write_param.length = LEN_X_TORQUE_ENABLE;

  for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    sync_write_param.xel[i].data[0] = onoff;
  }

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}



bool Turtlebot3MotorDriver::get_torque()
{
  bool all_torque = true;
  for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
    all_torque = all_torque && dxl.readControlTableItem(TORQUE_ENABLE, i);
  }

  if(all_torque){
    torque_ = true;
  }else{
    torque_ = false;
  }

  return torque_;
}


bool Turtlebot3MotorDriver::read_present_position(int32_t positions[MotorLocation::MOTOR_NUM_MAX])
{
  bool ret = false;

  sync_read_param.addr = ADDR_X_PRESENT_POSITION;
  sync_read_param.length = LEN_X_PRESENT_POSITION;

  if(dxl.syncRead(sync_read_param, read_result))
  {
    for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
    {
      memcpy(&positions[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}



bool Turtlebot3MotorDriver::read_present_velocity(int32_t velocities[MotorLocation::MOTOR_NUM_MAX])
{
  bool ret = false;

  sync_read_param.addr = ADDR_X_PRESENT_VELOCITY;
  sync_read_param.length = LEN_X_PRESENT_VELOCITY;

  if(dxl.syncRead(sync_read_param, read_result))
  {
    for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
    {
      memcpy(&velocities[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::read_present_current(int16_t currents[MotorLocation::MOTOR_NUM_MAX])
{
  bool ret = false;

  sync_read_param.addr = ADDR_X_PRESENT_CURRENT;
  sync_read_param.length = LEN_X_PRESENT_CURRENT;

  if(dxl.syncRead(sync_read_param, read_result))
  {
    for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
    {
      memcpy(&currents[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}



bool Turtlebot3MotorDriver::read_profile_acceleration(uint32_t profiles[MotorLocation::MOTOR_NUM_MAX])
{
  bool ret = false;

  sync_read_param.addr = ADDR_X_PROFILE_ACCELERATION;
  sync_read_param.length = LEN_X_PROFILE_ACCELERATION;

  if(dxl.syncRead(sync_read_param, read_result))
  {
    for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
    {
      memcpy(&profiles[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}



bool Turtlebot3MotorDriver::write_profile_acceleration(uint32_t profiles[MotorLocation::MOTOR_NUM_MAX])
{
  bool ret = false;

  sync_write_param.addr = ADDR_X_PROFILE_ACCELERATION;
  sync_write_param.length = LEN_X_PROFILE_ACCELERATION;

  for (int i = 0;i<MotorLocation::MOTOR_NUM_MAX;i++)
  {
      memcpy(sync_write_param.xel[i].data, &profiles[i], sync_write_param.length);
  }

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::controlJoints(int32_t *value)
{
  bool ret = false;

  sync_write_param.addr = ADDR_X_GOAL_POSITION;
  sync_write_param.length = LEN_X_GOAL_POSITION;


  memcpy(sync_write_param.xel[MotorLocation::JOINT_L_R].data, (uint32_t*)&value[MotorLocation::JOINT_L_R-4], sync_write_param.length);
  memcpy(sync_write_param.xel[MotorLocation::JOINT_R_R].data, (uint32_t*)&value[MotorLocation::JOINT_R_R-4], sync_write_param.length);
  memcpy(sync_write_param.xel[MotorLocation::JOINT_L_F].data, (uint32_t*)&value[MotorLocation::JOINT_L_F-4], sync_write_param.length);
  memcpy(sync_write_param.xel[MotorLocation::JOINT_R_F].data, (uint32_t*)&value[MotorLocation::JOINT_R_F-4], sync_write_param.length);


  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}



bool Turtlebot3MotorDriver::controlWheels(int32_t *value)
{
  bool ret = false;

  sync_write_param.addr = ADDR_X_GOAL_VELOCITY;
  sync_write_param.length = LEN_X_GOAL_VELOCITY;


  memcpy(sync_write_param.xel[MotorLocation::WHEEL_L_R].data, &value[MotorLocation::WHEEL_L_R], sync_write_param.length);
  memcpy(sync_write_param.xel[MotorLocation::WHEEL_R_R].data, &value[MotorLocation::WHEEL_R_R], sync_write_param.length);
  memcpy(sync_write_param.xel[MotorLocation::WHEEL_L_F].data, &value[MotorLocation::WHEEL_L_F], sync_write_param.length);
  memcpy(sync_write_param.xel[MotorLocation::WHEEL_R_F].data, &value[MotorLocation::WHEEL_R_F], sync_write_param.length);


  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}
