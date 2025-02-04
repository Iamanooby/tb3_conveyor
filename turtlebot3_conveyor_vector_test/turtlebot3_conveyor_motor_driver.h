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

#ifndef TURTLEBOT3_CONVEYOR_MOTOR_DRIVER_H_
#define TURTLEBOT3_CONVEYOR_MOTOR_DRIVER_H_


#include <DynamixelSDK.h>

#define TORQUE_ENABLE ControlTableItem::TORQUE_ENABLE



// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_PROFILE_ACCELERATION     108
#define ADDR_X_PROFILE_VELOCITY         112
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_CURRENT          126
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
//#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_PROFILE_ACCELERATION      4
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_CURRENT           2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

enum MotorLocation{
  WHEEL_L_R = 0,
  WHEEL_R_R=1,
  WHEEL_L_F=2,
  WHEEL_R_F=3,

  JOINT_L_R=4,
  JOINT_R_R=5,
  JOINT_L_F=6,
  JOINT_R_F=7,
  MOTOR_NUM_MAX
};

enum VelocityType{
  LINEAR_X = 0,
  LINEAR_Y,
  ANGULAR,
  TYPE_NUM_MAX
};


#define BAUDRATE                        57600 // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(void);
  void close(void);//
  void closeDynamixel(void);

  bool motor_is_connected(uint8_t id);//
  bool is_connected();//

  bool set_torque(bool onoff);//for all motors
  bool setMotorTorque(uint8_t id, bool onoff);
  bool get_torque();//
  bool getMotorTorque(uint8_t id);

  bool read_present_position(int32_t positions[MotorLocation::MOTOR_NUM_MAX]);
  bool read_present_velocity(int32_t velocities[MotorLocation::MOTOR_NUM_MAX]);
  bool read_present_current(int16_t currents[MotorLocation::MOTOR_NUM_MAX]);
  bool read_profile_acceleration(uint32_t profiles[MotorLocation::MOTOR_NUM_MAX]);

  bool setProfileAcceleration(uint8_t id, uint32_t value);
  bool setProfileVelocity(uint8_t id, uint32_t value);
  bool controlJoints(int32_t *value);
  bool controlWheels(int32_t *value);

  bool write_profile_acceleration(uint32_t profiles[MotorLocation::MOTOR_NUM_MAX]);

 private:
  uint32_t baudrate_;
  float  protocol_version_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncWrite *groupSyncWritePosition_;
  dynamixel::GroupSyncWrite *groupSyncWriteProfileAcc_;
  dynamixel::GroupSyncRead *groupSyncReadPosition_;
  dynamixel::GroupSyncRead *groupSyncReadVelocity_;
  dynamixel::GroupSyncRead *groupSyncReadCurrent_;
  dynamixel::GroupSyncRead *groupSyncReadProfileAcc_;
};

#endif // TURTLEBOT3_CONVEYOR_MOTOR_DRIVER_H_
