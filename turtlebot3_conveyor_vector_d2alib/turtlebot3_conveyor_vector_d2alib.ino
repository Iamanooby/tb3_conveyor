#include "turtlebot3_conveyor.h"
#include "turtlebot3_conveyor_motor_driver.h"

#define LOOP_TIME_SEC 0.010f

DynamixelStatus conveyor;
Turtlebot3MotorDriver motor_driver;



//0.2,0.0,0.0
double x_velocity = 0.0;
double y_velocity = 0.0;
double theta_velocity = 0.0;

void setup()
{
  Serial.begin(57600);

  motor_driver.init();
}

void loop()
{



  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n'); // Read input until newline
    parseInput(input);
  }

  
  static uint32_t previous_time = 0;
  uint32_t present_time = millis();
  
  getmanualData();


  if((present_time - previous_time) >= (LOOP_TIME_SEC * 1000))
  {
    motor_driver.controlJoints(conveyor.setJointAngle());
    motor_driver.controlWheels(conveyor.setWheelVel());
//    Serial.println(motor_ready);
//    Serial.print("Joint valid: ");
//    Serial.println(motor_driver.controlJoints(conveyor.setJointAngle()));
//    Serial.print("Wheel valid:");
//    Serial.println(motor_driver.controlWheels(conveyor.setWheelVel()));

    previous_time = millis();
  }     
  
  // Serial.println(motor_driver.is_connected());
  // Serial.println(motor_driver.get_torque());

  // uint32_t input_profiles[MotorLocation::MOTOR_NUM_MAX]= {1,1,1,1,1,1,1,1};
  uint32_t input_profiles[MotorLocation::MOTOR_NUM_MAX]= {0,};
  motor_driver.write_profile_acceleration(input_profiles);
  int32_t positions[MotorLocation::MOTOR_NUM_MAX];
  int32_t velocities[MotorLocation::MOTOR_NUM_MAX];
  uint32_t output_profiles[MotorLocation::MOTOR_NUM_MAX];
  int16_t currents[MotorLocation::MOTOR_NUM_MAX];
  // motor_driver.get_torque();
  motor_driver.read_present_position(positions);
  motor_driver.read_present_velocity(velocities);
  motor_driver.read_profile_acceleration(output_profiles);
  motor_driver.read_present_current(currents);
  // for(int i = 0; i < MotorLocation::MOTOR_NUM_MAX;i++)
  // {
  //   Serial.print(" Pos ");
  //   Serial.print(i);
  //   Serial.print(" : ");
  //   Serial.print(currents[i]);
  // }
  // Serial.println();



}

void getmanualData()
{

    conveyor.setParams(x_velocity,y_velocity,theta_velocity);
  
}

void parseInput(String input) {
  int firstComma = input.indexOf(',');
  int secondComma = input.indexOf(',', firstComma + 1);

  if (firstComma > 0 && secondComma > firstComma) {
    // Extract x, y, and theta as substrings
    String xStr = input.substring(0, firstComma);
    String yStr = input.substring(firstComma + 1, secondComma);
    String thetaStr = input.substring(secondComma + 1);

    // Convert to float
    x_velocity = xStr.toFloat();
    y_velocity = yStr.toFloat();
    theta_velocity = thetaStr.toFloat();

    // Print for debugging
    Serial.print("X Velocity: ");
    Serial.print(x_velocity,3);
    Serial.print(", Y Velocity: ");
    Serial.print(y_velocity,3);
    Serial.print(", Theta Velocity: ");
    Serial.println(theta_velocity,3);
  } else {
    Serial.println("Error: Invalid input format");
  }
}
