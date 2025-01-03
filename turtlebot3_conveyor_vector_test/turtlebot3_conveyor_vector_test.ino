#include "turtlebot3_conveyor.h"
#include "turtlebot3_conveyor_motor_driver.h"

#define LOOP_TIME_SEC 0.010f

RC100 rc100;
DynamixelStatus conveyor;
Turtlebot3MotorDriver motor_driver;

uint8_t conveyor_joint[4] = {JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F};
uint8_t conveyor_wheel[4] = {WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F};
bool motor_ready = false;

float x_velocity = 0.0;
float y_velocity = 0.0;
float theta_velocity = 0.0;

void setup()
{
  Serial.begin(57600);
  // while(!Serial);

//  rc100.begin(1);
  motor_ready= motor_driver.init();
}

void loop()
{

  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n'); // Read input until newline
    parseInput(input);


    static uint32_t previous_time = 0;
    uint32_t present_time = millis();
  
    getmanualData();
//  
//    if((present_time - previous_time) >= (LOOP_TIME_SEC * 1000))
//    {
//      motor_driver.controlJoints(conveyor.setJointAngle());
//      motor_driver.controlWheels(conveyor.setWheelVel());
//  
//      previous_time = millis();
//    }     
  }
  

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
    Serial.println(x_velocity);
    Serial.print("Y Velocity: ");
    Serial.println(y_velocity);
    Serial.print("Theta Velocity: ");
    Serial.println(theta_velocity);
  } else {
    Serial.println("Error: Invalid input format");
  }
}
