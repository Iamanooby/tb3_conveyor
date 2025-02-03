#include "math.h"
#include "turtlebot3_conveyor_motor_driver.h"


#define BODY_LENGTH           0.256
#define SPEED_ADD_ON          1
#define LIMIT_X_MAX_VELOCITY            250

class DynamixelStatus
{
 private:
//   int velocity = 200;
    double min_wheel_angle = -M_PI/2;
    double max_wheel_angle = M_PI/2;
   double wheel_linear_vel[4] = {0.0, };//WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F
   double wheel_radian_angle[4] = {0.0, };//JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F
   int32_t joint_angle[4] = {0, };//JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F //actual wheel angle
   int32_t wheel_vel[4] = {0, };//WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F


 public:

//   double wheel_l_r_vel, wheel_r_r_vel, wheel_l_f_vel, wheel_r_f_vel;
//   double joint_l_r_angle, joint_r_r_angle, joint_l_f_angle, joint_r_f_angle;

//   DynamixelStatus()
//   : wheel_l_r_vel(0.0), wheel_r_r_vel(0.0), wheel_l_f_vel(0.0), wheel_r_f_vel(0.0)
//   , joint_l_r_angle(0.0), joint_r_r_angle(0.0), joint_l_f_angle(0.0), joint_r_f_angle(0.0)
//   { }

   void setParams(double x, double y, double omega)
   {
    double L = BODY_LENGTH/2; // Length
    double W = BODY_LENGTH/2; // Width
    
//    double R = sqrt((L * L) + (W * W));

    // Adjust velocities based on robot dimensions
    double A = x - omega * (L);
    double B = x + omega * (L);
    double C = y - omega * (W);
    double D = y + omega * (W);//omega*R*W/R

//    Serial.println(A);
//    Serial.println(B);
//    Serial.println(C);
//    Serial.println(D);
    // Calculate speed and angle for each wheel

//add the correct angles to each
     wheel_linear_vel[MotorLocation::WHEEL_L_R]= sqrt(A * A + C * C);//wheel_l_r_vel
     wheel_radian_angle[MotorLocation::JOINT_L_R-4] = atan2(C, A)+(M_PI/4+M_PI/2*1);//wheel_l_r_vel

     wheel_linear_vel[MotorLocation::WHEEL_R_R]= sqrt(B * B + C * C);//wheel_r_r_vel
     wheel_radian_angle[MotorLocation::JOINT_R_R-4] = atan2(C, B)+(M_PI/4+M_PI/2*0);//joint_r_r_angle

     wheel_linear_vel[MotorLocation::WHEEL_L_F] = sqrt(A * A + D * D);//wheel_l_f_vel
     wheel_radian_angle[MotorLocation::JOINT_L_F-4] = atan2(D, A)-(M_PI/4+M_PI/2*1);//joint_l_f_angle

     wheel_linear_vel[MotorLocation::WHEEL_R_F] = sqrt(B * B + D * D);//wheel_r_f_vel
     wheel_radian_angle[MotorLocation::JOINT_R_F-4] = atan2(D, B)-(M_PI/4+M_PI/2*0);//joint_r_f_angle


// contraint angles to be between -pi and pi (since thats the range of atan2)
     for (int i=0;i<4;i++)
     {
        while (wheel_radian_angle[i] > M_PI) {
          wheel_radian_angle[i] -= 2 * M_PI;
        }
        while (wheel_radian_angle[i] < -M_PI) {
          wheel_radian_angle[i] += 2 * M_PI;
        }
     }




      //reverse wheel if wheel exceed max/min
     for (int i=0;i<4;i++)
     {
      if(wheel_radian_angle[i]<min_wheel_angle)
      {
        wheel_radian_angle[i]+= M_PI;
        wheel_linear_vel[i] = -wheel_linear_vel[i];
      }
      else if (wheel_radian_angle[i]>max_wheel_angle)
      {
        wheel_radian_angle[i]-= M_PI;
        wheel_linear_vel[i] = -wheel_linear_vel[i];
      }
//      Serial.print(i);
//      Serial.print(" Linear: ");
//      Serial.print(wheel_linear_vel[i]);
//      Serial.print(" , Angle: ");
//      Serial.println(wheel_radian_angle[i]);

     }

   }

   int32_t* setJointAngle()//actual joint value
   {



    //actual joint value

     for (int i=0;i<4;i++)
     {
      joint_angle[i] = static_cast<int>(round(wheel_radian_angle[i]/M_PI*4095/2)) + 2048;//because wheel goes from -pi/2 to pi/2 but joint goes from 1024 to 3072

//      Serial.print("Joint ");
//      Serial.print(i);
//      Serial.print(": ");
//      Serial.println(joint_angle[i]);
     }


     return joint_angle;
   }

   int32_t* setWheelVel()//actual rotational velocity
   {

    
     double linear_to_data = (1/(M_PI*2*0.033))*265/(60.69/60);//from m/s linear to rotational data. 0.033m is wheel radius, 265 data gives 60.69rpm


     //scale all wheels down proportionally based on fastest wheel that exceeded speedlimit
     const double limit_x_max_linear_veloctiy = LIMIT_X_MAX_VELOCITY/linear_to_data;//change max from data value to linear instead
//     Serial.print("MAX SPEED: ");
//     Serial.println(limit_x_max_linear_veloctiy);
     double currentHighestSpeed = 0.0;//temp
     double proportion = 1.0;



     for (int i=0;i<4;i++)
     {
      if(abs(wheel_linear_vel[i])>limit_x_max_linear_veloctiy && abs(wheel_linear_vel[i])>currentHighestSpeed)
      {
          currentHighestSpeed = abs(wheel_linear_vel[i]);
          proportion = limit_x_max_linear_veloctiy/abs(wheel_linear_vel[i]);
      }
     }
  
     for (int i=0;i<4;i++)
     {
      wheel_vel[i] = static_cast<int>(round(wheel_linear_vel[i]*proportion*linear_to_data));

//      Serial.print("Wheel ");
//      Serial.print(i);
//      Serial.print(": ");
//      Serial.println(wheel_vel[i]);
     }

     return wheel_vel;
   }
};


/*
1. Basic Motions

    FORWARD (RC100_BTN_U)
        Move forward with no rotation.
    BACKWARD (RC100_BTN_D)
        Move backward with no rotation.
    Rotate Left (RC100_BTN_L)
        Rotate the robot counterclockwise (pivot in place).
    Rotate Right (RC100_BTN_R)
        Rotate the robot clockwise (pivot in place).
    Point Left (RC100_BTN_2)
        The robot strafes left (translation) without rotation.
    Point Right (RC100_BTN_4)
        The robot strafes right (translation) without rotation.

2. Combined Motions

    Diagonal/Complex Translations:
        Left-Forward (RC100_BTN_U + RC100_BTN_L): Moves diagonally forward to the left.
        Right-Forward (RC100_BTN_U + RC100_BTN_R): Moves diagonally forward to the right.
        Left-Backward (RC100_BTN_D + RC100_BTN_L): Moves diagonally backward to the left.
        Right-Backward (RC100_BTN_D + RC100_BTN_R): Moves diagonally backward to the right.

    Pivoting While Moving:
        Point Left + Forward (RC100_BTN_U + RC100_BTN_2): Combines strafing left with forward motion.
        Point Right + Forward (RC100_BTN_U + RC100_BTN_4): Combines strafing right with forward motion.
        Similar motions for backward directions.

3. Rotation + Translation

    Rotate + Strafe Combinations:
        Rotate Left + Point Left (RC100_BTN_L + RC100_BTN_6): Rotates left while strafing left.
        Rotate Left + Point Right (RC100_BTN_L + RC100_BTN_5): Rotates left while strafing right.
        Rotate Right + Point Left (RC100_BTN_R + RC100_BTN_6): Rotates right while strafing left.
        Rotate Right + Point Right (RC100_BTN_R + RC100_BTN_5): Rotates right while strafing right.

4. Complex Compound Motions

    Triple Button Combinations:
        Rotate Left + Point Left + Forward (RC100_BTN_L + RC100_BTN_6 + RC100_BTN_U)
            Combines rotation, strafing, and forward motion.
        Rotate Right + Point Right + Backward (RC100_BTN_R + RC100_BTN_5 + RC100_BTN_D)
            Combines rotation, strafing, and backward motion.
        Additional triple button combinations create similar behaviors in different directions.

*/
