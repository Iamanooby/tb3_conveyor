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

     }

   }

   int32_t* setJointAngle()//actual joint value
   {



    //actual joint value

     for (int i=0;i<4;i++)
     {
      joint_angle[i] = static_cast<int>(round(wheel_radian_angle[i]/M_PI*4095/2)) + 2048;//because wheel goes from -pi/2 to pi/2 but joint goes from 1024 to 3072

     }


     return joint_angle;
   }

   int32_t* setWheelVel()//actual rotational velocity
   {

    
     double linear_to_data = (1/(M_PI*2*0.033))*265/(60.69/60);//from m/s linear to rotational data. 0.033m is wheel radius, 265 data gives 60.69rpm


     //scale all wheels down proportionally based on fastest wheel that exceeded speedlimit
     const double limit_x_max_linear_veloctiy = LIMIT_X_MAX_VELOCITY/linear_to_data;//change max from data value to linear instead

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

     }

     return wheel_vel;
   }
};

