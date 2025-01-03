#include "math.h"

#define BODY_LENGTH           25.6
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

 public:

   double wheel_l_r_vel, wheel_r_r_vel, wheel_l_f_vel, wheel_r_f_vel;
   double joint_l_r_angle, joint_r_r_angle, joint_l_f_angle, joint_r_f_angle;

//   DynamixelStatus()
//   : wheel_l_r_vel(0.0), wheel_r_r_vel(0.0), wheel_l_f_vel(0.0), wheel_r_f_vel(0.0)
//   , joint_l_r_angle(0.0), joint_r_r_angle(0.0), joint_l_f_angle(0.0), joint_r_f_angle(0.0)
//   { }

   void setParams(double x, double y, double omega)
   {
    double L = BODY_LENGTH/2; // Length
    double W = BODY_LENGTH/2; // Width
    
    double R = std::sqrt((L * L) + (W * W));

    // Adjust velocities based on robot dimensions
    double A = x - omega * (L);
    double B = x + omega * (L);
    double C = y - omega * (W);
    double D = y + omega * (W);//omega*R*W/R

    // Calculate speed and angle for each wheel


     wheel_linear_vel[0]= std::sqrt(A * A + C * C);//wheel_l_r_vel
     wheel_radian_angle[0] = std::atan2(C, A)-M_PI/4*3;//wheel_l_r_vel

     wheel_linear_vel[1]= std::sqrt(B * B + C * C);//wheel_r_r_vel
     wheel_radian_angle[1] = std::atan2(C, B)-M_PI/4*4;//joint_r_r_angle

     wheel_linear_vel[2] = std::sqrt(A * A + D * D);//wheel_l_f_vel
     wheel_radian_angle[2] = std::atan2(D, A)-M_PI/4*2;//joint_l_f_angle

     wheel_linear_vel[3] = std::sqrt(B * B + D * D);//wheel_r_f_vel
     wheel_radian_angle[3] = std::atan2(D, B)-M_PI/4*1;//joint_r_f_angle

    //add the correct angles to each

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

     int32_t joint_angle[4] = {0, };//JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F


     //actual wheel angle


    //actual joint value

     for (int i=0;i<4;i++)
     {
      joint_angle[i] = static_cast<int>(round(wheel_radian_angle[i]/M_PI*4095/2)) + 2048;//because wheel goes from -pi/2 to pi/2 but joint goes from 1024 to 3072
     }

    //  joint_angle[0] = JOINT_L_R;
    //  joint_angle[1] = joint_l_r_angle + 2048;
    //  joint_angle[2] = JOINT_R_R;
    //  joint_angle[3] = joint_r_r_angle + 2048;
    //  joint_angle[4] = JOINT_L_F;
    //  joint_angle[5] = joint_l_f_angle + 2048;
    //  joint_angle[6] = JOINT_R_F;
    //  joint_angle[7] = joint_r_f_angle + 2048;

     return joint_angle;
   }

   int32_t* setWheelVel()//actual rotational velocity
   {

     int32_t wheel_vel[4] = {0, };//WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F
    
     double linear_to_angular = 1/(M_PI*2*0.033)*265/(60.69/60)//from m/s linear to rotational data. 0.033m is wheel radius, 265 data gives 60.69rpm
    
     wheel_linear_vel[0] =wheel_l_r_vel*linear_to_angular;
     wheel_linear_vel[1] = wheel_r_r_vel*linear_to_angular;
     wheel_linear_vel[2] = wheel_l_f_vel*linear_to_angular;
     wheel_linear_vel[3] = wheel_r_f_vel*linear_to_angular;


     //scale all wheels down proportionally based on fastest wheel that exceeded speedlimit
     double highestSpeed = 0.0;//temp
     double proportion = 1.0;
     for (int i=0;i<4;i++)
     {
      if(wheel_linear_vel[i]>LIMIT_X_MAX_VELOCITY && wheel_linear_vel[i]>highestSpeed)
      {
          highestSpeed = wheel_linear_vel[i];
          proportion = LIMIT_X_MAX_VELOCITY/wheel_linear_vel[i];
      }
     }

     for (int i=0;i<4;i++)
     {
      wheel_vel[i] = static_cast<int>(round(wheel_linear_vel[i]*proportion));
     }

    //  wheel_vel[0] = WHEEL_L_R;
    //  wheel_vel[1] = wheel_l_r_vel;
    //  wheel_vel[2] = WHEEL_R_R;
    //  wheel_vel[3] = wheel_r_r_vel;
    //  wheel_vel[4] = WHEEL_L_F;
    //  wheel_vel[5] = wheel_l_f_vel;
    //  wheel_vel[6] = WHEEL_R_F;
    //  wheel_vel[7] = wheel_r_f_vel;

//    Serial.println(wheel_vel[3]);

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
