#include "ros/ros.h"
#include "std_msgs/string.h"
#include "std_msgs/Int64.h"
#include <stdio.h>
#include <math.h>
#include "Kraken_msgs/thrusterData6Thruster.h"
#include "Kraken_msgs/thrusterData4Thruster.h"
#include "kraken_msgs/imuData.h"
#include "kraken_msgs/absoluteRPY"
#include "resources/topicHeader.h"

#define kp_left 0.9
#define ki_left 0.5
#define kd_left 0.0
#define kp_right -0.9
#define ki_right -0.5
#define kd_right -0.0

float32 yaw=0,roll,pitch,prevError,errorP,errorI,errorD,inc_yaw;
float32 goal=0;
float32 count=1;

void rpyCallback(const kraken_msgs::absoluteRPY &msg)
{
  
 
  if (count==1)
    {   
      ini_yaw = yaw;
    }
  
  else
    {
      prevError  = errorP;
      errorP = (ini_yaw + inc_yaw - yaw);//intial yaw and  increase in yaw needed .
      //Inc_yaw is the difference between the values of the desired yaw value and current (i.e. starting yaw value)
      //The errorP value keeps on decreasing with each iteration and the desired goal is to make it equal to zero so that the error is minimized.
      
      errorI = errorI + errorP;//Integration term - addition of the errors
      errorD = prevError - errorP;//DIfference of the errors 
      //error = errorP + errorI + errorD;
     
      ROS_INFO ("The new position of yaw is [%ld], the error is [%ld]",msg->yaw,errorP);
     
    }
 
 
 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pidControl");
  ros::NodeHandle n;
 
  ros::Subscriber imu_data=n.subscribe<kraken_msgs::absoluteRPY> (topicHeader.ABSOLUTE_RPY,1000,rpyCallback);
  ros::Publisher thruster_6= n.advertise<kraken_msgs::thrusterData6Thruster>(topicHeader.CONTROL_PID_THRUSTER6,1000);
  // ros::Publisher thruster_4= n.advertise<kraken_msgs::thrusterData4Thruster>(topicHeader.CONTROL_PID_THRUSTER4,1000);

  kraken_msgs::thrusterData6Thruster= thruster6_output;

  while(ros::ok())
    {
      thruster6_output.data[0] = 0.0;
      thruster6_output.data[1] = 0.0;
      thruster6_output.data[2] =Kp_left*errorP + Kd_left*errorD + Ki_left*errorI;
      thruster6_output.data[3] =Kp_right*errorP + Kd_right*errorD + Ki_right*errorI;
      thruster6_output.data[4] = 0.0;
      thruster6_output.data[5] = 0.0;
      thruster_6.publish(thruster6_output);
      ros::spinOnce();
       ++count;
    }
  if(!ros::ok())
     {
      thruster6_output.data[0] = 0.0;
      thruster6_output.data[1] = 0.0;
      thruster6_output.data[2] = 0.0;
      thruster6_output.data[3] = 0.0;
      thruster6_output.data[4] = 0.0;
      thruster6_output.data[5] = 0.0;
     
    }
return 0;
}