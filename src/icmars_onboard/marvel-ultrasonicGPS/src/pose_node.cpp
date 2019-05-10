#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "marvelmind_nav/hedge_imu_raw.h"
#include <sstream>

ros::Publisher pose_pub;

geometry_msgs::Pose2D pose;

int hedge = 0;
double x_min = -744;
double x_max = 172;
double y_min = -855;
double y_max = 13;
bool calibrate = false;


void Callback(const marvelmind_nav::hedge_pos_ang::ConstPtr& msg)
{
//  ROS_INFO_STREAM(" Pose = "<< msg->x_m << ","<< msg->y_m<<","<< msg->z_m<<")");
  hedge = msg->address;

  if(hedge == 11 ){
    pose.x = msg->x_m;
    pose.y = msg->y_m;
  }
}

void Callback2(const marvelmind_nav::hedge_imu_raw::ConstPtr& msg)
{
//  ROS_INFO_STREAM("compass = "<<msg->compass_x << ","<<msg->compass_y << ","<<msg->compass_z<<")");

  pose_pub.publish(pose);

  float x_mean = (x_max+x_min)/2.0;
  float y_mean = (y_max+y_min)/2.0;
  float x_range = x_max-x_min;
  float y_range = y_max-y_min;
// change x_cal to negative to flip oreintation of rover to read positive theta in ccw direction and negative for cw direction.
  float x_cal = (msg->compass_x-x_mean)/(x_range/2.0);
  float y_cal = -(msg->compass_y-y_mean)/(y_range/2.0);

// added to offset theta by -90 degrees to align rover with x-axis
//  float modTheta = y_cal/x_cal;
  
  pose.theta = atan2(y_cal,x_cal); // changed atan2 to atan
//  ROS_INFO_STREAM("x_cal = " << x_cal << " y_cal =" << y_cal);

  if(calibrate)
  {
    if(msg->compass_x < x_min)
       x_min = msg->compass_x;
    if(msg->compass_x > x_max)
       x_max = msg->compass_x;
    if(msg->compass_y > y_max)
       y_max = msg->compass_y;
    if(msg->compass_y < y_min)
       y_min = msg->compass_y;
    ROS_INFO_STREAM("Calibration = x_min:" << x_min << ", x_max:" << x_max << ", y_min:" << y_min << ", y_max:" << y_max);
  }
}


void CalCallback(const std_msgs::Bool::ConstPtr& msg)
{
  calibrate = msg->data;

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("hedge_pos_ang", 100, Callback);
  ros::Subscriber sub2 = n.subscribe("hedge_imu_raw", 100, Callback2);
  ros::Subscriber cal = n.subscribe("calibrate", 1, CalCallback);
  pose_pub = n.advertise<geometry_msgs::Pose2D>("pose", 1000);

  ros::spin();

  return 0;
}

