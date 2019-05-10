#include "ros/ros.h"
#include "std_msgs/String.h"
#include "angles/angles.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "icmars_msgs/TwistDuration.h"
#include "icmars_onboard/MoveAction.h"
#include <actionlib/server/simple_action_server.h>
#include <queue>
#include <iostream>

#define ANGLE_DEADBAND 0.12
#define DISTANCE_DEADBAND 0.12

/***
  This node is responsible for driving the rover to position goals.
  it subscribes to "goal_topic" and publishes cmd_vel.

  when a goal pose is received, the node begins publihsing to cmd_vel in
  order to drive to the goal.  while it is driving it publishes a status message
  "DRIVING".  When it is finished, it publishes a message "ARRIVED"

  if it has no goal, it publihes 0 velocity commands
 */
geometry_msgs::Pose2D goal;
geometry_msgs::Pose2D pose;
geometry_msgs::Pose2D error;
geometry_msgs::Twist Kp;
ros::Publisher cmd_pub;
ros::Publisher status_pub;
float angLimit, forwardLimit;
icmars_onboard::MoveFeedback feedback;
icmars_onboard::MoveResult result;
bool driving = false;
bool moving = false;
bool first_time = true;
/* get a yaw (theta) angle from a quaternion */
float yawFromQ(geometry_msgs::Quaternion &q) 
{
  return atan2(2.0 * (q.z * q.w + q.x * q.y) , 
      - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}
void pose2Dfrom3D(geometry_msgs::Pose p3D, geometry_msgs::Pose2D &p2D) {
  p2D.x = p3D.position.x;
  p2D.y = p3D.position.y;
  p2D.theta = yawFromQ(p3D.orientation);
}

void ctlCallback(const std_msgs::String::ConstPtr &msg) {
  if(msg->data == "CLEAR") {
  }
}
/* if move time is greater than zero, then the move_twist command will
   be in effect
 */
double move_time = 0;
/*  this is the move cmmand for manual driving.  It is in effect for a specific
    time given by move_time
 */
geometry_msgs::Twist move_twist;
ros::Time last_callback_time;
typedef actionlib::SimpleActionServer<icmars_onboard::MoveAction> Server;
bool ActionBusy = false;
Server *actionServer;
double distance_to_goal = 0.0;

double distanceToGoal() {
  float dx = goal.x - pose.x;
  float dy = goal.y - pose.y;
  return sqrt(dx*dx+dy*dy);
}
/* 
   pid to drive cmd_vel to go to or hold a pose
   returns distance to goal
 */
double pid(void) {
  float dx = goal.x - pose.x ;
  float dy = goal.y - pose.y;
  float headingToGoal = atan2(dy,dx);
  float d = sqrt(dx*dx+dy*dy);

  float heading_error = angles::shortest_angular_distance(pose.theta,headingToGoal);
  geometry_msgs::Twist cmd;

  if(d > DISTANCE_DEADBAND) {
    if(heading_error > ANGLE_DEADBAND ||
      heading_error<-ANGLE_DEADBAND) {
      cmd.linear.x = 0.0;
      cmd.angular.z = Kp.angular.z * heading_error;
      ROS_INFO_STREAM("Goal:("<<goal.x<<","<<goal.y<<") Pose:("<<pose.x<<","<<pose.y<<") Heading: "<<headingToGoal<<" Error: "<<heading_error);
    } else {
      //ROS_INFO_STREAM("Goal:("<<goal.x<<","<<goal.y<<") Pose:("<<pose.x<<","<<pose.y<<") Forward!");
      ROS_INFO("FORWARD");
      //cmd.linear.x = Kp.linear.x * d;
      cmd.linear.x = forwardLimit;
      cmd.angular.z = 0.0;
    }
    if(cmd.linear.x>forwardLimit) cmd.linear.x = forwardLimit;
    if(cmd.angular.z>angLimit) cmd.angular.z = angLimit;
    else if(cmd.angular.z < -angLimit) cmd.angular.z = -angLimit;
    //    status_pub.publish(status);
  }
  cmd_pub.publish(cmd);
  distance_to_goal = d;
  return d;
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  // pose2Dfrom3D(msg->pose.pose, pose);
  pose = *msg;
  ros::Time now = ros::Time::now();
  std::ostringstream ss;
  std_msgs::String status;
  if(move_time > 0.0) {
    ss << "MOVING: ";
    ros::Duration dt = now - last_callback_time;
    move_time -= dt.toSec();
    cmd_pub.publish(move_twist);
    goal = pose;
  } else {
    pid();
    if(ActionBusy) {
      ss << "DRIVING: goal=("<< std::setprecision(1) << goal.x<<","<<goal.y<<","<<goal.theta<<")";
    } else {
      ss << "HOLDING : goal=("<< std::setprecision(1) <<goal.x<<","<<goal.y<<","<<goal.theta<<")";
    }
  }
  last_callback_time = now;
  if(first_time) {
    goal = pose;
    first_time = false;
  }
  status.data = ss.str();
  status_pub.publish(status);
}

void execute(const icmars_onboard::MoveGoalConstPtr& g, Server* as)
{
  ROS_INFO("DRIVE start");
  ActionBusy = true;
  ros::Rate r(10);
  ros::Time start = ros::Time::now();
  goal = g->goal;
  distance_to_goal = distanceToGoal();
  geometry_msgs::Twist cmd;
  do {
    feedback.distance_to_goal = distance_to_goal;
    actionServer->publishFeedback(feedback);
    r.sleep();
  } while(feedback.distance_to_goal > DISTANCE_DEADBAND); 
  cmd_pub.publish(cmd);
  icmars_onboard::MoveResult res;
  res.message = "OK";
  actionServer->setSucceeded(res);
  ROS_INFO("DRIVE success");
  ActionBusy = false;
}

void goalCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  goal = *msg;
}
/* move command has a twist and a duration.  When received,
   it sets the move_twist to the desired twist and sets the move_timer 
   variable to the amount of time for that twist to be active
 */
void moveCmdCallback(const icmars_msgs::TwistDuration::ConstPtr& msg)
{
  move_time = msg->duration;
  move_twist = msg->twist;
  moving = true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_node");

  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  std::string ctlTopic, cmdTopic, goalTopic, poseTopic, statusTopic;
  nh.param<std::string>("cmd_topic", cmdTopic, "cmd_vel");
  nh.param<std::string>("goal_topic", goalTopic, "goal");
  nh.param<std::string>("pose_topic", poseTopic, "pose");
  nh.param<std::string>("ctl_topic", ctlTopic, "drive_ctl");
  nh.param<std::string>("status_topic", statusTopic, "drive_status");
  nh.param<float>("angular_vel_limit", angLimit, 0.18);
  nh.param<float>("forward_vel_limit", forwardLimit, 0.25);
  ROS_INFO_STREAM("cmd = "<<cmdTopic<<"angLimit="<<angLimit<<" forwardLimit="<<forwardLimit);

  cmd_pub = n.advertise<geometry_msgs::Twist>(cmdTopic, 1);
  status_pub = n.advertise<std_msgs::String>(statusTopic, 1);
  ros::Subscriber goal_sub = n.subscribe(goalTopic, 100, goalCallback);
  ros::Subscriber pose_sub = n.subscribe(poseTopic, 100, poseCallback);
  ros::Subscriber ctl_sub = n.subscribe(ctlTopic, 100, ctlCallback);
  ros::Subscriber move_cmd_sub = n.subscribe("move", 1, moveCmdCallback);

  Kp.linear.x = 1.0;
  Kp.angular.z = -1.0;
  Server server(nh, "MoveAction", boost::bind(&execute, _1, &server), false);
  actionServer = &server;
  server.start();

  ros::spin();
  return 0;
}
