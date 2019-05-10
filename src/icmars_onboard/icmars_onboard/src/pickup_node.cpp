#include "ros/ros.h"
#include "std_msgs/String.h"
#include "angles/angles.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "icmars_msgs/TwistDuration.h"
#include "icmars_onboard/PickupAction.h"
#include <actionlib/server/simple_action_server.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <queue>
#include <iostream>

#define ANGLE_DEADBAND 0.1
#define DISTANCE_DEADBAND 0.1

#include "sipi_controller/PickUpController.h"
#include <iostream>
#include "sipi_controller/GripperController.h"
// offset to center of screen in X
#define X_OFFSET 0.054
#define X_TOLERANCE 0.02
//threshold distance to be from the target block before attempting pickup
#define DIST_TOLERANCE 0.05
// how far away should the cube be before just going forward
#define PICKUP_DISTANCE 0.25 
// how many times can you not see a target before deciding it is out of view
// this is to prevent state changes because of blurs
#define NUM_ALLOWED_MISSES 10
// timing constants
#define PICKUP_CENTER_TIMEOUT 2.0 // how long to try to center target
#define PICKUP_TURN_CONSTANT 0.2		// how fast to turn open loop
Result execute(
    int obstacleDetection,
    const std::vector<geometry_msgs::Pose2D> targets
    ); 

void reset();
bool ignore_cubes(void) {return ignore_cubes_;};
enum class ResultCode : int {
  SUCCESS = 0,
  BUSY,
  FAILED
};

enum class State : int {
  IDLE,	// starting point
  CENTER,	// turn so that cube is in center
  DISTANCE,	//move to the correct distance
  FORWARD,	// go forward to scoop up cube
  PICKUP,		// pick up cube
  VERIFY,		// check if successful
  BACKUP		// back up a little to reacquire lost target
};
struct Result {
  geometry_msgs::Twist cmd_vel;
  CGripCmd grip;
  bool pickedUp;
  bool giveUp;
  State state;
  State nextState;
  ResultCode result;
  std::string status;
};
void targetHandler(
    const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);

//yaw angle to target block 
double blockYaw;
//distance to target block from front of robot
double blockDist;
double distErr;

//struct for returning data to mobility
Result result;

ros::Time stateStartTime; // start time for states
ros::Duration stateRunTime; // time since last state change
geometry_msgs::Pose tagPose;
int missedTargetCount;
// flag to indicate that the target is no longer visible
bool targetLost;
int fail_count;
bool ignore_cubes_;
apriltags_ros::AprilTagDetectionArray tagDetectionArray;


void reset() {
  result.cmd_vel.linear.x = 0;
  result.cmd_vel.angular.z = 0;
  result.grip.fingersOpen = true;
  result.grip.wristPos = WRIST_UP;
  result.state = State::IDLE;
  result.nextState = State::IDLE;
  stateStartTime = ros::Time::now();
  targetLost = true;
  fail_count = 0;
  ignore_cubes_ = false;
}

bool selectNearestTarget(
    const std::vector<geometry_msgs::Pose2D>& targets,
    double &blockDist, double &blockYaw)
{ 
  if (targets.empty()) return false;
  double min_dist_sq = std::numeric_limits<double>::max();
  //this loop selects the closest visible block 
  geometry_msgs::Pose2D closest_target;
  for ( auto &t : targets) {
    float d_sq = t.x*t.x + t.y*t.y;
    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
      closest_target = t;
    }
  }
  blockDist = sqrt(min_dist_sq);
  //angle to block from center of chassis 
  blockYaw = atan2(closest_target.y, closest_target.x); 
  return true;
}

void loop(void) 
{
  // default is to send no velocity and be BUSY
  geometry_msgs::Twist cmd;
  // time since last state change
  if(result.nextState != result.state) {
    stateStartTime =  ros::Time::now();
    result.state = result.nextState;
  }
  stateRunTime = ros::Time::now() - stateStartTime;
  // this checks if it can see a target and gives its position
  bool targetVisible = selectNearestTarget(targets, blockDist, 
      blockYaw);
  // this allows the target to dissappear briefly but not consider it lost
  // at first (reset) targetLost is true. but then it only becomes true
  // again if the target is missed multiple times
  if(targetVisible) {
    missedTargetCount = 0;
    targetLost = false;
    distErr = blockDist - PICKUP_DISTANCE;
  } else {
    missedTargetCount++;
    if(missedTargetCount > NUM_ALLOWED_MISSES) {
      targetLost = true;
    }
  }
  // how far are we from the drive forward point
  switch(result.state) {
    case State::IDLE:
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_UP;
      if(stateRunTime > ros::Duration(1.0)) {
        if(targetLost) {
          ROS_WARN("PICKUP target lost from IDLE!");
          result.nextState = State::BACKUP;
        } else {
          result.nextState = State::CENTER;
        }
      }
      break;
    case State::CENTER:
      // turn until selected block is centered in front
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_DOWN;
      result.cmd_vel.angular.z = limit(blockYaw*2.0, 0.2);
      if(stateRunTime > ros::Duration(3.0)) {
        if(targetLost) {
          ROS_WARN("PICKUP target lost!");
          result.nextState = State::BACKUP;
        } else {
          result.nextState = State::DISTANCE;
        }
      }
      break;
    case State::DISTANCE:
      // move until cube is at right distance
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_DOWN;
      result.cmd_vel.linear.x = limit(distErr, 0.15);
      result.cmd_vel.angular.z = limit(blockYaw, 0.2);
      if(stateRunTime > ros::Duration(4) || fabs(distErr) < 0.05) {
        if(targetLost) {
          result.nextState = State::BACKUP;
        } else {
          result.nextState = State::FORWARD;
        }
      }
      break;
    case State::FORWARD:
      // drive forward until block should be in gripper
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_DOWN;
      result.cmd_vel.linear.x = 0.1;
      if(stateRunTime >= ros::Duration(1.0)) {
        result.nextState = State::PICKUP;
      }
      break;
    case State::PICKUP:
      result.grip.fingersOpen = false;
      result.grip.wristPos = WRIST_DOWN;
      if(stateRunTime >= ros::Duration(1.0)) {
        result.nextState = State::VERIFY;
      }
      break;
    case State::VERIFY:
      result.grip.fingersOpen = false;
      result.grip.wristPos = WRIST_VERIFY;
      if(obstacleDetected == 4 || (blockDist < 0.2 && targetVisible)) { 
        result.result =ResultCode::SUCCESS;
        result.nextState = State::IDLE;
      } else if(stateRunTime > ros::Duration(1.5)) {
        result.nextState = State::BACKUP;
      }
      break;
    case State::BACKUP:
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_UP;
      result.cmd_vel.linear.x = -0.1;
      if(stateRunTime >= ros::Duration(2.0)) {
        if(targetLost) {
          result.result = ResultCode::FAILED;
          result.nextState = State::IDLE;
        } else {
          if(fail_count++ > 3) {
            ignore_cubes_ = true;
            result.result = ResultCode::FAILED;
            result.nextState = State::IDLE;
          } else {
            result.nextState = State::IDLE;
          }
        }
      }
      break;
  }
  // create a status string
  std::ostringstream ss;
  ss << " state: " << (int)result.state <<
    std::setprecision(1) <<" time: " << stateRunTime <<
    " nextState "<< (int)result.nextState  <<
    " result= " << (int)result.result <<
    " obstacle= "<< obstacleDetected <<
    " visible= "<< targetVisible <<
    std::setprecision(2) <<
    " cube pos= "<<blockDist<<","<<blockYaw << 
    ") missed="<<missedTargetCount <<
    " distErr="<<distErr<<
    " vel="<<result.cmd_vel.linear.x<<","<<result.cmd_vel.angular.z;
  result.status = ss.str();
  return result;
  cmd_vel_pub.publish(cmd);
}

void getTagsVector( 
    const apriltags_ros::AprilTagDetectionArray& targets, 
    std::vector<geometry_msgs::Pose2D> &home_tags,
    int tag_id
    )
{
  geometry_msgs::Pose2D tagPose;
  geometry_msgs::Pose p;
  home_tags.clear();
  for (auto det : targets.detections)
  {
    if(det.id == tag_id) {
      p = det.pose.pose;
      tagPose.x = p.position.z;
      tagPose.y = -p.position.x;
      tagPose.theta = thetaFromQuat(p.orientation);
      home_tags.push_back(tagPose);
    }
  }
}

void execute(const icmars_onboard::PickupGoalConstPtr& g, Server* as)
{
  ROS_INFO("PICKUP start");
  ActionBusy = true;
  ros::Rate r(10);
  ros::Time start = ros::Time::now();
  do {
    feedback.distance_to_goal = distance_to_goal;
    actionServer->publishFeedback(feedback);
    r.sleep();
  } while(feedback.distance_to_goal > DISTANCE_DEADBAND); 
  cmd_pub.publish(cmd);
  icmars_onboard::MoveResult res;
  res.message = "OK";
  actionServer->setSucceeded(res);
  ROS_INFO("PICKUP success");
  ActionBusy = false;
}
void targetHandler(
    const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
  // store the detection data
  tagDetectionArray = *message;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pickup_node");

  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  nh.param<std::string>("tag_topic", tagTopic, "tags");
  Server server(nh, "PickupAction", boost::bind(&execute, _1, &server), false);
  ros::Subscriber tag_sub = n.subscribe(tagTopic, 1, tagCallback);
  actionServer = &server;
  server.start();

  ros::spin();
  return 0;
}
