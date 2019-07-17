/***
  Node to handle incoming commands from the rosbridge and interpret
  them into messages for the robot's ros topics.
  This is basically glue logic so that the cmd topic can be generic
  and the webserver/websocket client can just pass strings without
  knowing their contents.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/BatteryState.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "icmars_msgs/NamedImage.h"
#include "icmars_msgs/TwistDuration.h"
#include <sensor_msgs/CompressedImage.h>
#include <vector>
#include <sstream>
#include <queue>
#include <string>
#include <tf/transform_datatypes.h>
#include "rapidjson/document.h"
#include <actionlib/client/simple_action_client.h>
#include "icmars_onboard/MoveActionAction.h"

typedef actionlib::SimpleActionClient<icmars_onboard::MoveActionAction> Client;
class Cmd {
  public:
    // state machine for overall control
    enum class States {
      TIMEOUT, IDLE, MOVING, DRIVING, GRIPPER
    };

    Cmd(void);
    void cmdCallback(const std_msgs::String::ConstPtr& msg);
    void driveStatusCallback(const std_msgs::String::ConstPtr& msg);
    void navImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void cabooseImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void bellyImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void tagImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void chestImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void statusTimerCallback(const ros::TimerEvent& event);
    void loopTimerCallback(const ros::TimerEvent& event);
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
  private:
    Client ac;
    void parseCommand(std::string json_str);
    // this is the ros node
    ros::NodeHandle n;
    // these are the topics on the rosbridge to the server
    ros::Subscriber cmd_sub;
    ros::Publisher image_pub;
    ros::Publisher status_pub;
    // these are the topics to the rest of the roadrunner nodes	
    ros::Publisher motor_pub;
    ros::Publisher grip_finger_pub;
    ros::Publisher grip_wrist_pub;
    ros::Publisher twist_pub;
    // motor driver interface topics 
    ros::Publisher drive_control_pub;
    ros::Publisher goal_pub;
    ros::Subscriber drive_status_sub;

    ros::Subscriber nav_camera_sub;
    ros::Subscriber caboose_camera_sub;
    ros::Subscriber tag_camera_sub;
    ros::Subscriber chest_camera_sub;
    ros::Subscriber belly_camera_sub;
    ros::Subscriber pose_sub;

    ros::Timer statusTimer;
    ros::Timer loopTimer;

    std::string last_drive_status;
    int nav_image_count = 0;
    int caboose_image_count = 0;
    int statusCounter;
//    icmars_msgs::TwistDuration twisties;
    geometry_msgs::Pose2D latestPose;
    States state, next_state;
    std::queue<std::string> cmd_queue;
    ros::Duration timeout;
    ros::Duration energy;
    ros::Duration state_time;
    ros::Time state_start_time;
    std::string current_cmd;
    std::string cmd_status;
};
// called periodically to send out a status message
// data must be formatted as a JSON string for the webpage to
// parse it correctly
float yawFromQ(geometry_msgs::Quaternion &q)  
{ 
  return atan2(2.0 * (q.z * q.w + q.x * q.y) ,  
      - 1.0 + 2.0 * (q.w * q.w + q.x * q.x)); 
}

void Cmd::statusTimerCallback(const ros::TimerEvent& event)
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "{ \"heartbeat\":" << statusCounter++ << 
    ", \"pose\":{\"x\":" << latestPose.x  <<
    ", \"y\":" << latestPose.y << 
    ", \"theta\":" << latestPose.theta << "}" <<
    ", \"drive_status\":\"" << last_drive_status << "\"" <<
    ", \"cmd_status\":\"" << cmd_status << "\"" <<
    "}";
  msg.data = ss.str();
  status_pub.publish(msg);
}

Cmd::Cmd(void) : ac("drive_node/MoveAction", true)
{
  motor_pub = n.advertise<geometry_msgs::Point>("motor",10);
  grip_finger_pub = n.advertise<std_msgs::Float32>("/simEva_gripper/fingerAngle/cmd",10);
  grip_wrist_pub = n.advertise<std_msgs::Float32>("/simEva_gripper/wristAngle/cmd",10);
  goal_pub = n.advertise<geometry_msgs::Pose2D>("goal",10);
  drive_control_pub = n.advertise<std_msgs::String>("drive_ctl",10);
  image_pub = n.advertise<icmars_msgs::NamedImage>("imageJpg",10);
  twist_pub = n.advertise<icmars_msgs::TwistDuration>("move",10);
  cmd_sub = n.subscribe("cmd",1000,&Cmd::cmdCallback, this);
  pose_sub = n.subscribe("pose",1000,&Cmd::poseCallback, this);
  status_pub = n.advertise<std_msgs::String>("status",1000);
  drive_status_sub = n.subscribe("drive_status",10,
      &Cmd::driveStatusCallback, this);
  nav_camera_sub = n.subscribe("/nav_camera_image",10,&Cmd::navImageCallback, this);
  caboose_camera_sub = n.subscribe("/caboose_camera_image",10,&Cmd::cabooseImageCallback, this);
  tag_camera_sub = n.subscribe("/tag_camera_image",10,
      &Cmd::tagImageCallback, this);
  chest_camera_sub = n.subscribe("/chest_camera_image",10,
      &Cmd::chestImageCallback, this);
  belly_camera_sub = n.subscribe("/belly_camera_image",10,
      &Cmd::bellyImageCallback, this);


  statusTimer = n.createTimer(ros::Duration(1.0), 
      &Cmd::statusTimerCallback, this);
  loopTimer = n.createTimer(ros::Duration(0.1), 
      &Cmd::loopTimerCallback, this);
  nav_image_count = caboose_image_count = 0;
  statusCounter = 0;
  state = next_state = States::IDLE;
  state_start_time = ros::Time::now();
  timeout = ros::Duration(0.0);
}
void Cmd::driveStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  last_drive_status = msg->data;
}
void Cmd::navImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("navcam");
    image_pub.publish(image_msg);
}
void Cmd::cabooseImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("caboosecam");
    image_pub.publish(image_msg);
}
void Cmd::tagImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("tagscam");
    image_pub.publish(image_msg);
}
void Cmd::bellyImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("bellycam");
    image_pub.publish(image_msg);
}
void Cmd::chestImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("chestcam");
    image_pub.publish(image_msg);
}

void Cmd::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  latestPose = *msg;
}

/* called when a string is published to this topic
   commands are JSON strings so they must be parsed to extract
   their contents.
 */
void Cmd::cmdCallback(const std_msgs::String::ConstPtr& msg)
{
  cmd_queue.push(msg->data);
  ROS_INFO_STREAM("Got a cmd now have " << cmd_queue.size());
}

void Cmd::parseCommand(std::string json_str) {
  ROS_INFO_STREAM("cmd: " << json_str);
  rapidjson::Document d;
  d.Parse(json_str.c_str());
  if(d.HasParseError()) {
    ROS_ERROR_STREAM("JSON string could not be parsed: " << json_str);
    return;
  }

  // All commands must have a "cmd" field
  std::string cmd;
  if(d.HasMember("cmd")) {
    cmd = d["cmd"].GetString();
  } else {
    ROS_ERROR_STREAM("Invalid JSON command " << json_str);
    return;
  }

  if(d.HasMember("timeout")) {
    timeout = ros::Duration(d["timeout"].GetDouble());
    ROS_INFO_STREAM("timeout is " << timeout);
  } else {
    timeout = ros::Duration(0.0);
  }

  ROS_INFO_STREAM("Command is " << cmd);
  if(cmd == "goto") {
    geometry_msgs::Pose2D pose;
    if(d.HasMember("pose")) {
      if(d["pose"].HasMember("x")) {
        pose.x = d["pose"]["x"].GetDouble();
      }
      if(d["pose"].HasMember("y")) {
        pose.y = d["pose"]["y"].GetDouble();
      }
      if(d["pose"].HasMember("theta")) {
        pose.theta = d["pose"]["theta"].GetDouble();
      }
    }
    ROS_INFO_STREAM("pose is " << pose);
        goal_pub.publish(pose);
    icmars_onboard::MoveActionGoal g;
    g.goal = pose;
    ac.sendGoal(g);
    next_state = States::DRIVING;

  } else if(cmd == "move") {
    std::string direction;

      if(d.HasMember("direction")) {
        direction = d["direction"].GetString();
      } else {
        ROS_ERROR_STREAM("move command needs a direction! " << json_str);
        return;
        }

      double time;

      if(d.HasMember("time")) {
        time = d["time"].GetDouble();
      } else {
        ROS_ERROR_STREAM("move command needs a time! " << json_str);
        return;
        }

      float speed=0.2;
      icmars_msgs::TwistDuration twistMsg;
      twistMsg.duration = time;
      energy = ros::Duration(time);

      if(direction == "forward") {
        twistMsg.twist.linear.x = speed;
      } else if(direction == "back") {
        twistMsg.twist.linear.x = -speed;
      } else if(direction == "left") {
        twistMsg.twist.angular.z = speed;
      } else if(direction == "right") {
        twistMsg.twist.angular.z = -speed;
      } else if(direction == "stop") {
        twistMsg.twist.linear.x = 0.0;
        twistMsg.twist.angular.z = 0.0;
        twistMsg.duration = 1.0;
      } else {
        ROS_ERROR_STREAM("move command has invalid direction! " << json_str);
        return;
        }

      twist_pub.publish(twistMsg);
      next_state = States::MOVING;

  } else if(cmd == "stop") {
    for(int i = cmd_queue.size(); i == 0; i--) {
      cmd_queue.pop();
    }
    if(cmd_queue.empty()) {
      std_msgs::String Msg;
      Msg.data = "CLEAR";
      drive_control_pub.publish(Msg);
      next_state = States::IDLE;
    }
  } else if(cmd == "gripper") {
    if(d.HasMember("finger")) {
      std_msgs::Float32 m;
      m.data = d["finger"].GetDouble();
      grip_finger_pub.publish(m);
    }
    if(d.HasMember("wrist")) {
      std_msgs::Float32 m;
      m.data = d["wrist"].GetDouble();
      grip_wrist_pub.publish(m);
    }
    next_state = States::GRIPPER;
  }
}
/*
   this is the state machine that controls the operation and 
   sequence of behaviours for the rover
 */
void Cmd::loopTimerCallback(const ros::TimerEvent& event)
{
  if(state != next_state) {
    state = next_state;
    state_start_time = ros::Time::now();
  }
//  state_time = state_start_time - ros::Time::now();
  state_time = ros::Time::now() - state_start_time;
  if(timeout.toSec() > 0.0 && state_time > timeout) {
    ROS_INFO_STREAM("Timeout State");
    state = States::TIMEOUT;
  }
  switch(state) {
    case States::TIMEOUT:
      cmd_status = "TIMEOUT";
      break;
    case States::IDLE:
      if(!cmd_queue.empty()) {
        current_cmd = cmd_queue.front();
        cmd_queue.pop();
        parseCommand(current_cmd);
      }
      cmd_status = "IDLE";
      break;
    case States::DRIVING:
      {
        actionlib::SimpleClientGoalState s = ac.getState();
        if(s == actionlib::SimpleClientGoalState::ACTIVE) {
          cmd_status = "DRIVING, ACTIVE";
        } else if(s == actionlib::SimpleClientGoalState::PENDING) {
          cmd_status = "DRIVING, PENDING";
        } else if(s == actionlib::SimpleClientGoalState::SUCCEEDED) {
          cmd_status = "DRIVING, SUCCEEDED";
          next_state = States::IDLE;
        } else {
          cmd_status = "DRIVING, other";
        }
/*
        if(s == actionlib::SimpleClientGoalState::ACTIVE) {
          cmd_status = "DRIVING ";
        } else {
          next_state = States::IDLE;
        }
*/
      }
      break;
    case States::MOVING:

      // TODO need to check feedback
      if(state_time<energy) {
        ROS_INFO_STREAM(" Duration = "<<state_time<<" Timeout = "<<energy);
        cmd_status = "MOVING";
      } else {
        next_state = States::IDLE;
      }
        break;
    case States::GRIPPER:
      {
        cmd_status = "GRIPPER";
        // TODO need to check feedback
        next_state = States::IDLE;
      }
      break;
  }
  cmd_status = cmd_status + " cmds: " + std::to_string(cmd_queue.size());
}
/* main does nothing but subscribe and wait for callbacks */
int main(int argc, char **argv)
{
  ros::init(argc,argv,"cmd");
  Cmd c;
  ros::spin();
  return 0;
}
