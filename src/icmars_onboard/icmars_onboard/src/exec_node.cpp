/***
  Node to handle Twist messages and convert them into 
  commnad for the motor driver.
 */
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "icmars_msgs/TwistDuration.h"
#include "rapidjson/document.h"

class Crr_exec {
	public:
		Crr_exec(void);
		void moveCmdCallback(const icmars_msgs::TwistDuration::ConstPtr& msg);
		void loopTimerCallback(const ros::TimerEvent& event);
	private:
		// this is the ros node
		ros::NodeHandle n;
		// these are the topics on the rosbridge to the server
		ros::Subscriber move_cmd_sub;
		ros::Publisher status_pub;
		// these are the topics to the rest of the roadrunner nodes	
		ros::Publisher motor_pub;
		ros::Timer loopTimer;
		icmars_msgs::TwistDuration moveCmd;
		geometry_msgs::Twist motorCmd;
		float commandX, commandY;
		int moveCommandTimer;
		float loopRate;
};
Crr_exec::Crr_exec(void) 
{
	loopRate = 10.0;  // loop rate hertz
	motor_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);
	move_cmd_sub = n.subscribe("move",1000,&Crr_exec::moveCmdCallback, this);
	loopTimer = n.createTimer(ros::Duration(1.0/loopRate), 
			&Crr_exec::loopTimerCallback, this);
	moveCommandTimer = 0;
}
void Crr_exec::loopTimerCallback(const ros::TimerEvent& event)
{
	if(moveCommandTimer > 0) {
		if(--moveCommandTimer <= 0) {
			motorCmd.linear.x = motorCmd.angular.z = 0.0;
		}
	}
	//motor_pub.publish(motorCmd);
}
/* main does nothing but subscribe and wait for callbacks */
int main(int argc, char **argv)
{
	ros::init(argc,argv,"exec_node");
	Crr_exec c;
	ros::spin();
	return 0;
}
