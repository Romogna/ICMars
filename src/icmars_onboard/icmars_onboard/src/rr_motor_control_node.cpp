/***
  Node to handle Twist messages and convert them into 
  command for the motor driver node.

  It also takes the two Float32 nodes and converts them 
  into a single point command for the arduino.
 */
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

double x_hold = 0;
double y_hold = 0;

class Crr_motor_control {
	public:
		Crr_motor_control(void);
		void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
		void fingerCallback(const std_msgs::Float32::ConstPtr& msg);
		void wristCallback(const std_msgs::Float32::ConstPtr& msg);
	private:
		// this is the ros node
		ros::NodeHandle n;
		ros::Subscriber cmd_sub;
                ros::Subscriber grip_sub;
                ros::Subscriber wrist_sub;

		ros::Publisher motor_pub;
                ros::Publisher gripper_pub;
		geometry_msgs::Point cmd;
                geometry_msgs::Point angles;
};
Crr_motor_control::Crr_motor_control(void) 
{
	motor_pub = n.advertise<geometry_msgs::Point>("in_motor_cmd",10);
        gripper_pub = n.advertise<geometry_msgs::Point>("gripper_cmd",10);

	cmd_sub = n.subscribe("cmd_vel",1000,
		&Crr_motor_control::cmdCallback, this);
        grip_sub = n.subscribe("/simEva_gripper/fingerAngle/cmd",10,
                &Crr_motor_control::fingerCallback, this);
        wrist_sub = n.subscribe("/simEva_gripper/wristAngle/cmd",10,
                &Crr_motor_control::wristCallback ,this);
}
void Crr_motor_control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if(msg->linear.x != 0) {
		cmd.x = cmd.y = msg->linear.x;
	} else {
		cmd.x = msg->angular.z;
		cmd.y = -msg->angular.z;
	}
	motor_pub.publish(cmd);
}

void Crr_motor_control::fingerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angles.y = y_hold;
    angles.x = msg->data;
    x_hold = angles.x;
    gripper_pub.publish(angles);
}

void Crr_motor_control::wristCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angles.x = x_hold;
    angles.y = msg->data;
    y_hold = angles.y;
    gripper_pub.publish(angles);
}

/* main does nothing but subscribe and wait for callbacks */
int main(int argc, char **argv)
{
	ros::init(argc,argv,"rr_motor_control_node");
	Crr_motor_control c;
	ros::spin();
	return 0;
}
