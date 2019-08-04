#ifndef TURTLE_GRIP_H
#define TURTLE_GRIP_H
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <Servo.h>

class turtle_grip {
	public:
		turtle_grip(ros::NodeHandle *nh);
		void setup(void);
		void loop(void);
		void gripMessageCb( const geometry_msgs::Point& cmd_msg);
   
	private:
		ros::NodeHandle *nh;
		Servo grip;
		Servo wrist;
		geometry_msgs::Point status_msg;
		ros::Publisher *status_publisher;
		int grip_open_limit; //Gripper open position
		int grip_closed_limit; 
//		int fsr; //Force sensitive resistor value
		int pos; //Servo position
		int x; //State variable, 0 or 1
		int wrist_pos;
		int grip_pos;
		bool grip_open; // is gripper supposed to be open?
		int loop_count; 
};

#endif
