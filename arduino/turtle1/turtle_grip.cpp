#include "turtle_grip.h"
#include <Arduino.h>
#define WRIST_PIN 5
#define GRIP_PIN  4

// Constructor:  called once when class instance is created
turtle_grip::turtle_grip(ros::NodeHandle *nodeHandle)
{
	nh = nodeHandle;
	x = 0; //State variable, 0 or 1
	wrist_pos = 120;
	grip_open_limit = 180; //Gripper open position
	grip_closed_limit = 60;
	grip_pos = grip_open_limit;
	grip_open = true;
	loop_count = 0;
}

void turtle_grip::gripMessageCb (const geometry_msgs::Point& cmd_msg){
	if (cmd_msg.y > 120) { //Checks if wrist angle is too high
		wrist_pos = 120;
	} else if (cmd_msg.y < 85) { 
	//Checks if wrist angle is too low, this will prevent rover "push ups"
        	wrist_pos = 85;
      	} else {
		//If within limits then write the value directly to servo
        	wrist_pos = cmd_msg.y; 
      	}
	wrist.write(wrist_pos);

	/*
	This section of code will gradually close
	the gripper to allow time for the servo
	to move into position before trying to move
	to the next position
	Grip commands are:
		1 = open
		0 = close
		-1 = unchanged
	*/
	if (cmd_msg.x == 1) {
		grip_open = true;
	} else if (cmd_msg.x == 0) {
		grip_open = false;
	}
}

void turtle_grip::setup(){
	grip.attach(GRIP_PIN);  // attaches the servo on pin 9 to the servo object
	grip.write(grip_open_limit);
	wrist.attach(WRIST_PIN);  // attaches the servo on pin 9 to the servo object
	wrist.write(wrist_pos);	// set initial wrist angle
	status_publisher = new ros::Publisher("grip_status", &status_msg);
	nh->advertise(*status_publisher);
}

void turtle_grip::loop(){
	if(loop_count++ >= 100) {
		loop_count = 0;

//	grip.write(grip_pos);

	status_msg.y = wrist_pos;
	status_msg.z = grip_pos;
	status_publisher->publish(&status_msg);
	}
}
