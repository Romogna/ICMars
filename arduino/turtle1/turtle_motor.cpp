#include "turtle_motor.h"
#include <Arduino.h>
#include <geometry_msgs/Twist.h>

turtle_motor::turtle_motor(ros::NodeHandle *nodeHandle)
{
	nh = nodeHandle;
	loopCount = 0;
	dir[0] = 12;
	dir[1] = 13;
	pwm[0] = 3;
	pwm[1] = 11;
	//brake[0] = 9;
	//brake[1] = 8;
	cur[0] = A0;
	cur[1] = A1;

	// set the directions for the left and right wheels
	DIR_FWD[0] = HIGH;
	DIR_FWD[1] = LOW;
	DIR_REV[0] = LOW;
	DIR_REV[1] = HIGH;
	DIR_R[0] = HIGH;
	DIR_R[1] = HIGH;
	DIR_L[0] = LOW;
	DIR_L[1] = LOW;
	commandTimeoutMs = 50;
	loopMs = 50;
	efforts[0] = efforts[1] = 0;
}

void turtle_motor::motorMessageCb( const geometry_msgs::Twist& cmd_msg){
	efforts[0] = (int)(cmd_msg.linear.x * 200.0);
	efforts[1] = (int)(cmd_msg.angular.z * 200.0);
	// make sure the values are reasonable
	for(int i=0;i<2;++i) {
		if(efforts[i]<-100) efforts[i]=-100;
		if(efforts[i]>100) efforts[i]=100;
	}
	commandTimeout = commandTimeoutMs;
}

void turtle_motor::writeMotors(void)
{
	if ((efforts[0] == 0) & (efforts[1] == 0)) {
		for(int i=0;i<2;++i) {
			//digitalWrite(brake[i], HIGH);
			analogWrite(pwm[i], 0);
		}
	} else {
		for(int i=0;i<2;++i) {
			//digitalWrite(brake[i], LOW);
			if(efforts[0] < 0) {
				digitalWrite(dir[i], DIR_REV[i]);
				// the 2.55 is to scale 100% to full scale on pwm (255)
				analogWrite(pwm[i], -(int)((float)efforts[0]*2.55));
			}
			else if(efforts[0] > 0) {
				digitalWrite(dir[i], DIR_FWD[i]);
				analogWrite(pwm[i], (int)((float)efforts[0]*2.55));
      			}
			//Needs to enable skidsteering for low effort turns
      			else if(efforts[1] < 0) { //left
        			digitalWrite(dir[i], DIR_L[i]);
      				analogWrite(pwm[i], -(int)((float)efforts[1]*2.55));
      			}
      			else if(efforts[1] > 0) { //right
      				digitalWrite(dir[i], DIR_R[i]);
      				analogWrite(pwm[i], (int)((float)efforts[1]*2.55));
      			}
 		}
	}
}

void turtle_motor::setup(void)
{
	for(int i=0;i<2;++i) {
		pinMode(dir[i], OUTPUT);
		pinMode(pwm[i], OUTPUT);
		analogWrite(pwm[i], 100);
		//pinMode(brake[i], OUTPUT);
	}
	status_publisher = new ros::Publisher("motor_status", &status_msg);
	nh->advertise(*status_publisher);
}

void turtle_motor::loop(void)
{
	// only run every so many milliseconds
	if(loopCount++ >= loopMs) {
		loopCount = 0;
		//if(commandTimeout > 0) {
		//	if(--commandTimeout <= 0) {
		//		efforts[0]=efforts[1]=0;
		//	}
		//}
		status_msg.linear.x = efforts[0];
		status_msg.angular.z = efforts[1];
//		status_msg.angular.z = commandTimeout;
		status_publisher->publish(&status_msg);
	}
	writeMotors();
}
