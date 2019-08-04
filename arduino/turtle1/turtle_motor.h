#ifndef TURTLE_MOTOR_H
#define TURTLE_MOTOR_H
#include <ros.h>
#include <geometry_msgs/Twist.h>

class turtle_motor {
	public:
		turtle_motor(ros::NodeHandle *nh);
		void setup(void);
		void loop(void);
		void motorMessageCb( const geometry_msgs::Twist& cmd_msg);
    
	private:
		ros::NodeHandle *nh;
    geometry_msgs::Twist status_msg;
    ros::Publisher *status_publisher;
		void writeMotors(void);
		// map the pins to the motor shield functions
		int dir[2];
		int pwm[2];
		int brake[2];
		int cur[2];
		// set the directions for the left and right wheels
		int DIR_FWD[2];
		int DIR_REV[2];
    int DIR_R[2];
    int DIR_L[2];
		int efforts[2];
		/* commandTimeout is used as a timer to allow commands
		   to act over a limited time
		   this is necessary in case communication
		   is lost, the motors must 
		   automatically stop after some time
		 */
		int commandTimeout;
		int commandTimeoutMs;
	  float loopCount;
		int loopMs;

};

#endif
