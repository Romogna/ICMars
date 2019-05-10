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
#include "mars_msgs/mars_camera.h"
#include "rr_exec/TwistDuration.h"
#include <sensor_msgs/CompressedImage.h>
#include <vector>
#include <sstream>
#include <string>

class Crr_cmd {
public:
	Crr_cmd(void);
	void cmdCallback(const std_msgs::String::ConstPtr& msg);
	void frontImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
	void gripImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
	void mpu6050_temp_Callback(const std_msgs::Float32::ConstPtr& msg);
	void mpu6050_accel_Callback(const geometry_msgs::Point::ConstPtr& msg);
	void mpu6050_gyro_Callback(const geometry_msgs::Point::ConstPtr& msg);
	void lcd_button_Callback(const std_msgs::Int16::ConstPtr& msg);
	void ina219_Callback(const sensor_msgs::BatteryState::ConstPtr& msg);
	void irRangeCallback(const std_msgs::Int16::ConstPtr& msg);
	void statusTimerCallback(const ros::TimerEvent& event);
private:
	// this is the ros node
	ros::NodeHandle n;
	// these are the topics on the rosbridge to the server
	ros::Subscriber cmd_sub;
	ros::Publisher image_pub;
	ros::Publisher status_pub;
	// these are the topics to the rest of the roadrunner nodes	
	ros::Publisher motor_pub;
	ros::Publisher grip_pub;
	ros::Publisher arm_pub;
	ros::Publisher arm_grip_pub;
	ros::Publisher twist_pub;
	// sensor topic subscribers
	ros::Subscriber mpu6050_temp_sub;
	ros::Subscriber mpu6050_accel_sub;
	ros::Subscriber mpu6050_gyro_sub;
	ros::Subscriber lcd_button_sub;
	ros::Subscriber ina219_sub;

	ros::Subscriber front_camera_sub;
	ros::Subscriber grip_camera_sub;
	ros::Subscriber ir_range_sub;

	ros::Timer statusTimer;

	int front_image_count;
	int grip_image_count;
	int statusCounter;
	float mpu6050_temp;
	float Ax,Ay,Az,Gx,Gy,Gz;
	float ir_range;
	float ina219_voltage, ina219_current, ina219_percentage;
	float ultrasonic_range;
	int lcd_button;
};
// called periodically to send out a status message
// data must be formatted as a JSON string for the webpage to
// parse it correctly
void Crr_cmd::statusTimerCallback(const ros::TimerEvent& event)
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "{ \"heartbeat\" : \"" << statusCounter++ << "\"" <<
		", \"ir_range\" : \"" << ir_range << "\"" <<
		", \"ina219_voltage\" : \"" << ina219_voltage << "\"" <<
		", \"ina219_current\" : \"" << ina219_current << "\"" <<
		", \"ina219_percentage\" : \"" << ina219_percentage << "\"" <<
		", \"ultrasonic_range\" : \"" << ultrasonic_range << "\"" <<
		", \"lcd_button\" : \"" << lcd_button << "\"" <<
		", \"mpu6050_temp\" : \"" << mpu6050_temp << "\"" <<
		", \"Ax\" : \"" << Ax << "\"" <<
		", \"Ay\" : \"" << Ay << "\"" <<
		", \"Az\" : \"" << Az << "\"" <<
		", \"Gx\" : \"" << Gx << "\"" <<
		", \"Gy\" : \"" << Gy << "\"" <<
		", \"Gz\" : \"" << Gz << "\"" <<
		" }";
	msg.data = ss.str();
	status_pub.publish(msg);	
}

Crr_cmd::Crr_cmd(void) 
{
	motor_pub = n.advertise<geometry_msgs::Point>("motor_cmd",10);
	grip_pub = n.advertise<geometry_msgs::Point>("grip_cmd",10);
	arm_pub = n.advertise<geometry_msgs::Point>("arm_cmd",10);
	arm_grip_pub = n.advertise<geometry_msgs::Point>("gripper_cmd",10);
	image_pub = n.advertise<mars_msgs::mars_camera>("imageJpg",10);
	twist_pub = n.advertise<rr_exec::TwistDuration>("move",10);
	cmd_sub = n.subscribe("cmd",1000,&Crr_cmd::cmdCallback, this);
	status_pub = n.advertise<std_msgs::String>("status",1000);
	front_camera_sub = n.subscribe("/front_camera_image",10,&Crr_cmd::frontImageCallback, this);
	grip_camera_sub = n.subscribe("/grip_camera_image",10,
		&Crr_cmd::gripImageCallback, this);
	// sensor topic subscriptions
	mpu6050_temp_sub = n.subscribe("mpu6050_temp",10,&Crr_cmd::mpu6050_temp_Callback, this);
	mpu6050_accel_sub = n.subscribe("mpu6050_accels",10,
		&Crr_cmd::mpu6050_accel_Callback, this);
	mpu6050_gyro_sub = n.subscribe("mpu6050_gyro",10,
		&Crr_cmd::mpu6050_gyro_Callback, this);
	lcd_button_sub = n.subscribe("lcd_button",10,
		&Crr_cmd::lcd_button_Callback, this);
	ina219_sub = n.subscribe("battery_state",10,
		&Crr_cmd::ina219_Callback, this);

	statusTimer = n.createTimer(ros::Duration(1.0), 
		&Crr_cmd::statusTimerCallback, this);
	front_image_count = grip_image_count = 0;
	statusCounter = 0;
	mpu6050_temp = 0.0;
	Ax = Ay = Az = Gx = Gy = Gz = 0.0;
	ir_range = 0.0;
	ina219_voltage = ina219_current = ina219_percentage = 0.0;
	ultrasonic_range = 0.0;
	lcd_button = -1;
}
void Crr_cmd::ina219_Callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	ina219_voltage = msg->voltage;
	ina219_current = msg->current;
	ina219_percentage = msg->percentage;
}
void Crr_cmd::lcd_button_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	lcd_button = msg->data;
}
void Crr_cmd::mpu6050_temp_Callback(const std_msgs::Float32::ConstPtr& msg)
{
	mpu6050_temp = msg->data;
}
void Crr_cmd::mpu6050_accel_Callback(const geometry_msgs::Point::ConstPtr& msg)
{
	Ax = msg->x;
	Ay = msg->y;
	Az = msg->z;
}
void Crr_cmd::mpu6050_gyro_Callback(const geometry_msgs::Point::ConstPtr& msg)
{
	Gx = msg->x;
	Gy = msg->y;
	Gz = msg->z;
}
void Crr_cmd::frontImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
	if(++front_image_count >= 20) {
		mars_msgs::mars_camera image_msg;
		image_msg.image = *image;
		image_msg.name = std::string("raspicam");
		image_pub.publish(image_msg);
		front_image_count = 0;
	}
}
void Crr_cmd::gripImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
	if(++grip_image_count >= 20) {
		mars_msgs::mars_camera image_msg;
		image_msg.image = *image;
		image_msg.name = std::string("webcam");
		image_pub.publish(image_msg);
		grip_image_count = 0;
	}
}

void Crr_cmd::irRangeCallback(const std_msgs::Int16::ConstPtr& msg)
{
	ir_range = msg->data;
}

/* called when a String is published to this topic */
void Crr_cmd::cmdCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("rr_cmd: %s", msg->data.c_str());
	/* parse the string into tokens sep by whitespace */
	std::istringstream s(msg->data);
	std::string tmp;
	std::vector<std::string> tokens;
	while(s >> tmp) {
		tokens.push_back(tmp);
	}
	printf("Got %i tokens\n",(int)tokens.size());
	if(tokens[0] == "MT") {
		ROS_INFO("Move for a Time cmd");
		geometry_msgs::Point msg;
		float effort, time;
		sscanf(tokens[1].c_str(), "%f", &effort);
		sscanf(tokens[2].c_str(), "%f", &time);
		rr_exec::TwistDuration twistMsg;
		twistMsg.twist.linear.x = effort;
		twistMsg.duration = time;
		twist_pub.publish(twistMsg);
		
	} else if(tokens[0] == "TT") {
		ROS_INFO("Turn for a Time cmd");
		float effort, time;
		sscanf(tokens[1].c_str(), "%f", &effort);
		sscanf(tokens[2].c_str(), "%f", &time);
		rr_exec::TwistDuration twistMsg;
		twistMsg.twist.angular.z = effort;
		twistMsg.duration = time;
		twist_pub.publish(twistMsg);
	} else if(tokens[0] == "ARM") {
		ROS_INFO("Move arm joints");
		geometry_msgs::Point msg;
		sscanf(tokens[1].c_str(), "%lf", &msg.x); // base
		sscanf(tokens[2].c_str(), "%lf", &msg.y); // shoulder
		sscanf(tokens[3].c_str(), "%lf", &msg.z); // elbow
		arm_pub.publish(msg);
	} else if(tokens[0] == "ARM_GRIP") {
		ROS_INFO("Move arm wrist and gripper joints");
		geometry_msgs::Point msg;
		sscanf(tokens[1].c_str(), "%lf", &msg.x); // wrist
		sscanf(tokens[2].c_str(), "%lf", &msg.y); // rot
		sscanf(tokens[3].c_str(), "%lf", &msg.z); // grip
		arm_grip_pub.publish(msg);
	} else if(tokens[0] == "GP") {
		ROS_INFO("Gripper command");
		geometry_msgs::Point msg;
		float grip, wrist;
		sscanf(tokens[1].c_str(), "%f", &grip);
		sscanf(tokens[2].c_str(), "%f", &wrist);
		msg.x = grip;
		msg.y = wrist;
		msg.z = 0.0;
		grip_pub.publish(msg);
	}
};

/* main does nothing but subscribe and wait for callbacks */
int main(int argc, char **argv)
{
	ros::init(argc,argv,"rr_cmd");
	Crr_cmd c;
	ros::spin();
	return 0;
}
