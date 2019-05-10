#include "ros/ros.h"
#include <std_msgs/String.h>
#include "icmars_msgs/NamedImage.h"
#include <sensor_msgs/CompressedImage.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
int count = 0;
int image_count = 0;
ros::Publisher image_pub;
ros::Publisher status_pub;
geometry_msgs::PoseWithCovarianceStamped latestPose; 

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  latestPose = *msg;
}
void frontImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
        if(++image_count >= 5) {
                icmars_msgs::NamedImage image_msg;
                image_msg.image = *image;
                image_msg.name = std::string("raspicam");
                image_pub.publish(image_msg);
               image_count = 0;
        }
}
float yawFromQ(geometry_msgs::Quaternion &q)  
{ 
  return atan2(2.0 * (q.z * q.w + q.x * q.y) ,  
      - 1.0 + 2.0 * (q.w * q.w + q.x * q.x)); 
}
void timerCallback(const ros::TimerEvent&)
{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "{\"heartbeat\" : \"" << count++ << "\"" <<
		", \"pose_x\" : \"" << latestPose.pose.pose.position.x << "\"" <<
		", \"pose_y\" : \"" << latestPose.pose.pose.position.y << "\"" <<
		", \"pose_z\" : \"" << latestPose.pose.pose.position.z << "\"" <<
		", \"pose_yaw\" : \"" << yawFromQ(latestPose.pose.pose.orientation) << "\"" << 
		" }";
		msg.data = ss.str();
		status_pub.publish(msg);	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mars_pi_node");
	ros::NodeHandle n;
	status_pub = n.advertise<std_msgs::String>("status",1000);
	image_pub = n.advertise<icmars_msgs::NamedImage>("imageJpg",1);

  ros::Subscriber camera_sub = n.subscribe("image/compressed",10,
		frontImageCallback);

	ros::Subscriber pose_sub = n.subscribe("pose",1000, poseCallback);
	ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
	ros::spin();
	return 0;
}

