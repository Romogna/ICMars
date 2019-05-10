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
#include "icmars_onboard/MoveAction.h"
#include "icmars_onboard/PickupAction.h"

class Image {
  public:
    // state machine for overall control
    enum class States {
      TIMEOUT, IDLE, MOVING, DRIVING, GRIPPER, PICKUP
    };

    Image(void);
    void navImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void cabooseImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void bellyImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void tagImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
    void chestImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image);
  private:
    // this is the ros node
    ros::NodeHandle n;
    // these are the topics on the rosbridge to the server
    ros::Publisher image_pub;
    ros::Subscriber nav_camera_sub;
    ros::Subscriber caboose_camera_sub;
    ros::Subscriber tag_camera_sub;
    ros::Subscriber chest_camera_sub;
    ros::Subscriber belly_camera_sub;
};

Image::Image(void) 
{
  image_pub = n.advertise<icmars_msgs::NamedImage>("imageJpg",10);
  nav_camera_sub = n.subscribe("/nav_camera_image",10,&Image::navImageCallback, this);
  caboose_camera_sub = n.subscribe("/caboose_camera_image",10,&Image::cabooseImageCallback, this);
  tag_camera_sub = n.subscribe("/tag_camera_image",10,
      &Image::tagImageCallback, this);
  chest_camera_sub = n.subscribe("/chest_camera_image",10,
      &Image::chestImageCallback, this);
  belly_camera_sub = n.subscribe("/belly_camera_image",10,
      &Image::bellyImageCallback, this);
}
void Image::navImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("navcam");
    image_pub.publish(image_msg);
}
void Image::cabooseImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("caboosecam");
    image_pub.publish(image_msg);
}
void Image::tagImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("tagscam");
    image_pub.publish(image_msg);
}
void Image::bellyImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("bellycam");
    image_pub.publish(image_msg);
}
void Image::chestImageCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
{
    icmars_msgs::NamedImage image_msg;
    image_msg.image = *image;
    image_msg.name = std::string("chestcam");
    image_pub.publish(image_msg);
}
/* main does nothing but subscribe and wait for callbacks */
int main(int argc, char **argv)
{
  ros::init(argc,argv,"image");
  Image img;
  ros::spin();
  return 0;
}
