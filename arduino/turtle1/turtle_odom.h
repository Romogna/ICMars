#ifndef TURTLE_ODOM_H
#define TURTLE_ODOM_H
#include <ros.h>
#include <geometry_msgs/Twist.h>

class turtle_odom {
  public:
    turtle_odom(ros::NodeHandle *nh);
    void setup(void);
    void loop(void);
    static void leftEncoderEvent();
    static void rightEncoderEvent();

  private:
    ros::NodeHandle *nh;
    geometry_msgs::Twist odom;
    ros::Publisher *odomPub;
    int loop_count;
};

#endif
