#ifndef TURTLE_SONAR_H
#define TURTLE_SONAR_H
#include <ros.h>
#include <geometry_msgs/Point.h>

class turtle_sonar {
  public:
    turtle_sonar(ros::NodeHandle *nh);
    void setup(void);
    void loop(void);
    unsigned long ping(int sensor);
    int i;

  private:
    ros::NodeHandle *nh;
    ros::Publisher *ultraSonic_pub;
    geometry_msgs::Point rangeSonar_msg;
    
    int ultraSoundSignalPins[3];
    unsigned long ultraSoundValue;
    int loop_count;
};

#endif
