#ifndef TURTLE_LIGHT_H
#define TURTLE_LIGHT_H
#include <ros.h>


class turtle_light {
  public:
    turtle_light(ros::NodeHandle *nh);
    void setup(void);
    void loop(void);

  private:
    ros::NodeHandle *nh;
    ros::Publisher *ultraSonic_pub;
    
    int headlightPin;
    int sensorPin;
    int lightLevel;
    int lightState;
    int loop_count;
};

#endif
