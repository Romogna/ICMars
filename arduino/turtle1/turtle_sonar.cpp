#include "turtle_sonar.h"
#include <Arduino.h>
#include <geometry_msgs/Point.h>

turtle_sonar::turtle_sonar(ros::NodeHandle *nodeHandle) {
  
  nh = nodeHandle;
  //Left = 31, Center = 33, Right = 35
  ultraSoundSignalPins[0] = 31;
  ultraSoundSignalPins[1] = 33;  
  ultraSoundSignalPins[2] = 35;
  loop_count = 0;
  
}

void turtle_sonar::setup() {

  ultraSonic_pub = new ros::Publisher("sonar", &rangeSonar_msg);
  nh->advertise(*ultraSonic_pub);
}

void turtle_sonar::loop() {

  if(i >2) {
    i = 0;
    ultraSonic_pub->publish(&rangeSonar_msg);
  }

  if(loop_count++ >= 100){
    loop_count = 0;

      ultraSoundValue = ping(i);

      switch(i) {
        case 0:
          rangeSonar_msg.x = ultraSoundValue;
          break;
        case 1:
          rangeSonar_msg.y = ultraSoundValue;
          break;
        case 2:
          rangeSonar_msg.z = ultraSoundValue;
          break;
      }
      
    i++;
  }
}

unsigned long turtle_sonar::ping(int sensor) {

  unsigned long echo;

  pinMode(ultraSoundSignalPins[sensor], OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignalPins[sensor], LOW); // Send low pulse
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignalPins[sensor], HIGH); // Send high pulse
  delayMicroseconds(5); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignalPins[sensor], LOW); // Holdoff
  pinMode(ultraSoundSignalPins[sensor], INPUT); // Switch signalpin to input
  digitalWrite(ultraSoundSignalPins[sensor], HIGH); // Turn on pullup resistor
  echo = pulseIn(ultraSoundSignalPins[sensor], HIGH); //Listen for echo
  
  return echo / 29 / 2;

}
