#include "turtle_light.h"
#include <Arduino.h>

turtle_light::turtle_light(ros::NodeHandle *nodeHandle) {
  
  nh = nodeHandle;
  //LightSensor = analog 15, Led = digital 3 
  sensorPin = A15;
  headlightPin = 39;
  lightState = 0;
  lightLevel = 0;
  loop_count = 0; 
  
}

void turtle_light::setup() {
// Can put publisher here

}

void turtle_light::loop() {

  if(loop_count++ >= 100){
    loop_count = 0;
      
    //Serial.println(lightLevel);
    lightLevel = map(analogRead(sensorPin), 80, 675, 0, 10);

    if ((lightState % 2) == 0) {
      if (lightLevel <= 4) {
        digitalWrite(headlightPin, HIGH);
      }else{
        digitalWrite(headlightPin, LOW);
      }
    }
  }
}
