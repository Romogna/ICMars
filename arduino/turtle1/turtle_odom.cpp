#include "turtle_odom.h"
#include <Arduino.h>

const byte rf_encoder_A = 19;
const byte rf_encoder_B = 28;
const byte lf_encoder_A = 18;
const byte lf_encoder_B = 24;

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

turtle_odom::turtle_odom(ros::NodeHandle *nodeHandle)
{
  nh = nodeHandle;
  loop_count = 0;
}

void turtle_odom::setup(void){
  
  pinMode(rf_encoder_A, INPUT_PULLUP);
  pinMode(rf_encoder_B, INPUT_PULLUP);
  pinMode(lf_encoder_A, INPUT_PULLUP);
  pinMode(lf_encoder_B, INPUT_PULLUP);
  
  attachInterrupt(5, leftEncoderEvent, RISING);
  attachInterrupt(4, rightEncoderEvent, RISING);
  
  odomPub = new ros::Publisher("odom", &odom);
  nh->advertise(*odomPub);
}

void turtle_odom::loop(){
  if(loop_count++ >= 100){
    loop_count = 0;
  
    odom.linear.x = leftCount;
    odomPub->publish(&odom);
  }
}

// encoder event for the interrupt call
void turtle_odom::leftEncoderEvent(){
  if (digitalRead(lf_encoder_A) == HIGH){
    if (digitalRead(lf_encoder_B) == LOW){
      leftCount--;
    } else{
      leftCount++;
    }
  }
}

// encoder event for the interrupt call
void turtle_odom::rightEncoderEvent(){
  if (digitalRead(rf_encoder_A) == HIGH){
    if (digitalRead(rf_encoder_B) == LOW){
        rightCount++;
      } else{
        rightCount--;
      }
  }
}
