/******************************************************
 * Arduino code for the roadrunner with a motor shield for
 * gearmotors and a servo powered gripped with a wrist joint
 * ****************************************************/
/******************************************************
 * Reminder: Change the message values on Rover to
 * reflect the arduino
 * 
 */

#include <ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
//#include <std_msgs/Float32.h>
#include <Servo.h>
#include "turtle_motor.h"
#include "turtle_odom.h"
#include "turtle_grip.h"
#include "turtle_sonar.h"
#include "turtle_light.h"
//#include "rr_wrist.h"
//#include "INA219.h"


// handle for the ros node interface
ros::NodeHandle  nh;

turtle_motor motor(&nh);
turtle_odom odom(&nh);
turtle_grip grip(&nh);
turtle_sonar sonar(&nh);
//turtle_wrist wrist(&nh);
//INA219 ina219(&nh);
turtle_light headlight(&nh);

// callback function when a motor command ROS message is received
// create the ros topic subscribers
void motorMessageCb( const geometry_msgs::Twist& cmd_msg){
  motor.motorMessageCb(cmd_msg);
}

ros::Subscriber<geometry_msgs::Twist> motorSubscriber("/rover/speed", motorMessageCb );

// callback function when a arm command ROS message is received
void gripMessageCb( const geometry_msgs::Point& cmd_msg){
  grip.gripMessageCb(cmd_msg);
}
  
ros::Subscriber<geometry_msgs::Point> gripSubscriber("/walle/simEva_gripper/fingerAngle/cmd", gripMessageCb );


//void wristMessageCb(const std_msgs::Float32& cmd_msg){
//  wrist.wristMessageCb(cmd_msg);
//  }
//ros::Subscriber<std_msgs::Float32> wristSubscriber("/walle/simEva_wrist/wristAngle/cmd", wristMessageCb );


void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(motorSubscriber);

//  nh.subscribe(gripSubscriber);
//  nh.subscribe(wristSubscriber);
  motor.setup();
  odom.setup();
  grip.setup();
  sonar.setup();
//  wrist.setup();
//  ina219.setup();
  headlight.setup();
}

void loop()
{

  //int used =  Serial.available();
  //Serial.print("TX Buffer Used: ");
  //Serial.print(used);
  //Serial.println();
  motor.loop();
  odom.loop();
  grip.loop();
  sonar.loop();
  //wrist.loop();
  //ina219.readbattery();
  headlight.loop();
  nh.spinOnce();
  delay(10);
}
