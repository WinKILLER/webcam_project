#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle  nh;

int servoX = 9; //pwm pins
int servoY = 10;

void servo_y_cb( const std_msgs::Int32MultiArray& cmd_msg)
{ 
  int y;
  y=cmd_msg.data[0];
 
  analogWrite(servoY, y);
}

void servo_cb( const std_msgs::Int32MultiArray& cmd_msg)
{ 
  int x;
  x=cmd_msg.data[0];
 
  analogWrite(servoX, x);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("/ros_pid_controller/pwm_output_x", servo_cb);
ros::Subscriber<std_msgs::Int32MultiArray> sub_y("/ros_pid_controller/pwm_output_y", servo_y_cb);

void setup(){
  pinMode(servoX, OUTPUT); // sets the pin as output
  pinMode(servoY, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_y);

}

void loop(){
  nh.spinOnce();
  delay(10);
}


