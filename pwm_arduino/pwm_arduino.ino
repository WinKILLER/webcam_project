#include "Arduino.h"
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle  nh;
Servo servoX;
Servo servoY;

void servo_cb( const std_msgs::Int32MultiArray& cmd_msg)
{  
  int y,x;
  int b = 90;
  float m = -0.5;

  x=cmd_msg.data[0];
  y=((m*x)+b);

  if (y < 0) y=0;

  if (y > 180) y=180;

  servoX.write(120); //set servo angle, should be from 0-180  
}

void servo_y_cb( const std_msgs::Int32MultiArray& cmd_msg)
{ 
  int y,x;
  int b = 90;
  float m = -0.5;

  x=cmd_msg.data[0];
  y=((m*x)+b);

  if (y < 0) y=0;

  if (y > 180) y=180;
  servoY.write(cmd_msg.data[0]);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("/ros_pid_controller/pwm_output_x", servo_cb);
ros::Subscriber<std_msgs::Int32MultiArray> sub_y("/ros_pid_controller/pwm_output_y", servo_y_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_y);

  servoX.attach(9); //attach it to pin 9
  servoY.attach(10);
}

void loop(){
  nh.spinOnce();
  delay(10);
}


