/*

 Flex sensor reading and broadcasting for ROS
 
 */

#include <math.h>
#include <ros.h>
#include <std_msgs/Float32.h>

const int analogInPin = A0;
const int led = 13;

// Value read from the flexo
int sensorValue = 0;
float voltage = 0.0;

std_msgs::Float32 flexvoltage;
ros::Publisher p("flexvoltage", &flexvoltage);
ros::NodeHandle nh;
long publisher_timer = 0;

// The setup routine runs once when you press reset:
void setup() {
  // Initialize the digital pin as an output.
  pinMode(led, OUTPUT);     

  nh.initNode();
  nh.advertise(p);
}

// The loop routine runs over and over again forever:
void loop() {
  if (millis() > publisher_timer) {
    // Read the analog in value:
    sensorValue = analogRead(analogInPin);            
    
    // Map it to the range of the analog out:
    voltage = map(sensorValue, 0, 1023, 0, 5);

    flexvoltage.data = voltage;
    p.publish( &flexvoltage );
    publisher_timer = millis() + 10; // publish ~100 times a second
  }
  nh.spinOnce();
}




