/**
 ******************************************************************************
 * @file        pid_main.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       Main ROS app to control PID node.
 ******************************************************************************
 */

#include <stdio.h>
#include "pid_node.h"

int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "ros_pid");

    double Input, Output, Setpoint;

    //create ros wrapper object
    PidNode pid(&Input, &Output, &Setpoint,2,5,1, DIRECT);

    Setpoint = 320;

    pid.SetMode(AUTOMATIC);

    //set node loop rate
    ros::Rate loopRate(pid.getRate());

    //node loop
    while ( ros::ok() ) {
        //execute pending callbacks
        ros::spinOnce();

        pid.Compute();

        //publish things
        pid.publish();

        //relax to fit output rate
        loopRate.sleep();
    }

    //exit program
    return 0;
}