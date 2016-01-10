#include <stdio.h>
#include "pid_node.h"

int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "ros_pid");

    //create ros wrapper object
    PidNode pid;

    //set node loop rate
    ros::Rate loopRate(pid.getRate());

    //node loop
    while ( ros::ok() )
    {
          //execute pending callbacks
          ros::spinOnce();

          //ConvertVelocity
          pid.convertVelocity_X(20);
          pid.convertVelocity_Y(30);

          //PID
          pid.pid_X();

          //publish things
          pid.publish();

          //relax to fit output rate
          loopRate.sleep();
    }

    //exit program
    return 0;
}

