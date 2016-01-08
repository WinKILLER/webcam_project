#include <iostream>

//ros dependencies
#include "ros_kalman_filter_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "ros_kalman_filter");

      //create ros wrapper object
      RosKalmanFilterNode kalman_filter;

      //set node loop rate
      ros::Rate loopRate(kalman_filter.getRate());

      //node loop
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce();

            //Predition
            kalman_filter.prediction();

            //Correction
            kalman_filter.correction();

            //publish things
            kalman_filter.publish();

            //relax to fit output rate
            loopRate.sleep();
      }

      //exit program
      return 0;
}

