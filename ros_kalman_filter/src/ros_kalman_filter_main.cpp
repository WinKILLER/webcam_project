#include <iostream>

//ros dependencies
#include "ros_kalman_filter_node.h"

//node main
int main(int argc, char **argv)
{
      std::cout << "////Ping 1" << std::endl;
      //init ros
      ros::init(argc, argv, "ros_kalman_filter");

      std::cout << "////Ping 2" << std::endl;
      //create ros wrapper object
      RosKalmanFilterNode kalman_filter;

      std::cout << "////Ping 3" << std::endl;
      //set node loop rate
      ros::Rate loopRate(kalman_filter.getRate());

      //node loop
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce();

            std::cout << "////Ping 4" << std::endl;
            //Predition
            kalman_filter.prediction();

            std::cout << "////Ping 5" << std::endl;
            //Correction
            kalman_filter.correction();

            std::cout << "////Ping 6" << std::endl;
            //publish things
            kalman_filter.publish();

            std::cout << "////Ping 7" << std::endl;
            //relax to fit output rate
            loopRate.sleep();
      }

      //exit program
      return 0;
}

