/**
 ******************************************************************************
 * @file        ros_kalman_filter_main.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       Main ROS app to control Kalman filter node.
 ******************************************************************************
 */

#define ROS_KALMAN_FILTER_MAIN_C_

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

#include <iostream>
#include "ros_kalman_filter_node.h"

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/

/******************************************************************************
 * PROTOTYPES OF LOCAL FUNCTIONS
 *****************************************************************************/

/******************************************************************************
 * LOCAL VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/

/******************************************************************************
 * LOCAL FUNCTIONS
 *****************************************************************************/

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
    while ( ros::ok() ) {

        //execute pending callbacks
        ros::spinOnce();

        std::cout << "SpinOnce DONE" << std::endl;
        //Predition
        kalman_filter.prediction();

        std::cout << "Predciton DONE" << std::endl;
        //Correction
        kalman_filter.correction();

        std::cout << "Correction DONE" << std::endl;
        //publish things
        kalman_filter.publish();

        std::cout << "Publish DONE!" << std::endl;
        //relax to fit output rate
        loopRate.sleep();

        std::cout << "Sleep DONE" << std::endl;
    }

    //exit program
    return 0;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
