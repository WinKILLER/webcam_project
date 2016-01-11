/**
 ******************************************************************************
 * @file        ros_img_processor_main.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Andreu Corominas, Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       Main ROS app to control image processor node.
 ******************************************************************************
 */

//ros dependencies
#include "ros_img_processor_node.h"

//node main
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "ros_img_processor");

    //create ros wrapper object
    RosImgProcessorNode imgp;

    //set node loop rate
    ros::Rate loopRate(imgp.getRate());

    //node loop
    while ( ros::ok() ) {
        //execute pending callbacks
        ros::spinOnce();

        //do things
        imgp.process();

        //publish things
        imgp.publish();

        //relax to fit output rate
        loopRate.sleep();
    }

    //exit program
    return 0;
}
