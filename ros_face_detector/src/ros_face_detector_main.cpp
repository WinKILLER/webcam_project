/**
 ******************************************************************************
 * @file        ros_face_detector_main.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       Main ROS app to control face detection node.
 ******************************************************************************
 */

#include "ros_face_detector_node.h"

//node main
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "ros_face_detector");

    //create ros wrapper object
    RosFaceDetectorNode face_detector;  //no es darle un nombre, crea un objeto de dicha clase

    // double ratenode = imgp.getRate();
    // set node loop rate
    ros::Rate loopRate(face_detector.getRate());


    //node loop
    while ( ros::ok() ) {
        //execute pending callbacks
        ros::spinOnce();

        //Detect faces
        face_detector.detect_face();

        //publish things
        face_detector.publish();

        //relax to fit output rate
        loopRate.sleep();
    }

    //exit program
    return 0;
}
