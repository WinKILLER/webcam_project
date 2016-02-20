/**
 ******************************************************************************
 * @file        ros_face_detector_node.h
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       ROS face detection node (header).
 ******************************************************************************
 */

#ifndef ROS_FACE_DETECTOR_NODE_H
#define ROS_FACE_DETECTOR_NODE_H

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <sstream>

//ROS headers for image I/O
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Bool.h>

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/

/** \brief Simple Image Processor
 *
 * Simple Image Processor with opencv calls
 *
 */
class RosFaceDetectorNode
{
protected:

    //ros node handle
    ros::NodeHandle nh_;

    //image transport
    image_transport::ImageTransport img_tp_;

    // subscribers to the image and camera info topics
    image_transport::Subscriber image_subs_;

    //publishers
    std_msgs::UInt32MultiArray detect_msg_;
    ros::Publisher detector_publi;

    bool isDemo_ = true;
    std::stringstream ss_;
    std_msgs::Bool faceDetected_ ;
    ros::Publisher uvic_demo_publi;

    //pointer to received (in) and published (out) images
    cv_bridge::CvImagePtr cv_img_ptr_in_;

    //image encoding label
    std::string img_encoding_;

    //wished process rate, [hz]
    double rate_;

    //face detector variables
    cv::CascadeClassifier face_detect_;
    cv::Mat image_, gray_;
    std::vector<cv::Rect> faces_;

protected:
    // callbacks
    void imageCallback(const sensor_msgs::ImageConstPtr& _msg);

public:
    RosFaceDetectorNode(const char *filename);
    ~RosFaceDetectorNode();

    void publish();
    double getRate() const;

    void detectFace();
};

/******************************************************************************
 * EXPORTED VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/

#endif /* ROS_FACE_DETECTOR_NODE_H */

/******************************************************************************
 * EOF
 *****************************************************************************/
