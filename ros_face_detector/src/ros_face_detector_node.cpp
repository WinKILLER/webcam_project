/**
 ******************************************************************************
 * @file        ros_face_detector_node.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       ROS face detection node.
 ******************************************************************************
 */

#include "ros_face_detector_node.h"
#include <ros/console.h>

RosFaceDetectorNode::RosFaceDetectorNode():
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
    //loop rate [hz], Could be set from a yaml file
    rate_=10;

    //sets publishers
    detect_msg_.layout.dim.resize(1);
    detect_msg_.layout.dim[0].label = "face_position";
    detect_msg_.layout.dim[0].size = 4;
    detect_msg_.data.resize(4);

    detector_publi = nh_.advertise<std_msgs::UInt32MultiArray>("/ros_face_detector/detector_out",100);

    //sets subscribers
    image_subs_ = img_tp_.subscribe("/ros_img_processor/image_out", 10, &RosFaceDetectorNode::imageCallback, this);

    //face detector init
    face_detect_.load("./src/webcam_project/ros_face_detector/filters/lbdcascade_frontlface.xml");
}

RosFaceDetectorNode::~RosFaceDetectorNode()
{
    //
}

void RosFaceDetectorNode::detect_face()
{
    if (cv_img_ptr_in_ != nullptr) {
        //copy the input image to the out one
        image_ = cv_img_ptr_in_->image;
        cv::cvtColor(image_, gray_, CV_BGR2GRAY);
        face_detect_.detectMultiScale(gray_, faces_, 1.3, 4, cv::CASCADE_SCALE_IMAGE, cv::Size(48, 48), cv::Size(480,480));
    }
}

void RosFaceDetectorNode::publish()
{
    detect_msg_.data.clear();
    detect_msg_.data.resize(4);

    if (faces_.size() > 0) {
        detect_msg_.data[0] = faces_[0].x;
        detect_msg_.data[1] = faces_[0].y;
        detect_msg_.data[2] = faces_[0].width;
        detect_msg_.data[3] = faces_[0].height;
        detector_publi.publish(detect_msg_);
    }
}

double RosFaceDetectorNode::getRate() const
{
    return rate_;
}

void RosFaceDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}
