/**
 ******************************************************************************
 * @file        ros_img_processor_node.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Andreu Corominas, Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       ROS image processor node.
 ******************************************************************************
 */

#define ROS_IMG_PROCESSOR_NODE_C_

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

#include "ros_img_processor_node.h"
#include <ros/console.h>

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

RosImgProcessorNode::RosImgProcessorNode() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
    //loop rate [hz], Could be set from a yaml file
    rate_=100;

    //sets publishers
    image_pub_ = img_tp_.advertise("/ros_img_processor/image_out", 10);

    //sets subscribers
    image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
    camera_info_subs_ = nh_.subscribe("camera_info_in", 10, &RosImgProcessorNode::cameraInfoCallback, this);
    detector_subs_ = nh_.subscribe("/ros_face_detector/detector_out", 100, &RosImgProcessorNode::detectorFacePixelsCallbacks, this);
    kalman_subs_ = nh_.subscribe("/ros_kalman_filter/kalman_out", 100, &RosImgProcessorNode::kalmanFacePixelsCallbacks, this);

}

RosImgProcessorNode::~RosImgProcessorNode()
{
    //
}

void RosImgProcessorNode::process()
{
    cv::Rect_<int> box;

    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr ) {
        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;


        //mark a rectangle in the center: http://docs.opencv.org/2.4.11/modules/core/doc/drawing_functions.html#rectangle
        if(box_detector_.x > 0) {

            cv::rectangle(cv_img_out_.image, box_detector_, cv::Scalar(255,0,0), 1);
        }
        if(box_kalman_.x >0) {

            cv::rectangle(cv_img_out_.image, box_kalman_, cv::Scalar(0,255,0), 2);
        }
    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}

void RosImgProcessorNode::publish()
{
    //image_raw topic
    cv_img_out_.header.seq ++;
    cv_img_out_.header.stamp = ros::Time::now();
    cv_img_out_.header.frame_id = "camera";
    cv_img_out_.encoding = img_encoding_;
    image_pub_.publish(cv_img_out_.toImageMsg());
}

double RosImgProcessorNode::getRate() const
{
    return rate_;
}

void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try {
        if (_msg->data.size() > 0) {
            img_encoding_ = _msg->encoding;//get image encodings
            cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo& _msg)
{
    //
}

void RosImgProcessorNode::detectorFacePixelsCallbacks(const std_msgs::UInt32MultiArrayConstPtr& _msg)
{
    try {
        box_detector_.x = _msg -> data[0];
        box_detector_.y = _msg -> data[1];
        box_detector_.width = _msg -> data[2];
        box_detector_.height = _msg -> data[3];


    } catch(ros::Exception& e) {
        ROS_ERROR("RosKalmanFilterNode::centerFacePixelsCallbacks(): exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::kalmanFacePixelsCallbacks(const std_msgs::UInt32MultiArrayConstPtr& _msg)
{
    try {

        box_kalman_.x = _msg -> data[0];
        box_kalman_.y = _msg -> data[1];

        if(box_detector_.width != 0 && box_detector_.height != 0) {
            box_kalman_.width = box_detector_.width;
            box_kalman_.height = box_detector_.height;
        } else {
            box_kalman_.width = 20;
            box_kalman_.height = 20;
        }
    } catch(ros::Exception& e) {
        ROS_ERROR("RosImgProcessorNode::kalmanFacePixelsCallbacks(): exception: %s", e.what());
        return;
    }
}

/******************************************************************************
 * EOF
 *****************************************************************************/
