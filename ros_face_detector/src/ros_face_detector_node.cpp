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
    detect_msg_.layout.dim[0].label = "states_detected";
    detect_msg_.layout.dim[0].size = 4;
    detect_msg_.data.resize(4);

    detector_publi = nh_.advertise<std_msgs::Float32MultiArray>("detector_out",100);

    //sets subscribers
    image_subs_ = img_tp_.subscribe("/ros_img_processor/image_out", 1, &RosFaceDetectorNode::imageCallback, this);
    //camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosFaceDetectorNode::cameraInfoCallback, this);

    //face detector init
    face_detect_.load("./src/webcam_project/ros_face_detector/filters/lbdcascade_frontlface.xml");
}

RosFaceDetectorNode::~RosFaceDetectorNode()
{
    //
}

void RosFaceDetectorNode::detect_face()
{
    if (cv_img_ptr_in_ != nullptr)
    {
        //copy the input image to the out one
        image_ = cv_img_ptr_in_->image;
        cv::cvtColor(image_, gray_, CV_BGR2GRAY);
        face_detect_.detectMultiScale(gray_, faces_, 1.3, 4, cv::CASCADE_SCALE_IMAGE, cv::Size(50, 50), cv::Size(200,200));
    }
}

void RosFaceDetectorNode::publish()
{
    detect_msg_.data.clear();

    if (faces_.size() > 0)
    {
        ROS_INFO("Face detected at (%d,%d) (w=%d,h=%d)", faces_[0].x, faces_[0].y, faces_[0].width, faces_[0].height);
        detect_msg_.data[0] = (float)faces_[0].x;
        detect_msg_.data[1] = (float)faces_[0].y;
        detect_msg_.data[2] = (float)faces_[0].width;
        detect_msg_.data[3] = (float)faces_[0].height;
        detector_publi.publish(detect_msg_);
    }
}

double RosFaceDetectorNode::getRate() const
{
    return rate_;
}

void RosFaceDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}
