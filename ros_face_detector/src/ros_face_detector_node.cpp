#include "ros_face_detector_node.h"

RosFaceDetectorNode::RosFaceDetectorNode() :
    nh_(ros::this_node::getName())
{
    //loop rate [hz], Could be set from a yaml file 
    rate_=10; 
    
    //sets publishers
    detector_publi = nh_.advertise<std::vector<cv::Rect>("detector_out",100);
    
    //sets subscribers
    image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
    camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);

    //face detector init
    face_detect_.load("/home/oos/catkin_ws/src/webcam_project/ros_face_detector/filters/lbdcascade_frontlface.xml");
}

RosFaceDetectorNode::~RosFaceDetectorNode()
{
    //
}

void RosFaceDetectorNode::detect_face()
{

    image_ = cv_img_ptr_in_->image.clone();
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    face_detect.detectMultiScale(image_, faces_, 1.3, 4, cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
}

void RosFaceDetectorNode::publish()
{

    detector_publi.publish(faces_);
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


