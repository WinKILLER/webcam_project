#include "ros_img_processor_node.h"
#include <ros/console.h>

RosImgProcessorNode::RosImgProcessorNode() : 
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
    //loop rate [hz], Could be set from a yaml file 
    rate_=10; 
    
    //sets publishers
    image_pub_ = img_tp_.advertise("image_out", 100);
    
    //sets subscribers
    image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
    camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);
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
    if ( cv_img_ptr_in_ != nullptr )
    {
        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;
        

        //mark a rectangle in the center: http://docs.opencv.org/2.4.11/modules/core/doc/drawing_functions.html#rectangle
        if(box_detector_.x > 0){

            cv::rectangle(cv_img_out_.image, box_detector_, cv::Scalar(0,0,255), 3);
        }
        if(box_kalman_.x >0){

            cv::rectangle(cv_img_out_.image, box_kalman_, cv::Scalar(255,0,0), 3);
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

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo& _msg)
{
    //
}

void RosImgProcessorNode::detectorFacePixelsCallbacks(const std_msgs::Float32MultiArrayConstPtr& _msg)
{
    try{
            box_detector_.x = (int)_msg -> data[0];
            box_detector_.y = (int)_msg -> data[1];
            box_detector_.width = (int)_msg -> data[2];
            box_detector_.height = (int)_msg -> data[3];

    }catch(ros::Exception& e)
    {
        ROS_ERROR("RosKalmanFilterNode::centerFacePixelsCallbacks(): exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::kalmanFacePixelsCallbacks(const std_msgs::Float32MultiArrayConstPtr& _msg)
{
    try{

        box_kalman_.x = (int)_msg -> data[0];
        box_kalman_.y = (int)_msg -> data[1];

        if(box_detector_.width != 0 && box_detector_.height != 0){
            box_kalman_.width = box_detector_.width;
            box_kalman_.height = box_kalman_.height;
        }else{
            box_kalman_.width = 20;
            box_kalman_.height = 20;
        }
    }catch(ros::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::kalmanFacePixelsCallbacks(): exception: %s", e.what());
        return;
    }
}
