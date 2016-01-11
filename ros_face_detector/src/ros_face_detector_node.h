#ifndef ros_face_detector_node_H
#define ros_face_detector_node_H

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
    /** \brief Constructor
    *
    * Constructor
    *
    */

    RosFaceDetectorNode();

    /** \brief Destructor
    *
    * Destructor
    *
    */
    ~RosFaceDetectorNode();

    /** \brief Process input image
    *
    * Detects faces
    *
    **/
    void detect_face();

    /** \brief Publish output image
    *
    * Publish output image
    *
    */
    void publish();

    /** \brief Returns rate_
     *
     * Returns rate_
     *
     **/
    double getRate() const;
};
#endif
