#ifndef ROS_KALMAN_FILTER_NODE_H
#define ROS_KALMAN_FILTER_NODE_H

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


//ROS
#include <ros/ros.h>

//std C++
#include <iostream>

//OpenCV
#include <opencv/cv.h>

//ROS headers for image I/O
#include <std_msgs/Int32MultiArray.h>

class RosKalmanFilterNode {
protected:
    ros::NodeHandle nh_;

    //subscriber to faces detector topic
    ros::Subscriber detected_pixels;

    //publisher
    std_msgs::Int32MultiArray kalman_msg_;
    ros::Publisher kalman_publi;

    //wished process rate [hz]
    double rate_;

protected:
    //callbacks
    void centerFacePixelsCallbacks(const std_msgs::Int32MultiArrayConstPtr& _msg);

public:
    /** \brief Constructor
    *
    * Constructor
    *
    */
    RosKalmanFilterNode();

    /** \brief Destructor
    *
    * Destructor
    *
    */
    ~RosKalmanFilterNode();

    /** \brief Prediction x(t) & Cx(t)
     *
     * Prediction x(t) & Cx(t)
     *
     */
    void prediction();

    /** \brief Correction part kalman filter
     *
     * Correction part kalman filter
     *
     */
    void correction();

    /** \brief get distance Malanovich
     *
     * get distance Malanovich
     *
     */
    double distanceMalanovich();

    /** \brief Publish kalman msg (center face pixel predicted)
    *
    * Publish kalman msg (center face pixel predicted)
    *
    */
    void publish();

    /** \brief Returns rate_
     *
     * Returns rate_
     *
     **/
    double getRate();


};


#endif // ROS_KALMAN_FILTER_NODE_H
