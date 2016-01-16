/**
 ******************************************************************************
 * @file        ros_kalman_filter_node.h
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       ROS Kalman filter node (header).
 ******************************************************************************
 */

#ifndef ROS_KALMAN_FILTER_NODE_H
#define ROS_KALMAN_FILTER_NODE_H

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

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
#include <std_msgs/UInt32MultiArray.h>

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/

class RosKalmanFilterNode
{
protected:
    ros::NodeHandle nh_;

    //subscriber to faces detector topic
    ros::Subscriber detected_pixels;

    //publisher
    std_msgs::UInt32MultiArray kalman_msg_;
    ros::Publisher kalman_publi;

    //wished process rate [hz]
    double rate_;
    double lastCallback;

protected:
    //callbacks
    void centerFacePixelsCallbacks(const std_msgs::UInt32MultiArrayConstPtr& _msg);

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

/******************************************************************************
 * EXPORTED VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/

#endif /* ROS_KALMAN_FILTER_NODE_H */

/******************************************************************************
 * EOF
 *****************************************************************************/
