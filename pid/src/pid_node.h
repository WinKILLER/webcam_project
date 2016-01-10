#ifndef PID_NODE_H
#define PID_NODE_H

//std C++
#include <iostream>

//OpenCV
#include "cv.h"

//ROS headers for image I/O
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class PidNode{
    protected:
        ros::NodeHandle nh_;

        //subscriber to kalman filter
        ros::Subscriber kalman_publi;

        //publisher
        std_msgs::Float32MultiArray pid_msg_;
        ros::Publisher pid_publi;

        //wished process rate [hz]
        double rate_;

    protected:
        //callbacks
        void kalmanfiltercallback(const std_msgs::Float32MultiArrayConstPtr& msg);

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */
        PidNode();

        /** \brief Destructor
        *
        * Destructor
        *
        */
        ~PidNode();

        /** \brief Prediction x(t) & Cx(t)
         *
         * Prediction x(t) & Cx(t)
         *
         */
         float convertVelocity_X(float);
         float convertVelocity_Y(float);

         /** \brief Convert the position error to velocity
          *
          * Convert the position error to velocity
          *
          */
         void pid_X();
         void pid_Y();

         /** \brief do the pid correction
          *
          * do the pid correction
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


#endif // PID_NODE_H
