#ifndef PID_NODE_H
#define PID_NODE_H

//std C++
#include <iostream>

//OpenCV
#include "cv.h"

//ROS headers for image I/O
#include <std_msgs/Float32MultiArray.h>

class PidNode{
    protected:
        ros::NodeHandle nh_;

        //subscriber to kalman filter
        ros::Subscriber kalman_msg;

        //publisher
        ros::Publisher pid_msg;

        //wished process rate [hz]
        double rate_;

    protected:
        //callbacks
        void ros_kalman_filter_node(const std_msgs::Float32MultiArray::ConstPtr& msg);

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
         void convertVelocity();

         /** \brief Convert the position error to velocity
          *
          * Convert the position error to velocity
          *
          */
         void pid();

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
