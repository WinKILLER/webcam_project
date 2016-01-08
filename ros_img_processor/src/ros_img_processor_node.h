#ifndef ros_img_processor_node_H
#define ros_img_processor_node_H

//std C++
#include <iostream>

//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>

/** \brief Simple Image Processor
 *
 * Simple Image Processor with opencv calls
 * 
 */
class RosImgProcessorNode
{
    protected: 
        //ros node handle
        ros::NodeHandle nh_;
        
        //image transport
        image_transport::ImageTransport img_tp_;
        
        // subscribers to the image and camera info topics
        image_transport::Subscriber image_subs_;
        ros::Subscriber camera_info_subs_;
        ros::Subscriber detector_subs_;
        ros::Subscriber kalman_subs_;

        //publishers
        image_transport::Publisher image_pub_;      
        
        //pointer to received (in) and published (out) images
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        cv_bridge::CvImage cv_img_out_;
        
        //image encoding label
        std::string img_encoding_;
        
        //wished process rate, [hz]
        double rate_;

        //variables for rectangles to print
        cv::Rect_<int> box_detector_;
        cv::Rect_<int> box_kalman_;
        
    protected: 
        // callbacks
        void imageCallback(const sensor_msgs::ImageConstPtr& _msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfo & _msg);
        void detectorFacePixelsCallbacks(const std_msgs::Float32MultiArrayConstPtr& _msg);
        void kalmanFacePixelsCallbacks(const std_msgs::Float32MultiArrayConstPtr& _msg);

    public:
        /** \brief Constructor
        * 
        * Constructor
        * 
        */
        RosImgProcessorNode();

        /** \brief Destructor
        * 
        * Destructor
        * 
        */
        ~RosImgProcessorNode();

        /** \brief Process input image
        * 
        * Process input image
        * 
        **/
        void process();
                    
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
