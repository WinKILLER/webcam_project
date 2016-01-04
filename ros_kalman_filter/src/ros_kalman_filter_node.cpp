#include <ros_kalman_filter_node.h>
#include <ros/ros.h>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

RosKalmanFilterNode::RosKalmanFilterNode():
    nh_(ros::this_node::getName())
{

    //loop rate [hz], Could be set from a yaml file
    rate_=10;

    //set publishers
    kalman_msg = nh_.advertise<std_msgs::Float32MultiArray>("kalman_out", 100);

    //set subscribers
    detected_pixels = nh_.subscribe("detector_out", 1, &RosKalmanFilterNode::centerFacePixelsCallbacks, this);

    //SET KALMAN PARAMETERS
    // state vectors
    Eigen::Vector4f x_before;
    Eigen::Vector4f x_predicted;
    Eigen::Vector4f x_t;

    //noise state added
    Eigen::Matrix4f C_nx;
    double sigma_p_x = 10**2;
    double sigma_v_x = 5**2;
    C_nx << sigma_p_x, 0, 0, 0,
            0, sigma_p_x, 0, 0,
            0, 0, sigma_v_x, 0,
            0, 0, 0, sigma_v_x;

    //variance of the state
    Eigen::Matrix4f C_x;
    Eigen::Matrix4f C_x_predicted;
    Eigen::Matrix4f C_x_before;
    C_x_before << 10**2, 0, 0, 0,
            0, 10**2, 0, 0,
            0, 0, 5**2, 0,
            0, 0, 0, 5**2;

    //z_t = H*x_t + n_z
    Eigen::MatrixXf H(2, 4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    //Measurements vectors
    Eigen::Vector2f z_t;
    Eigen::Vector2f z_predicted;
    //noise measurements added
    Eigen::Matrix2f C_nz;
    double sigma_nz = 20**2;
    C_nz << sigma_nz, 0,
            0, sigma_nz;

    //x_t = F*x_before + nx
    Eigen::Matrix4f F;
    double dT = 0;
    double precTick = 0;
    double ticks = 0;

    F << 1, 0, dT, 0,
         0, 1, 0, dT,
         0, 0, 1, 0,
         0, 0, 0, 1;

    Eigen::MatrixXd K(4,2);
    Eigen::Matrix4i I;
    I << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0 ,1;
}

RosKalmanFilterNode::~RosKalmanFilterNode()
{
    //
}

void RosKalmanFilterNode::prediction()
{

    x_before = x_t;
    C_x_before = C_X;
    x_predicted = F * x_before;

    precTick = ticks;
    ticks = (double) cv::getTickCount();
    dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

    C_x_predicted = F * C_x_before * F.transpose() + C_nx;
}

void RosKalmanFilterNode::correction()
{

    z_predicted = H * x_predicted;

    if(distanceMalanovich() <= 5){
        K = C_x_predicted * H.transpose() * (H * C_x_predicted * H.transpose() + C_nz);
        x_t = x_predicted + K*(z_t - z_predicted);

        Eigen::Matrix4f TEMP = (I - K * H);
        C_x = TEMP * C_x_predicted * TEMP.transpose() + K * C_nz * K.transpose();
    }
}

double RosKalmanFilterNode::distanceMalanovich()
{
    double distance;
    Eigen::Vector2f error_z = z_t - z_predicted;
    Eigen::Matrix2f TEMP = H * C_x * H.transpose();
    Eigen::Matrix2f inverse = (C_nz + TEMP).inverse();

    distance = error_z * inverse + error_z.transpose();

    return  distance;

}

void RosKalmanFilterNode::publish()
{

    std_msgs::Float32MultiArray msg;
    msg.data(x_t);


}

double RosKalmanFilterNode::getRate()
{

    return rate_;
}

void RosKalmanFilterNode::centerFacePixelsCallbacks(const std_msgs::Int16MultiArray_::ConstPtr &msg)
{

}