/**
 ******************************************************************************
 * @file        pid_node.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Brett Beauregard, Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       ROS PID node.
 ******************************************************************************
 */

#define PID_NODE_C_

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

#include "pid_node.h"

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

#define MAX_SERVO_SPEED (10)

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/

/******************************************************************************
 * PROTOTYPES OF LOCAL FUNCTIONS
 *****************************************************************************/

/******************************************************************************
 * LOCAL VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/

/******************************************************************************
 * LOCAL FUNCTIONS
 *****************************************************************************/

PidNode::PidNode(double* Input, double* Output, double* Setpoint,
                 double Kp, double Ki, double Kd, int ControllerDirection,
                 const std::string publi_name ):
    nh_(ros::this_node::getName())
{
    //loop rate [hz], Could be set from a yaml file
    rate_=10;

    //sets publishers
    pid_msg_.layout.dim.resize(1);
    pid_msg_.layout.dim[0].label = "servo_pid_values";
    pid_msg_.layout.dim[0].size = 1;
    pid_msg_.data.resize(1);

    //set publishers
    pid_publi = nh_.advertise<std_msgs::Int32MultiArray>(publi_name, 10);

    //set subscribers
    kalman_subscriber = nh_.subscribe("/ros_kalman_filter/kalman_out", 1, &PidNode::kalmanfiltercallback, this);
    face_subscriber = nh_.subscribe("/ros_face_detector/detector_out", 1, &PidNode::detectorFacePixelsCallbacks, this);

    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PidNode::SetOutputLimits(-9, 3);

    SampleTime = 0.1;							//default Controller Sample Time is 0.1 seconds

    PidNode::SetControllerDirection(ControllerDirection);
    PidNode::SetTunings(Kp, Ki, Kd);

    lastTime = ros::Time::now().toSec() - SampleTime;
}

PidNode::~PidNode()
{
    //
}

bool PidNode::Compute()
{
    if(!inAuto) return false;
    double now = ros::Time::now().toSec();
    double timeChange = (now - lastTime);
    if(timeChange>=SampleTime) {
        /*Compute all the working error variables*/
        double input = *myInput;
        double error = *mySetpoint - input;
        ITerm+= (ki * error);
        if(ITerm > outMax) ITerm= outMax;
        else if(ITerm < outMin) ITerm= outMin;
        double dInput = (input - lastInput);

        /*Compute PID Output*/
        double output = kp * error + ITerm- kd * dInput;

        if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;
        *myOutput = output;

        /*Remember some variables for next time*/
        lastInput = input;
        lastTime = now;
        return true;
    } else return false;
}

void PidNode::SetTunings(double Kp, double Ki, double Kd)
{
    if (Kp<0 || Ki<0 || Kd<0) return;

    dispKp = Kp;
    dispKi = Ki;
    dispKd = Kd;

    kp = Kp;
    ki = Ki * SampleTime;
    kd = Kd / SampleTime;

    if(controllerDirection == REVERSE) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

void PidNode::SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0) {
        double ratio  = (double)NewSampleTime
                        / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (double)NewSampleTime;
    }
}

void PidNode::SetOutputLimits(double Min, double Max)
{
    if(Min >= Max) return;
    outMin = Min;
    outMax = Max;

    if(inAuto) {
        if(*myOutput > outMax) *myOutput = outMax;
        else if(*myOutput < outMin) *myOutput = outMin;

        if(ITerm > outMax) ITerm= outMax;
        else if(ITerm < outMin) ITerm= outMin;
    }
}

void PidNode::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto) {
        /*we just went from manual to auto*/
        PidNode::Initialize();
    }
    inAuto = newAuto;
}

void PidNode::Initialize()
{
    ITerm = *myOutput;
    lastInput = *myInput;
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;
}

void PidNode::SetControllerDirection(int Direction)
{
    if(inAuto && Direction !=controllerDirection) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
    controllerDirection = Direction;
}

double PidNode::GetKp()
{
    return  dispKp;
}
double PidNode::GetKi()
{
    return  dispKi;
}
double PidNode::GetKd()
{
    return  dispKd;
}
int PidNode::GetMode()
{
    return  inAuto ? AUTOMATIC : MANUAL;
}
int PidNode::GetDirection()
{
    return controllerDirection;
}

void PidNode::publish()
{
    pid_msg_.data.clear();
    pid_msg_.data.resize(1);

    pid_msg_.data[0] = *myOutput;

    pid_publi.publish(pid_msg_);
}

double PidNode::getRate() const
{
    return rate_;
}

void PidNode::kalmanfiltercallback(const std_msgs::UInt32MultiArrayConstPtr& msg)
{
    double horizontalCenter = msg->data[0] + width/2;
    double verticalCenter = msg->data[1] + height/2;
    *myInput = horizontalCenter;
}

void PidNode::detectorFacePixelsCallbacks(const std_msgs::UInt32MultiArrayConstPtr& _msg)
{
    try {
        if (_msg->data.size() > 0)
        {
          width = _msg->data[2];
          height = _msg->data[3];
        }
    } catch(ros::Exception& e) {
        ROS_ERROR("PidNode::centerFacePixelsCallbacks(): exception: %s", e.what());
        return;
    }
}

/******************************************************************************
 * EOF
 *****************************************************************************/
