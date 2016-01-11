/**
 ******************************************************************************
 * @file        pid_node.h
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       ROS PID node (header).
 ******************************************************************************
 */

#ifndef PID_NODE_H
#define PID_NODE_H

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

/******************************************************************************
* TYPEDEFS AND STRUCTURES
*****************************************************************************/

class PidNode
{
protected:
    ros::NodeHandle nh_;

    ros::Subscriber kalman_subscriber;

    std_msgs::UInt32MultiArray pid_msg_;
    ros::Publisher pid_publi;

    double rate_;

    void Initialize();

    double dispKp;				// * we'll hold on to the tuning parameters in user-entered
    double dispKi;				//   format for display purposes
    double dispKd;				//

    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;

    double *myInput;        // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;       //   This creates a hard link between the variables and the
    double *mySetpoint;     //   PID, freeing the user from having to constantly tell us
    //   what these values are.  with pointers we'll just know.

    double lastTime;
    double ITerm, lastInput;

    double SampleTime;
    double outMin, outMax;
    bool inAuto;

protected:
    //callbacks
    void kalmanfiltercallback(const std_msgs::UInt32MultiArrayConstPtr& msg);

public:
    /** \brief Constructor
    *
    * Constructor
    *
    */
    PidNode(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
            double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here

    /** \brief Destructor
    *
    * Destructor
    *
    */
    ~PidNode();

    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
    //   called every time loop() cycles. ON/OFF and
    //   calculation frequency can be set using SetMode
    //   SetSampleTime respectively

    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
    //it's likely the user will want to change this depending on
    //the application

    //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the
                    double);         	  //   constructor, this function gives the user the option
    //   of changing tunings during runtime for Adaptive control
    void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
    //   means the output will increase when error is positive. REVERSE
    //   means the opposite.  it's very unlikely that this will be needed
    //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which
    //   the PID calculation is performed.  default is 100

    //Display functions ****************************************************************
    double GetKp();						  // These functions query the pid for interal values.
    double GetKi();						  //  they were created mainly for the pid front-end,
    double GetKd();						  // where it's important to know what is actually
    int GetMode();						  //  inside the PID.
    int GetDirection();					  //

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

/******************************************************************************
 * EXPORTED VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/

#endif // PID_NODE_H

/******************************************************************************
 * EOF
 *****************************************************************************/