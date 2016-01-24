/**
 ******************************************************************************
 * @file        pid_main.cpp
 * @version     1.00
 * @date        1/01/2016
 * @author      Carles Oró, Oriol Orra, Ismael Rodríguez, Juan Pedro López
 * @brief       Main ROS app to control PID node.
 ******************************************************************************
 */

#define PID_MAIN_C_

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

#include <stdio.h>
#include "pid_node.h"

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

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

int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "ros_pid_controller");

    double Input, Output, Setpoint;
    double InputY, OutputY, SetpointY;

    //create ros wrapper object
    PidNode pid(&Input, &Output, &Setpoint,0.001,0.5,500, DIRECT, "pwm_output_x");
    PidNode pidY(&InputY, &OutputY, &SetpointY,0.001,0.5,500, DIRECT, "pwm_output_y");

    Setpoint = 320;
    SetpointY = 240;

    pid.SetOutputLimits(120, 134);
    pidY.SetOutputLimits(120, 134);

    pid.SetMode(AUTOMATIC);
    pidY.SetMode(AUTOMATIC);

    //set node loop rate
    ros::Rate loopRate(pid.getRate());

    //node loop
    while ( ros::ok() ) {
        //execute pending callbacks
        ros::spinOnce();

        // Update PID output
        pid.Compute();
        pidY.Compute();

        //publish things
        pid.publish();
        pidY.publish();

        //relax to fit output rate
        loopRate.sleep();
    }

    //exit program
    return 0;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
