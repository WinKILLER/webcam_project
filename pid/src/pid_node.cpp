#include "pid_node.h"

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include <velocity_controllers/joint_velocity_controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

float velx, vely, posx, posy;


PidNode::PidNode():
    nh_(ros::this_node::getName())
{
    //loop rate [hz], Could be set from a yaml file
    rate_=10;

    //set publishers
    pid_publi = nh_.advertise<std_msgs::Float32MultiArray>("pid_out", 100);

    //set subscribers
    kalman_publi = nh_.subscribe("/ros_kalman_filter/kalman_out", 1, &PidNode::kalmanfiltercallback, this);

}

PidNode::~PidNode()
{
    //
}

void PidNode::convertVelocity(posx, posy, velx, vely)
{
    //Passem l'error a velocitat. Depèn dels pixels que està, anem més ràpid o més lents (X)

    if (0 > posx < 319){          //girem motor esquerra
        velx = 3/16 * posx;
        velx = velx * -1; // Canviem el número de signe
    }

    if (321 > posx < 640){          //girem motor dreta
        posx = posx - 320;
        velx = 3/16 * posx;
    }

    if (posx == 320){ // No hi ha error. Aturem motor
        velx = 0;
    }

    //Passem l'error a velocitat. Depèn dels pixels que està, anem més ràpid o més lents (Y)
    if (0 > posy < 239){          //girem motor esquerra
        vely = 1/4 * posy;
        vely = vely * -1; // Canviem el número de signe
    }

    if (241 > posy < 480){          //girem motor dreta
        posy = posy - 240;
        vely = 1/4 * posy;
   }

    if (posy == 320){ // No hi ha error. Aturem motor
        vely = 0;
    }
}

void PidNode::pid()
{
    //PID

    velocity_controllers::JointVelocityController vel_controller;
    hardware_interface::JointHandle joint1;
    control_toolbox::Pid pidx;
    control_toolbox::Pid pidy;
    hardware_interface::VelocityJointInterface hard_interface;

    pidx.setGains(1,2,3,4,5);
    joint1.name_ = "joint1";

    hard_interface.getNames("world");
    vel_controller.init(hard_interface, nh);
    vel_controller.

     /*pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
     double position_desi_ = 0.5;

     ros::Time last_time = ros::Time::now();
     while (true) {
     ros::Time time = ros::Time::now();
     double effort = pid.updatePid(currentPosition() - position_desi_, time - last_time);
       last_time = time;
       }*/

}

void PidNode::publish()
{

    pid_msg_.data.clear();

    pid_msg_.data[0] = (float)velx;
    pid_msg_.data[1] = (float)vely;

    pid_publi.publish(pid_msg_);


}

double PidNode::getRate()
{

    return rate_;
}
