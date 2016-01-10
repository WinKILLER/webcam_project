#include "pid_node.h"

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <pid/pid_header.h>
#include <control_toolbox/pid.h>



float velx, vely, posx, posy;
double velx_correction, vely_correction;
double p = 1.0;
double ii = 1.0;
double d = 1.0;
double i_max = 1.0;
double i_min = 1.0;

ros::Time time_of_last_cycle_;

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

float PidNode::convertVelocity_Y(float posy)
{
  float vely;

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

  time_of_last_cycle_ = ros::Time::now();

  return vely;
}

float PidNode::convertVelocity_X(float posx)
{
    float velx;
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

    //time_of_last_cycle_ = robot_->getTime();
    time_of_last_cycle_ = ros::Time::now();

    return velx;
}


void PidNode::pid_X()
{
    //PID
    control_toolbox::Pid pid_x;

    pid_x.initPid(p, ii, d, i_max, i_min);


    ros::Duration dt = ros::Time::now() - time_of_last_cycle_;

    velx_correction = pid_x.updatePid(velx, dt);


}

void PidNode::publish()
{

    pid_msg_.data.clear();

    pid_msg_.data[0] = (float)velx;
    pid_msg_.data[1] = (float)vely;

    pid_publi.publish(pid_msg_);
}

double PidNode::getRate() const
{

    return rate_;
}

void PidNode::kalmanfiltercallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  std::cout << msg->data[0] << std::endl;
}
