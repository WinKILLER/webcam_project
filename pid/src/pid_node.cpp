#include "pid_node.h"

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <pid/pid_header.h>


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

    return velx;
}

void PidNode::pid()
{
    //PID


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
