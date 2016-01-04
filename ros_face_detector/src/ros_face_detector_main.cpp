#include "ros_face_detector_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "ros_img_processor");
      
      //create ros wrapper object
      RosImgProcessorNode imgp;  //no es darle un nombre, crea un objeto de dicha clase

      // double ratenode = imgp.getRate();
      // set node loop rate
      ros::Rate loopRate(imgp.getRate());


      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce();

            //do things
            imgp.process();

            
            //publish things
            imgp.publish(); 
                        
            //relax to fit output rate
            loopRate.sleep();            
      }
            
      //exit program
      return 0;
}

