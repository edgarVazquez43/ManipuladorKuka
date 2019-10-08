#include <iostream>
#include "ros/ros.h"
#include "vision_msgs/DetectObjects.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A OBJECT SEGMENTATION TEST EDGAR..." << std::endl;
    ros::init(argc, argv, "test_object_seg");
    ros::NodeHandle n;

    //Messagge type wich is sending in the service
    vision_msgs::DetectObjects detectObj_msg;

    //Declaration of service client
    ros::ServiceClient cltDetectObjectsPCA;

    cltDetectObjectsPCA = n.serviceClient<vision_msgs::DetectObjects>("vision/detect_object/PCA_calculator");

    
    ros::Rate loop(10);

    while(ros::ok())
    {
      
        if(!cltDetectObjectsPCA.call(detectObj_msg))
        {
            std::cout << std::endl << "ERROOOOOR calling object segmentation Service" << std::endl << std::endl;
            return false;
        }
	
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
