#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "vision_msgs/DetectObjects.h"


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR OBJECT FEATURES BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "obj_features_test");
    ros::NodeHandle n;

    int count=0;
    float height_obj_yisus;
    std::ofstream fileHeight;

    vision_msgs::DetectObjects srv_detectObj_edd, srv_detectObj_yisus;

    geometry_msgs::Pose centroid_edd, centroid_yisus;
    geometry_msgs::Vector3 size_obj;
    

    ros::ServiceClient cltDetectObjectsEdd;
    ros::ServiceClient cltDetectObjectsYisus;


    cltDetectObjectsEdd = n.serviceClient<vision_msgs::DetectObjects>("/vision/detect_object/PCA_calculator");
    cltDetectObjectsYisus = n.serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_all_objs");

    fileHeight.open("/home/edgar/height_cerealBox.txt");

    ros::Rate loop(10);

    while(ros::ok())
    {
      
        if(!cltDetectObjectsEdd.call(srv_detectObj_edd))
        {
            std::cout << std::endl << "Justina::Vision --->  Don't detect object (Edd-2 version)" << std::endl << std::endl;
        }
	else
	{
	  centroid_edd = srv_detectObj_edd.response.recog_objects[0].pose;
	  
	  size_obj = srv_detectObj_edd.response.recog_objects[0].size;
	  
	  std::cout << " --- Edd-II  -- " << std::endl;
	  std::cout << "Centroid: " << centroid_edd.position << std::endl;
	}

	if(!cltDetectObjectsYisus.call(srv_detectObj_yisus))
        {
            std::cout << std::endl << "Justina::Vision ---> Don't detect object (Yisus version)" << std::endl << std::endl;
        }
	else
	{
	  centroid_yisus = srv_detectObj_yisus.response.recog_objects[0].pose;
	  height_obj_yisus = srv_detectObj_yisus.response.recog_objects[0].height;
	  
	  std::cout << " --- Yisus  -- " << std::endl;
	  std::cout << "Centroid: " << centroid_yisus.position << std::endl;
	}
	
	std::cout << "            Edd-II  ---  Yisus " << std::endl;
	std::cout << "Height: " << size_obj.z << "    " << height_obj_yisus << std::endl;
        std::cout << std::endl << std::endl;
	
	fileHeight << " " << size_obj.z << " " << height_obj_yisus << "\n";
        
        std::cout << "---------------------------" << std::endl;
        ros::spinOnce();
        loop.sleep();

	std::cout << "Count: " << count << std::endl;
	count++;
	   
    }

    fileHeight.close();
    return 0;
}
