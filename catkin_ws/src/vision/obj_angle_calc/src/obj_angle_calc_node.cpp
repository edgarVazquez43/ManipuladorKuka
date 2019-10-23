#include <iostream>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include "hardware_tools/VisionTools.h"
#include "ros/ros.h"

#include "nn.hpp"
#include "plane3D.hpp"
#include "objExtract.hpp"
#include "findPlaneRansac.hpp"
#include "segmentedObject.hpp"
#include "vision_msgs/DetectObjects.h"

//std::ofstream myFile;

ros::ServiceClient cltRgbdKinect;

point_cloud_manager::GetRgbd srv;

bool callbackPCAobject(vision_msgs::DetectObjects::Request &req,
		       vision_msgs::DetectObjects::Response &resp)
{
	std::cout << "Calling service to calculate PCA....." << std::endl;

	vision_msgs::VisionObject objectDetected;

	std::vector<float> centroid_coord;
	std::vector<float> dimensions;
	std::vector<float> analogVector;
	std::vector<int> P;
	std::vector<int> Tout;

	std::vector<cv::Point3f> principal_axis_calculated;
	std::vector<segmentedObject> objectList;
	segmentedObject object_1;

	int xmin, ymin, H, W;
	int attemps;
	int points_obj;
	float x_obj, y_obj, z_obj;
	float threshold;
	float h_table;
	
	std::string object_name;
	
	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat planeBGR;
	cv::Mat objectsBGR;
	cv::Mat objectsDepth;
	cv::Mat croppedDepth;
	cv::Mat croppedBRG;

	cv::Vec4f planeComp;
	cv::Point3f px;
	cv::Point3f normal_plane;

	plane3D bestPlane;

	// *** Parametros de RANSAC *** //
	attemps = 150;		// Numero de iteraciones para RANSAC
	threshold = 0.009;	// Distancia al plano en metros

	x_obj = 0.0;
	y_obj = 0.0;
	z_obj = 0.0;
	h_table = 0.0;

	points_obj = 0;

	xmin = 100;
	ymin = 120;

	W = 400;
	H = 320;

	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);


	// Calling a service to get the PointCloud
	if(!cltRgbdKinect.call(srv))
	{
		std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
		return -1;
	}

	// This line should be replaced with a code to transform a PointClod (ROS mensaje) to -> CV::MAT(OpenCV data type) 
	VisionTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);


	// Here is done a cropp over the image
	cv::Rect myROI(xmin, ymin, W, H);
	croppedDepth = imgDepth(myROI);
	croppedBRG = imgBGR(myROI);

	planeBGR = croppedBRG.clone();
	objectsBGR = croppedBRG.clone();


	// ##### Find best fit model to point cloud
	// Return plane with Normal = (1, 1, 1) if didn't find plane

	/* Best plane is a data structure that contains the model of a plane
	   expresed by a normal vector, in addittion it contains wich point belog to plane
	   and their respective coordinates (x, y, z)
	 */
        
	// clock_t begin = std::clock();                          // It already measure time
	bestPlane = FindPlaneRANSAC(croppedDepth, threshold, attemps );

	//clock_t end = std::clock();                            // It already measure time
	//double duration = double(end-begin) / CLOCKS_PER_SEC;  // It already measure time
	// std::cout << "EDGAR PLANE plane" << std::endl
	// 	  << "Duration: " << duration << std::endl;

	
	// /* Code for coloring the plane of green color
	//std::cout << "BestModel[Normal]: " << bestPlane.GetNormal() << std::endl;
	//if(bestPlane.GetNormal() != cv::Point3f(1.0, 1.0, 1.0) )

	normal_plane = bestPlane.GetNormal();
	// std::cout << "BestModel[Normal]: " << normal_plane << std::endl;
	if(normal_plane.x != 0.0  & normal_plane.y != 0.0 & normal_plane.z != 0.0)
	{
		for(int j = 0; j < planeBGR.rows; j++)
			for (int i = 0; i < planeBGR.cols; i++)
			{
			  // We calculate the distance of each one point to the plane
			  px = croppedDepth.at<cv::Point3f>(j, i);
			  // If the distance is less that a predefined threshold then
			  // we coloring green it 
			  if (bestPlane.DistanceToPoint(px, false) < threshold)
			    planeBGR.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);

			  // Camparamos si la distancia está dentro de la tolerancia
			  // if (bestPlane.DistanceToPoint(px, false) > threshold)
			  //   planeBGR.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
			}

	  
	        // Code for object segmentation
		// ##### Return the point cloud of objects cropped
		object_1 = ExtractObj(bestPlane, croppedBRG, croppedDepth);
		h_table = CalculateZPlane(bestPlane, croppedDepth);
		objectsDepth = object_1.pointsObject;
		objectsBGR = object_1.pointsBRG;
		cv::imshow("Original RGB", imgBGR);
		cv::imshow("plane", planeBGR);
	}
	else
	{
	        std::cout << "I can't found the plane....   :( " << std::endl;
		objectsDepth = cv::Mat(50, 50, CV_8UC3);
	}

	// cv::imshow("Original RGB", imgBGR);
	// cv::imshow("plane", planeBGR);
	// return true;


	// Search the centroid of object PointCloud
	if(objectsDepth.size() != cv::Size(50, 50) )
	{
	        //this is a class method to get the centroid of the object 
		object_1.getCentroid();
		//This is a class method to get principal axis PCA
		object_1.getPrincipalAxis();

		//This is for response
		resp.recog_objects.push_back(objectDetected);
		resp.recog_objects[0].pose.position.x = object_1.centroid[0];
		resp.recog_objects[0].pose.position.y = object_1.centroid[1];
		resp.recog_objects[0].pose.position.z = object_1.centroid[2];


		geometry_msgs::Vector3 q1;
		resp.recog_objects[0].principal_axis.push_back(q1);
		resp.recog_objects[0].principal_axis.push_back(q1);
		resp.recog_objects[0].principal_axis.push_back(q1);

		//This is the bigger axis
		resp.recog_objects[0].principal_axis[0].x = float(object_1.principalAxis[0].x);
		resp.recog_objects[0].principal_axis[0].y = float(object_1.principalAxis[0].y);
		resp.recog_objects[0].principal_axis[0].z = float(object_1.principalAxis[0].z);

		resp.recog_objects[0].principal_axis[1].x = float(object_1.principalAxis[1].x);
		resp.recog_objects[0].principal_axis[1].y = float(object_1.principalAxis[1].y);
		resp.recog_objects[0].principal_axis[1].z = float(object_1.principalAxis[1].z);

		resp.recog_objects[0].principal_axis[2].x = float(object_1.principalAxis[2].x);
		resp.recog_objects[0].principal_axis[2].y = float(object_1.principalAxis[2].y);
		resp.recog_objects[0].principal_axis[2].z = float(object_1.principalAxis[2].z);
		

		cv::rectangle(imgBGR, cv::Point(xmin, ymin),
			      cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0), 1, 8, 0 );
		cv::rectangle(imgDepth, cv::Point(xmin, ymin),
			      cv::Point(xmin+W, ymin+H), cv::Scalar(50, 255, 0), 2, 8, 0 );
		cv::rectangle(croppedBRG, object_1.ROI, cv::Scalar(150, 50, 0), 2, 8, 0);

		
		cv::imshow("Original RGB", imgBGR);
		cv::imshow("OBJECT RECONIZING", croppedBRG);
		cv::imshow("objects", objectsBGR);
		cv::imshow("Objects Point Cloud", objectsDepth);


	}
	else
		std::cout << "    I can't find a object on the table..... :(" << std::endl;
	return true;
}



int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT ANGLE CALCULATOR BY  EDGAR-II" << std::endl;


	// Initializing ROS node
	ros::init(argc, argv, "obj_angle_calc");
	ros::NodeHandle n;

	ros::ServiceServer srvPCAobject;   // This service is attendend by this node
	ros::Publisher marker_pub;         // This is a publisher for Rviz Visualizator

       
	srvPCAobject = n.advertiseService("/vision/detect_object/PCA_calculator", callbackPCAobject);
	cltRgbdKinect = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");


	ros::Rate loop(10);

	// This is a endless-loop... Is doing nothing but the node is waiting for response the service
	// when a service request is recived, the node execute the function named "callbacjPCAobject"
	std::cout << "Waiting for image.." << std::endl;
	while( ros::ok() && cv::waitKey(15) != 27)
	{
		// ROS
		ros::spinOnce();
		loop.sleep();

		if( cv::waitKey(5) == 'q' )
			break;
	}
	cv::destroyAllWindows();
	return 0;
}
