#include <iostream>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include "justina_tools/JustinaTools.h"
#include "ros/ros.h"

#include "nn.hpp"
#include "plane3D.hpp"
#include "objExtract.hpp"
#include "findPlaneRansac.hpp"
#include "segmentedObject.hpp"
#include "vision_msgs/DetectObjects.h"

//std::ofstream myFile;

ros::ServiceClient cltRgbdRobot;
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
	std::string object_price;
	std::string object_calories;
	std::string object_grasas;
	std::string object_carbohidratos;
	
	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat planeBGR;
	cv::Mat objectsBGR;
	cv::Mat objectsDepth;
	cv::Mat croppedDepth;
	cv::Mat croppedBRG;

	cv::Vec4f planeComp;
	cv::Point3f px;

	plane3D bestPlane;

	// *** Parametros de RANSAC *** //
	attemps = 250;		// Numero de iteraciones para RANSAC
	threshold = 0.02;	// Distancia al plano en metros

	x_obj = 0.0;
	y_obj = 0.0;
	z_obj = 0.0;
	h_table = 0.0;

	points_obj = 0;

	xmin = 150;
	ymin = 170;

	W = 350;
	H = 280;

	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);



	if(!cltRgbdRobot.call(srv))
	{
		std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
		return -1;
	}

	JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);


	cv::Rect myROI(xmin, ymin, W, H);
	croppedDepth = imgDepth(myROI);
	croppedBRG = imgBGR(myROI);

	planeBGR = croppedBRG.clone();
	objectsBGR = croppedBRG.clone();


	// ##### Find best fit model to point cloud
	// Return plane with Normal = (1, 1, 1) is didn't find plane

	/****
	     AL READY MEASURE THE TIME
	 ***/
	// It already measure time
	clock_t begin = std::clock();
	bestPlane = FindPlaneRANSAC(croppedDepth, threshold, attemps );
	clock_t end = std::clock();
	double duration = double(end-begin) / CLOCKS_PER_SEC;
	// myFile << " " << duration << " " << bestPlane.inliers << "\n"; 

	// std::cout << "EDGAR PLANE plane" << std::endl
	// 	  << "Duration: " << duration << std::endl
	// 	  << "Inliers: " << bestPlane.inliers << std::endl
	// 	  << std::endl;

	// /* Code for coloring the plane
	if(bestPlane.GetNormal() != cv::Point3f(1.0, 1.0, 1.0) )
	{
		for(int j = 0; j < planeBGR.rows; j++)
			for (int i = 0; i < planeBGR.cols; i++)
			{
			  // Calculamos la distancia de cada uno de los puntos al plano
			  px = croppedDepth.at<cv::Point3f>(j, i);
			  // Camparamos si la distancia está dentro de la tolerancia
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
		object_1.getCentroid();
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
		
		dimensions = getFeatures(objectsBGR, objectsDepth); 

		resp.recog_objects[0].size.x = dimensions[0];
		resp.recog_objects[0].size.y = dimensions[1];
		resp.recog_objects[0].size.z = dimensions[2];

		resp.recog_objects[0].colors.x = dimensions[3]/255;
		resp.recog_objects[0].colors.y = dimensions[4]/255;
		resp.recog_objects[0].colors.z = dimensions[5]/255;

		// **************************//
		//   CODE FOR ARTIFICAL-NN   //
		
	        // analogVector.push_back(dimensions[0]);
		// analogVector.push_back(dimensions[1]);
		// analogVector.push_back(dimensions[2]);
	        // analogVector.push_back(dimensions[3]/255);
	        // analogVector.push_back(dimensions[4]/255);
	        // analogVector.push_back(dimensions[5]/255);

		// P = normalizingVector(analogVector);
		// // std::cout << "Caracteristicas del objeto:    " << std::endl;
		// // std::cout << "	Dimensiones: " << std::endl;
		// // std::cout << "		x:  " << object_1.dimensions[0] << "   " << P[0] << std::endl;
		// // std::cout << "		y:  " << object_1.dimensions[1] << "   " << P[1] << std::endl;
		// // std::cout << "		z:  " << object_1.dimensions[2] << "   " << P[2] << std::endl << std::endl;  
		// // std::cout << "	Color: " << std::endl;
		// // std::cout << "		Red:  "   << object_1.colors[0] << "   " << P[3] << std::endl;
		// // std::cout << "		Green:  " << object_1.colors[1] << "   " << P[4] << std::endl;
		// // std::cout << "		Blue:  "  << object_1.colors[2] << "   " << P[5] << std::endl; 
		
		// Tout = nn_calculate(P);

		// std::cout << "--------------------------------------" << std::endl;

		// if(Tout[0] == 0 && Tout[1] == 0 && Tout[2] == 0)
		// {
		// 	object_name = "Coca-cola";
		// 	object_price = "$ 12.00";
		// 	object_calories = "151 [kcal]";
		// 	object_grasas = "0.0 [g]";
		// 	object_carbohidratos = "38.9 [g]";
		// }
		// else if(Tout[0] == 0 && Tout[1] == 1 && Tout[2] == 1)
		// {
		// 	object_name = "Jugo";
		// 	object_price = "$ 18.00";
		// 	object_calories = "58 [kcal]";
		// 	object_grasas = "0.0 [g]";
		// 	object_carbohidratos = "15.0 [g]";
		// }
		// else if(Tout[0] == 0 && Tout[1] == 0 && Tout[2] == 1)
		// {
		// 	object_name = "Milk";
		// 	object_price = "$ 8.00";
		// 	object_calories = "139.5 [kcal]";
		// 	object_grasas = "4.7 [g]";
		// 	object_carbohidratos = "22.0 [g]";
		// }
		// else if(Tout[0] == 0 && Tout[1] == 1 && Tout[2] == 0)
		// {
		// 	object_name = "Desodorante 150 ml";
		// 	object_price = "$ 24.00";
		// 	object_calories = "Butano";
		// 	object_grasas = "Alcohol  - fragancia";
		// 	object_carbohidratos = "Agua";
		// }

		// cv::putText(croppedBRG, object_name, 
		// 	cv::Point(30,30),
		// 	cv::FONT_HERSHEY_COMPLEX_SMALL,
		// 	1.0,
		// 	cv::Scalar(200, 20, 0),
		// 	1,
		// 	CV_AA);

		// cv::putText(croppedBRG, object_price, 
		// 	cv::Point(30,60),
		// 	cv::FONT_HERSHEY_COMPLEX_SMALL,
		// 	1.0,
		// 	cv::Scalar(200, 20, 0),
		// 	1,
		// 	CV_AA);
		
		// cv::putText(croppedBRG, object_calories, 
		// 	cv::Point(30,90),
		// 	cv::FONT_HERSHEY_COMPLEX_SMALL,
		// 	1.0,
		// 	cv::Scalar(10, 30, 150),
		// 	1,
		// 	CV_AA);

		// cv::putText(croppedBRG, object_grasas, 
		// 	cv::Point(30,120),
		// 	cv::FONT_HERSHEY_COMPLEX_SMALL,
		// 	1.0,
		// 	cv::Scalar(10, 30, 150),
		// 	1,
		// 	CV_AA);
		
		// cv::putText(croppedBRG, object_carbohidratos, 
		// 	cv::Point(30,150),
		// 	cv::FONT_HERSHEY_COMPLEX_SMALL,
		// 	1.0,
		// 	cv::Scalar(10, 30, 150),
		// 	1,
		// 	CV_AA);


		cv::rectangle(imgBGR, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0), 1, 8, 0 );
		cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(50, 255, 0), 2, 8, 0 );
		cv::rectangle(croppedBRG, object_1.ROI, cv::Scalar(150, 50, 0), 2, 8, 0);

		
		cv::imshow("Original RGB", imgBGR);
		// cv::imshow("OBJECT RECONIZING", croppedBRG);
		// cv::imshow("plane", planeBGR);
		cv::imshow("objects", objectsBGR);
		// cv::imshow("Objects Point Cloud", objectsDepth);


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
	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");


	ros::Rate loop(10);

	// This is a endless-loop... Is doing nothing but the node is waiting for response the service
	// when a service request is recived, the node execute the function named "callbacjPCAobject"
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
