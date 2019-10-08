#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "vision_msgs/DetectObjects.h"

visualization_msgs::Marker centroid_marker, pca1, pca2, pca3, vectPlane;

bool markerSetup()
{
    centroid_marker.header.frame_id = "base_link";
    pca1.header.frame_id = "base_link";
    pca2.header.frame_id = "base_link";
    pca3.header.frame_id = "base_link";
    vectPlane.header.frame_id = "base_link";
    
    centroid_marker.header.stamp = ros::Time::now();
    pca1.header.stamp = ros::Time::now();
    pca2.header.stamp = ros::Time::now();
    pca3.header.stamp = ros::Time::now();
    vectPlane.header.stamp = ros::Time::now();

    centroid_marker.ns = "centroid";
    pca1.ns = "principal axis1";
    pca2.ns = "principal axis2";
    pca3.ns = "principal axis3";
    vectPlane.ns = "principal axis3";
    
    centroid_marker.pose.orientation.w = 1.0;
    pca1.pose.orientation.w = 1.0;
    pca2.pose.orientation.w = 1.0;
    pca3.pose.orientation.w = 1.0;
    vectPlane.pose.orientation.w = 1.0;

    centroid_marker.id = 0;
    pca1.id = 1;
    pca2.id = 2;
    pca3.id = 3;
    vectPlane.id = 4;

    centroid_marker.type = visualization_msgs::Marker::SPHERE;
    pca1.type = visualization_msgs::Marker::ARROW;
    pca2.type = visualization_msgs::Marker::ARROW;
    pca3.type = visualization_msgs::Marker::ARROW;
    vectPlane.type = visualization_msgs::Marker::ARROW;

    // POINTS markers use x and y scale for width/height respectively
    centroid_marker.scale.x = 0.035;
    centroid_marker.scale.y = 0.035;
    centroid_marker.scale.z = 0.035;

    centroid_marker.color.r = 0.9f;
    centroid_marker.color.g = 0.9f;
    centroid_marker.color.b = 0.2f;
    centroid_marker.color.a = 1.0;

    
    pca1.scale.x = 0.015;     // Shaft diameter 
    pca1.scale.y = 0.035;    // Head diameter
    pca1.scale.z = 0.03;     // Head lenght
    pca1.color.b = 1.0f;
    pca1.color.a = 1.0; 

    pca2.scale.x = 0.015;
    pca2.scale.y = 0.035;
    pca2.scale.z = 0.03;
    pca2.color.g = 1.0f;
    pca2.color.a = 1.0;

    pca3.scale.x = 0.015;
    pca3.scale.y = 0.035;
    pca3.scale.z = 0.03;
    pca3.color.r = 1.0f;
    pca3.color.a = 1.0;

    vectPlane.scale.x = 0.015;
    vectPlane.scale.y = 0.035;
    vectPlane.scale.z = 0.03;
    vectPlane.color.r = 0.80f;
    vectPlane.color.g = 0.00f;
    vectPlane.color.b = 0.80f;
    vectPlane.color.a = 1.0;

    return true;
}

bool buildMarkerAxis(geometry_msgs::Vector3 PCA_axis_0,
		     geometry_msgs::Vector3 PCA_axis_1,
		     geometry_msgs::Vector3 PCA_axis_2,
		     geometry_msgs::Vector3 aux,
		     geometry_msgs::Pose centroid_pose)
{
  float alpha = 2.5;
  pca1.points.clear();
  pca2.points.clear();
  pca3.points.clear();
  vectPlane.points.clear();
    
  geometry_msgs::Point px_centroid;
  geometry_msgs::Point p_0, p_1, p_2, pPlane;
    
  px_centroid = centroid_pose.position;
  
  p_0.x = px_centroid.x + PCA_axis_0.x;
  p_0.y = px_centroid.y + PCA_axis_0.y;
  p_0.z = px_centroid.z + PCA_axis_0.z;
    
  p_1.x = px_centroid.x + PCA_axis_1.x;
  p_1.y = px_centroid.y + PCA_axis_1.y;
  p_1.z = px_centroid.z + PCA_axis_1.z;
    
  p_2.x = px_centroid.x + PCA_axis_2.x * alpha;
  p_2.y = px_centroid.y + PCA_axis_2.y * alpha;
  p_2.z = px_centroid.z + PCA_axis_2.z * alpha;

  pPlane.x = px_centroid.x + aux.x * alpha;
  pPlane.y = px_centroid.y + aux.y * alpha;
  pPlane.z = px_centroid.z + aux.z * alpha;
  
  pca1.points.push_back( px_centroid );
  pca1.points.push_back( p_0 );
    
  pca2.points.push_back( px_centroid );
  pca2.points.push_back( p_1 );
  
  pca3.points.push_back( px_centroid );
  pca3.points.push_back(p_2 );

  pPlane.x -= 0.10;
  px_centroid.x -= 0.10;
  vectPlane.points.push_back(px_centroid);
  vectPlane.points.push_back(pPlane);
  
  return true;
}




int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR GRASP OBJECT BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "pca_obj");
    ros::NodeHandle n;

    int count = 0;
    float objectYaw, objectYaw1;
    
    std::vector<float> alpha, beta, gamma;
    std::vector<float> mag_axis;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    geometry_msgs::Pose centroid;
    geometry_msgs::Vector3 axis_resp_0, axis_resp_1, axis_resp_2, aux;
    vision_msgs::DetectObjects srv_detectObj;
    
    ros::ServiceClient cltDetectObjectsPCA;
    ros::Publisher marker_pub;

    std::ofstream fileAngleObjs;

    
    cltDetectObjectsPCA = n.serviceClient<vision_msgs::DetectObjects>("vision/detect_object/PCA_calculator");
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    fileAngleObjs.open("/home/edgar/angle_cereal_90.txt");
    
    alpha.resize(3);
    beta.resize(3);
    gamma.resize(3);
    mag_axis.resize(4);

    
    markerSetup();

    ros::Rate loop(10);
    
    while(ros::ok())
    {
      
        if(!cltDetectObjectsPCA.call(srv_detectObj))
        {
            std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
            return false;
        }

        centroid = srv_detectObj.response.recog_objects[0].pose;
        axis_resp_0 = srv_detectObj.response.recog_objects[0].principal_axis[0];
        axis_resp_1 = srv_detectObj.response.recog_objects[0].principal_axis[1];
        axis_resp_2 = srv_detectObj.response.recog_objects[0].principal_axis[2];

        if( fabs(axis_resp_0.y) > fabs(aux.y)  )
	  aux = axis_resp_0;

	if( fabs(axis_resp_1.y) > fabs(aux.y)  )
	  aux = axis_resp_1;
	
	if( fabs(axis_resp_2.y) > fabs(aux.y)  )
	  aux = axis_resp_0;

	std::cout << "Axis on plane: " << std::endl
		  << aux << std::endl; 
	

	//Calculate magnitude principal axis
	mag_axis[0] = sqrt(axis_resp_0.x*axis_resp_0.x +
			   axis_resp_0.y*axis_resp_0.y +
			   axis_resp_0.z*axis_resp_0.z);

	mag_axis[1] = sqrt(axis_resp_1.x*axis_resp_1.x +
			   axis_resp_1.y*axis_resp_1.y +
			   axis_resp_1.z*axis_resp_1.z);

	mag_axis[2] = sqrt(axis_resp_2.x*axis_resp_2.x +
			   axis_resp_2.y*axis_resp_2.y +
			   axis_resp_2.z*axis_resp_2.z);
	
	mag_axis[3] = sqrt(aux.x*aux.x +
			   aux.y*aux.y +
			   aux.z*aux.z);

	//Calculate directions cosines
	alpha[0] = acos(axis_resp_0.x/mag_axis[0]);
	beta[0]  = acos(axis_resp_0.y/mag_axis[0]);
	gamma[0] = acos(axis_resp_0.z/mag_axis[0]);

	alpha[1] = acos(axis_resp_1.x/mag_axis[1]);
	beta[1]  = acos(axis_resp_1.y/mag_axis[1]);
	gamma[1] = acos(axis_resp_1.z/mag_axis[1]);

	alpha[2] = acos(axis_resp_2.x/mag_axis[2]);
	beta[2]  = acos(axis_resp_2.y/mag_axis[2]);
	gamma[2] = acos(axis_resp_2.z/mag_axis[2]);

	objectYaw = acos(aux.y/mag_axis[3])*180/3.141592 ;
	objectYaw1 = atan2(aux.y, aux.x)*180/3.141592;

	if(objectYaw > 90.0)
	  objectYaw = 180.0 - objectYaw;

	if(objectYaw1 > 90.0 || aux.y > 0.0)
	  objectYaw1 -= 90.0;

	if(aux.y < 0.0)
	  objectYaw1 += 90.0;


        std::cout << "Centroid: " << std::endl
		  << centroid.position << std::endl;
	
	std::cout << "Axis magnitude: " << std::endl
		  << mag_axis[0] << std::endl
		  << mag_axis[1] << std::endl
		  << mag_axis[2] << std::endl << std::endl;
	
	// std::cout << "Directions cosines : " << std::endl
	// 	  << "Axis[0]: alpha = " << alpha[0]
	// 	  << "  beta = " << beta[0]
	// 	  << "  gamma = " << gamma[0] << std::endl
	// 	  << "Axis[1]: alpha = " << alpha[1]
	// 	  << "  beta = " << beta[1]
	// 	  << "  gamma = " << gamma[1] << std::endl
	// 	  << "Axis[2]: alpha = " << alpha[2]
	// 	  << "  beta = " << beta[2]
	// 	  << "  gamma = " << gamma[2] << std::endl << std::endl;

	std::cout << "Directions cosines [ANGLES]: " << std::endl
		  << "Axis[0]: alpha = " << alpha[0]*180/3.141592
		  << "  beta = " << beta[0]*180/3.141592
		  << "  gamma = " << gamma[0]*180/3.141592 << std::endl
		  << "Axis[1]: alpha = " << alpha[1]*180/3.141592
		  << "  beta = " << beta[1]*180/3.141592
		  << "  gamma = " << gamma[1]*180/3.141592 << std::endl
		  << "Axis[2]: alpha = " << alpha[2]*180/3.141592
		  << "  beta = " << beta[2]*180/3.141592
		  << "  gamma = " << gamma[2]*180/3.141592 << std::endl << std::endl;

	std::cout << "  roll  = " << gamma[0]*180/3.141592
		  << "  pitch = " << beta[1]*180/3.141592
		  << "  yaw   = " << alpha[2]*180/3.141592 << std::endl << std::endl;

	std::cout << "object yaw:  " << objectYaw << std::endl << std::endl;
	std::cout << "object yaw:  " << objectYaw1 << std::endl << std::endl;


	fileAngleObjs << " " << objectYaw1 << " 90.0" <<"\n";
  
        centroid_marker.pose.position = centroid.position;
        buildMarkerAxis(axis_resp_0, axis_resp_1,
			axis_resp_2, aux, centroid);
	
	marker_pub.publish(centroid_marker);
        marker_pub.publish(pca1);
	marker_pub.publish(pca2);
	marker_pub.publish(pca3);
	marker_pub.publish(vectPlane);

	aux.x = 0.0;
	aux.y = 0.0;
	aux.z = 0.0;

	std::cout << "Count: " << count << std::endl;
	
        std::cout << std::endl << "---------------------------" << std::endl;
	count++;
	ros::spinOnce();
        loop.sleep();
    }
    fileAngleObjs.close();
    return 0;
}
