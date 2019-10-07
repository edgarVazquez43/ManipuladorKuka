#pragma once
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "plane3D.hpp"
#include "segmentedObject.hpp"

float CalculateZPlane(plane3D plane, cv::Mat points);
segmentedObject ExtractObj(plane3D plane, cv::Mat pointsBRG, cv::Mat points);
std::vector<segmentedObject> clusterObjects(cv::Mat pointsBRG, cv::Mat pointsObject);

std::vector<float> getFeatures(cv::Mat pointsBRG, cv::Mat pointsObject);


float CalculateZPlane(plane3D plane, cv::Mat points)
{
	int z_numbers;
	float z;

	cv::Point3f px;

	z_numbers = 0;
	z = 0.0;

	// Delete the points on the plane ///
	for(int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			px = points.at<cv::Point3f>(i, j);
			if( plane.DistanceToPoint(px, false) < 0.2)
			{
				z_numbers++;
				z += px.z;
			}
		}

	return z / z_numbers;;
}


segmentedObject ExtractObj(plane3D plane,cv::Mat pointsBRG ,cv::Mat points)
{
	int z_numbers;
	int x_min, x_max;
	int y_min, y_max;
	float z_plane;
	float threshold;

	cv::Mat objectsPC;
	cv::Mat objectBRG;
	cv::Point3f px;

	z_numbers = 0;
	x_min = 1000;
	x_max = 0;
	y_min = 1000;
	y_max = 0;
	z_plane = 0.0;
	threshold = 0.02;

	objectsPC = points.clone();
	objectBRG = pointsBRG.clone();

	//std::cout << "obj_extr--->  planeComp:  " << plane.GetPlaneComp() << std::endl;
	//std::cout << "obj_extr--->  points_size: " << points.size() << std::endl;

	// Delete the points on the plane ///
	for(int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			px = points.at<cv::Point3f>(i, j);
			if( plane.DistanceToPoint(px, false) < threshold)
			{
				z_numbers++;
				z_plane += px.z;
				objectsPC.at<cv::Point3f>(i, j) = cv::Point3f(0.0, 0.0, 0.0);
				objectBRG.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
			}
		}

	z_plane = z_plane / z_numbers;

	// Delete points under plane
	for(int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			px = points.at<cv::Point3f>(i, j);
			if( px.z < (z_plane - 0.01) || px.x < 0.10 || px.x > 0.80)
			{
				objectsPC.at<cv::Point3f>(i, j) = cv::Point3f(0.0, 0.0, 0.0);
				objectBRG.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
			}
		}


	//  ### Code for crop objects from image

	for(int i = 0; i < objectsPC.rows; i++)
		for(int j = 0; j < objectsPC.cols; j++)
		{
			if( objectsPC.at<cv::Point3f>(i, j) != cv::Point3f(0.0, 0.0, 0.0) )
			{
				y_min = (i < y_min) ? i : y_min;
				x_min = (j < x_min) ? j : x_min;
				y_max = (i > y_max) ? i : y_max;
				x_max = (j > x_max) ? j : x_max;
			}
		}

	cv::rectangle(objectsPC, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(0, 255, 0));

	if(y_min == 1000 && x_min == 1000 && x_max == 0 && y_max == 0)
		objectsPC = cv::Mat(50, 50, CV_8UC3);
	else
	{
		cv::Rect myCrop(x_min, y_min, x_max -x_min, y_max - y_min);
		objectsPC = objectsPC(myCrop);
		objectBRG = objectBRG(myCrop);
	}

	segmentedObject object_1(objectBRG, objectsPC, x_min, y_min, x_max, y_max);

	return object_1;
}


std::vector<segmentedObject> clusterObjects(cv::Mat pointsBRG, cv::Mat pointsObject)
{
	float threshold = 0.0002;
	float norma_i = 0.0;
	float norma_i_1 = 100000.0;

	cv::Point3f px;
	cv::Mat pointsObj_1;

	std::vector<segmentedObject> objectList;

	pointsObj_1 = pointsObject.clone();


	for(int j = 0; j < pointsObject.rows; j++)
		for (int i = 0; i < pointsObject.cols; i++)
		{
			px = pointsObject.at<cv::Point3f>(j,i);
			if ( px != cv::Point3f(0.0, 0.0, 0.0) && px != cv::Point3f(0, 255, 0))
			{
				norma_i = sqrt(px.x*px.x + px.y*px.y + px.z*px.z);
				std::cout << "norma_i: " << norma_i << std::endl;
				std::cout << "norma_i_1: " << norma_i_1 << std::endl;
				std::cout << "V_abs: " << fabs(norma_i_1 - norma_i) << std::endl;
				if ( fabs(norma_i_1 - norma_i) < threshold )
					pointsObj_1.at<cv::Point3f>(j,i) = cv::Point3f(100, 255, 100);
				else
					pointsObj_1.at<cv::Point3f>(j,i) = cv::Point3f(0.0, 0.0, 0.0);
			}
			norma_i_1 = norma_i;
		}

	/*
	for(int i = 0; i < objectsDepth.rows; i++)
		for(int j = 0; j < objectsDepth.cols; j++)
		{
			if( objectsDepth.at<cv::Point3f>(i, j) != cv::Point3f(0.0, 0.0, 0.0) )
			{
				y_min = (i < y_min) ? i : y_min;
				x_min = (j < x_min) ? j : x_min;
				y_max = (i > y_max) ? i : y_max;
				x_max = (j > x_max) ? j : x_max;
			}
		}
	*/


	segmentedObject object_1(pointsBRG, pointsObj_1, 0, 0, 100, 100);

	objectList.push_back(object_1);

	return objectList;
}

std::vector<float> getFeatures(cv::Mat pointsBRG, cv::Mat pointsObject)
{
	cv::Vec3b intensity;
	int blue;
	int green;
	int red;

	float xMin, xMax;
	float yMin, yMax;
	float zMin, zMax;

	float x_lenght;
	float y_lenght;
	float z_lenght;

	float r;
	float g;
	float b;

	int pixelNumber = 0;

	xMin = 10000000.0;
	yMin = 10000000.0;
	zMin = 10000000.0;

	xMax = 0.0;
	xMax = 0.0;
	xMax = 0.0;

	blue = 0;
	red = 0;
	green = 0;


	std::vector<float> features;
	cv::Point3f px;

	for(int j = 0; j < pointsObject.rows; j++)
		for (int i = 0; i < pointsObject.cols; i++)
		{
			px = pointsObject.at<cv::Point3f>(j,i);
			intensity = pointsBRG.at<cv::Vec3b>(j, i);
			if ( px != cv::Point3f(0.0, 0.0, 0.0) && px != cv::Point3f(0, 255, 0))
			{
				xMin = (px.x < xMin) ? px.x : xMin;
				xMax = (px.x > xMax) ? px.x : xMax;
				
				yMin = (px.y < yMin) ? px.y : yMin;
				yMax = (px.y > yMax) ? px.y : yMax;
				
				zMin = (px.z < zMin) ? px.z : zMin;
				zMax = (px.z > zMax) ? px.z : zMax;
				
				blue  += intensity.val[0];
				green += intensity.val[1];
				red   += intensity.val[2];

				pixelNumber++;
			}
		}

	x_lenght = xMax - xMin;
	y_lenght = yMax - yMin;
	z_lenght = zMax - zMin;

	blue = (blue/pixelNumber);
	green = (green/pixelNumber);
	red = (red/pixelNumber);

	/*
	std::cout << "x_lenght: " << xMax - xMin << std::endl;
	std::cout << "y_lenght: " << yMax - yMin << std::endl;
	std::cout << "z_lenght: " << zMax - zMin << std::endl;

	std::cout << "blue: "  << blue  << std::endl;
	std::cout << "green: " << green << std::endl;
	std::cout << "red: "   << red   << std::endl;
	*/

	features.push_back(x_lenght);
	features.push_back(y_lenght);
	features.push_back(z_lenght);

	//Valores RGB medios
	features.push_back(red);
	features.push_back(green);
	features.push_back(blue);

	return features;
}
