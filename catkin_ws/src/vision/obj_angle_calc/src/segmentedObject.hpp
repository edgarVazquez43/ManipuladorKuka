#pragma once
#include <cmath>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class segmentedObject{
  
public:
  segmentedObject();
  segmentedObject(cv::Mat pointsBRG, cv::Mat pointsObject, int xMin, int yMin, int xMax, int yMax);

  bool getCentroid();
  bool getPrincipalAxis();

  cv::Mat pointsObject;
  cv::Mat pointsBRG;
  cv::Rect ROI;
  std::vector<float> centroid;
  std::vector<cv::Point3f> principalAxis;
  std::vector<float> colors;
  std::vector<float> dimensions;

  std::vector<float> standartDeviations;

private:

  
};

segmentedObject::segmentedObject()
{
	
}


segmentedObject::segmentedObject(cv::Mat pointsBRG, cv::Mat pointsObject, int xMin, int yMin, int xMax, int yMax)
{
  cv::Rect myCrop(xMin, yMin, xMax - xMin, yMax - yMin);
  this-> pointsBRG = pointsBRG;
  this-> pointsObject = pointsObject;
  this-> ROI = myCrop;
}

bool segmentedObject::getCentroid()
{
  int points_obj;
  float x_obj;
  float y_obj;
  float z_obj;
  cv::Point3f px;

  x_obj = 0.0;
  y_obj = 0.0;
  z_obj = 0.0;

  points_obj = 0;

  // Search the centroid of object PointCloud
  for(int j = 0; j < this->pointsObject.rows; j++)
    for (int i = 0; i < this->pointsObject.cols; i++)
      {
	px = this->pointsObject.at<cv::Point3f>(j,i);
	if ( px != cv::Point3f(0.0, 0.0, 0.0) && px != cv::Point3f(0, 255, 0))
	  {
	    x_obj += px.x;
	    y_obj += px.y;
	    z_obj += px.z;
	    points_obj++;
	    //std::cout << "x_obj: " << px.x << " - y_obj: " << px.y << " - z_obj: " << px.z << std::endl;
	  }
      }

  x_obj = x_obj/points_obj;
  this->centroid.push_back(x_obj);

  y_obj = y_obj/points_obj;
  this->centroid.push_back(y_obj);

  z_obj = (z_obj/points_obj) ;
  this->centroid.push_back(z_obj);

  return true;

}

bool segmentedObject::getPrincipalAxis()
{
  int n;
  float var_x;
  float var_y;
  float var_z;
  float cov_xy;
  float cov_xz;
  float cov_yz;
  std::vector<cv::Point3f> principal_comp;

  cv::Point3f px;
  cv::Point3f axis_1, axis_2, axis_3;

  Eigen::Matrix3f cov_matrix(3,3);
  //Eigen::Vector3f eivals;

  n = 0;
  var_x = 0.0;
  var_y = 0.0;
  var_z = 0.0;
  cov_xy = 0.0;
  cov_xz = 0.0;
  cov_yz = 0.0;

  //Calculate covariance matrix
  for(int j = 0; j < this->pointsObject.rows; j++)
    for (int i = 0; i < this->pointsObject.cols; i++)
      {
	px = this->pointsObject.at<cv::Point3f>(j,i);
	if ( px != cv::Point3f(0.0, 0.0, 0.0) && px != cv::Point3f(0, 255, 0))
	  {
	    var_x += pow( (px.x - this->centroid[0]), 2 );
	    var_y += pow( (px.y - this->centroid[1]), 2 );
	    var_z += pow( (px.z - this->centroid[2]), 2 );
	    cov_xy += ( px.x - this->centroid[0] )*( px.y - this->centroid[1] );
	    cov_xz += ( px.x - this->centroid[0] )*( px.z - this->centroid[2] );
	    cov_yz += ( px.y - this->centroid[1] )*( px.z - this->centroid[2] );
	    n++;
	  }
      }

  var_x /= n;
  var_y /= n;
  var_z /= n;

  cov_xy /= n;
  cov_xz /= n;
  cov_yz /= n;

  // Fill Covariance Matrix
  cov_matrix << var_x,  cov_xy, cov_xz,
    cov_xy,  var_y, cov_yz,
    cov_xz, cov_yz,  var_z;
  //std::cout << "    cov_matrix: " << std::endl << cov_matrix << std::endl;

  
  // Calculate eigenvalues and eigenvectors of covariance matrix
  // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov_matrix);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov_matrix);
  
  //std::cout << "    eigenvectors: " << std::endl << eig.eigenvectors().col(0) <<
  //			std::endl << eig.eigenvectors().col(1) << std::endl <<
  //			std::endl << eig.eigenvectors().col(2) << std::endl;

  //std::cout << "    eigenvalues: " << std::endl << eig.eigenvalues().transpose() << std::endl;

  axis_1 = cv::Point3f(eig.eigenvectors()(0,2),
		       eig.eigenvectors()(1,2),
		       eig.eigenvectors()(2,2));
  axis_2 = cv::Point3f(eig.eigenvectors()(0,1),
		       eig.eigenvectors()(1,1),
		       eig.eigenvectors()(2,1));
  axis_3 = cv::Point3f(eig.eigenvectors()(0,0),
		       eig.eigenvectors()(1,0),
		       eig.eigenvectors()(2,0));

  //Multiply axis for the corresponding standart deviation
  this->standartDeviations.push_back( sqrt(eig.eigenvalues()(0))*2 );
  this->standartDeviations.push_back( sqrt(eig.eigenvalues()(1))*2 );
  this->standartDeviations.push_back( sqrt(eig.eigenvalues()(2))*2 );

  this->principalAxis.push_back(axis_1* sqrt( eig.eigenvalues()(2) )*2.0);
  this->principalAxis.push_back(axis_2* sqrt( eig.eigenvalues()(1) )*2.0);
  this->principalAxis.push_back(axis_3* sqrt( eig.eigenvalues()(0) )*2.0);

  return true;

}
