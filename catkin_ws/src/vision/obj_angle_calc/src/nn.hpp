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

std::vector<int> normalizingVector(std::vector<float> analogVector);
std::vector<int> nn_calculate(std::vector<int> P);


std::vector<int> normalizingVector(std::vector<float> analogVector)
{
	std::vector<int> p;

	if (analogVector[0] > 0.035)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[1] > 0.06)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[2] > 0.105)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[3] > 0.55)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[4] < 0.38)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[5] > 0.0275)
		p.push_back(1);
	else
		p.push_back(0);

	return p;
}


std::vector<int> nn_calculate(std::vector<int> Pin)
{
	std::vector<int> a;
	a.push_back(0);
	a.push_back(0);
	a.push_back(0);

	Eigen::MatrixXf Pt_e(6,1);
	Eigen::MatrixXf T(3,4);
	Eigen::MatrixXf T_s(3,1);
  	Eigen::MatrixXf P(6,4);
  	Eigen::MatrixXf Pt(4,6);
  	Eigen::MatrixXf P_x_Pt(4,4);
  	Eigen::MatrixXf W(3,6);

  	// Vector de entrada propuesto
  	/* 
  	Pt_e << 0,
    		0,
    		1,
    		0,
    		1,
    		1;
	*/

   	//Vector de entrada Real
	Pt_e << Pin[0],
    		Pin[1],
    		Pin[2],
    		Pin[3],
    		Pin[4],
    		Pin[5]; 
  	
  	//Vector de salida propuesto
  	T << 0, 0, 0, 0,
	     0, 1, 0, 1,
	     0, 0, 1, 1;


  	//std::cout << "T = " << std::endl << T << std::endl;

  	P << 1, 1, 1, 0,
	     0, 1, 1, 0,
	     0, 1, 0, 1,
    	     1, 0, 0, 1,
    	     1, 1, 1, 0,
	     1, 1, 1, 0;
    //std::cout << "P = " << std::endl << P << std::endl;

    //P -- transpuesta
   	Pt << 1, 0, 0, 1, 1, 1,
       	      1, 1, 1, 0, 1, 1,
       	      1, 1, 0, 0, 1, 1,
       	      0, 0, 1, 1, 0, 0;
    //std::cout << "P_t = " << std::endl << Pt << std::endl;


    // Red neuronal con aprendizaje Hebbiano
    // y regla de la pseudoinversa
    P_x_Pt = Pt*P;				// = 
    W = ( T*P_x_Pt.inverse() )*Pt ;

    // Calculo del vector de salida	
	T_s = W*Pt_e;


	//Program of Hardlim Function
	if( T_s(0) >= 0.00005)
		a[0] = 1;
	else if( T_s(0) < 0.00005)
		a[0] = 0;

	if( T_s(1) >= 0.00005)
		a[1] = 1;
	else if( T_s(1) < 0.00005)
		a[1] = 0;

	if( T_s(2) >= 0.00005)
		a[2] = 1;
	else if( T_s(2) < 0.00005)
		a[2] = 0;

	std::cout << "P_in=  " << Pt_e << std::endl;
	std::cout << "T_s=  " << std::endl
			  << a[0] << std::endl
			  << a[1] << std::endl
			  << a[2] << std::endl;
    return a;
}

