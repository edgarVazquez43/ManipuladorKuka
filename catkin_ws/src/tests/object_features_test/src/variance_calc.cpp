#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR WORKSPACE RIGHT ARM BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "variance_calculator");
    ros::NodeHandle n;

    //std::fstream openFile("/home/edgar/ws_points.txt", std::ios_base::in);
    std::fstream openFile("/home/edgar/github/tesis/resultados/objetos/Alturas/height_cerealBox.txt", std::ios_base::in);

    int i=0;
    
    float x, h_edd, h_yisus;
    float h_eddMedian, h_yisusMedian;
    float errorRel_edd, errorRel_yisus;
    
    double var_edd, var_yisus;
    std::vector<float> eddList, yisusList;
    std::vector<float> ideal;

    ideal.resize(5);

    ideal[0] = 0.27;  // Cereal box
    ideal[1] = 0.05;  // Joystick
    ideal[2] = 0.15;  // Juice
    ideal[3] = 0.11;  // Milk
    ideal[4] = 0.03;  // Chocolate

    h_eddMedian = 0;
    h_yisusMedian = 0;
    
    for(int i = 0; i < 100; i++)
    {
      std::string line;
      std::getline(openFile, line);   
      std::istringstream in(line);      //make a stream for the line itself
      in >> h_edd >> h_yisus;      //now read the whitespace-separated floats

      h_eddMedian += h_edd;
      h_yisusMedian += h_yisus;
      
      eddList.push_back(h_edd);
      yisusList.push_back(h_yisus);  
    }

    h_eddMedian /= 100;
    h_yisusMedian /= 100;

    std::cout << "Median: " << std::endl
	      << h_eddMedian << "  "
	      << h_yisusMedian << std::endl;

    for(int i = 0; i < eddList.size(); i++)
    {
      var_edd += (eddList[i]-h_eddMedian)*(eddList[i]-h_eddMedian);
      var_yisus += (yisusList[i]-h_yisusMedian)*(yisusList[i]-h_yisusMedian);

      errorRel_edd += fabs(eddList[i]-ideal[0])/ideal[0];
      errorRel_yisus += fabs(yisusList[i]-ideal[0])/ideal[0];
    }

    var_edd /= 99;
    var_yisus /= 99;

    errorRel_edd /= 99;
    errorRel_yisus /= 99;

    std::cout << "Var: " << std::endl
	      << var_edd << "  "
	      << var_yisus << std::endl;

    std::cout << "Error relativo promedio: " << std::endl
	      << errorRel_edd << "  "
	      << errorRel_yisus << std::endl;
    
    return 0;
}
