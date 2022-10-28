#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

int main(int argc, char **argv)
{
  /*** 画像を読み込む ***/
  // std::string img_path ="/home/user/lidar_data/reflectionintensity_map/build/map_tsukuba_color2.png";

  /*** 境界線を出す ***/
  {
    cv::Mat boundary;
    cv::Mat canny;
    /*** 画像をグレースケールで読み込み ***/
    boundary = cv::imread("/home/rhihara/reflectionintensitymap/build/sensor_20221023_123902.png");
    if(boundary.empty()){
      std::cout << "error" << std::endl;
      return 0;
    }
  Canny(boundary, canny, 900, 1000);
  // cv::imshow("boundary.png", canny);
  for(int x = 0; x < canny.cols; x++){
    for(int y = 0; y < canny.rows; y++){
  if(boundary.at<cv::Vec3b>(y, x)[0] = 255){
    if(boundary.at<cv::Vec3b>(y, x)[2] = 0){
    canny.at<cv::Vec3b>(y,x)[0] = 0;
    canny.at<cv::Vec3b>(y,x)[1] = 0;
    canny.at<cv::Vec3b>(y,x)[2] = 255;
    }
  }
    }
  }
  cv::imwrite("sensor_20221023_123902_gray.png", canny); 
}
}