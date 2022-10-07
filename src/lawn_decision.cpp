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
  std::string img_path ="/home/user/lidar_data/reflectionintensity_map/build/sample_tsukuba_ref_map5.png";

  /*** 境界線を出す ***/
  {
    cv::Mat boundary;
    cv::Mat canny;
    /*** 画像をグレースケールで読み込み ***/
    boundary = cv::imread("/home/user/lidar_data/reflectionintensitymap/build/boundary_5.png");
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
  cv::imwrite("boundary8.png", canny); 
}
}