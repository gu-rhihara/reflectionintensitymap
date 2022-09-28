#include <opencv2/opencv.hpp>
#include <string>

int main(int argc, char **argv)
{
  /*** 画像を読み込む ***/
  std::string img_path ="/home/user/lidar_data/reflectionintensity_map/build/sample_tsukuba_map5";

  /*** 境界線を出す ***/
  {
    cv::Mat boundary;
    cv::Mat canny;
    /*** 画像をグレースケールで読み込み ***/
    boundary = cv::imread("img_path", 0);
    if(boundary.empty()){
      std::cout << "error" << std::endl;
      return 0;
    }
  Canny(boundary, canny, 50, 100);
  cv::imshow("boundary.png", canny);
}