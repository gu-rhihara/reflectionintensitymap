#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>

//#define dst480_640
using namespace std;

int main(int argc, char **argv)
{
/*** 出力画像の初期化 ***/
#ifdef dst480_640
  constexpr int W_i = 480;
  constexpr int H_i = 640;
  cv::Mat dst(H_i, W_i, CV_8UC1);
#else
  constexpr int W_i = 1000;
  constexpr int H_i = 1000;
#endif

  /*** 使用する定数 ***/
  const int range = 10;
  constexpr int X_min = -range;
  constexpr int X_max = range;
  constexpr int Y_min = -range;
  constexpr int Y_max = range;

  /*** 使用する変数 ***/
  double x, y, z, a, b;
  double px, py, pz, ox, oy, oz;
  double i, j, k, pi, pj;
  constexpr int W_x = Y_max - Y_min;
  constexpr int H_y = X_max - X_min;
  constexpr double i_a = (double)W_x / W_i;
  constexpr double j_b = (double)H_y / H_i;
  const double Range_min = 3.5;
  const double Range_max = 3.6;

  /*** getline関数を用いてデータを取得する ***/
  string line, s, str, line3;
  int file_cnt1 = 0;
  int file_cnt2 = 0;

  /*** 桐生キャンパスで取得したデータ (2021/12/03) ***/
  string path1 = "/home/user/lidar_data/12_03/2021_12_03_11_04_04/pandar_40/pandar_";
  string img_path1 = "/home/user/lidar_data/12_03/2021_12_03_11_04_04/image/center/";
  /*** 桐生キャンパスで取得したデータ (2022/08/01) ***/
  string path2 = "/home/user/lidar_data/map_20220801_133836_S-1-2-3-S/pandar_40/";
  string img_path2 = "/home/user/lidar_data/map_20220801_133836_S-1-2-3-S/image/";
  string location_path2 = "/home/user/lidar_data/map_20220801_133836_S-1-2-3-S/location/location.csv";
  /*** つくばで取得したデータ (2022/07/02) ***/
  string path = "/home/user/lidar_data/2022_07_02/2022_07_02_15_22_25/pandar_40/pandar_";
  string img_path = "/home/user/lidar_data/2022_07_02/2022_07_02_15_22_25/image/center/";

  /*** 点群画像とimg画像の作成 ***/
  while (1)
  {
    cv::Mat dst(H_i, W_i, CV_8UC3);                         //　点群画像
    cv::Mat dst_rf(H_i, W_i, CV_8UC3); // 芝生判定画像
    cv::Mat dst_hsv(H_i, W_i, CV_8UC3);                     // image画像
    for (int x = 0; x < dst.cols; x++)
    {
      for (int y = 0; y < dst.rows; y++)
      {
        dst.at<cv::Vec3b>(y, x) = 0;
        dst_rf.at<cv::Vec3b>(y, x) = 0;
        dst_hsv.at<cv::Vec3b>(y, x) = 0;
      }
    }
    /*** 点群画像の読み込み ***/
    char num_str1[10];
    snprintf(num_str1, sizeof(num_str1), "%06d", file_cnt1);
    /*** 点群データのlogファイルオープン ***/
    ifstream ifs(path2 + num_str1 + ".csv");
    file_cnt1++;
    // ifstream ifs("/home/user/lidar_data/pandar_40-20220601T065559Z-001/pandar_40/pandar_000399.dat"); //ファイル一個の読み込み
    if (!ifs)
    {
      cerr << "Failed to open file. " << endl;
      return -1;
    }

    /*** 桐生キャンパスのマップ読み込み ***/
    cv::Mat campus_map = cv::imread("/home/user/Downloads/campus_map.png");
    getline(ifs, line);       //入力文字列を取得しlineに格納
    stringstream line2(line); // stringstream型にする

    
    /*** locationの読み込み ***/
    ifstream ifs2(location_path2);

    /*** Lidar座標からマップ座標への変換式 ***/
    while (getline(ifs, str))
    {
      sscanf(str.c_str(), "%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &a, &b); // c_str()・・・string型からchar*型に変換, %lf・・・double型変数に対する入力
      //while (getline(ifs2, str))
      //{
      // sscanf(str.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &px, &py, &pz, &ox, &oy, &oz);
      //{
      //  sscanf(str.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &px, &py, &pz, &ox, &oy, &oz);
      if (x * x + y * y < range * range && x * x + y * y > 0)
      {
        // cout << str << endl;
        j = -(x / i_a) + (X_max / i_a);
        i = -(y / j_b) + (Y_max / j_b);
        // k = (255 * z + 255) / 2;
        k = 120 * b / 10;
        // cout << i << "," << j << endl;
        if (0 <= i && i < W_i && 0 <= j && j < H_i && -1 <= z && z <= 1 && b <= 5)
        {
          /*** 画像に描画する ***/
          dst.at<cv::Vec3b>(j, i)[0] = 120 - k;
          dst.at<cv::Vec3b>(j, i)[1] = 255;
          dst.at<cv::Vec3b>(j, i)[2] = 255;
          /*** Lidar座標からマップ座標への変換式 (location)***/
          // pj = px + 2250;
          // pi = py + 2250;

     /*** 拡大画像の表示 ***/
     cv::Mat img_map(500, 500, CV_8UC3);
     for(j = 0; j < 500; j++){
       for( i = 0; i < 500; i++){
         int Px,Py;
         Px = 2500;
         Py = 2500;
         img_map.at<cv::Vec3b>(j,i) = campus_map.at<cv::Vec3b>(j-250+Px, i-250+Py);
       }
     }
          /*** マップに描画 ***/
          img_map.at<cv::Vec3b>(j + 2500, i + 2500)[0] = dst_rf.at<cv::Vec3b>(j, i)[0];
          img_map.at<cv::Vec3b>(j + 2500, i + 2500)[1] = dst_rf.at<cv::Vec3b>(j, i)[1];
          img_map.at<cv::Vec3b>(j + 2500, i + 2500)[2] = dst_rf.at<cv::Vec3b>(j, i)[2];

          /*** マップに描画したものを保存する ***/
        }
        else if (x * x + y * y < range * range && x * x + y * y > 0 && b > 5)
        {
          dst.at<cv::Vec3b>(j, i)[0] = 240;
          dst.at<cv::Vec3b>(j, i)[1] = 255;
          dst.at<cv::Vec3b>(j, i)[2] = 255;
        }
        if (x * x + y * y < Range_max * Range_max && x * x + y * y > Range_min * Range_min && 0 <= i && i < W_i && 0 <= j && j < H_i && -1 <= z && z <= 1 && b > 5)
        {
          /*** 画像に描画する ***/
          dst_rf.at<cv::Vec3b>(j, i)[0] = 120 - k;
          dst_rf.at<cv::Vec3b>(j, i)[1] = 255;
          dst_rf.at<cv::Vec3b>(j, i)[2] = 255;
        }
      }
    }

    /*** img画像のデータ読み込み ***/
    char num_str2[10];
    snprintf(num_str2, sizeof(num_str2), "%06d", file_cnt2);
    cv::Mat img = cv::imread(img_path2 + num_str2 + ".png");
    // cout << "num: " <<num_str2 << ", cnt: " << file_cnt2 <<  endl;
    file_cnt2++;

    /*** dstを描画ウインドウに表示 ***/
    cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
    cv::namedWindow("preview1");
    cv::namedWindow("preview2");
    cv::imshow("preview1", dst);
    //resize(campus_map, campus_map, cv::Size(), 0.5, 0.5);
    cv::imshow("preview2", img_map);
    cv::imshow("preview3", dst_rf);
    /*** 0.5秒ごとに画像切り替わり ***/
    cv::waitKey(500);
  }
  return 0;
}
