#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>
//#define dst480_640

using namespace std;

int main(int argc, char **argv)
{
/*** 出力画像の初期化 ***/
#ifdef p_c480_640
  constexpr int W_i = 480;
  constexpr int H_i = 640;
  cv::Mat p_c(H_i, W_i, CV_8UC1);
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
  const double Range_max = 10;

  /*** getline関数を用いてデータを取得する ***/
  string line, s, pc_str, location_str;
  int file_cnt1 = 0;
  int file_cnt2 = 0;
  int file_cnt3 = 0;
  int file_cnt4 = 0;

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
  /*** つくばで取得したデータ (2022/07/23) ***/
  string path3 = "/home/user/lidar_data/map_20220723_121659 S-4-5-6-10-9-15-14-13-12-11-S/sensor_20220723_121659 S-4-5-6-10-9-15-14-13-12-11-S/pandar_40/";
  string img_path3 = "/home/user/lidar_data/map_20220723_121659 S-4-5-6-10-9-15-14-13-12-11-S/sensor_20220723_121659 S-4-5-6-10-9-15-14-13-12-11-S/image/";
  string location_path3 = "/home/user/lidar_data/map_20220723_121659 S-4-5-6-10-9-15-14-13-12-11-S/sensor_20220723_121659 S-4-5-6-10-9-15-14-13-12-11-S/location/location.csv";
  /*** locationの読み込み (１行目飛ばす) ***/
  ifstream location_file(location_path3);
  getline(location_file, location_str);
  /*** resize count 1 ***/
  int resize_cnt = 0;
  /*** 桐生キャンパスのマップ読み込み ***/
  cv::Mat campus_map = cv::imread("/home/user/Downloads/campus_map.png");
  cv::Mat resized_map;
  /*** つくばのマップ読み込み ***/
  cv::Mat tsukuba_map = cv::imread("/home/user/Downloads/tsukuba_map.png");
  while (location_file)
  {
    cv::Mat p_c(H_i, W_i, CV_8UC3); //　点群画像
    // cv::Mat dst_hsv(H_i, W_i, CV_8UC3);
    cv::Mat dst_rf(H_i, W_i, CV_8UC3); // 芝生判定画像
    /*** 点群画像とimg画像の作成 ***/
    for (int x = 0; x < p_c.cols; x++)
    {
      for (int y = 0; y < p_c.rows; y++)
      {
        p_c.at<cv::Vec3b>(y, x) = 0;
        // dst_hsv.at<cv::Vec3b>(y, x) = 0;
      }
    }
    /*** 点群画像の読み込み ***/
    char num_str1[10];
    snprintf(num_str1, sizeof(num_str1), "%06d", file_cnt1);
    ifstream pc_ifs(path3 + num_str1 + ".csv");
    getline(pc_ifs, line);    //入力文字列を取得しlineに格納
    stringstream line2(line); // stringstream型にする
    file_cnt1++;
    // ifstream ifs("/home/user/lidar_data/pandar_40-20220601T065559Z-001/pandar_40/pandar_000399.dat"); //ファイル一個の読み込み
    // if (!pc_ifs)
    // {
    //   cerr << "Failed to open file. " << endl;
    //   return -1;
    // }

    /*** locationの読み込み ***/
    getline(location_file, location_str);
    // cout<< "location_data: " << location_str << endl;
    sscanf(location_str.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &px, &py, &pz, &ox, &oy, &oz);

    /*** locator座標から画像座標への変換式 ***/
    double x_i, y_i, res, c_r, s_r, r, x_c, y_c;
    res = 0.10;
    r = oz;
    c_r = (double)cos(r);
    s_r = (double)sin(r);

    /*** Lidar座標からマップ座標への変換式 ***/
    while (getline(pc_ifs, pc_str))
    {
      if (!location_file)
        cerr << "location file err" << endl;
      sscanf(pc_str.c_str(), "%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &a, &b); // c_str()・・・string型からchar*型に変換, %lf・・・double型変数に対する入力
      // 回転と平行移動 //
      x_c = (double)(x * c_r) - (y * s_r) + px;
      y_c = (double)(x * s_r) + (y * c_r) + py;
      x_i = (double)(tsukuba_map.rows / 2) - (y_c / res);
      y_i = (double)(tsukuba_map.cols / 2) - (x_c / res);

      // cout << "x_i: " << x_i << ", " << "y_i: " << y_i << endl;
      if (x * x + y * y < range * range)
      {
        j = -(x / i_a) + (X_max / i_a);
        i = -(y / j_b) + (Y_max / j_b);
        double h = -(1.5 * b) + 150;
        double calc_h = -(1.5 * b) + 150;
        // cout << "h: " << h << endl;
        // if (0 <= i && i < W_i && 0 <= j && j < H_i && z <= -1 && b <= 5)
        // {
        // /*** 画像に描画する ***/
        // p_c.at<cv::Vec3b>(j, i)[0] = 120;
        //  p_c.at<cv::Vec3b>(j, i)[1] = 255;
        //  p_c.at<cv::Vec3b>(j, i)[2] = 255;
        //  //   //cout<< i << "," << j << endl;
        // }
        // else if (x * x + y * y < range * range && z <= 0.15 && b > 5)
        // {
        //  p_c.at<cv::Vec3b>(j, i)[0] = 240;
        //  p_c.at<cv::Vec3b>(j, i)[1] = 255;
        //  p_c.at<cv::Vec3b>(j, i)[2] = 255;
        // }
        if (x * x + y * y < Range_max * Range_max && x * x + y * y > Range_min * Range_min && 0 <= i && i < W_i && 0 <= j && j < H_i && z <= 0.20)
        {
          /*** 画像に描画する ***/
          // dst_rf.at<cv::Vec3b>(j, i)[0] = 120 - k;
          // dst_rf.at<cv::Vec3b>(j, i)[1] = 255;
          // dst_rf.at<cv::Vec3b>(j, i)[2] = 255;
          /*** マップに描画 ***/
          // campus_map.at<cv::Vec3b>(y_i, x_i)[0] = 255;
          // campus_map.at<cv::Vec3b>(y_i, x_i)[1] = 0;
          // campus_map.at<cv::Vec3b>(y_i, x_i)[2] = 0;
          // tsukuba_map.at<cv::Vec3b>(y_i, x_i)[0] = h;     //B
          // tsukuba_map.at<cv::Vec3b>(y_i, x_i)[1] = 255;   //G
          // tsukuba_map.at<cv::Vec3b>(y_i, x_i)[2] = 255;   //R

          unsigned char R, G, B;
          if (h < 37.5)
          {
            B = 0;
            G = (double)6.8 * h;
            R = 255;
          }
          else if (h < 75)
          {
            B = 0;
            G = 255;
            R = (double)-(6.8 * h) + 510;
          }
          else if (h < 112.5)
          {
            B = (double)(6.8 * h) - 510;
            G = 255;
            R = 0;
          }
          else if (h < 150)
          {
            B = 255;
            G = (double)-(6.8 * h) + 1020;
            R = 0;
          }
          /*** calc_hの値が大きければ色を塗る ***/
          if (calc_h > h)
          {
            tsukuba_map.at<cv::Vec3b>(y_i, x_i)[0] = B;
            tsukuba_map.at<cv::Vec3b>(y_i, x_i)[1] = G;
            tsukuba_map.at<cv::Vec3b>(y_i, x_i)[2] = R;
          }
        }
      }
    }
    /*** img画像のデータ読み込み ***/
    char num_str2[10];
    snprintf(num_str2, sizeof(num_str2), "%06d", file_cnt2);
    cv::Mat img = cv::imread(img_path3 + num_str2 + ".png");
    // cout << "num: " <<num_str2 << ", cnt: " << file_cnt2 <<  endl;
    file_cnt2++;

    /*** 拡大画像の表示 ***/
    // cv::Mat img_map(500, 500, CV_8UC3);
    // for (j = 0; j < 500; j++)
    // {
    //   for (i = 0; i < 500; i++)
    //   {
    //     int Px, Py;
    //     Px = 2250;
    //     Py = 2250;
    //     img_map.at<cv::Vec3b>(j, i) = campus_map.at<cv::Vec3b>(j + Px, i + Py);
    //   }
    // }

    /*** p_cを描画ウインドウに表示 ***/
    cv::cvtColor(p_c, p_c, cv::COLOR_HSV2BGR);
    // cv::namedWindow("preview1");
    // cv::namedWindow("preview2");
    // resize(p_c, p_c, cv::Size(), 0.8, 0.8);
    // resize(dst_rf, dst_rf, cv::Size(), 0.8, 0.8);
    resize(tsukuba_map, resized_map, cv::Size(), 0.05, 0.05);
    cv::imshow("つくば", resized_map);
    // cv::imshow("点群画像", p_c);
    // cv::imshow("芝生画像", img);
    /*** 0.5秒ごとに画像切り替わり ***/
    cv::waitKey(1);
    // cout << file_cnt4 << endl;
    // file_cnt4++;
    //  resize_cnt++;
  }
  cv::imwrite("sample_tsukuba_ref_map8.png", tsukuba_map);
  return 0;
}
