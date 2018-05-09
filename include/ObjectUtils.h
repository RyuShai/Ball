#ifndef OJECT_UTILS_DATA_H
#define OJECT_UTILS_DATA_H

#include <iostream>
#include <vector>
#include "IOData.h"

using namespace cv;
using namespace std;

#define CLOCK_NOW std::chrono::system_clock::now
namespace pr
{
class ObjectUtils
{
private:
  Mat img;
  string test;

public:
  void wtfile_demo(string link, string content);
  string FloatToStr(float tt);
  string getCurrentDate();
  string getCurrentDateTime();
  string getCurrentDateTime_pushSQL();
  bool isplate(cv::Mat img);
  long convert_time(string s);
  int minResult(int x, int y, int z);
  int editDist(string str1, string str2, int m, int n);
  float calcBlurriness(const Mat &src);
  string kindcascade(int mode);
  int kindsizecascade(int mode);
  vector<Point2f> transformOffset(vector<Point2f> points, float x, float y);
  Point get_cross_points(Point pA, Point pB, Rect vehicle_rect, int direction);
  float up_down(Point p, Point pA, Point pB);
};
}
#endif