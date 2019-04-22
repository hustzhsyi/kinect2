#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
using namespace cv;
using namespace std;
int main()
{
  vector<Point2f>scene_corners(4);
  scene_corners[0] = cvPoint(213,14);
  scene_corners[1] = cvPoint(779, 105);
  scene_corners[2] = cvPoint(868, 516);
  scene_corners[3] = cvPoint(95, 507);

  Mat dst;
  Mat img=imread("/home/zhsyi/camera/src/camera2/src/1.jpg",1);
  Mat roi=Mat::zeros(img.size(),CV_8U);
  vector<vector <Point2f> >contour;
  contour.push_back(scene_corners);
  drawContours(roi,contour,-1,Scalar::all(255),3);
  img.copyTo(dst,roi);
  //imshow("roi",roi);
  imshow("img",img);
  waitKey(1000);
  imshow("dst",dst);
  waitKey(1000);

  return 0;
}
