#include "opencv2/opencv.hpp"
#include "iostream"
#include "zimiaoshiyan/define.h"
#pragma once
using namespace cv;
using namespace std;

namespace rm
{
  class Result_Show
  {
public:
    Scalar COLOR_BLUE = Scalar(255, 0, 0);
    Scalar COLOR_RED = Scalar(0, 0, 255);
    Scalar COLOR_GREEN = Scalar(0, 255, 0);
    Scalar COLOR_YELLOW = Scalar(0, 255, 255);
    Scalar COLOR_ORANGE = Scalar(0, 97, 255);
    Scalar COLOR_PURPLE = Scalar(240, 32, 160);

    Size RSIZE = Size(WIDTH, HEIGHT); //show image resolution

    bool makeRectSafe(cv::Rect &rect, cv::Size size)
    {
      if (rect.x < 0)
        rect.x = 0;
      if (rect.width > size.width)
        rect.width = size.width;
      if (rect.x + rect.width > size.width)
        rect.x = size.width - rect.width;
      if (rect.y < 0)
        rect.y = 0;
      if (rect.height > size.height)
        rect.height = size.height;
      if (rect.y + rect.height > size.height)
        rect.y = size.height - rect.height;
      if (rect.width <= 0 || rect.height <= 0)
      {

         rect.x = 0;
         rect.y = 0;
         rect.width = size.width;
         rect.height = size.height;

         return false;
      }
      return true;
    }

    void to_show_img(Mat img, string windowNames, Size size)
    {
      cv::namedWindow(windowNames, 0);
      cv::resizeWindow(windowNames, size.width, size.height);
      cv::imshow(windowNames, img);
      cv::waitKey(1);
    }

    void to_draw_RotatedRect(RotatedRect Rec, Mat draw_show, Scalar color)
    {
      cv::Point2f *p = new cv::Point2f[4];
      Rec.points(p);
      for (int i = 0; i < 4; i++)
        cv::line(draw_show, p[i], p[(i + 1) % 4], color, 4, 0);
    }

    void to_draw_rect(Rect rec, Mat draw_show, Scalar color)
    {
      rectangle(draw_show, rec, color, 4, LINE_8, 0);
    }
  };
};
