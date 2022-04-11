#pragma once

#define Log(n, str) std::cout << "\033[" << n << ";40H" << str << "\033[0m" << std::endl
#define LOGG 0
#define WIDTH 1280
#define HEIGHT 1024

#define DEBUG 1

#define DEBUG_PLOT 0

#define RECORD 0
#define USE_LIGHTBAR_P4P_POINTS 0

#define USE_MIX_ROTATED_RECT 1

#define USE_SOFT_TRIGGER o
/*********************ARMOR*******************/

#define LIGHT_BAR_BINARY 0
#define LIGHT_BAR_SPLIT_ERODE 0
#define LIGHT_BAR_RECT 0    // the rotated rect of the contour
#define LIGHT_BAR_CONTOUR 0 // the corners of the contour
#define WRONG_LIGHTBAR 0
#define RECT_MATCH 0
#define WRONG_RECT 0
#define TARGET_SHOW 0
#define USE_SVM 0
#define SVM_IMG 0
//#define TEST_VIDEO  "/media/star/whx1/2021-RMUC-SZGS-CB02.mp4"
/*********************WIND*******************/

#define WIND_THRESOLD_FINAL 0
#define WIND_TARGET_CIRCLE_CENTER 0
#define WIND_TARGET_PREDICT 0

#define ANGLE_PERSPECTIVE_COMPARE 0

/*********************GLOG*******************/

//#include "glog/logging.h"   // glog 头文件
//#include "glog/raw_logging.h"
