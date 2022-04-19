#pragma once

#define Log(n, str) std::cout << "\033[" << n << ";40H" << str << "\033[0m" << std::endl
#define LOG 1

#define DIST(p1, p2) std::sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))

#define USE_360P
//#define USE_720P

#ifndef USE_720P
#define WIDTH 640
#define HEIGHT 480
#define RESOLUTION_SCALE 1
#else
#define WIDTH 1280
#define HEIGHT 720
#define RESOLUTION_SCALE 2
#endif

#define DEBUG 1

#define RECORD 0

#define USE_WRONG_P4P_POINTS 0
#define ARMOR_SIZE_SCALE 0.75
/*********************ARMOR*******************/
#define LIGHT_BAR_SPLIT 1*0
#define LIGHT_BAR_BINARY 1*0
#define LIGHT_BAR_SPLIT_ERODE 1*0

#define LIGHT_BAR_RECT 0    // the rotated rect of the contour
#define LIGHT_BAR_CONTOUR 0 // the corners of the contour

#define RECT_MATCH 1*0
#define TARGET_SHOW 1

/*********************WIND*******************/

#define WIND_THRESOLD_FINAL 0
#define WIND_TARGET_CIRCLE_CENTER 0

#define WIND_TARGET_PREDICT 0

#define ANGLE_PERSPECTIVE_COMPARE 0
