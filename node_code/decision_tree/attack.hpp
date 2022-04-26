#pragma once
#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>
#include <string>

#include "TCP.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#define WINDOW_NAME "watch_map"
#define USE_TCP 1
#define PI 3.1415926

#define IS_CROSS(x1,y1,x2,y2,x3,y3,x4,y4)								\
		do																\
		{																\
			is_cross_ans = 0;											\
			ab_ac = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);		\
			ab_ad = (x2 - x1) * (y4 - y1) - (x4 - x1) * (y2 - y1);		\
			dc_da = (x3 - x4) * (y1 - y4) - (x1 - x4) * (y3 - y4);		\
			dc_db = (x3 - x4) * (y2 - y4) - (x2 - x4) * (y3 - y4);		\
			if ((ab_ac * ab_ad <= 0) && (dc_da * dc_db <= 0))			\
			{															\
				is_cross_ans = 1;										\
			}															\
		}while (0)														\

#define IS_MEET(x1,y1,nowx,nowy)																											\
	do																																		\
	{																																		\
		is_meet_ans = 1;																													\
		for (int i = 1; i <= 35; i++){																										\
			IS_CROSS(x1, y1, nowx, nowy, all_wall_line[i].c1x, all_wall_line[i].c1y, all_wall_line[i].c2x, all_wall_line[i].c2y);			\
			if (is_cross_ans == 1){is_meet_ans = 0;}																						\
		}																																	\
	}while(0)																																\
	
/*------Class define-------*/

//其实就是line
class wall_line
{
public:
	int c1x;
	int c1y;
	int c2x;
	int c2y;
	wall_line() { }
	wall_line(int c1x_, int c1y_, int c2x_, int c2y_)
	{
		c1x = c1x_;
		c1y = c1y_;
		c2x = c2x_;
		c2y = c2y_;
	}
};

class Strike_Point
{
	double dis;
	double angle;
};

//class My_pair
//{
//public:
//	short a;
//	short b;
//	My_pair() {};
//	My_pair(short aa, short bb) { a = aa; b = bb; }
//};
int can_strike_angle[2001];
/*-------------------------*/


/*------Const define---------*/
double Optimum_strike_distance = 300;	//(cm)
double Optimum_strike_distance_pix = Optimum_strike_distance / 2;
double Min_strike_distance = 70;   //(cm)
double Min_strike_distance_pix = Min_strike_distance / 2;
double Max_strike_distance = 550;   //(cm)
double Max_strike_distance_pix = Max_strike_distance / 2;
double Best_strike_distance = 170;   //(cm)
double Best_strike_distance_pix = Best_strike_distance / 2;
double Best_strike_distance2 = 270;   //(cm)
double Best_strike_distance_pix2 = Best_strike_distance2 / 2;
/*-------------------------*/


/*------Var define---------*/
wall_line all_wall_line[36], erode_wall_line[25];
cv::Mat map_save, erode_map_save, pure_fire_map, white_map;
cv::Mat fire_map(226, 406, CV_8UC1, 1);
//存储该角度、该距离下是否是可打击区域
bool circle_strike_area_flag[2000];
/*-------------------------*/


/*------Function decare---------*/
void init();
void system_init();
void mouse_handle(int event, int x, int y, int flags, void* param);
double is_met(int x1, int y1, int nowx, int nowy);
bool is_cross(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4);
void Update_Apriltag_Callback();
void Update_Radar_Callback();
//用途：获得火力地图
//输入：敌方1号(x1,y1)，二号(x2,y2)；坐标均为自定义坐标系下的坐标
//返回值：Mat类型的火力地图，接敌区域使用100灰度标出
//副产物：全局变量pure_fire_map，原本是纯白，计算之后接敌用黑色标注，对其进行膨胀得到合适的打击区域
cv::Mat get_fire_map(int x1, int x2, int y1, int y2);
void Update_Odom_Callback();
void Update_UltrasonicRanging_Callback();
void init_load_erode_wall_line();
void init_load_wall_line();
void refresh_window(int x, int y);
cv::Mat dilate_strike_area(int enemyx, int enemyy);
void pick_strike_area(int enemyx, int enemyy);
/*-------------------------*/


/*------Function define---------*/
void init()
{
	init_load_wall_line();
	init_load_erode_wall_line();
	map_save = cv::imread("map.png", -1);
	std::cout << "imgrows:" << map_save.rows << "    imgcols: " << map_save.cols << std:: endl;
	white_map = cv::Mat(map_save.rows, map_save.cols, CV_8UC1, 255);
	pure_fire_map = cv::Mat(map_save.rows, map_save.cols, CV_8UC1, 255);
	//腐蚀操作取结构元所指定的领域内值的最小值作为该位置的输出灰度值
	cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(35, 35), cv::Point(-1, -1));
	erode(map_save, erode_map_save, structureElement);
	cv::imshow(WINDOW_NAME, map_save);
	cv::waitKey(10000000);
	printf("listen to server");
	return;
}

cv::Point search_this_angle_area(int enemyx, int enemyy, int r_min,int r_max)
{
	memset(circle_strike_area_flag, 0, sizeof(circle_strike_area_flag));
	memset(can_strike_angle, 0, sizeof(can_strike_angle));

	int xx = 0; int imgrows = map_save.rows;
	int yy = 0; int imgcols = map_save.cols;
	/*cv::imshow("adad", fire_map);
	cv::imshow("adad2", pure_fire_map);*/
	int count = 0;
	//是不是每一个角度都不行
	bool every_angle_flag = 0;
	bool find_angle_flag = 0;
	uchar* map_data = (uchar*)pure_fire_map.data;
	uchar* map_data2 = (uchar*)fire_map.data;
	for (double angle = 0; angle <= 3.1415 * 2.0; angle += 0.005)
	{
		every_angle_flag = 1;
		for (double r = r_min; r <= r_max; r += 1)
		{
			yy = int(enemyx + r * sin(angle));
			xx = (enemyy + int(r * cos(angle)));//这是在图像中的第几行，即x
			if (xx * yy <= 0)
			{
				if (xx <= 0) xx = 0;
				if (yy <= 0) yy = 0;
			}
			if (xx >= imgcols) xx = imgcols;
			if (yy >= imgrows) yy = imgrows;
			//cv::circle(fire_map, cv::Point(xx, yy), 10, cv::Scalar(0), 5);
			//cv::circle(fire_map, cv::Point(enemyy, enemyx), 10, cv::Scalar(100), 5);
			//return;
			//*(map_data2 + xx + yy * imgcols) = 200;
			//可以打到并且不是墙
			if ((*(map_data + xx + yy * imgcols) != 0) && (*(map_data2 + xx + yy * imgcols) != 0))
			{
				circle_strike_area_flag[count] = 0;
				every_angle_flag = 0;
				count++;
				break;
			}
			*(map_data2 + xx + yy * imgcols) = 200;
		}
		if (every_angle_flag == 1)
		{
			circle_strike_area_flag[count] = 1;
			yy = int(enemyx + ((r_min + r_max) / 2) * sin(angle));
			xx = (enemyy + int(((r_min + r_max) / 2) * cos(angle)));//这是在图像中的第几行，即x
			if (xx * yy <= 0) circle_strike_area_flag[count] = 0;
			if (xx >= imgcols) circle_strike_area_flag[count] = 0;
			if (yy >= imgrows) circle_strike_area_flag[count] = 0;
			find_angle_flag = 1;
			count++;
		}
	}
	count--;
	return cv::Point(find_angle_flag, count);
}

//这一部分是否可以、有必要并入画打击图部分的函数？？
void pick_strike_area(int enemyx, int enemyy)
{
	memcpy((uchar*)fire_map.data, (uchar*)map_save.data, map_save.total() * sizeof(uchar));
	//这里今后改成clone
	cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10), cv::Point(-1, -1));
	cv::dilate(pure_fire_map, pure_fire_map, structureElement);

	cv::Point return_value = search_this_angle_area(enemyx, enemyy, Best_strike_distance_pix, Best_strike_distance_pix2);
	//std::cout << return_value.x << ' ' << return_value.y << std::endl;
	if (return_value.x == 0)
	{
		std::cout << "Narrow scope of Strike, and searching..." << std::endl;
		return_value = search_this_angle_area(enemyx, enemyy, Min_strike_distance_pix, Best_strike_distance_pix);
		if (return_value.x == 0)
		{
			std::cout << "Cannot find right area!!" << std::endl;
			return;
		}
	}

	int count = return_value.y;
	//寻找连续打击区域
	int beginof1 = 0; bool finding1 = 0;
	int ii = 0;//遍历角度指针
	int jj = 0;//可打击角度数组指针
	int begin_position;

	while (circle_strike_area_flag[ii] == 1 && ii <= count - 1) //找到第一个0
		ii++;

	begin_position = ii;

	while (1)
	{
		if (circle_strike_area_flag[ii % count] == 1 && finding1 == 0)
		{
			finding1 = 1;
			beginof1 = ii;
		}
		if (circle_strike_area_flag[ii % count] == 0) {
			if (finding1 == 1)
			{
				jj++;
				can_strike_angle[jj] = ((ii + beginof1) / 2) % count;
				finding1 = 0;
			}
		}
		//std::cout << ii << ' ' << finding1 << ' ' << beginof1 << ' ' << jj << std::endl;
		if (ii % count == begin_position && ii >= count)break;
		ii++;
	}

	//for (int i = 1; i <= jj; i++)
	//	std::cout << can_strike_angle[i] << std::endl;
	//std::cout << "find area num:  " << jj << std::endl;

	double angle1;
	double my_r = (Best_strike_distance_pix2 + Best_strike_distance_pix) / 2.0;

	int mmin_y = 9999;
	int mmin_i = 0;

	for (int i = 1; i <= jj; i++) {
		//取出最靠左边的点来拍视频，同时实战中也是离家近方便直接润
		if (enemyy + int(my_r * cos(can_strike_angle[i] * 0.005)) < mmin_y) {
			mmin_y = enemyy + int(my_r * cos(can_strike_angle[i] * 0.005));
			mmin_i = i;
		}

		cv::circle(fire_map, cv::Point(enemyy + int(my_r*cos(can_strike_angle[i]*0.005)), enemyx + my_r*sin(can_strike_angle[i]*0.005)), 3, cv::Scalar(0), 3);
		cv::line(fire_map, cv::Point(enemyy + int(my_r * cos(can_strike_angle[i] * 0.005)), enemyx + my_r * sin(can_strike_angle[i] * 0.005)), cv::Point(enemyy, enemyx), cv::Scalar(0), 2);
	}
	
	if (jj == 0) return;
	//准备发布选中的点
	//注意这里发出去的全部都是以opencv图像坐标系为参考坐标系的
	int temp_x = int(enemyy + int(my_r * cos(can_strike_angle[mmin_i] * 0.005)));
	int temp_y = int(enemyx + int(my_r * sin(can_strike_angle[mmin_i] * 0.005)));
	cv::circle(fire_map, cv::Point(temp_x, temp_y), 5, cv::Scalar(0), 3);

	std::string string_x = std::to_string(temp_x);
	std::string string_y = std::to_string(temp_y);

	if (temp_x < 100) string_x = '0' + string_x;
	if (temp_y < 100) string_y = '0' + string_y;

	std::string temp_string = string_x + ' ' + string_y;
	need_to_send_queue.push(temp_string);
	return;
}

cv::Mat get_fire_map(int x1, int y1, int x2, int y2)
{
	cv::Mat img(map_save.rows, map_save.cols, CV_8UC1,1);
	memcpy((uchar*)img.data, (uchar*)map_save.data, map_save.total() * sizeof(uchar));
	//这里很坑，如何一个图像是纯黑的，那么是不能进行memcpy的，甚至不能获取data指针
	//因为，opencv认为这是一个空图像，返回的有可能是空指针，即使它不是
	memcpy((uchar*)pure_fire_map.data, (uchar*)white_map.data, white_map.total() * sizeof(uchar));
	//如果场上只有一个敌人，把两个点重合即可
	if ((x2 == -1) && (y2 == -1)) { x2 = x1; y2 = y1; }
	/*for (int i = 0; i < img.rows; i++)
		for (int j = 0; j < img.cols; j++)
			if (img.at<uchar>(i, j) != 0)
				if ((is_met(x1, y1, i, j) == 1) || (is_met(x2, y2, i, j) == 1)) {
					img.at<uchar>(i,j) = 100;
					pure_fire_map.at<uchar>(i, j) = 0;
				}*/
	int pix_total = img.cols * img.rows;
	uchar* data = (uchar*) img.data;
	int i = 0, j = 0;
	bool temp1, temp2;//cv::circle(img, cv::Point(y1,x1),5,cv::Scalar(200),5);
	bool temp3, temp4;
	for (int k = 0; k < pix_total; k++)
	{
		i = k / img.cols;
		j = k - i * img.cols;
		if (*(data + k) != 0)
		{	
			//bool is_cross_ans = 0;
			//bool is_meet_ans = 0;
			//int ab_ac, ab_ad, dc_da, dc_db;
			//IS_MEET(x1, y1, i, j);
			//temp1 = is_meet_ans;
			//temp3 = is_met(x1, y1, i, j);
			//if (temp1 != temp3) std::cout << "error1:" << x1 << ' ' << y1 << ' ' << i << ' ' << j << ' ' << temp1 << ' ' << temp3 << std::endl;
			//IS_MEET(x2, y2, i, j);
			//temp2 = is_meet_ans;
			//temp4 = is_met(x2, y2, i, j);
			//if (temp2 != temp4) std::cout << "error2:" << x2 << ' ' << y2 << ' ' << i << ' ' << j << ' ' << temp2 << ' ' << temp4 << std::endl;
			//if ((temp1 == 1) || (temp2 == 1)) {
			//	//*(data + k) = 100;
			// 
			//	img.at<uchar>(i, j) = 100;
			//	pure_fire_map.at<uchar>(i, j) = 0;
			//}

			if ((is_met(x1, y1, i, j) == 1) || (is_met(x2, y2, i, j) == 1)) {
				img.at<uchar>(i, j) = 100;
				pure_fire_map.at<uchar>(i, j) = 0;
			}

		}
			
	}
	return img;
}
/*----------
这里需要注意的是为什么我这个调用的pick_strike_area()参数是y, x
是因为原本测试的时候使用的是鼠标事件回调函数，x，y为鼠标在图像坐标系中的坐标值，注意不是窗口坐标系；
但是我的坐标系规定是和opencv反过来的，为什么呢，因为拿行作为x轴我认为应该是正常人的思维，或者说，它符合二维数组的思维方式
|----------------------->y
|
|
|
x
------------*/
void refresh_window(int x, int y)
{
	//if (x <= 0 || y <= 0) return;
	//IS_MEET(20, 20, y, x);
	//IS_CROSS(0, 0, 200, 200, 200, 0, y, x);
	//cv::Mat temp_map = map_save.clone();
	//cv::circle(temp_map, cv::Point(x, y), 3, cv::Scalar(200),3);
	//cv::circle(temp_map, cv::Point(0, 0), 5, cv::Scalar(200),3);
	//cv::circle(temp_map, cv::Point(200, 200), 5, cv::Scalar(200),3);
	//cv::circle(temp_map, cv::Point(0, 200), 3, cv::Scalar(200),3);
	//cv::imshow(WINDOW_NAME, temp_map);
	//std::cout << is_meet_ans << ' ' << is_met(20, 20, y, x)<<' ' <<is_cross_ans << ' '<< is_cross(0, 0, 200, 200, 200, 0, y, x)<<std::endl;
	cv::Mat temp_map = get_fire_map(y, x, y, x);
	cv::circle(temp_map, cv::Point(x, y), 3, cv::Scalar(200));
	cv::circle(temp_map, cv::Point(x, y), Optimum_strike_distance_pix, cv::Scalar(0));
	cv::imshow(WINDOW_NAME, temp_map);
	pick_strike_area(y, x);
	cv::imshow("222", fire_map);
	cv::waitKey(1);
	return;
}

//判断线段AB,CD是否相交,A(x1,y1),B(x2,y2),C(x3,y3),D(x4,y4),判断AB×AC 与AB×AD是否异号或有一方为零，若确实，则相交，相交返回True
inline bool is_cross(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
{
	int ab_ac = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
	int ab_ad = (x2 - x1) * (y4 - y1) - (x4 - x1) * (y2 - y1);
	int dc_da = (x3 - x4) * (y1 - y4) - (x1 - x4) * (y3 - y4);
	int dc_db = (x3 - x4) * (y2 - y4) - (x2 - x4) * (y3 - y4);
	if ((ab_ac * ab_ad <= 0) && (dc_da * dc_db <= 0)) return 1;//相交了
	return 0;//没有相交
}

//判断(nowx,nowy)和enemy的位置连线是否经过墙，注意这里不是腐蚀之后的，因为子弹可以擦着墙
//如果连线经过墙，也就是没有相遇，那么返回0，没有遮挡相遇了返回1，因为这个函数的返回值要和enemy_fire_cost相乘。
double is_met(int x1,int y1, int nowx, int nowy)
{
	int flag = 0;
	for (int i = 1; i <= 35; i++)
		if (is_cross(x1, y1, nowx, nowy, all_wall_line[i].c1x, all_wall_line[i].c1y, all_wall_line[i].c2x, all_wall_line[i].c2y) == 1)
			return 0;
	return 1;
}

void mouse_handle(int event, int x, int y, int flags, void* param)
{
	switch (event)
	{
	case cv::EVENT_MOUSEMOVE:
		refresh_window(x, y);
		break;
	case cv::EVENT_LBUTTONDOWN:
		break;
	case cv::EVENT_LBUTTONUP:
		break;
	}
}

void Update_Apriltag_Callback()
{
}

void Update_Radar_Callback()
{
}

void Update_Odom_Callback()
{
}

void Update_UltrasonicRanging_Callback()
{
}

//注意，这里的xy坐标系是和opencv cv::Mat坐标系是反过来的，也就是行是x，列是y
void init_load_wall_line()
{
	//上围挡
	all_wall_line[1] = wall_line(46, 177, 46, 227);
	all_wall_line[2] = wall_line(46, 227, 56, 227);
	all_wall_line[3] = wall_line(56, 227, 56, 177);
	all_wall_line[4] = wall_line(56, 177, 46, 177);
	//rectangle(gggmap, Rect(Point(177, 46), Point(227, 56)),Scalar(255));
	//左围挡
	all_wall_line[5] = wall_line(107, 75, 107, 115);
	all_wall_line[6] = wall_line(107, 115, 117, 115);
	all_wall_line[7] = wall_line(117, 115, 117, 75);
	all_wall_line[8] = wall_line(117, 75, 107, 75);
	//rectangle(gggmap, Rect(Point(75, 107), Point(115, 117)), Scalar(255));
	all_wall_line[9] = wall_line(167, 177, 167, 227);
	all_wall_line[10] = wall_line(167, 227, 177, 227);
	all_wall_line[11] = wall_line(177, 227, 177, 177);
	all_wall_line[12] = wall_line(177, 177, 167, 177);
	//rectangle(gggmap, Rect(Point(177, 167), Point(227, 177)), Scalar(255));
	all_wall_line[13] = wall_line(107, 289, 107, 329);
	all_wall_line[14] = wall_line(107, 329, 117, 329);
	all_wall_line[15] = wall_line(117, 329, 117, 289);
	all_wall_line[16] = wall_line(117, 289, 107, 289);
	//rectangle(gggmap, Rect(Point(289, 107), Point(329, 117)), Scalar(255));
	all_wall_line[17] = wall_line(118, 202, 112, 195);
	all_wall_line[18] = wall_line(112, 195, 105, 202);
	all_wall_line[19] = wall_line(105, 202, 112, 208);
	all_wall_line[20] = wall_line(112, 208, 118, 202);
	//启动区围挡
	//左上，顺时针
	all_wall_line[20] = wall_line(50, 1, 50, 50);
	all_wall_line[21] = wall_line(50, 50, 60, 50);
	all_wall_line[22] = wall_line(60, 50, 60, 1);
	all_wall_line[23] = wall_line(60, 1, 50, 1);
	//右上，顺
	all_wall_line[24] = wall_line(1, 319, 1, 329);
	all_wall_line[25] = wall_line(1, 329, 50, 329);
	all_wall_line[26] = wall_line(50, 329, 50, 319);
	all_wall_line[27] = wall_line(50, 319, 1, 319);
	//右下，顺
	all_wall_line[28] = wall_line(164, 354, 164, 404);
	all_wall_line[29] = wall_line(164, 404, 174, 404);
	all_wall_line[30] = wall_line(174, 404, 174, 354);
	all_wall_line[31] = wall_line(174, 354, 164, 354);

	all_wall_line[32] = wall_line(174, 75, 174, 85);
	all_wall_line[33] = wall_line(174, 85, 224, 85);
	all_wall_line[34] = wall_line(224, 85, 224, 75);
	all_wall_line[35] = wall_line(224, 75, 224, 85);
}

//加载腐蚀之后的墙壁，用于优化路径，所有点都是按照顺时针输入的
void init_load_erode_wall_line()
{
	//左围挡
	erode_wall_line[0] = wall_line(95, 63, 95, 127);
	erode_wall_line[1] = wall_line(95, 127, 129, 127);
	erode_wall_line[2] = wall_line(129, 127, 129, 63);
	erode_wall_line[4] = wall_line(129, 63, 95, 63);

	//右围挡
	erode_wall_line[5] = wall_line(95, 277, 95, 341);
	erode_wall_line[6] = wall_line(95, 341, 129, 341);
	erode_wall_line[7] = wall_line(129, 341, 129, 277);
	erode_wall_line[8] = wall_line(129, 277, 95, 277);

	//上围挡
	erode_wall_line[9] = wall_line(34, 165, 34, 239);
	erode_wall_line[10] = wall_line(34, 239, 68, 239);
	erode_wall_line[11] = wall_line(68, 239, 68, 165);
	erode_wall_line[12] = wall_line(68, 165, 34, 165);

	//下围挡
	erode_wall_line[13] = wall_line(155, 165, 155, 239);
	erode_wall_line[14] = wall_line(155, 239, 189, 239);
	erode_wall_line[15] = wall_line(189, 239, 189, 165);
	erode_wall_line[16] = wall_line(189, 165, 155, 165);

	//中障碍
	erode_wall_line[17] = wall_line(93, 190, 93, 214);
	erode_wall_line[18] = wall_line(93, 214, 100, 220);
	erode_wall_line[19] = wall_line(100, 220, 124, 220);
	erode_wall_line[20] = wall_line(124, 220, 130, 214);
	erode_wall_line[21] = wall_line(130, 214, 130, 190);
	erode_wall_line[22] = wall_line(130, 190, 124, 183);
	erode_wall_line[23] = wall_line(124, 183, 100, 183);
	erode_wall_line[24] = wall_line(100, 183, 93, 190);
}
/*-------------------------*/

/*----------嘱来者-----------
* 由于编写时间极为仓促，所以很多地方明明可以使用算法或者是
* 编程技巧优化，最后却没有去实施。所以，希望后来的你能够完善这
* 份代码，我能够想到的地方都列在下面：
* 
* 1.本代码中大量使用函数返回值传递图像，这实际上是极其不合适的。
* 不应该让图像拷来拷去，非常耗时间。希望你能把这些全部改成传递
* 图像指针。
* 
* 2.opencv circle()函数非常快(0.038ms)，比我自己写的沿圆周检
* 测（0.09ms）快得多，不知道为什么。有兴趣可以探究一下opencv究
* 竟用了什么奇妙的算法
* 
* 3.圆环内空白检测，肯定有更好的数学方法，我赶进度来不及思考了
* 后来者务必给我优化了，虽然只是个零点几毫秒的事，但是叠起来就是
* 一毫秒乃至两毫秒
-------------------------*/

/*----------后记-------------
*   你能看到这里，想必是已经大致阅读过程序了。我不喜欢干预别人的事
* 情，但是我相信每个人应该都挺喜欢读故事吧，那也不妨读一读我的文字。
* 我也不知道我写了什么，但是说实话，整个项目我的开发过程还是比较孤独
* 的，没有人可以交流进度，所以我有很多东西想说，但是却无从下笔，只能
* 写出这一堆混乱的文字。如果你能够通过这些文字和我共情，那也算是跨时空交流了。
* 
*   我本来以为这个东西十天就可以写完，看来我还是太高估我自己了。
* 主要是什么把我搞崩溃了呢，就是is_cross换成宏定义函数的方法。
* 我本来是想换成宏函数，这样可以极大加速代码，可是弄了好久都不行
* 我到现在也不知道为什么。
*   RMUA这个项目工程量是很大的。我为什么愿意几乎一个人去做这个项目。
* 写这个代码遇到困难的时候我也不停问过自己。因为这个，我失去了很多
* 可是又得到了什么呢，可能最后失败了什么也没有得到。但是怎么说呢，
* 毕竟年轻时还是有想要自己做机器人、自己写程序的梦想。在重重困难中
* 我渐渐退缩了。我知道这是可耻的，但是我只能说我能力有限，承认自己
* 的无能和锻炼自己的能力一样重要。不过，我始终坚信，对人类的赞歌
* 就是对勇气的赞歌。我钦佩所有后来者，来接手这个项目，不论你们是
* 为了什么，也不论最后放弃、失败与否，至少你们曾经和我一样，
* 追寻过梦想。（写这段文字的时候，正好在播放《你的答案》，可谓应景。）
*   本科生科研虽然说做不出来什么东西，但是可以说，目的都是纯粹的。
* 注意我没说本科生比赛，懂得都懂。《华严经》有言，勿忘初心，方得始终。
* 而此话想要实现，多么困难！年少的锋芒都已消磨殆尽，等着我的只有虚无。
* 毕竟梦想归梦想，学习还是得学的。大二下课程真的很多，我不得不放弃。
*   不过，真的非常感激张兰勇老师一直以来对我研究的支持，他真的非常好。
* 创梦之翼实验室也是一个追逐梦想的地方，你今后不一定再能找到这样积极向上、
* 融洽和谐的实验室团队了。
*			 --2022/1/14 Shi JiaHao@Harbin Engeering University
-------------------------*/