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

//��ʵ����line
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
//�洢�ýǶȡ��þ������Ƿ��ǿɴ������
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
//��;����û�����ͼ
//���룺�з�1��(x1,y1)������(x2,y2)�������Ϊ�Զ�������ϵ�µ�����
//����ֵ��Mat���͵Ļ�����ͼ���ӵ�����ʹ��100�Ҷȱ��
//�����ȫ�ֱ���pure_fire_map��ԭ���Ǵ��ף�����֮��ӵ��ú�ɫ��ע������������͵õ����ʵĴ������
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
	//��ʴ����ȡ�ṹԪ��ָ����������ֵ����Сֵ��Ϊ��λ�õ�����Ҷ�ֵ
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
	//�ǲ���ÿһ���Ƕȶ�����
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
			xx = (enemyy + int(r * cos(angle)));//������ͼ���еĵڼ��У���x
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
			//���Դ򵽲��Ҳ���ǽ
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
			xx = (enemyy + int(((r_min + r_max) / 2) * cos(angle)));//������ͼ���еĵڼ��У���x
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

//��һ�����Ƿ���ԡ��б�Ҫ���뻭���ͼ���ֵĺ�������
void pick_strike_area(int enemyx, int enemyy)
{
	memcpy((uchar*)fire_map.data, (uchar*)map_save.data, map_save.total() * sizeof(uchar));
	//������ĳ�clone
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
	//Ѱ�������������
	int beginof1 = 0; bool finding1 = 0;
	int ii = 0;//�����Ƕ�ָ��
	int jj = 0;//�ɴ���Ƕ�����ָ��
	int begin_position;

	while (circle_strike_area_flag[ii] == 1 && ii <= count - 1) //�ҵ���һ��0
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
		//ȡ�����ߵĵ�������Ƶ��ͬʱʵս��Ҳ����ҽ�����ֱ����
		if (enemyy + int(my_r * cos(can_strike_angle[i] * 0.005)) < mmin_y) {
			mmin_y = enemyy + int(my_r * cos(can_strike_angle[i] * 0.005));
			mmin_i = i;
		}

		cv::circle(fire_map, cv::Point(enemyy + int(my_r*cos(can_strike_angle[i]*0.005)), enemyx + my_r*sin(can_strike_angle[i]*0.005)), 3, cv::Scalar(0), 3);
		cv::line(fire_map, cv::Point(enemyy + int(my_r * cos(can_strike_angle[i] * 0.005)), enemyx + my_r * sin(can_strike_angle[i] * 0.005)), cv::Point(enemyy, enemyx), cv::Scalar(0), 2);
	}
	
	if (jj == 0) return;
	//׼������ѡ�еĵ�
	//ע�����﷢��ȥ��ȫ��������opencvͼ������ϵΪ�ο�����ϵ��
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
	//����ܿӣ����һ��ͼ���Ǵ��ڵģ���ô�ǲ��ܽ���memcpy�ģ��������ܻ�ȡdataָ��
	//��Ϊ��opencv��Ϊ����һ����ͼ�񣬷��ص��п����ǿ�ָ�룬��ʹ������
	memcpy((uchar*)pure_fire_map.data, (uchar*)white_map.data, white_map.total() * sizeof(uchar));
	//�������ֻ��һ�����ˣ����������غϼ���
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
������Ҫע�����Ϊʲô��������õ�pick_strike_area()������y, x
����Ϊԭ�����Ե�ʱ��ʹ�õ�������¼��ص�������x��yΪ�����ͼ������ϵ�е�����ֵ��ע�ⲻ�Ǵ�������ϵ��
�����ҵ�����ϵ�涨�Ǻ�opencv�������ģ�Ϊʲô�أ���Ϊ������Ϊx������ΪӦ���������˵�˼ά������˵�������϶�ά�����˼ά��ʽ
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

//�ж��߶�AB,CD�Ƿ��ཻ,A(x1,y1),B(x2,y2),C(x3,y3),D(x4,y4),�ж�AB��AC ��AB��AD�Ƿ���Ż���һ��Ϊ�㣬��ȷʵ�����ཻ���ཻ����True
inline bool is_cross(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
{
	int ab_ac = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
	int ab_ad = (x2 - x1) * (y4 - y1) - (x4 - x1) * (y2 - y1);
	int dc_da = (x3 - x4) * (y1 - y4) - (x1 - x4) * (y3 - y4);
	int dc_db = (x3 - x4) * (y2 - y4) - (x2 - x4) * (y3 - y4);
	if ((ab_ac * ab_ad <= 0) && (dc_da * dc_db <= 0)) return 1;//�ཻ��
	return 0;//û���ཻ
}

//�ж�(nowx,nowy)��enemy��λ�������Ƿ񾭹�ǽ��ע�����ﲻ�Ǹ�ʴ֮��ģ���Ϊ�ӵ����Բ���ǽ
//������߾���ǽ��Ҳ����û����������ô����0��û���ڵ������˷���1����Ϊ��������ķ���ֵҪ��enemy_fire_cost��ˡ�
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

//ע�⣬�����xy����ϵ�Ǻ�opencv cv::Mat����ϵ�Ƿ������ģ�Ҳ��������x������y
void init_load_wall_line()
{
	//��Χ��
	all_wall_line[1] = wall_line(46, 177, 46, 227);
	all_wall_line[2] = wall_line(46, 227, 56, 227);
	all_wall_line[3] = wall_line(56, 227, 56, 177);
	all_wall_line[4] = wall_line(56, 177, 46, 177);
	//rectangle(gggmap, Rect(Point(177, 46), Point(227, 56)),Scalar(255));
	//��Χ��
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
	//������Χ��
	//���ϣ�˳ʱ��
	all_wall_line[20] = wall_line(50, 1, 50, 50);
	all_wall_line[21] = wall_line(50, 50, 60, 50);
	all_wall_line[22] = wall_line(60, 50, 60, 1);
	all_wall_line[23] = wall_line(60, 1, 50, 1);
	//���ϣ�˳
	all_wall_line[24] = wall_line(1, 319, 1, 329);
	all_wall_line[25] = wall_line(1, 329, 50, 329);
	all_wall_line[26] = wall_line(50, 329, 50, 319);
	all_wall_line[27] = wall_line(50, 319, 1, 319);
	//���£�˳
	all_wall_line[28] = wall_line(164, 354, 164, 404);
	all_wall_line[29] = wall_line(164, 404, 174, 404);
	all_wall_line[30] = wall_line(174, 404, 174, 354);
	all_wall_line[31] = wall_line(174, 354, 164, 354);

	all_wall_line[32] = wall_line(174, 75, 174, 85);
	all_wall_line[33] = wall_line(174, 85, 224, 85);
	all_wall_line[34] = wall_line(224, 85, 224, 75);
	all_wall_line[35] = wall_line(224, 75, 224, 85);
}

//���ظ�ʴ֮���ǽ�ڣ������Ż�·�������е㶼�ǰ���˳ʱ�������
void init_load_erode_wall_line()
{
	//��Χ��
	erode_wall_line[0] = wall_line(95, 63, 95, 127);
	erode_wall_line[1] = wall_line(95, 127, 129, 127);
	erode_wall_line[2] = wall_line(129, 127, 129, 63);
	erode_wall_line[4] = wall_line(129, 63, 95, 63);

	//��Χ��
	erode_wall_line[5] = wall_line(95, 277, 95, 341);
	erode_wall_line[6] = wall_line(95, 341, 129, 341);
	erode_wall_line[7] = wall_line(129, 341, 129, 277);
	erode_wall_line[8] = wall_line(129, 277, 95, 277);

	//��Χ��
	erode_wall_line[9] = wall_line(34, 165, 34, 239);
	erode_wall_line[10] = wall_line(34, 239, 68, 239);
	erode_wall_line[11] = wall_line(68, 239, 68, 165);
	erode_wall_line[12] = wall_line(68, 165, 34, 165);

	//��Χ��
	erode_wall_line[13] = wall_line(155, 165, 155, 239);
	erode_wall_line[14] = wall_line(155, 239, 189, 239);
	erode_wall_line[15] = wall_line(189, 239, 189, 165);
	erode_wall_line[16] = wall_line(189, 165, 155, 165);

	//���ϰ�
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

/*----------������-----------
* ���ڱ�дʱ�伫Ϊ�ִ٣����Ժܶ�ط���������ʹ���㷨������
* ��̼����Ż������ȴû��ȥʵʩ�����ԣ�ϣ�����������ܹ�������
* �ݴ��룬���ܹ��뵽�ĵط����������棺
* 
* 1.�������д���ʹ�ú�������ֵ����ͼ����ʵ�����Ǽ��䲻���ʵġ�
* ��Ӧ����ͼ������ȥ���ǳ���ʱ�䡣ϣ�����ܰ���Щȫ���ĳɴ���
* ͼ��ָ�롣
* 
* 2.opencv circle()�����ǳ���(0.038ms)�������Լ�д����Բ�ܼ�
* �⣨0.09ms����ö࣬��֪��Ϊʲô������Ȥ����̽��һ��opencv��
* ������ʲô������㷨
* 
* 3.Բ���ڿհ׼�⣬�϶��и��õ���ѧ�������ҸϽ���������˼����
* ��������ظ����Ż��ˣ���Ȼֻ�Ǹ���㼸������£����ǵ���������
* һ��������������
-------------------------*/

/*----------���-------------
*   ���ܿ������������Ѿ������Ķ��������ˡ��Ҳ�ϲ����Ԥ���˵���
* �飬����������ÿ����Ӧ�ö�ͦϲ�������°ɣ���Ҳ������һ���ҵ����֡�
* ��Ҳ��֪����д��ʲô������˵ʵ����������Ŀ�ҵĿ������̻��ǱȽϹ¶�
* �ģ�û���˿��Խ������ȣ��������кܶණ����˵������ȴ�޴��±ʣ�ֻ��
* д����һ�ѻ��ҵ����֡�������ܹ�ͨ����Щ���ֺ��ҹ��飬��Ҳ���ǿ�ʱ�ս����ˡ�
* 
*   �ұ�����Ϊ�������ʮ��Ϳ���д�꣬�����һ���̫�߹����Լ��ˡ�
* ��Ҫ��ʲô���Ҹ�������أ�����is_cross���ɺ궨�庯���ķ�����
* �ұ������뻻�ɺ꺯�����������Լ�����ٴ��룬����Ū�˺þö�����
* �ҵ�����Ҳ��֪��Ϊʲô��
*   RMUA�����Ŀ�������Ǻܴ�ġ���ΪʲôԸ�⼸��һ����ȥ�������Ŀ��
* д��������������ѵ�ʱ����Ҳ��ͣ�ʹ��Լ�����Ϊ�������ʧȥ�˺ܶ�
* �����ֵõ���ʲô�أ��������ʧ����ʲôҲû�еõ���������ô˵�أ�
* �Ͼ�����ʱ��������Ҫ�Լ��������ˡ��Լ�д��������롣������������
* �ҽ��������ˡ���֪�����ǿɳܵģ�������ֻ��˵���������ޣ������Լ�
* �����ܺͶ����Լ�������һ����Ҫ����������ʼ�ռ��ţ���������޸�
* ���Ƕ��������޸衣���������к����ߣ������������Ŀ������������
* Ϊ��ʲô��Ҳ������������ʧ���������������������һ����
* ׷Ѱ�����롣��д������ֵ�ʱ�������ڲ��š���Ĵ𰸡�����νӦ������
*   ������������Ȼ˵��������ʲô���������ǿ���˵��Ŀ�Ķ��Ǵ���ġ�
* ע����û˵���������������ö����������Ͼ������ԣ��������ģ�����ʼ�ա�
* ���˻���Ҫʵ�֣���ô���ѣ����ٵķ�â������ĥ�����������ҵ�ֻ�����ޡ�
* �Ͼ���������룬ѧϰ���ǵ�ѧ�ġ�����¿γ���ĺܶ࣬�Ҳ��ò�������
*   ��������ķǳ��м���������ʦһֱ���������о���֧�֣�����ķǳ��á�
* ����֮��ʵ����Ҳ��һ��׷������ĵط�������һ�������ҵ������������ϡ�
* ��Ǣ��г��ʵ�����Ŷ��ˡ�
*			 --2022/1/14 Shi JiaHao@Harbin Engeering University
-------------------------*/