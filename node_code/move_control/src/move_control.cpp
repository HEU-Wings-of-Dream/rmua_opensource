#include <iostream>
#include <fstream>
#include <queue>
#include <math.h>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <move_control/velocity.h>
#include <move_control/my_control_frame.h>
#include <move_control/race_state.h>

#include <move_control/data_frame.h>
#include <move_control/Serial.h>

#define WATCH_ROAD_PLAN 0

using namespace std;
using namespace cv;

/*---------------------------*/
/*Mat:  ---------->x    my_map: ------->y
 *      |                       |
 *      |                       |
 *      |                       |
 *      !y                      !x
 */
/*---------------------------*/

int map_x = 228, map_y = 408, memory_pool_i = 0, memory_pool_optimizer_i = 0;

//����Ŀ��㶨�塢�Ż���Ŀ��㶨��
int goalx, goaly, min_dist_goal_x, min_dist_goal_y;

int a[228][408]; //��ͼ
double enemy_fire_cost[228][408];//�з��������۵�ͼ
bool flag[228][408], flag_optimizer[228][408];//A*�������
int corner_array[93024]; int number_of_corner = 0;//������ŵ��ǹյ���path_array������±꣬����ͬʱ���ã��������Ǵ�1��ʼ���
cv::Mat img, img_plain_save, img_erode_save, simulation_map, path_map(226,406,CV_8UC1,Scalar(255)), mask, img_final_path;//�����ͼɶ��

int zone[6]{}, zone_status[6]{};
bool have_i_pass_zone[6]{};

bool is_race_start = 0;
bool bullet_buff_state = 0;
//ȫ��ͨ�ű�Ƕ���
bool need_replan_path = 1, first_in_simulation_function = 0, stop_navigation_flag = 0;

//��������ȫ��vector����
std::vector<cv::Point2f>corner_vector;
std::vector<int>angle_vector;

//ʱ���������
auto program_start_time = std::chrono::high_resolution_clock::now();
auto last_receive_time = std::chrono::high_resolution_clock::now();//��һ���յ��ٶ���Ϣ��ʱ��
bool frist_receive_flag = 0;

double total_start_time, last_simulation_time;

//·���滮����ר����
class my_point
{
public:
	int x;
	int y;
	int father;//���ڵ����ڴ���е��±�
	int self;//�Լ����ڴ���е��±�
	my_point(int fa = 0, int selff = 0, int xx = 0, int yy = 0) {
		x = xx;
		y = yy;
		father = fa;
		self = selff;
	}
	//������ʹ��������������ķ����Զ������ȶ���˳�򣬽����ô�������У��Ա��˶��죬û�취�����ǻ�����cmp����
	//bool operator < (const my_point* b);
};

class enemy
{
public:
	int x;
	int y;
	int HP;
	bool is_alive;
	bool can_use_its_position;
	enemy(int xx, int yy, int HPP) {
		is_alive = 1;
		//ע�⣡�������������Լ����Ե�ʱ��������ó�1��ʵ��ʹ�õ�ʱ��һ��Ĭ��ֵ��0������Ȼ���군�ˣ���
		can_use_its_position = 1;
		x = xx;
		y = yy;
		HP = HPP;
	}
};

//����֡�ṹ��
class ControlFrame
{
public:
	double vx;
	double vy;
        int angle;
};
ControlFrame controlframe, lastcontrolframe;

class self_robot
{
//����㶨����vmaxΪ1.8(m/s), vrmax = 1.5��(rad/s)
//�ٳ������ܷǳ��ã�������Ϊ�ٶȶ��Ǻ㶨�ģ�û�м���ʱ�䡢����ʱ�䣬�Դ˼򻯼���
//������90�ȵ�ת��뾶Ҳֻ��0.18m�������Ҿ����н��ٶ�ͦ����
public:
	int x;
	int y;
	double angle;//˵������ǰλ���ǵ���������������ˮƽ�߼н�
	int HP;
	int MP;    
	double vx;
	double vy;
	double vr;
	self_robot() {}
	self_robot(double xx, double yy, int HP_ = 2000, int MP_ = 240, double vx_ = 0, double vy_ = 0, double vr_ = 0, double angle_ = 0){
		x = xx; y = yy; HP = HP_; MP = MP_;
		vx = vx_; vy = vy_; vr = vr_; angle = angle_;
	}
};

//��ʵ����line
class wall_line
{
public:
	int c1x;
	int c1y;
	int c2x;
	int c2y;
	wall_line( ) { }
	wall_line(int c1x_, int c1y_, int c2x_, int c2y_)
	{
		c1x = c1x_;
		c1y = c1y_;
		c2x = c2x_;
		c2y = c2y_;
	}
};

//�����γɵ�·����ͳ��Ϊ�˸�ʽ������my_point�������������ർ����Ϣ��ȥ���˲���Ҫ��father��self��
class navigation_point
{
public:
	int x;
	int y;
	int should_angle;
	int nextcorner;
	double next_corner_dis; //����һ���սǵľ���
	int next_state_change_node; //��һ���ӵ�״̬�ı��
	navigation_point( ){ }
	navigation_point(int xx, int yy)
	{
		this->x = xx;
		this->y = yy;
        }
	navigation_point(int xx, int yy, int _next_corner_dis)
	{
		this->x = xx;
		this->y = yy;
		this->next_corner_dis = _next_corner_dis;
	}
};

navigation_point final_path[93024]; int final_path_i = 1;//final_path���±�1��ʼ�ŵģ�������ʼ����final_path[1]

//path_planner
my_point* memory_pool[93024];
my_point* path_array[93024]; int path_long;//path_array���±�1��ʼ�ŵģ�������ʼ����path_array[1]

//path_optimizer
my_point* memory_pool_optimizer[93024];
my_point* short_path_array[65536];

//others
enemy enemy1(187, 36, 2000);
self_robot robot1(0, 0, 2000, 240, 0, 0, 0, 0);
wall_line all_wall_line[21], erode_wall_line[25];

//��ʼ������ǽ��ĺ���
void init_load_wall_line();
//���ظ�ʴ��ǽ�庯��
void init_load_erode_wall_line();
//���滷�����ʱ��ص�����
void mouse_handle(int event, int x, int y, int flags, void* param);
void keep_xy_safe(int& x, int& y);
//�����������������Ը��õģ��������º�����Ū��������ô����
//����Ū�����������������������
void add_cant_go(cv::Point, int);
void delete_cant_go(cv::Point, int);


//����룬����Ҫ�ǿ��������������е㺩
double my_dist(int x1, int y1, int x2, int y2)
{
	return std::sqrt((double(x1 - x2) * double(x1 - x2) + double(y1 - y2) * double(y1 - y2)));
}

//�ж��߶�AB,CD�Ƿ��ཻ,A(x1,y1),B(x2,y2),C(x3,y3),D(x4,y4)
//�ж�AB��AC ��AB��AD�Ƿ���Ż���һ��Ϊ�㣬��ȷʵ�����ཻ���ཻ����True
//����yyds (^_^)
bool is_cross(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
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
double is_met(enemy e, int nowx, int nowy)
{
	int flag = 0;
	for (int i = 1; i <= 20; i++)
		if (is_cross(e.x, e.y, nowx, nowy, all_wall_line[i].c1x, all_wall_line[i].c1y, all_wall_line[i].c2x, all_wall_line[i].c2y) == 1)
			return 0;
	return 1;
}

//�ж�(x1,y1)-(x2,y2)�����߶��Ƿ񾭹��˸�ʴ֮���ǽ��
//��ǽ���غϻ��߲��ཻ��Ϊ����ͨ��������1
//��ǽ���ཻ��Ϊ������ͨ��������0
double can_pass(int x1, int y1, int x2, int y2)
{
	for (int i = 0; i <= 24; i++)
		if (is_cross(x1, y1, x2, y2, erode_wall_line[i].c1x, erode_wall_line[i].c1y, erode_wall_line[i].c2x, erode_wall_line[i].c2y) == 1)
			return 0;
	return 1;
}

void init_load_map()
{
    img_plain_save = cv::imread("/home/ubuntu/catkin/src/move_control/src/map2.png", -1);
	if (img_plain_save.empty() == 1)cout<<"MAP EMPTY!!"<<std::endl;
	//��ʴ����ȡ�ṹԪ��ָ����������ֵ����Сֵ��Ϊ��λ�õ�����Ҷ�ֵ
    cv::Mat structureElement = getStructuringElement(MORPH_RECT, Size(40, 40), Point(-1, -1));
	//�ṹԪ�����Ƕ���д����ۣ�Ŀǰ��25 * 2cm = 50cm��Լ���ڵ��̰뾶
	erode(img_plain_save, img_erode_save, structureElement);
    //cv::imwrite("erode_map.jpg",img_erode_save);
	cout<<img_erode_save.rows<<' '<<img_erode_save.cols<<endl;
    imshow("Simulation_Environment@HEUsjh", img_erode_save);cv::waitKey(100);
	for (int i = 0; i < img_erode_save.rows; i++)
	{
		for (int j = 0; j < img_erode_save.cols; j++)
		{
			if (img_erode_save.at<uchar>(i, j) == 255) {
				a[i][j] = 0; 
			}
			else a[i][j] = -1;
		}
	}
	// cv::Mat temp_map(226, 406, CV_8UC1, cv::Scalar(255));

	// for (int i = 0; i < temp_map.rows; i++)
	// {
	// 	for (int j = 0; j < temp_map.cols; j++)
	// 	{
	// 		if (a[i][j] == -1) {
	// 			temp_map.at<uchar>(i, j)  = 0; continue;
	// 		}
	// 	}
	// }
	// imshow("1111", temp_map);
	// cv::waitKey(10000);
	std::cout << "read map complete" << std::endl << std::endl;
}

void Refresh_enemy_fire_cost()
{
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			//A = 1,sigma(x) = 50,sigma(y) = 50;
			enemy_fire_cost[i][j] = is_met(enemy1, i, j) * (50000.0 * exp(-(((i - enemy1.x) * (i - enemy1.x)) / (2 * 50 * 50) + ((j - enemy1.y) * (j - enemy1.y)) / (2 * 50 * 50))));
			//�˴�ӦΪenemy2�������
		}
	}
}

//������һ��Ĺ�ֵ����
double h(int x, int y)
{
	double dis = sqrt((x - goalx) * (x - goalx) + (y - goaly) * (y - goaly));
        return dis;
        //return  dis + 1000 * enemy_fire_cost[x][y];
}

//�����·�õĹ�ֵ����
double min_dist_h(int x, int y)
{
	double dis = sqrt((x - min_dist_goal_x) * (x - min_dist_goal_x) + (y - min_dist_goal_y) * (y - min_dist_goal_y));
	return  dis;
}

//�Զ������ȶ������򷽷�
struct cmp
{ 
	bool operator () (my_point* a, my_point* b)
	{
		return h(a->x, a->y) > h(b->x, b->y);
	}
};

struct cmp_path_optimizer_queue
{
	bool operator () (my_point* a, my_point* b)
	{
		return min_dist_h(a->x, a->y) > min_dist_h(b->x, b->y);
	}
};

bool sort_cmp(cv::Point a, cv::Point b)
{
	return my_dist(a.x, a.y, final_path[1].x, final_path[1].y) < my_dist(b.x, b.y, final_path[1].x, final_path[1].y);
}

priority_queue <my_point*, std::vector<my_point*>, cmp> search_queue;
priority_queue <my_point*, std::vector<my_point*>, cmp_path_optimizer_queue> Q;
//�����������ԵĶ���
//priority_queue <my_point*, std::vector<my_point*>, cmp> test;

//����д����ʹ��txt�ĵ����ԣ�������Ȼ����Ҫ��
//void build_game_map()
//{
//	fstream fin;
//	char ch;
//	fin.open("map.txt");
//	for (int i = 0; i < map_x; i++)
//	{
//		for (int j = 0; j < map_y; j++)
//		{
//			fin >> ch;
//			if (ch == '0') a[i][j] = 0;
//			if (ch == '1') a[i][j] = 1;
//		}
//	}
//	fin.close();
//	return;
//}

//һ��ʼʹ�û��ݵķ�������·���������������п��ܱ�ջ���������˰ɣ��Ż���while��
//����д�ķ��㣬˼ά�Ѷȵͣ���ϧ�ᱬջ
//void path_recall(my_point now, int step)
//{
//	//�ݹ���ֹ����
//	if ((now.self == now.father))
//	{
//		std::cout << step << std::endl;
//		return;
//	}
//	//std::cout << '=';
//	my_point father_point = *(memory_pool[now.father]);
//	path_recall(father_point, step + 1);
//	//std::cout << "Callback " << step << " : " << now.x << ' ' << now.y << std::endl;
//}

//������С���˺���
double get_k(std::vector<cv::Point>);

void draw_path()
{
	img_final_path = img_plain_save.clone();
	for (int i = 1; i <= final_path_i; i++)
	{
		img_final_path.at<uchar>(final_path[i].x, final_path[i].y) = 0;
	}
}

//��С���˷���ϳ����ĽǶ��������ģ���������ģ��һ�£������Ҳû�й�ϵ��������дһ�濪�����Ƶ�
//�ջ��Ļ����Ӿ���ǩ���ڸڳ�������˵
int correct_angle(double angle)
{
	/*
	|
        O������������>y (0)
	|
        |   angle>0
	x
	*/
    /* !x
     * |   k>0
     * |
     * *-------->y (90)
     * |
     * |   k<0
     * |
     * |
     */
	//�ݶ�˳ʱ��Ϊ��
	//��90����һ�������������̣���Ϊ��90��

        if (abs(angle)<10)return 90;
        if (abs(abs(angle) - 90) <= 30) //80~100 && -80~-100
                return 0;

	//if (abs(angle - 55) <=8)
	//		return 90;

	if (abs(angle - 45) < 7) // 35~55
		return -45;

	if (abs(angle + 45) < 7) // -35~-55
		return 45;

	if ((angle < 80) && (angle > 55)) // 55~80
		return -30;

	if ((angle > -80) && (angle < -55))
		return 30;
        return (int)angle;
	
}

//�ҹյ�ĺ���
void find_corner()
{
	//ԭ����Ϊ��ôд���㷨�����޷죬���б����չ����ɢ����ʱ��ͻ���־���͵�·��
	//Ȼ������㷨�ͼ�⵽һ��б������ȫ�ǹյ㣡��ʵ��������֮�⣡
	//����������뵽������opencv�Դ��Ľǵ����㷨��ʵ����̫���ˣ�
	//int count = 0;
	//my_point* front_point;
	//my_point* middle_point;
	//my_point* next_point;
	//for (int i = 2; i <= path_long - 1; i++)
	//{
	//	double vec1x = path_array[i]->x - path_array[i - 1]->x;
	//	double vec1y = path_array[i]->y - path_array[i - 1]->y;
	//	double vec2x = path_array[i]->x - path_array[i + 1]->x;
	//	double vec2y = path_array[i]->y - path_array[i + 1]->y;
	//	//б���Ƿ���ȣ�������ǿ϶��ǹյ���
	//	if ((vec1x == 0) && (vec2x == 0)) continue;
	//	else
	//	{
	//		if ((vec1y / vec1x) != (vec2y / vec2x))
	//		{
	//			//std::cout << "Error: " << (vec1y / vec1x) << ' ' << (vec2y / vec2x) << ' '<<std::endl;
	//			//std::cout << path_array[i]->x << ' ' << path_array[i]->y << ' ' << path_array[i - 1]->x << ' ' << path_array[i - 1]->y << std::endl;
	//			//std::cout << path_array[i]->x << ' ' << path_array[i]->y << ' ' << path_array[i + 1]->x << ' ' << path_array[i + 1]->y << std::endl;
	//			number_of_corner++;
	//			corner_array[number_of_corner] = i;
	//		}
	//	}	
	//}
	//for (int i = 1; i <= number_of_corner; i++) {
	//	//std::cout << path_array[corner_array[i]]->x << ' ' << path_array[corner_array[i]]->y << std::endl;
	//	circle(img, cv::Point(path_array[corner_array[i]]->y, path_array[corner_array[i]]->x), 2, Scalar(0), -1);
	//}
	//cv::Mat temp_map = path_map.clone();
	corner_vector.clear();
	draw_path();
	std::vector<cv::Point> corner_vector_temp;
	cv::goodFeaturesToTrack(path_map, corner_vector_temp , 100, 0.04, 4, mask, 5, true, 0.04);
	
	//cout << "coner size : " << corner_vector_temp.size() << std::endl;

	//after goodFeaturesToTrack, the corner vector need sort
	//the output of that function is not reguarly
	bool break_flag = 0;
	for (int i = 1; i <= final_path_i; i++)
	{
			std::vector<cv::Point>::iterator it = corner_vector_temp.begin();
			//����ɣ��������˶�vector��Ϥ��������δ���һ���ɻ�û�аɣ��������˻���ȥ�����ϰɣ�
			while (it != corner_vector_temp.end())
            {
				if (my_dist(final_path[i].x, final_path[i].y, (*it).y, (*it).x) <= 2)
				{
                    corner_vector.push_back(*it);
					//��ģ���������񵰵ط������Ұ��죬����STL�������ǡ��뵱���ʦ��Ȼ�Ǻ���˵�˵ģ���ϧ����������
					it = corner_vector_temp.erase(it);
					if (corner_vector_temp.size() == 0)
					{
						break_flag = 1;
						break;
					}
					//std::cout << "erase one!!" << std::endl;
				}
				else it++;
			}
		if (break_flag == 1)break;
	}

	for (const auto& corner : corner_vector)
	{
		circle(path_map, cv::Point(corner.x, corner.y), 2, Scalar(0), -1);
	}
	//std::cout << "final path"<<final_path_i << std::endl;
	cv::imshow("2345534", path_map);
        cv::waitKey(130);

	std::cout << "Corner vector size: " << corner_vector.size() << ' ' << "Path size: " << final_path_i << std::endl;
	std::vector <cv::Point> short_path;
	int next_corner = 1;
	//��һ���ǵ���final_path�е�λ��
	int last_corner_locate = 0;
	for (int i = 1; i <= final_path_i; i++)
	{
            //���Ｋ����ˣ�����Ҫ�ر��ر�ע�⣡���������ǵ�����ϵ��������x�ᣬ����opencv API goodFeaturesToTrack()
            //���ص���opencv��ʽ��cv::Point�������������ɺ���Ϊx���ģ�������Ŀ��ˣ�����
            //��������Լ������ݽṹȥ�����㣬�Ǿ�û�����������⣬���������õ���cv��API�ͱ���ע������������

            //������ʵ����һ��Сע�⣺c++�ڼ���if���������ʽ��ʱ����������ʽ����ֻ��&&�������������if
            //��ô�����ȼ����һ�����������Ϊfalse����ô�Ͳ������¼����ˣ��������i==1ֻ��һ�Σ����������my_dist�����������ǰ��
            //ÿ�ζ������һ�飬�ǳ���ʱ�䣬���˳���ǿ��Ա������my_dist�ġ���Ȼ����ͬ�ı��������Ż���ʽ��һ��
            //�����Ĳ��Խ�����������ģ���ֻ��˵�������������⣬���˵�������Ǵ��ڵġ�

		//std::cout << i << ' ' << final_path[i].x << ' ' << final_path[i].y << ' ' << corner_vector[next_corner].y << ' ' << corner_vector[next_corner].x << ' ' << my_dist(final_path[i].x, final_path[i].y, corner_vector[next_corner].y, corner_vector[next_corner].x) << std::endl;
		
		if ((i == 1) && (my_dist(final_path[1].x, final_path[1].y, corner_vector[0].y, corner_vector[0].x) >= 2.0))
		{
			//std::cout << final_path[1].x << ' ' << final_path[1].y << ' ' << corner_vector[0].x << ' ' << corner_vector[0].y << ' '<< std::sqrt(pow(final_path[1].x - corner_vector[0].x, 2.0) + std::pow(final_path[1].y - corner_vector[0].y, 2.0))<<std::endl;
			std::cout << "dong't have first node!!I will push it!" << std::endl;
                        corner_vector.insert(corner_vector.begin(), cv::Point(final_path[1].y, final_path[1].x));
			next_corner = 1;
			continue;
		}

		if ((i == final_path_i))
		{
			std::cout << "Point num: " << short_path.size();
                        std::cout << ";   " << "Now path: " << next_corner - 2 << '-' << next_corner - 1 << ";   " << "k == " << std::floor(correct_angle(get_k(short_path))) << std::endl;
			short_path.clear();
			break;
		}
		final_path[i].next_corner_dis = my_dist(final_path[i].x, final_path[i].y, corner_vector[next_corner].y, corner_vector[next_corner].x);
		short_path.push_back(cv::Point(final_path[i].x, final_path[i].y));
		
		//�����ǰ������һ���ǵ㣬Ӧ���ͽ�ȥ��С������ϣ�Ȼ������ǰshort_path����
		if ((my_dist(final_path[i].x, final_path[i].y, corner_vector[next_corner].y, corner_vector[next_corner].x) <= 2))
		{
			
			next_corner++;final_path[i].nextcorner = next_corner;

			int temp = correct_angle(get_k(short_path));
                        //cout<<"   temp = "<<temp<<"   ";
                        //position correction (very important)
                        //ATTENTION: cornver_vector is front OpenCV API, (x,y) is relate to Mat, not my position!!!
                        if (int(temp) == 90)
                        {
                                //cout << "last corner  " << corner_vector[next_corner - 2].x << "   next corner " << corner_vector[next_corner - 1].x << std::endl;
                                if (corner_vector[next_corner - 2].y < corner_vector[next_corner - 1].y)
                                {
                                        final_path[i].should_angle = 90;
                                        temp = 90;
                                }
                                if (corner_vector[next_corner - 2].y > corner_vector[next_corner - 1].y)
                                {
                                        final_path[i].should_angle = -90;
                                        temp = -90;
                                }
                        }

			if (int(temp) == 0)
                        {
                                cout << "last corner  " << corner_vector[next_corner - 2].y << "   next corner " << corner_vector[next_corner - 1].y << std::endl;
                                if (corner_vector[next_corner - 2].x > corner_vector[next_corner - 1].x)
				{
                                        final_path[i].should_angle = -180;
                                        temp = -180;
				}
                                if (corner_vector[next_corner - 2].x < corner_vector[next_corner - 1].x)
				{
                                        final_path[i].should_angle = 0;
                                        temp = 0;
				}
			}

                        if (int(temp) == -45)
                        {
                            cout << "last corner  " << corner_vector[next_corner - 2].x << "   next corner " << corner_vector[next_corner - 1].x << std::endl;
                            if (corner_vector[next_corner - 2].x > corner_vector[next_corner - 1].x)
                            {
                                    final_path[i].should_angle = 135;
                                    temp = 135;
                            }
                            if (corner_vector[next_corner - 2].x < corner_vector[next_corner - 1].x)
                            {
                                    final_path[i].should_angle = -45;
                                    temp = -45;
                            }
                        }

                        if (int(temp) == 45)
                        {
                            cout << "last corner  " << corner_vector[next_corner - 2].x << "   next corner " << corner_vector[next_corner - 1].x << std::endl;
                            if (corner_vector[next_corner - 2].x > corner_vector[next_corner - 1].x)
                            {
                                    final_path[i].should_angle = -135;
                                    temp = -135;
                            }
                            if (corner_vector[next_corner - 2].x < corner_vector[next_corner - 1].x)
                            {
                                    final_path[i].should_angle = 45;
                                    temp = 45;
                            }
                        }

			for (int j = last_corner_locate; j <= i; j++)
				final_path[j].should_angle = temp;
			last_corner_locate = i;
			//final_path[i].should_angle = temp;
			angle_vector.push_back(temp);
			std::cout << "Point num: " << short_path.size();
                        std::cout << ";   " << "Now path: " << next_corner - 2 << '-' << next_corner - 1 << ";   " << "k = " << get_k(short_path) << "   k2= "<<correct_angle(get_k(short_path))<<"  correct angle is  " << temp<< std::endl;
			short_path.clear();
			if (next_corner == corner_vector.size()) break;
			continue;
		}
		final_path[i].nextcorner = next_corner;
	}

	//���½�Ϊ�ƺ�������Ϊ�ǵ��⵱���һ���ս����յ�ܽ���ʱ���յ��Ǽ�ⲻ�����ģ�����ֻ��ȥ�ֶ���ȫ���һ��·��·����Ϣ

	//�Ȳ�ȫ�Ƕȿ�����Ϣ

	//�ҵ����ĽǶ�
	int j = final_path_i-1;
	while ((abs(final_path[j].should_angle) > 360))
	{
		j--;
	}
	//���䱣��֮
	for (int k = j + 1; k <= final_path_i; k++)
		final_path[k].should_angle = final_path[j].should_angle;

	//��ȫ�յ����λ����Ϣ
	j = final_path_i - 1;
	while (final_path[j].nextcorner!=next_corner)
	{
		j--;
	}

	for (int k = j + 1; k <= final_path_i; k++)
		final_path[k].nextcorner = next_corner;

	final_path[final_path_i].x = goalx;
	final_path[final_path_i].y = goaly;

	corner_vector.push_back(cv::Point(goaly, goalx));
	for (int i = 1; i <= final_path_i; i++)
	{
		final_path[i].next_corner_dis = my_dist(final_path[i].x, final_path[i].y, corner_vector[final_path[i].nextcorner].y, corner_vector[final_path[i].nextcorner].x);
	}

        for (int i = 1; i <= final_path_i; i++)
                {
                    std::cout << final_path[i].should_angle << ' '<<i<< ' ' << final_path[i].nextcorner<<std::endl;
                }
        waitKey(30);
	
	/*std::cout << "Point num: " << short_path.size() << ';' << "k = " << get_k(short_path) << std::endl;
	short_path.clear();*/
	//cv::imshow("123123", temp_map);
	
}

//ʹ�ñ����������Ż�·���ĺ���
//void Bezier_curve_optimizer(int x1, int y1, int x2, int y2, int x3, int y3){}

void refresh_simulation_environment()
{
	auto t1 = std::chrono::high_resolution_clock::now();
	if (first_in_simulation_function == 0) {//û�з�����Ͳ�Ҫ�����ˣ���һ����ʼʱ���
		last_simulation_time = (static_cast<std::chrono::duration<double, std::milli>>(t1 - program_start_time)).count();
		return;
	}
	simulation_map = img;//ˢ�µ�ͼ
	total_start_time = (static_cast<std::chrono::duration<double, std::milli>>(t1 - program_start_time)).count();
	double delta_time = double(total_start_time - last_simulation_time) / 1000.0;//ת������
	//��������
	robot1.x += robot1.vx * delta_time;
	robot1.y += robot1.vy * delta_time;
	robot1.angle += robot1.angle * delta_time;
	cv::circle(simulation_map, cv::Point(int(robot1.y), int(robot1.x)), 2, Scalar(0), -1);
	imshow("Simulation_Environment@HEUsjh", simulation_map);
	last_simulation_time = total_start_time;//ʱ�����
	waitKey(1);
}

void path_optimizer()
{
	std::cout << "Running path_optimizer..." << std::endl;

	bool now_state = 0, last_state = 0, first_flag = 0;
	my_point* path_optimizer_array[100]; int path_optimizer_array_size = 0;

	//Ѱ�����нӵ�״̬�ı�ĵ㣬�������飬׼�������Ż�
	for (int i = 1; i <= path_long; i++)
	{
		now_state = is_met(enemy1, path_array[i]->x, path_array[i]->y);

		if (now_state == 1) {
			circle(img, cv::Point(path_array[i]->y, path_array[i]->x), 2, Scalar(0), -1);
			circle(img_final_path, cv::Point(path_array[i]->y, path_array[i]->x), 2, Scalar(0), -1);
		}

		if ((first_flag == 0))
		{
			last_state = now_state;
			first_flag = 1;
			path_optimizer_array_size++;
			path_optimizer_array[path_optimizer_array_size] = path_array[i];//�洢״̬�仯�Ĺյ�
			continue;
		}

		if ((now_state != last_state))
		{
			path_optimizer_array_size++;
			path_optimizer_array[path_optimizer_array_size] = path_array[i];//�洢״̬�仯�Ĺյ�
			//std::cout << "need :  " << path_array[i]->y << ' ' << path_array[i]->x << std::endl;
			circle(img_final_path, cv::Point(path_array[i]->y, path_array[i]->x), 2, Scalar(0), -1);
		}

		last_state = now_state;
	}
	//imshow("123", img);
	//waitKey(1);
	path_optimizer_array_size++;
	path_optimizer_array[path_optimizer_array_size] = path_array[path_long];//�洢���һ����

	std::cout << "Path_optimizer: " << "Plan to optimize " << path_optimizer_array_size << " part path. "<<std::endl;
	//����ÿ�����ӵе㣬���еڶ���A*���·�Ż�����ȥ���յ�
	for (int i = 1; i <= path_optimizer_array_size - 1; i++)
	{
		//std::cout << "Optimize " << i << " to " << i + 1 << std::endl;
		bool end_flag = 0, now_meet_state = is_met(enemy1, path_optimizer_array[i]->x, path_optimizer_array[i]->y);
		//std::cout << now_meet_state << std::endl;
		min_dist_goal_x = path_optimizer_array[i + 1]->x;
		min_dist_goal_y = path_optimizer_array[i + 1]->y;
		memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(0, memory_pool_optimizer_i, path_optimizer_array[i]->x, path_optimizer_array[i]->y);
		Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
		//std::cout << Q.size() << std::endl;
		while (end_flag == 0)
		{
			my_point* now = Q.top();
			Q.pop();
			flag_optimizer[now->x][now->y] = 1;
			//std::cout << now->x << ' ' << now->y << ' ' << now_meet_state << ' '<< path_optimizer_array[i + 1]->x<<' '<< path_optimizer_array[i + 1]->y<< ' ' << is_met(enemy1, now->x, now->y)<<std::endl;
			//std::cout << is_met(enemy1, now->x + 1, now->y) << ' ' << is_met(enemy1, now->x, now->y + 1) << ' ' << is_met(enemy1, now->x, now->y - 1) << ' ' << is_met(enemy1, now->x - 1, now->y) << std::endl << std::endl;
			if ((now->x == path_optimizer_array[i + 1]->x) && (now->y == path_optimizer_array[i + 1]->y))
			{
				end_flag = 1;
				//���ݲ�����·��
				my_point now_point = *now;
				path_long = 1;
				while (now_point.father != now_point.self)
				{
					//img.at<uchar>(now_point.x, now_point.y) = 0;
					//path_map.at<uchar>(now_point.x, now_point.y) = 0;
					my_point father_point = *(memory_pool_optimizer[now_point.father]);
					now_point = father_point;
					short_path_array[path_long] = memory_pool_optimizer[now_point.self]; //�ڴ���е�ָ�뿽����һ��
					path_long++;
				}
				path_long--;

				//���ݴ����·���ǵ������ģ� ����Ҫ����������
				for (int i = 1; i <= path_long; i++)
				{
					//�����ż����ѭ����һ����У�����ѭ�����м��ǰһ��
					if (path_long % 2 == 0) {
						if (i == (path_long) / 2 + 1) break;
					}
					else if (i == (path_long + 1) / 2) break;

					//������ûʲô��˵��
					my_point* temp = short_path_array[i];
					short_path_array[i] = short_path_array[path_long - i + 1];
					short_path_array[path_long - i + 1] = temp;
				}

				//��ô����һ�����⣬����ÿһ���ӵнڵ�ᱻ��final_path�м�¼2�飬����Ӱ�첻��
				//���������ĳ���Ӧ����������覴ã���󵱸�֮
				for (int i = 1; i <= path_long; i++)
				{
					path_map.at<uchar>(short_path_array[i]->x, short_path_array[i]->y) = 0;
					final_path[final_path_i] = navigation_point(short_path_array[i]->x, short_path_array[i]->y);
					final_path_i++;
				}

				//find_corner();

				//һ�����·��������֮����ƺ�������
				while (Q.empty() != 1) Q.pop();
				//����ڴ��
				while (memory_pool_optimizer_i != -1)
				{
					delete(memory_pool_optimizer[memory_pool_optimizer_i]);
					memory_pool_optimizer_i = memory_pool_optimizer_i - 1;
				}
				memory_pool_optimizer_i = 0;
				//���A*������飬��Ȼ�´��Ѳ�����
				std::memset(flag_optimizer, 0, sizeof(flag_optimizer));
				break;
			}
			//(now_meet_state == 0) && 
			if (((now->x + 1 == path_optimizer_array[i + 1]->x) && (now->y == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x+1, now->y) == now_meet_state))  && (end_flag == 0) && (now->x + 1 <= map_x - 1) && (flag_optimizer[now->x + 1][now->y] == 0) && (a[now->x + 1][now->y] == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x + 1][now->y] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x + 1, now->y);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//y+1  ��(now_meet_state == 0) && 
			if (((now->x == path_optimizer_array[i + 1]->x) && (now->y + 1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x, now->y + 1) == now_meet_state)) && (now->y + 1 <= map_y - 1) && (flag_optimizer[now->x][now->y + 1] == 0) && (a[now->x][now->y + 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x][now->y + 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x, now->y + 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//y-1 ��(now_meet_state == 0) && 
			if (((now->x == path_optimizer_array[i + 1]->x) && (now->y - 1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x, now->y - 1) == now_meet_state)) && (now->y - 1 >= 0) && (flag_optimizer[now->x][now->y - 1] == 0) && (a[now->x][now->y - 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x][now->y - 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x, now->y - 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//x-1  ��(now_meet_state == 0) && 
			if (((now->x - 1 == path_optimizer_array[i + 1]->x) && (now->y == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x - 1, now->y) == now_meet_state)) && (now->x - 1 >= 0) && (flag_optimizer[now->x - 1][now->y] == 0) && (a[now->x - 1][now->y] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x - 1][now->y] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x - 1, now->y);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//����
			if (((now->x - 1 == path_optimizer_array[i + 1]->x) && (now->y-1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x - 1, now->y-1) == now_meet_state)) && (now->x - 1 >= 0) && (now->y - 1 >= 0) && (flag_optimizer[now->x - 1][now->y - 1] == 0) && (a[now->x - 1][now->y - 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x - 1][now->y - 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x - 1, now->y - 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//
			if (((now->x - 1 == path_optimizer_array[i + 1]->x) && (now->y+1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x - 1, now->y+1) == now_meet_state)) && (now->x - 1 >= 0) && (now->y + 1 <= map_y - 1) && (flag_optimizer[now->x - 1][now->y + 1] == 0) && (a[now->x - 1][now->y + 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x - 1][now->y + 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x - 1, now->y + 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//
			if (((now->x + 1 == path_optimizer_array[i + 1]->x) && (now->y-1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x + 1, now->y-1) == now_meet_state)) && (now->x + 1 <= map_x - 1) && (now->y - 1 >= 0) && (flag_optimizer[now->x + 1][now->y - 1] == 0) && (a[now->x + 1][now->y - 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x + 1][now->y - 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x + 1, now->y - 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//
			if (((now->x + 1 == path_optimizer_array[i + 1]->x) && (now->y+1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x + 1, now->y+1) == now_meet_state)) && (now->x + 1 <= map_x - 1) && (now->y + 1 <= map_y - 1) && (flag_optimizer[now->x + 1][now->y + 1] == 0) && (a[now->x + 1][now->y + 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x + 1][now->y + 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x + 1, now->y + 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
		}
	}
}

void path_planner()
{
		if (need_replan_path == 1) {
			std::cout << "Running path_planner..." << std::endl;
			std::cout<<"need to plan to "<<goalx << ' '<< goaly<<std::endl;
			img = img_plain_save.clone();
			//img_final_path = img_plain_save.clone();
			bool end_flag = 0;
			Refresh_enemy_fire_cost();//ˢ�»�����ͼ
			cv::circle(img_erode_save, cv::Point(goaly, goalx), 2, cv::Scalar(0),2);
			imshow("Simulation_Environment@HEUsjh", img_erode_save);
			waitKey(10);
			std::cout<<"now x  "<<robot1.x <<"nowy   "<<robot1.y<<std::endl;
			std::cout<<memory_pool_i<<endl;
			keep_xy_safe(robot1.x, robot1.y);
			//����·����㣬�������丸�ڵ�Ϊ�Լ���������ֹ����
			memory_pool[memory_pool_i] = new my_point(0, memory_pool_i, robot1.x, robot1.y);
			search_queue.push(memory_pool[memory_pool_i]);
			//cv::waitKey(1000);
			auto t1 = std::chrono::high_resolution_clock::now();
			//���������ı�������:����û�б�ǿ����ֹ�� �������зǿգ� ����̨����û�б���ֹ
			while (end_flag == 0 && search_queue.empty() == 0 && ros::ok())
			{
				if (search_queue.empty() == 1){
					std::cout<<"Search queue is empty, please check now position or goal point. Is they are in the wall??"<<std::endl;
					break;
				}
				//ȡ����
				my_point* now = search_queue.top();
				std::cout<<"nowx, nowy, goalx, goaly"<<now->x<<' '<<now->y<<' ' <<goalx<<' '<<goaly<<std::endl;
				std::cout<<"queue size = " << search_queue.size()<<std::endl;
				if (search_queue.size() == -1){
					return;
				}
				//cv::waitKey(500);
				search_queue.pop();
				flag[now->x][now->y] = 1;

				//�˶δ������ڴ�ӡ��ǰ������һ���ڵ�
				//std::cout << '(' << now->x << ',' << now->y << ")    h(x,y) = " << h(now->x, now->y) << std::endl;
				//�˶δ������ڴ�ӡ��ǰ���������е����нڵ㣬�����ֽ׶���Ȼ����Ҫ�ˣ�ֻ�ڱ�д�����õ���
				//���ǰ�search_queue������һ��Ȼ��ȫ��ȡ������һ�顣���ȶ��в��ܹ�ʹ�õ���������
				//��Ϊû���ṩobject.begin(),object.end()��Ա����
				//if (search_queue.size()!=0)test = search_queue;
				//while (test.size() != 0)
				//{
				//	my_point* now2 = test.top();
				//	std::cout << '(' << now2->x << ',' << now2->y << ")    h(x,y) = " << h(now2->x, now2->y)  << std::endl;
				//	test.pop();
				//}
				//std::cout << std::endl << std::endl;

				//������ֹ
				if ((now->x == goalx) && (now->y == goaly))
				{
					std::cout << "search complete" << std::endl;
					end_flag = 1;
					auto t2 = std::chrono::high_resolution_clock::now();

					//���ݲ�����·��
					my_point now_point = *now;
					int now_step = 0;
					need_replan_path = 0;
					path_long = 1;
					while (now_point.father != now_point.self)
					{
						img.at<uchar>(now_point.x, now_point.y) = 0;
                                                //path_map.at<uchar>(now_point.x, now_point.y) = 0;
						my_point father_point = *(memory_pool[now_point.father]);
						now_point = father_point;
						path_array[path_long] = memory_pool[now_point.self]; //�ڴ���е�ָ�뿽����һ��
						path_long++;
						now_step++;
					}
					path_long--;

					//��ӡ·��������Ϣ
					std::cout << "Use point = " << memory_pool_i << std::endl;
					std::cout << "Queue size = " << search_queue.size() << std::endl;
					std::cout << "Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << std::endl;
					std::cout << "Total step = " << now_step << std::endl;
					std::cout << std::endl;

					//���ݴ����·���ǵ������ģ� ����Ҫ����������
					for (int i = 1; i <= path_long; i++)
					{
						//�����ż����ѭ����һ����У�����ѭ�����м��ǰһ��
						if (path_long % 2 == 0) {
							if (i == (path_long) / 2 + 1) break;
						}
						else if (i == (path_long + 1) / 2) break;

						//������ûʲô��˵��
						my_point* temp = path_array[i];
						path_array[i] = path_array[path_long - i + 1];
						path_array[path_long - i + 1] = temp;
					}

					/*for (int i = 1; i <= path_long; i++)
						std::cout << path_array[i]->x << ' ' << path_array[i]->y << std::endl;*/
					/*for (int i = 0; i < img.rows; i++) {
						for (int j = 0; j < img.cols; j++)
							std::cout << enemy_fire_cost[i][j] << ' ';
						std::cout << std::endl;
					}*/
					//std::cout << std::endl << " is_met :   " << is_met(enemy1, 187, 39) << std::endl;
					//std::cout << std::endl << "is cross:   " << is_cross(enemy1.x, enemy1.y, 187, 39, 186, 37, 190, 37);
					circle(img, cv::Point(enemy1.y, enemy1.x), 2, Scalar(0), -1);
					//imwrite("path_out.png", img);
					//imshow("path_img", path_map);

					//robot1.x = goalx;
					//robot1.y = goaly;

					//���������ͼ
					//img = imread("map2.png", -1);
					//simulation_map = img;
					end_flag = 1;
					need_replan_path = 0;
				}
				
				std::cout<<now->x+1<<' ' << now->y << ' '<<flag[now->x+1][now->y]<<' '<<a[now->x+1][now->y]<<' '<<end_flag<<endl;
				std::cout<<now->x+1<<' ' << now->y << ' '<<flag[now->x+1][now->y]<<' '<<a[now->x+1][now->y]<<' '<<end_flag<<endl;
				std::cout<<now->x+1<<' ' << now->y << ' '<<flag[now->x+1][now->y]<<' '<<a[now->x+1][now->y]<<' '<<end_flag<<endl;

				//cv::waitKey(500);
				//�������������������ֱ��ǣ�û��Խ����ͼ�߽磬û���߹�����������ȥ������ǽ��
				//��ʵԽ���������ɾ���ģ���Ϊ�һ���ͼ��ʱ��ͻ��˱߽�
				//�������˸����ģ��������ð��ˣ�
				//x+1  ��
				if (now->x + 1 <= map_x - 1 && (flag[now->x + 1][now->y] == 0) && (a[now->x + 1][now->y] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x + 1][now->y] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//y+1  ��
				if (now->y + 1 <= map_y - 1 && (flag[now->x][now->y + 1] == 0) && (a[now->x][now->y + 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x][now->y + 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y + 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//y-1 ��
				if (now->y - 1 >= 0 && (flag[now->x][now->y - 1] == 0) && (a[now->x][now->y - 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x][now->y - 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y - 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//x-1  ��
				if (now->x - 1 >= 0 && (flag[now->x - 1][now->y] == 0) && (a[now->x - 1][now->y] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x - 1][now->y] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//����
				if ((now->x - 1 >= 0) && (now->y - 1 >= 0) && (flag[now->x - 1][now->y - 1] == 0) && (a[now->x - 1][now->y - 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x - 1][now->y - 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y - 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//
				if ((now->x - 1 >= 0) && (now->y + 1 <= map_y - 1) && (flag[now->x - 1][now->y + 1] == 0) && (a[now->x - 1][now->y + 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x - 1][now->y + 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y + 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//
				if ((now->x + 1 <= map_x - 1) && (now->y - 1 >= 0) && (flag[now->x + 1][now->y - 1] == 0) && (a[now->x + 1][now->y - 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x + 1][now->y - 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y - 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//
				if ((now->x + 1 <= map_x - 1) && (now->y + 1 <= map_y - 1) && (flag[now->x + 1][now->y + 1] == 0) && (a[now->x + 1][now->y + 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x + 1][now->y + 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y + 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}

			}
			printf("search complete!!!\n");
			//std::cout << "out while";
		}
}

void delete_now_path()
{
	int test = 0;
	while (search_queue.empty() != 1) search_queue.pop();
	//����ڴ��
	while (memory_pool_i != -1)
	{
		
		delete(memory_pool[memory_pool_i]);
		memory_pool_i = memory_pool_i - 1;
		test++;
	}
	std::cout << "Cleaner Function:  Delete " << test-1 << " node" << std::endl;
	memory_pool_i = 0;
	//���A*������飬��Ȼ�´��Ѳ�����
	std::memset(flag, 0, sizeof(flag));
	//���·����������
	std::memset(path_array, 0, sizeof(path_array));
	path_long = 0;
	//��չյ�����
	std::memset(corner_array, 0, sizeof(corner_array));
	number_of_corner = 0;
	//��Ҫ�滮·����־λ����
	need_replan_path = 0;

	path_map = cv::Mat(226, 406, CV_8UC1, Scalar(255));
	final_path_i = 1;
	memset(final_path, 0, sizeof(final_path));
	return;
}

void Position_correction()
{

}

//��ȡpath_array[now]����һ���յ�ķ�������
//ע�⣬һ������һ���յ㣬��������һ��·���㣬��Ȼ�ͻ����֮ǰ�ҹյ�ʱ���ֵ�ͬ��������
void get_angle(int now)      
{

}

double get_k(std::vector<cv::Point>my_line)
{
        /* !x
         * |   k>0
         * |
         * *-------->y (90)
         * |
         * |   k<0
         * |
         * |
         */
	double pointCount = (double)my_line.size();
	double xCount = 0, yCount = 0, xyCount = 0, xxCount = 0;
	for (const auto& p : my_line)
	{
		xCount += p.x;
		yCount += p.y;
		xyCount += p.x * p.y;
		xxCount += p.x * p.x;
	}
	if ((double)(pointCount * xxCount - xCount * xCount) == 0) return 90.0;
	double k = (pointCount * xyCount - xCount * yCount) / (pointCount * xxCount - xCount * xCount);
	double sinValue = -k / (std::sqrt(1 + k * k));
	double rad = std::asin(sinValue);
	double pi = 3.14159265354;
	double angle = rad * 180.0 / pi;
	return angle;
}

// void update_velocity_callback(const move_control:: velocity :: ConstPtr& msg_receive)
// {
// 	robot1.x = (msg_receive->vx) / 1000 / 2;
// 	robot1.y = (msg_receive->vy) / 1000 / 2;
// 	//auto t1 = std::chrono::high_resolution_clock::now();
//         // auto dt = ((static_cast<std::chrono::duration<float, std::milli>>(t1- last_receive_time)).count()) / (double)(1000.0);
//         // last_receive_time = t1;
//         // if (abs(msg_receive->omiga) > 500) return;
//         // robot1.angle = msg_receive->omiga;
//         // if (std::abs((msg_receive->vx * std::cos(robot1.angle / 180.0 * 3.14159265354) - msg_receive->vy * std::sin(robot1.angle / 180.0 * 3.14159265354)) * (double)(dt / 1000.0))>10000) return;
//         // if (std::abs((msg_receive->vx * std::sin(robot1.angle / 180.0 * 3.14159265354) + msg_receive->vy * std::cos(robot1.angle / 180.0 * 3.14159265354)) * (double)(dt / 1000.0))>10000) return;
//         // //robot1.y += ((msg_receive->vx * std::cos(robot1.angle) - msg_receive->vy * std::sin(robot1.angle)) * (double)(dt / 1000.0)) / 2;
//         // //robot1.x += ((msg_receive->vx * std::sin(robot1.angle) + msg_receive->vy * std::cos(robot1.angle)) * (double)(dt / 1000.0)) / 2;
//         // //if (robot1.angle < 0) {
//         //     robot1.y += (((msg_receive->vx * std::cos(robot1.angle / 180.0 * 3.14159265354) + msg_receive->vy * std::sin(robot1.angle / 180.0 * 3.14159265354)) )* (double)(dt )) / 2;
//         //     robot1.x += (((msg_receive->vy * std::cos(robot1.angle / 180.0 * 3.14159265354) - msg_receive->vx * std::sin(robot1.angle / 180.0 * 3.14159265354)) )* (double)(dt )) / (2);
//         //ROS_INFO("dy= %lf* %lf+ %lf* %lf= %lf\n",msg_receive->vx, std::cos(robot1.angle / 180.0 * 3.14159265354), msg_receive->vy, std::sin(robot1.angle / 180.0 * 3.14159265354), ((msg_receive->vx * std::cos(robot1.angle / 180.0 * 3.14159265354) + msg_receive->vy * std::sin(robot1.angle / 180.0 * 3.14159265354))));
//         //ROS_INFO("dx= %lf* %lf- %lf* %lf= %lf\n",msg_receive->vy, std::cos(robot1.angle / 180.0 * 3.14159265354), msg_receive->vx, std::sin(robot1.angle / 180.0 * 3.14159265354), ((msg_receive->vy * std::cos(robot1.angle / 180.0 * 3.14159265354) - msg_receive->vx * std::sin(robot1.angle / 180.0 * 3.14159265354))));
//         //}
//         //else
//         //{
//         //    robot1.x += (((msg_receive->vx * std::sin(robot1.angle / 180.0 / 3.14159265354) + msg_receive->vy * std::cos(robot1.angle / 180.0 / 3.14159265354)) )* (double)(dt / 1000.0)) / 2;
//         //    robot1.y += (((msg_receive->vy * std::cos(robot1.angle / 180.0 / 3.14159265354) - msg_receive->vx * std::sin(robot1.angle / 180.0 / 3.14159265354)) )* (double)(dt / 1000.0)) / 2;

//         //}
//         //robot1.y += 0.0;
//         //ROS_INFO("vx = %lf y = %f ", robot1.x, robot1.y);
//         //ROS_INFO("vx = %lf vy = %lf angle:%lf", msg_receive->vx, msg_receive->vy,msg_receive->omiga);
// }


#define USE_PID_MOVE_CONTROL 1
double last_errorx, last_errory, errorx, errory, PID_dt;
auto lasttime = std::chrono::high_resolution_clock::now();
double kd = 0.1*0, kp = 0.02; bool PID_first_flag = 0;

void PID_move_control(const auto&, float, float);

void Navigation_base_on_odom(const auto& move_control_publisher)
{
	//�ָ�·�β������·��б��
	find_corner();
#if WATCH_ROAD_PLAN
        while(1);
#endif

#if USE_PID_MOVE_CONTROL
	int next_corner = 0;
	while (next_corner < corner_vector.size() && ros::ok()){
		//Attention, corner_vector is the output of OpenCV API, so the xy of its point is oppisite of ours
		if (my_dist(robot1.x, robot1.y, corner_vector[next_corner].y, corner_vector[next_corner].x) < 2)
			next_corner++;	//switch to next goal point
		
		//if (check_path() == 0) return;
		//׼�����Ϳ���֡
		// move_control :: my_control_frame _Contraldata;
		// _Contraldata.vx = float(corner_vector[next_corner].y) * 2;
		// _Contraldata.vy = float(corner_vector[next_corner].x) * 2;
		// _Contraldata.angle = 1;
		// move_control_publisher.publish(_Contraldata);
		// printf("sending control frame, x,y = (%f, %f), my x,y = (%f. %f)\n", _Contraldata.vx, _Contraldata.vy, robot1.x, robot1.y);
		PID_move_control(move_control_publisher, float(corner_vector[next_corner].y), float(corner_vector[next_corner].x));

		//printf("corner vector size = %d, next_corner = %d  x,y = (%f, %f)\n", corner_vector.size(),  next_corner, float(corner_vector[next_corner].y), float(corner_vector[next_corner].x));
		printf("%d  %d", robot1.x, robot1.y);
		circle(img_final_path, cv::Point(robot1.y, robot1.x), 2, Scalar(0), -1);
		circle(img_final_path, cv::Point(corner_vector[next_corner].x, corner_vector[next_corner].y), 3, Scalar(0), -1);
        imshow("Simulation_Environment2@HEUsjh", img_final_path);
        waitKey(10);
		ros::spinOnce();
	}
	printf("PID control succed!!!\n");
	PID_first_flag = 0;
#else

	//��ǰ����corner[now_path]-corner[now_path+1]·����
	int now_path = 0;
	//��ǰ��·���������һ���ڵ㣬��Ӧ��final_path[now[
	int now = 0;
        ros::Rate rate(100);
	auto lasttime = std::chrono::high_resolution_clock::now();
	while (stop_navigation_flag == 0)
	{
		//auto t1 = std::chrono::high_resolution_clock::now();
                circle(img_final_path, cv::Point(robot1.y, robot1.x), 2, Scalar(0), -1);
                imshow("Simulation_Environment2@HEUsjh", img_final_path);
                waitKey(10);
		//ˢ��ROS���Ļ���
		ros::spinOnce();
		
		//ÿ�δ�·������ȡһ������ĵ�
		int mmin = 9999999, mmin_number;
		for (int i = 1; i <= final_path_i; i++)
		{
			if (my_dist(final_path[i].x, final_path[i].y, robot1.x, robot1.y) < mmin)
			{
				mmin = my_dist(final_path[i].x, final_path[i].y, robot1.x, robot1.y);
				mmin_number = i;
			}
		}
        now = mmin_number;
        controlframe.angle = final_path[now].should_angle;
        //std::cout << "Now position:   X: " << robot1.x << "  Y: " << robot1.y << "  Angle: " << robot1.angle << "  next corner dist is  " << final_path[now].next_corner_dis << "  next corner is: " << final_path[now].nextcorner<<std::endl;
        //ROS_INFO("X: %lf;  Y: %lf; goalx= %d  goaly= %d;  dis=%lf;  angle=%lf;  cont_ang= %d;  next_c: %d;  now:%d; total:%d \n",robot1.x, robot1.y, corner_vector[final_path[now].nextcorner].y, corner_vector[final_path[now].nextcorner].x, final_path[now].next_corner_dis,robot1.angle, controlframe.angle,final_path[now].nextcorner, now,final_path_i );
        ROS_INFO("X:%lf; Y:%lf; dis=%lf; now=%d; angle=%lf; cont_ang= %d; c_vx= %lf; goal=(%d,%d) next_c: %d;  now:%d; total:%d\n",robot1.x, robot1.y, final_path[now].next_corner_dis, now, robot1.angle, controlframe.angle,controlframe.vx,  goalx, goaly,final_path[now].nextcorner, now,final_path_i);
        //std::cout << "now is:  "<<now << ' ';
		//ת����ǰ��10cm / 2cm = 5
		
		//�����ٶȿ��ƾ���
        if (final_path[now].next_corner_dis <= 2 && now!=1 && now!=2){
			//cout << "---------------arrive corner----" << std::endl;
			controlframe.vx = 0;
			controlframe.vy = 0;
		}
		else{
            controlframe.vx = 0.4;
			controlframe.vy = 0;
		}

        if (my_dist(goalx,goaly, robot1.x, robot1.y) <= 2){//��10cm��Ϊ�ִ����յ�
			stop_navigation_flag = 1;
            need_replan_path = 0;
        }
		
		move_control :: my_control_frame _Contraldata;
		_Contraldata.vx = controlframe.vx;
		_Contraldata.vy = controlframe.vy;
		_Contraldata.angle = controlframe.angle;
		move_control_publisher.publish(_Contraldata);
		ros::spinOnce();

		//<< "   now is in path : "<< now_path
                //ROS_INFO("control to vx = %lf   control to angle = %d; goalx=%d  goaly=%d\n",controlframe.vx, controlframe.angle, corner_vector[final_path[now].nextcorner].y, corner_vector[final_path[now].nextcorner].x);//<< "   now in final_path is: " << now

		//auto t2 = std::chrono::high_resolution_clock::now();
		//double deltatime = (static_cast<std::chrono::duration<double, std::milli>>(t2 - lasttime)).count();
		//lasttime = t2;
		//robot1.angle = controlframe.angle;
		//if (controlframe.vx == 0)
		//{
		//	robot1.y = corner_vector[final_path[now].nextcorner].x;
		//	robot1.x = corner_vector[final_path[now].nextcorner].y;
		//}
		//	robot1.x += controlframe.vx * deltatime * std::sin(robot1.angle / 180.0 * 3.1415926);
		//	robot1.y += controlframe.vx * deltatime * std::cos(robot1.angle / 180.0 * 3.1415926);
		//std::cout << "next corner x:  " << corner_vector[final_path[now].nextcorner].y << "  y: "<< corner_vector[final_path[now].nextcorner].x<<"   cos(angle) is:  " << std::cos(robot1.angle / 180.0 * 3.1415926)<<std::endl;
		//�����ٶȿ�����Ϣ
		//ros::
                rate.sleep();
	}
#endif
	stop_navigation_flag = 0;
    need_replan_path = 0;
}

void Navigation()
{
	int nowx = robot1.x;
	int nowy = robot1.y;
	int now_path_step = 1;//��ǰ����ָ�룬ָ���������·���ϵ���һ����
	double mindis = 999999;
	int min_dis_point = 1;//�ݴ��������·����������path_array[]�±�
	int last_corner = 1, next_corner = 2;//�����յ���ɵ�������������·������ķ�������
	while (true)
	{
		//refresh_simulation_environment();
		//ÿ����·������һ�������������ĵ㣬�����߶�������һ�������̷�Χ�ڵ�ʱ�򣬼�������
		//�������һ����Χ���������ģʽ������һ���ٶ�����ָ��·������������ĵ㣬����ִ�õ�֮�󣬼�������
		//���ò�˵��ROSȷʵ�������Ƶģ�����ƾc++��ѷ��滷���ӽ�����������ܹ���ĺ�����ƣ���Ϊ����ܹ�������ԥ�˺þã����������ֲ�����ROS��
		for (int i = 1; i <= path_long; i++)
		{
			if (sqrt((robot1.x - path_array[i]->x) * (robot1.x - path_array[i]->x) + (robot1.y - path_array[i]->y) * (robot1.y - path_array[i]->y)) < mindis) {
				mindis = sqrt((robot1.x - path_array[i]->x) * (robot1.x - path_array[i]->x) + (robot1.y - path_array[i]->y) * (robot1.y - path_array[i]->y));
				min_dis_point = i;
			}
		}
		//25 * 2 = 50cm����ƫ��50cm��ʱ����Ϊ�Ѿ�ƫ������ô����λ�ý���ģʽ�����ٻع�·��
		//if (mindis > 25) Position_correction();
		//if (abs(robot1.angle - get_angle())
		//ע�⣬�ڷ���ת���ʱ��һ��Ҫ���ע�⡣������corner���жϳ�������ƫ������ô���ع��ʱ����Ѿ�����ת����
		//���������Է���ת�򣬷�����군��

	}
}

#if USE_PID_MOVE_CONTROL

//��һ������������������ROSͨ�ž��
//�ڶ��������ǵ�ǰ���Ƶ�Ŀ���
void PID_move_control(const auto& move_control_publisher, float goalpoint_x, float goalpoint_y)
{
	//���㵱ǰλ�����
	errorx = (int(goalpoint_x) - robot1.x);
	errory = int(goalpoint_y) - robot1.y;

	if (PID_first_flag == 0){
		PID_first_flag = 1;
		last_errorx = errorx;
		last_errory = errory;
		lasttime = std::chrono::high_resolution_clock::now();
		return;
	}
	
	if (PID_first_flag == 1){
		auto lasttime2 = std::chrono::high_resolution_clock::now();
		PID_dt = ((static_cast<std::chrono::duration<double, std::milli>>(lasttime2 - lasttime)).count())/1000;
		lasttime = std::chrono::high_resolution_clock::now();
	}

	//����vx, vy
	double control_vx, control_vy;
	control_vx = kp * (errorx  + kd * (last_errorx - errorx) / PID_dt);
	control_vy = kp * (errory  + kd * (last_errory - errory) / PID_dt);

	//�޷�
	if (abs(control_vx) >= 0.2){
		if (control_vx > 0) control_vx = 0.2;
			else control_vx = -0.2;
	}
	if (abs(control_vy) >= 0.2){
		if (control_vy > 0) control_vy = 0.2;
			else control_vy = -0.2;
	}

	//׼�����Ϳ���֡
	move_control :: my_control_frame _Contraldata;
	_Contraldata.vx = control_vy * 1000.0;
	_Contraldata.vy = control_vx * 1000.0;
	_Contraldata.angle = 0;
	move_control_publisher.publish(_Contraldata);

	printf("control_v = (%1f, %1f);   goal = (%1f, %1f)  now = (%d, %d)  error = (%1f, %1f);\n",control_vx, control_vy, goalpoint_x, goalpoint_y, robot1.x, robot1.y, errorx, errory);

	ros::spinOnce();

	//����last_error
	last_errorx = errorx;
    last_errory = errory;
}
#endif


void update_command_callback(const geometry_msgs :: Point ::ConstPtr& msg_receive)
{	
	//���յĵ�����Ϣ�Ĵ���
	if (int(msg_receive -> z) == 2020080119){
		//delete_cant_go(cv::Point(enemy1.x, enemy1.y), 20);
		enemy1.x = msg_receive -> y; enemy1.y = msg_receive -> x;
		//printf("recv enemy = (%d,   %d    %lf)\n", enemy1.x, enemy1.y, msg_receive -> z);
		return;
	}
	// //��ֹ��Ϊ���˵�ʶ���֡���°��Լ�����ʴ��
	// if (((enemy1.x - robot1.x) * (enemy1.x - robot1.x) + (enemy1.y - robot1.y) * (enemy1.y - robot1.y)) > 10){
	// 	add_cant_go(cv::Point(enemy1.x, enemy1.y), 20);
	// 	printf("enemy position (x, y) = (%d, %d)\n", enemy1.x, enemy1.y);
	// }
	if (is_race_start == 0) return;
	if ((need_replan_path == 0) && (int(msg_receive -> z) != 2020080119)){
		if ((int(msg_receive->y) - robot1.x) * (int(msg_receive->y) - robot1.x) + (int(msg_receive->x) - robot1.y) * (int(msg_receive->x) - robot1.y) >= 10){
			goalx = msg_receive->y;
			goaly = msg_receive->x;
			keep_xy_safe(goalx, goaly);
			need_replan_path = 1;

			//printf("Listend command from TCP server, will plan to go to :\n");
			printf("recv goalx = %d,       goaly = %d\n", goalx, goaly);
		}
		else printf("too near,  dist = %d now = (%d, %d); goal = (%d, %d)\n", (goalx - robot1.x) * (goalx - robot1.x) + (goaly - robot1.y) * (goaly - robot1.y), robot1.x, robot1.y, goalx, goaly);
	}
	else printf("Busy\n");
}

//set robot init position
//1       2
//4       3
void init_robot_position(int num)
{
    int nowx, nowy, nowangle;
    if (num == 1) {nowx = 30; nowy = 25; nowangle = 0;}
    if (num == 2) {nowx = 30; nowy = map_y-25; nowangle = 180;}
    if (num == 3) {nowx = map_x-30; nowy = map_y-25; nowangle = 180;}
    if (num == 4) {nowx = map_x-30; nowy = 25; nowangle = -90;}
    robot1.x = nowx;
    robot1.y = nowy;
    robot1.angle = nowangle;
}

void update_position_callback (const geometry_msgs::Point::ConstPtr& msg_recv);

void update_race_state_callback(const move_control :: race_state :: ConstPtr& msg_recv);

int main(int argc, char **argv)
{	
	//��ʼ����̼ƽڵ�
    ros::init(argc, argv, "move_control");
	//����ͨ�ž��
	ros::NodeHandle communication_handle_obj;
	ros::Publisher move_control_publisher = 
				communication_handle_obj.advertise <move_control :: my_control_frame> ("/robot_control",10);

    //������Ϣ
   // ros::Subscriber race_state_subscriber = communication_handle_obj.subscribe("/velocity", 10, update_race_state_callback);
	ros::Subscriber now_pos_subscriber = communication_handle_obj.subscribe("/now_pos", 10, update_position_callback);

	ros::Subscriber my_command_listener = communication_handle_obj.subscribe("/TCP_command_listener", 10, update_command_callback);

	ros::Subscriber race_state_listener = communication_handle_obj.subscribe("/race_state", 10, update_race_state_callback);


	//program_start_time = std::chrono::high_resolution_clock::now();//ʱ������֮ʼҲ

	init_load_map();//���ص�ͼ
	init_load_wall_line();//����ǽ����Ϣ
	init_load_erode_wall_line();//���ظ�ʴ֮��ĵ�ͼ��Ϣ

	namedWindow("Simulation_Environment@HEUsjh");//�������滷������
	setMouseCallback("Simulation_Environment@HEUsjh", mouse_handle);//��������¼��ص�����

    //init_robot_position(1);

	need_replan_path = 0;
	img = img_plain_save.clone(); 
	img_final_path = img_plain_save.clone();
	
	auto t_start = std::chrono::high_resolution_clock::now();
	bool first_flag = 0;
	bool zone_start_change = 0;

	//���빤��
	while (ros::ok())
	{
		if (is_race_start == 0){
			move_control :: my_control_frame _Contraldata;
			_Contraldata.vx = 0;
			_Contraldata.vy = 0;
			_Contraldata.angle = 0;
			move_control_publisher.publish(_Contraldata);

			circle(img_final_path, cv::Point(robot1.y, robot1.x), 2, Scalar(0), -1);
			imshow("Simulation_Environment2@HEUsjh", img_final_path);
			waitKey(10);

			imshow("Simulation_Environment@HEUsjh", img_erode_save);
			//imshow("Simulation_Environment3@HEUsjh", img);
			//imshow("Simulation_Environment2@HEUsjh", img_final_path);
			ros::spinOnce();
			//ROS_INFO("Searching hit goal!!!\n");
			waitKey(1);
			continue;
		}

		if (is_race_start == 1 && first_flag == 0){
			t_start = std::chrono::high_resolution_clock::now();
			first_flag = 1;
			bullet_buff_state = 1;
		}

		if (need_replan_path == 1) { 
			//delete_now_path();
			auto t1 = std::chrono::high_resolution_clock::now();
			path_planner(); 
            path_optimizer();
			Navigation_base_on_odom(move_control_publisher);
			delete_now_path(); //��Ҫע�����final_path��ɾ��ʱ��
			auto t2 = std::chrono::high_resolution_clock::now();
			std::cout << "Memory pool Clear!" << std::endl; 
			std::cout << "Total Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << std::endl;
			std::cout << "----------------------------" << std::endl<<std::endl;
		}
		
		//Navigation();
		//draw_path();

		//printf("now time = %lf\n",(static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - t_start)).count() / 1000);
		//second buff
		if ((static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - t_start)).count() / 1000 >= 160.0)
			{
				if (need_replan_path == 0){
					goalx = 30;
					goaly = 25;
					need_replan_path = 1;
					break;
				}
			}

		if ((static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - t_start)).count() / 1000 >= 125.0)
			bullet_buff_state = 1;


		move_control :: my_control_frame _Contraldata;
		_Contraldata.vx = 0;
		_Contraldata.vy = 0;
		_Contraldata.angle = 0;
		move_control_publisher.publish(_Contraldata);

		circle(img_final_path, cv::Point(robot1.y, robot1.x), 2, Scalar(0), -1);
        imshow("Simulation_Environment2@HEUsjh", img_final_path);
        waitKey(10);

        imshow("Simulation_Environment@HEUsjh", img_erode_save);
        //imshow("Simulation_Environment3@HEUsjh", img);
		//imshow("Simulation_Environment2@HEUsjh", img_final_path);
		ros::spinOnce();
        //ROS_INFO("Searching hit goal!!!\n");
		waitKey(1);
	// 	goalx = 30; goaly = 100;
	// need_replan_path = 1;
		
	}
	while (ros::ok())
	{
		move_control :: my_control_frame _Contraldata;
		_Contraldata.vx = 0;
		_Contraldata.vy = 0;
		_Contraldata.angle = 0;
		move_control_publisher.publish(_Contraldata);
		ros::spinOnce();
		waitKey(10);
	}
}

//����ÿһ���ӳɳͷ�����λ��
//��Ҫע�����������Ȼ���ص���cv::Point
//������ֻ����Ϊ�˴������õ���������
//������������opencv Mat����ϵ�µ����ꡣʵ����Ȼ�������Լ��ĵ�ͼ����ϵ�µ�����
cv::Point get_zone_locate(int num)
{
	if (num == 1)
		return cv::Point(144, 367);
	if (num == 2)
		return cv::Point(80, 322);
	if (num == 4)
		return cv::Point(24, 204);
	if (num == 3)
		return cv::Point(201, 202);
	if (num == 5)
		return cv::Point(146, 85);
	if (num == 6)
		return cv::Point(84, 40);
}

void add_cant_go(cv::Point center,int aa)
{
	int x = center.x;
	int y = center.y;
	
	//��һ����
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 1;

	//�ڶ�����
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 1;

	//��������
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 1;

	//��������
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 1;

	//���ö�̬�������ɹ�
	return;
}

void delete_cant_go(cv::Point center, int aa)
{
	int x = center.x;
	int y = center.y;
	
	//��һ����
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 0;

	//�ڶ�����
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 0;

	//��������
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 0;

	//��������
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 0;

	//���ö�̬�������ɹ�
	return;
}

void update_race_state_callback(const move_control :: race_state :: ConstPtr& msg_recv)
{
	is_race_start = msg_recv -> robot_id;
	robot1.HP = msg_recv->self_1_HP_left;
	robot1.MP = msg_recv->self_1_bullet_left;
	printf("\nnow HP = %d   MP =   %d   race_state = %d\n, bullet_buff_state = %d", robot1.HP, robot1.MP, is_race_start, bullet_buff_state);
	for (int i=0;i<=5;i++)
	{
		zone[i] = msg_recv->zone[i];
		zone_status[i] = msg_recv->zone_status[i];
		//red 2 blue 1
		//�м��ӵ���buff��ֱ�ӳ�
		if (msg_recv->myteam == 2 && zone[i] == 2 && bullet_buff_state == 1 ){
			if (need_replan_path == 0){
				goalx = get_zone_locate(i+1).x;
				goaly = get_zone_locate(i+1).y;
				//printf("keep\n");
				keep_xy_safe(goalx, goaly);
				//printf("keep\n");
				need_replan_path = 1;
				bullet_buff_state = 0;
				printf("ready to go to area %d\n", i);
			}
		}

		if (msg_recv->myteam == 1 && zone[i] == 4 && bullet_buff_state == 1)
		{
			if (need_replan_path == 0){
				goalx = get_zone_locate(i+1).x;
				goaly = get_zone_locate(i+1).y;
				//printf("keep\n");
				keep_xy_safe(goalx, goaly);
				//printf("keep\n");
				need_replan_path = 1;
				bullet_buff_state = 0;
				printf("ready to go to area %d\n", i);
			}
		}

		//�м�Ѫ��buff���ȵ�Ѫ���ٳ�
		if (msg_recv->myteam == 2 && zone[i] == 1){
			if (2000 - robot1.HP >= 100){
				goalx = get_zone_locate(i+1).x;
				goaly = get_zone_locate(i+1).y;
				//printf("keep\n");
				keep_xy_safe(goalx, goaly);
				//printf("keep\n");
				need_replan_path = 1;
				printf("ready to go to area %d\n", i);
			}
		}

		if (msg_recv->myteam == 1 && zone[i] == 3){
			if (abs(2000 - robot1.HP) >= 100){
				goalx = get_zone_locate(i+1).x;
				goaly = get_zone_locate(i+1).y;
				//printf("keep\n");
				keep_xy_safe(goalx, goaly);
				//printf("keep\n");
				need_replan_path = 1;
				printf("ready to go to area %d\n", i);
			}
		}

		//�ͷ������a����
		if (zone[i] == 5){
			//add_cant_go(get_zone_locate(i), 30);
		}

		//�ͷ������a����
		if (zone[i] == 6){
			//add_cant_go(get_zone_locate(i), 30);
		}
	}
	return;
}


void update_position_callback (const geometry_msgs::Point::ConstPtr& msg_recv)
{
	robot1.x = (msg_recv->x) * 1000 / 20;
	robot1.y = (msg_recv->y) * 1000 / 20;

	// if (my_dist(robot1.x, robot1.y, 144, 367) <= 25 && is_race_start == 1 && zone) {have_i_pass_zone[0] = 1; printf("set 0 zone passed!!"); }
	// if (my_dist(robot1.x, robot1.y, 80, 322) <= 25 && is_race_start == 1) {have_i_pass_zone[1] = 1; printf("set 1 zone passed!!");}
	// if (my_dist(robot1.x, robot1.y, 24, 204) <= 25 && is_race_start == 1) {have_i_pass_zone[2] = 1; printf("set 2 zone passed!!");}
	// if (my_dist(robot1.x, robot1.y, 201, 202) <= 25 && is_race_start == 1) {have_i_pass_zone[3] = 1; printf("set 3 zone passed!!");}
	// if (my_dist(robot1.x, robot1.y, 146, 85) <= 25 && is_race_start == 1) {have_i_pass_zone[4] = 1; printf("set 4 zone passed!!");}
	// if (my_dist(robot1.x, robot1.y, 84, 40) <= 25 && is_race_start == 1) {have_i_pass_zone[5] = 1; printf("set 5 zone passed!!");}
	//printf("recv = (%f, %f)  now robotx = %d, now roboty = %d\n", msg_recv->x, msg_recv->y, robot1.x, robot1.y);
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
	all_wall_line[15] = wall_line(117, 329, 177, 289);
	all_wall_line[16] = wall_line(177, 289, 107, 289);
	//rectangle(gggmap, Rect(Point(289, 107), Point(329, 117)), Scalar(255));
	all_wall_line[17] = wall_line(118, 202, 112, 195);
	all_wall_line[18] = wall_line(112, 195, 105, 202);
	all_wall_line[19] = wall_line(105, 202, 112, 208);
	all_wall_line[20] = wall_line(112, 208, 118, 202);
	//imshow("1123123", gggmap);
	//waitKey(1000 );
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

void mouse_handle(int event, int x, int y, int flags, void* param)
{
	//ע��cv::Mat����ϵ�����ǵĵ�ͼ����ϵxy���Ƿ�������
	switch (event)
	{
	//������������յ�
	case EVENT_LBUTTONDOWN:
		
		goalx = y;//y
		goaly = x;//x
		keep_xy_safe(goalx, goaly);
		cout<<"need replan path "<< x << "   "<<y<<endl;
		need_replan_path = 1;
		break;
	//�Ҽ�����������������
	case EVENT_RBUTTONDOWN:
		enemy1.x = y;
		enemy1.y = x;
		break;
	}
}

//��֤(x,y)����ǽ���棬����·���滮ը��
//��Ϊ�����ã�ֱ�Ӵ��ξͿ�����
void keep_xy_safe(int &x, int &y)
{
	//printf("%d %d \n", x, y);
	if (a[x][y] == -1){
		int my_mmin = 99999, mmin_x,mmin_y;
		////for (int i = 0; i < img_erode_save.rows; i++)
		//	for (int j = 0; j < img_erode_save.cols; j++)

		for (int i=0;i<img_erode_save.rows;i++)
			for (int j=0;j<img_erode_save.cols;j++)
				if (((i-x) * (i-x) + (j-y) * (j-y) < my_mmin) && (a[i][j] == 0)){
					my_mmin = (i-x) * (i-x) + (j-y)*(j-y);
					mmin_x = i;
					mmin_y = j;
				}
		x = mmin_x;
		y = mmin_y;
	}
	else
		return;
}