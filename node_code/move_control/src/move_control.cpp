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

//导航目标点定义、优化器目标点定义
int goalx, goaly, min_dist_goal_x, min_dist_goal_y;

int a[228][408]; //地图
double enemy_fire_cost[228][408];//敌方火力代价地图
bool flag[228][408], flag_optimizer[228][408];//A*标记数组
int corner_array[93024]; int number_of_corner = 0;//这里面放的是拐点在path_array里面的下标，方便同时调用，两个都是从1开始存的
cv::Mat img, img_plain_save, img_erode_save, simulation_map, path_map(226,406,CV_8UC1,Scalar(255)), mask, img_final_path;//定义地图啥的

int zone[6]{}, zone_status[6]{};
bool have_i_pass_zone[6]{};

bool is_race_start = 0;
bool bullet_buff_state = 0;
//全局通信标记定义
bool need_replan_path = 1, first_in_simulation_function = 0, stop_navigation_flag = 0;

//导航所需全局vector定义
std::vector<cv::Point2f>corner_vector;
std::vector<int>angle_vector;

//时间参数变量
auto program_start_time = std::chrono::high_resolution_clock::now();
auto last_receive_time = std::chrono::high_resolution_clock::now();//上一次收到速度信息的时间
bool frist_receive_flag = 0;

double total_start_time, last_simulation_time;

//路径规划计算专用类
class my_point
{
public:
	int x;
	int y;
	int father;//父节点在内存池中的下标
	int self;//自己在内存池中的下标
	my_point(int fa = 0, int selff = 0, int xx = 0, int yy = 0) {
		x = xx;
		y = yy;
		father = fa;
		self = selff;
	}
	//本来想使用类重载运算符的方法自定义优先队列顺序，结果怎么调都不行，自闭了都快，没办法，还是换成了cmp函数
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
		//注意！！！！！！！自己调试的时候可以设置成1，实际使用的时候一定默认值是0！！不然就完蛋了！！
		can_use_its_position = 1;
		x = xx;
		y = yy;
		HP = HPP;
	}
};

//控制帧结构体
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
//现令：恒定各项vmax为1.8(m/s), vrmax = 1.5π(rad/s)
//官车的性能非常好，不妨认为速度都是恒定的，没有加速时间、减速时间，以此简化计算
//这样，90度的转弯半径也只有0.18m，而且我觉得行进速度挺合适
public:
	int x;
	int y;
	double angle;//说明：当前位姿是底盘正方向射线与水平线夹角
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

//其实就是line
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

//最终形成的路径，统编为此格式而不用my_point，以其囊括更多导航信息，去除了不必要的father、self域
class navigation_point
{
public:
	int x;
	int y;
	int should_angle;
	int nextcorner;
	double next_corner_dis; //距下一个拐角的距离
	int next_state_change_node; //下一个接敌状态改变点
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

navigation_point final_path[93024]; int final_path_i = 1;//final_path从下标1开始放的，所以起始点是final_path[1]

//path_planner
my_point* memory_pool[93024];
my_point* path_array[93024]; int path_long;//path_array从下标1开始放的，所以起始点是path_array[1]

//path_optimizer
my_point* memory_pool_optimizer[93024];
my_point* short_path_array[65536];

//others
enemy enemy1(187, 36, 2000);
self_robot robot1(0, 0, 2000, 240, 0, 0, 0, 0);
wall_line all_wall_line[21], erode_wall_line[25];

//初始化加载墙体的函数
void init_load_wall_line();
//加载腐蚀后墙体函数
void init_load_erode_wall_line();
//仿真环境鼠标时间回调函数
void mouse_handle(int event, int x, int y, int flags, void* param);
void keep_xy_safe(int& x, int& y);
//这两个函数本来可以复用的，但是我怕后来人弄不明白怎么复用
//还是弄成两个函数，用名字区别吧
void add_cant_go(cv::Point, int);
void delete_cant_go(cv::Point, int);


//算距离，这你要是看不出来那你是有点憨
double my_dist(int x1, int y1, int x2, int y2)
{
	return std::sqrt((double(x1 - x2) * double(x1 - x2) + double(y1 - y2) * double(y1 - y2)));
}

//判断线段AB,CD是否相交,A(x1,y1),B(x2,y2),C(x3,y3),D(x4,y4)
//判断AB×AC 与AB×AD是否异号或有一方为零，若确实，则相交，相交返回True
//向量yyds (^_^)
bool is_cross(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
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
double is_met(enemy e, int nowx, int nowy)
{
	int flag = 0;
	for (int i = 1; i <= 20; i++)
		if (is_cross(e.x, e.y, nowx, nowy, all_wall_line[i].c1x, all_wall_line[i].c1y, all_wall_line[i].c2x, all_wall_line[i].c2y) == 1)
			return 0;
	return 1;
}

//判断(x1,y1)-(x2,y2)这条线段是否经过了腐蚀之后的墙壁
//与墙壁重合或者不相交认为可以通过，返回1
//与墙壁相交认为不可以通过，返回0
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
	//腐蚀操作取结构元所指定的领域内值的最小值作为该位置的输出灰度值
    cv::Mat structureElement = getStructuringElement(MORPH_RECT, Size(40, 40), Point(-1, -1));
	//结构元到底是多大有待讨论，目前是25 * 2cm = 50cm，约等于底盘半径
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
			//此处应为enemy2，待填充
		}
	}
}

//对任意一点的估值函数
double h(int x, int y)
{
	double dis = sqrt((x - goalx) * (x - goalx) + (y - goaly) * (y - goaly));
        return dis;
        //return  dis + 1000 * enemy_fire_cost[x][y];
}

//找最短路用的估值函数
double min_dist_h(int x, int y)
{
	double dis = sqrt((x - min_dist_goal_x) * (x - min_dist_goal_x) + (y - min_dist_goal_y) * (y - min_dist_goal_y));
	return  dis;
}

//自定义优先队列排序方法
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
//定义用来调试的队列
//priority_queue <my_point*, std::vector<my_point*>, cmp> test;

//供编写初期使用txt文档测试，现在显然不需要了
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

//一开始使用回溯的方法遍历路径链表，不过发现有可能爆栈，还是算了吧，优化成while了
//回溯写的方便，思维难度低，可惜会爆栈
//void path_recall(my_point now, int step)
//{
//	//递归终止条件
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

//声明最小二乘函数
double get_k(std::vector<cv::Point>);

void draw_path()
{
	img_final_path = img_plain_save.clone();
	for (int i = 1; i <= final_path_i; i++)
	{
		img_final_path.at<uchar>(final_path[i].x, final_path[i].y) = 0;
	}
}

//最小二乘法拟合出来的角度是有误差的，不妨把它模糊一下，有误差也没有关系，反正先写一版开环控制的
//闭环的话等视觉标签、哨岗出来了再说
int correct_angle(double angle)
{
	/*
	|
        O——————>y (0)
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
	//暂定顺时针为正
	//与90度有一定的误差，可以容忍，认为是90°

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

//找拐点的函数
void find_corner()
{
	//原本以为这么写的算法天衣无缝，结果斜向扩展在离散化的时候就会出现锯齿型的路径
	//然后这个算法就检测到一条斜线上面全是拐点！！实在是意料之外！
	//最后王洪玺想到了利用opencv自带的角点检测算法，实在是太高了！
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
	//	//斜率是否相等，不相等那肯定是拐点了
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
			//不会吧，不会有人对vector熟悉到看到这段代码一点疑惑都没有吧？不会有人还不去查资料吧？
			while (it != corner_vector_temp.end())
            {
				if (my_dist(final_path[i].x, final_path[i].y, (*it).y, (*it).x) <= 2)
				{
                    corner_vector.push_back(*it);
					//妈的，就是这个鸟蛋地方卡了我半天，重修STL了属于是。想当年恩师必然是和我说了的，可惜我忘了来着
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
	//上一个角点在final_path中的位置
	int last_corner_locate = 0;
	for (int i = 1; i <= final_path_i; i++)
	{
            //这里极其坑人！！需要特别特别注意！！由于我们的坐标系是竖着是x轴，但是opencv API goodFeaturesToTrack()
            //返回的是opencv格式的cv::Point，其坐标轴是由横向为x轴编的，这里真的坑人！！！
            //如果都是自己的数据结构去做运算，那就没有这样的问题，但是这里用到了cv的API就必须注意这样的问题

            //这里其实是有一个小注意：c++在计算if的条件表达式的时候，如果你这个式子中只有&&，形如下面这个if
            //那么他会先计算第一个条件，如果为false，那么就不会向下计算了，例如这里，i==1只有一次，但是如果把my_dist这个条件放在前面
            //每次都会调用一遍，非常费时间，这个顺序是可以避免调用my_dist的。当然，不同的编译器的优化方式不一样
            //如果你的测试结果不是这样的，那只能说，编译器的问题，这个说法绝对是存在的。

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
		
		//如果当前就是下一个角点，应该送进去最小二乘拟合，然后清理当前short_path数组
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

	//以下皆为善后工作，因为角点检测当最后一个拐角离终点很近的时候，终点是检测不出来的，我们只能去手动补全最后一段路的路径信息

	//先补全角度控制信息

	//找到最后的角度
	int j = final_path_i-1;
	while ((abs(final_path[j].should_angle) > 360))
	{
		j--;
	}
	//令其保持之
	for (int k = j + 1; k <= final_path_i; k++)
		final_path[k].should_angle = final_path[j].should_angle;

	//补全终点控制位置信息
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

//使用贝塞尔曲线优化路径的函数
//void Bezier_curve_optimizer(int x1, int y1, int x2, int y2, int x3, int y3){}

void refresh_simulation_environment()
{
	auto t1 = std::chrono::high_resolution_clock::now();
	if (first_in_simulation_function == 0) {//没有仿真过就不要再做了，搞一个初始时间点
		last_simulation_time = (static_cast<std::chrono::duration<double, std::milli>>(t1 - program_start_time)).count();
		return;
	}
	simulation_map = img;//刷新地图
	total_start_time = (static_cast<std::chrono::duration<double, std::milli>>(t1 - program_start_time)).count();
	double delta_time = double(total_start_time - last_simulation_time) / 1000.0;//转换成秒
	//更新坐标
	robot1.x += robot1.vx * delta_time;
	robot1.y += robot1.vy * delta_time;
	robot1.angle += robot1.angle * delta_time;
	cv::circle(simulation_map, cv::Point(int(robot1.y), int(robot1.x)), 2, Scalar(0), -1);
	imshow("Simulation_Environment@HEUsjh", simulation_map);
	last_simulation_time = total_start_time;//时间迭代
	waitKey(1);
}

void path_optimizer()
{
	std::cout << "Running path_optimizer..." << std::endl;

	bool now_state = 0, last_state = 0, first_flag = 0;
	my_point* path_optimizer_array[100]; int path_optimizer_array_size = 0;

	//寻找所有接敌状态改变的点，存入数组，准备进行优化
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
			path_optimizer_array[path_optimizer_array_size] = path_array[i];//存储状态变化的拐点
			continue;
		}

		if ((now_state != last_state))
		{
			path_optimizer_array_size++;
			path_optimizer_array[path_optimizer_array_size] = path_array[i];//存储状态变化的拐点
			//std::cout << "need :  " << path_array[i]->y << ' ' << path_array[i]->x << std::endl;
			circle(img_final_path, cv::Point(path_array[i]->y, path_array[i]->x), 2, Scalar(0), -1);
		}

		last_state = now_state;
	}
	//imshow("123", img);
	//waitKey(1);
	path_optimizer_array_size++;
	path_optimizer_array[path_optimizer_array_size] = path_array[path_long];//存储最后一个点

	std::cout << "Path_optimizer: " << "Plan to optimize " << path_optimizer_array_size << " part path. "<<std::endl;
	//对于每两个接敌点，进行第二次A*最短路优化，以去除拐点
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
				//回溯并储存路径
				my_point now_point = *now;
				path_long = 1;
				while (now_point.father != now_point.self)
				{
					//img.at<uchar>(now_point.x, now_point.y) = 0;
					//path_map.at<uchar>(now_point.x, now_point.y) = 0;
					my_point father_point = *(memory_pool_optimizer[now_point.father]);
					now_point = father_point;
					short_path_array[path_long] = memory_pool_optimizer[now_point.self]; //内存池中的指针拷贝了一份
					path_long++;
				}
				path_long--;

				//回溯储存的路径是倒过来的， 我们要把它正过来
				for (int i = 1; i <= path_long; i++)
				{
					//如果是偶数，循环到一半就行，奇数循环到中间的前一个
					if (path_long % 2 == 0) {
						if (i == (path_long) / 2 + 1) break;
					}
					else if (i == (path_long + 1) / 2) break;

					//交换，没什么好说的
					my_point* temp = short_path_array[i];
					short_path_array[i] = short_path_array[path_long - i + 1];
					short_path_array[path_long - i + 1] = temp;
				}

				//这么做有一个问题，就是每一个接敌节点会被在final_path中记录2遍，不过影响不大
				//但是优美的程序不应当出现这种瑕疵，今后当改之
				for (int i = 1; i <= path_long; i++)
				{
					path_map.at<uchar>(short_path_array[i]->x, short_path_array[i]->y) = 0;
					final_path[final_path_i] = navigation_point(short_path_array[i]->x, short_path_array[i]->y);
					final_path_i++;
				}

				//find_corner();

				//一次最短路搜索结束之后的善后清理工作
				while (Q.empty() != 1) Q.pop();
				//清空内存池
				while (memory_pool_optimizer_i != -1)
				{
					delete(memory_pool_optimizer[memory_pool_optimizer_i]);
					memory_pool_optimizer_i = memory_pool_optimizer_i - 1;
				}
				memory_pool_optimizer_i = 0;
				//清空A*标记数组，不然下次搜不了了
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
			//y+1  右(now_meet_state == 0) && 
			if (((now->x == path_optimizer_array[i + 1]->x) && (now->y + 1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x, now->y + 1) == now_meet_state)) && (now->y + 1 <= map_y - 1) && (flag_optimizer[now->x][now->y + 1] == 0) && (a[now->x][now->y + 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x][now->y + 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x, now->y + 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//y-1 左(now_meet_state == 0) && 
			if (((now->x == path_optimizer_array[i + 1]->x) && (now->y - 1 == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x, now->y - 1) == now_meet_state)) && (now->y - 1 >= 0) && (flag_optimizer[now->x][now->y - 1] == 0) && (a[now->x][now->y - 1] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x][now->y - 1] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x, now->y - 1);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//x-1  上(now_meet_state == 0) && 
			if (((now->x - 1 == path_optimizer_array[i + 1]->x) && (now->y == path_optimizer_array[i + 1]->y)) || (((is_met(enemy1, now->x - 1, now->y) == now_meet_state)) && (now->x - 1 >= 0) && (flag_optimizer[now->x - 1][now->y] == 0) && (a[now->x - 1][now->y] == 0) && (end_flag == 0)))
			{
				memory_pool_optimizer_i = memory_pool_optimizer_i + 1;
				flag_optimizer[now->x - 1][now->y] = 1;
				memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(now->self, memory_pool_optimizer_i, now->x - 1, now->y);
				Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
			}
			//左上
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
			Refresh_enemy_fire_cost();//刷新火力地图
			cv::circle(img_erode_save, cv::Point(goaly, goalx), 2, cv::Scalar(0),2);
			imshow("Simulation_Environment@HEUsjh", img_erode_save);
			waitKey(10);
			std::cout<<"now x  "<<robot1.x <<"nowy   "<<robot1.y<<std::endl;
			std::cout<<memory_pool_i<<endl;
			keep_xy_safe(robot1.x, robot1.y);
			//构造路径起点，并且令其父节点为自己，便于终止回溯
			memory_pool[memory_pool_i] = new my_point(0, memory_pool_i, robot1.x, robot1.y);
			search_queue.push(memory_pool[memory_pool_i]);
			//cv::waitKey(1000);
			auto t1 = std::chrono::high_resolution_clock::now();
			//搜索继续的必须条件:搜索没有被强制终止， 搜索队列非空， 控制台程序没有被终止
			while (end_flag == 0 && search_queue.empty() == 0 && ros::ok())
			{
				if (search_queue.empty() == 1){
					std::cout<<"Search queue is empty, please check now position or goal point. Is they are in the wall??"<<std::endl;
					break;
				}
				//取队首
				my_point* now = search_queue.top();
				std::cout<<"nowx, nowy, goalx, goaly"<<now->x<<' '<<now->y<<' ' <<goalx<<' '<<goaly<<std::endl;
				std::cout<<"queue size = " << search_queue.size()<<std::endl;
				if (search_queue.size() == -1){
					return;
				}
				//cv::waitKey(500);
				search_queue.pop();
				flag[now->x][now->y] = 1;

				//此段代码用于打印当前处于哪一个节点
				//std::cout << '(' << now->x << ',' << now->y << ")    h(x,y) = " << h(now->x, now->y) << std::endl;
				//此段代码用于打印当前搜索队列中的所有节点，不过现阶段显然不需要了，只在编写初期用到过
				//就是把search_queue复制了一遍然后全部取出来看一遍。优先队列不能够使用迭代器遍历
				//因为没有提供object.begin(),object.end()成员函数
				//if (search_queue.size()!=0)test = search_queue;
				//while (test.size() != 0)
				//{
				//	my_point* now2 = test.top();
				//	std::cout << '(' << now2->x << ',' << now2->y << ")    h(x,y) = " << h(now2->x, now2->y)  << std::endl;
				//	test.pop();
				//}
				//std::cout << std::endl << std::endl;

				//搜索终止
				if ((now->x == goalx) && (now->y == goaly))
				{
					std::cout << "search complete" << std::endl;
					end_flag = 1;
					auto t2 = std::chrono::high_resolution_clock::now();

					//回溯并储存路径
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
						path_array[path_long] = memory_pool[now_point.self]; //内存池中的指针拷贝了一份
						path_long++;
						now_step++;
					}
					path_long--;

					//打印路径调试信息
					std::cout << "Use point = " << memory_pool_i << std::endl;
					std::cout << "Queue size = " << search_queue.size() << std::endl;
					std::cout << "Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << std::endl;
					std::cout << "Total step = " << now_step << std::endl;
					std::cout << std::endl;

					//回溯储存的路径是倒过来的， 我们要把它正过来
					for (int i = 1; i <= path_long; i++)
					{
						//如果是偶数，循环到一半就行，奇数循环到中间的前一个
						if (path_long % 2 == 0) {
							if (i == (path_long) / 2 + 1) break;
						}
						else if (i == (path_long + 1) / 2) break;

						//交换，没什么好说的
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

					//重新载入地图
					//img = imread("map2.png", -1);
					//simulation_map = img;
					end_flag = 1;
					need_replan_path = 0;
				}
				
				std::cout<<now->x+1<<' ' << now->y << ' '<<flag[now->x+1][now->y]<<' '<<a[now->x+1][now->y]<<' '<<end_flag<<endl;
				std::cout<<now->x+1<<' ' << now->y << ' '<<flag[now->x+1][now->y]<<' '<<a[now->x+1][now->y]<<' '<<end_flag<<endl;
				std::cout<<now->x+1<<' ' << now->y << ' '<<flag[now->x+1][now->y]<<' '<<a[now->x+1][now->y]<<' '<<end_flag<<endl;

				//cv::waitKey(500);
				//向下搜索，三个条件分别是：没有越过地图边界，没有走过，可以走上去（不是墙）
				//其实越界这个可以删掉的，因为我画地图的时候就画了边界
				//不过加了更放心（心理作用罢了）
				//x+1  下
				if (now->x + 1 <= map_x - 1 && (flag[now->x + 1][now->y] == 0) && (a[now->x + 1][now->y] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x + 1][now->y] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//y+1  右
				if (now->y + 1 <= map_y - 1 && (flag[now->x][now->y + 1] == 0) && (a[now->x][now->y + 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x][now->y + 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y + 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//y-1 左
				if (now->y - 1 >= 0 && (flag[now->x][now->y - 1] == 0) && (a[now->x][now->y - 1] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x][now->y - 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y - 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//x-1  上
				if (now->x - 1 >= 0 && (flag[now->x - 1][now->y] == 0) && (a[now->x - 1][now->y] == 0) && (end_flag == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x - 1][now->y] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//左上
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
	//清空内存池
	while (memory_pool_i != -1)
	{
		
		delete(memory_pool[memory_pool_i]);
		memory_pool_i = memory_pool_i - 1;
		test++;
	}
	std::cout << "Cleaner Function:  Delete " << test-1 << " node" << std::endl;
	memory_pool_i = 0;
	//清空A*标记数组，不然下次搜不了了
	std::memset(flag, 0, sizeof(flag));
	//清空路径保存数组
	std::memset(path_array, 0, sizeof(path_array));
	path_long = 0;
	//清空拐点数组
	std::memset(corner_array, 0, sizeof(corner_array));
	number_of_corner = 0;
	//需要规划路径标志位置零
	need_replan_path = 0;

	path_map = cv::Mat(226, 406, CV_8UC1, Scalar(255));
	final_path_i = 1;
	memset(final_path, 0, sizeof(final_path));
	return;
}

void Position_correction()
{

}

//获取path_array[now]与下一个拐点的方向向量
//注意，一定是下一个拐点，而不是下一个路径点，不然就会出现之前找拐点时出现的同样的问题
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
	//分割路段并计算各路段斜率
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
		//准备发送控制帧
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

	//当前处于corner[now_path]-corner[now_path+1]路段上
	int now_path = 0;
	//当前是路径上面的哪一个节点，对应于final_path[now[
	int now = 0;
        ros::Rate rate(100);
	auto lasttime = std::chrono::high_resolution_clock::now();
	while (stop_navigation_flag == 0)
	{
		//auto t1 = std::chrono::high_resolution_clock::now();
                circle(img_final_path, cv::Point(robot1.y, robot1.x), 2, Scalar(0), -1);
                imshow("Simulation_Environment2@HEUsjh", img_final_path);
                waitKey(10);
		//刷新ROS订阅话题
		ros::spinOnce();
		
		//每次从路径上面取一个最近的点
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
		//转弯提前量10cm / 2cm = 5
		
		//做出速度控制决策
        if (final_path[now].next_corner_dis <= 2 && now!=1 && now!=2){
			//cout << "---------------arrive corner----" << std::endl;
			controlframe.vx = 0;
			controlframe.vy = 0;
		}
		else{
            controlframe.vx = 0.4;
			controlframe.vy = 0;
		}

        if (my_dist(goalx,goaly, robot1.x, robot1.y) <= 2){//误差＜10cm认为抵达了终点
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
		//发布速度控制信息
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
	int now_path_step = 1;//当前导航指针，指向机器人在路径上的哪一个点
	double mindis = 999999;
	int min_dis_point = 1;//暂存机器人离路径上最近点的path_array[]下标
	int last_corner = 1, next_corner = 2;//两个拐点组成的向量才是这条路径上面的方向向量
	while (true)
	{
		//refresh_simulation_environment();
		//每次在路径上找一个离机器人最近的点，当两者都距离在一定可容忍范围内的时候，继续导航
		//如果超出一定范围，进入矫正模式，发送一个速度向量指向路径上离它最近的点，待其抵达该点之后，继续导航
		//不得不说，ROS确实是有优势的，单纯凭c++想把仿真环境加进导航代码里，架构真的很难设计，因为这个架构问题犹豫了好久，但是现在又不想上ROS测
		for (int i = 1; i <= path_long; i++)
		{
			if (sqrt((robot1.x - path_array[i]->x) * (robot1.x - path_array[i]->x) + (robot1.y - path_array[i]->y) * (robot1.y - path_array[i]->y)) < mindis) {
				mindis = sqrt((robot1.x - path_array[i]->x) * (robot1.x - path_array[i]->x) + (robot1.y - path_array[i]->y) * (robot1.y - path_array[i]->y));
				min_dis_point = i;
			}
		}
		//25 * 2 = 50cm，当偏离50cm的时候认为已经偏航，那么进入位置矫正模式，速速回归路径
		//if (mindis > 25) Position_correction();
		//if (abs(robot1.angle - get_angle())
		//注意，在发布转向的时候一定要万般注意。假如在corner处判断出机器人偏航，那么它回归的时候就已经进行转向了
		//不能再无脑发布转向，否则就完蛋了

	}
}

#if USE_PID_MOVE_CONTROL

//第一个参数是用来发布的ROS通信句柄
//第二个参数是当前控制的目标点
void PID_move_control(const auto& move_control_publisher, float goalpoint_x, float goalpoint_y)
{
	//计算当前位置误差
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

	//计算vx, vy
	double control_vx, control_vy;
	control_vx = kp * (errorx  + kd * (last_errorx - errorx) / PID_dt);
	control_vy = kp * (errory  + kd * (last_errory - errory) / PID_dt);

	//限幅
	if (abs(control_vx) >= 0.2){
		if (control_vx > 0) control_vx = 0.2;
			else control_vx = -0.2;
	}
	if (abs(control_vy) >= 0.2){
		if (control_vy > 0) control_vy = 0.2;
			else control_vy = -0.2;
	}

	//准备发送控制帧
	move_control :: my_control_frame _Contraldata;
	_Contraldata.vx = control_vy * 1000.0;
	_Contraldata.vy = control_vx * 1000.0;
	_Contraldata.angle = 0;
	move_control_publisher.publish(_Contraldata);

	printf("control_v = (%1f, %1f);   goal = (%1f, %1f)  now = (%d, %d)  error = (%1f, %1f);\n",control_vx, control_vy, goalpoint_x, goalpoint_y, robot1.x, robot1.y, errorx, errory);

	ros::spinOnce();

	//更新last_error
	last_errorx = errorx;
    last_errory = errory;
}
#endif


void update_command_callback(const geometry_msgs :: Point ::ConstPtr& msg_receive)
{	
	//接收的敌人信息的处理
	if (int(msg_receive -> z) == 2020080119){
		//delete_cant_go(cv::Point(enemy1.x, enemy1.y), 20);
		enemy1.x = msg_receive -> y; enemy1.y = msg_receive -> x;
		//printf("recv enemy = (%d,   %d    %lf)\n", enemy1.x, enemy1.y, msg_receive -> z);
		return;
	}
	// //防止因为敌人的识别掉帧导致把自己给腐蚀了
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
	//初始化里程计节点
    ros::init(argc, argv, "move_control");
	//创建通信句柄
	ros::NodeHandle communication_handle_obj;
	ros::Publisher move_control_publisher = 
				communication_handle_obj.advertise <move_control :: my_control_frame> ("/robot_control",10);

    //订阅消息
   // ros::Subscriber race_state_subscriber = communication_handle_obj.subscribe("/velocity", 10, update_race_state_callback);
	ros::Subscriber now_pos_subscriber = communication_handle_obj.subscribe("/now_pos", 10, update_position_callback);

	ros::Subscriber my_command_listener = communication_handle_obj.subscribe("/TCP_command_listener", 10, update_command_callback);

	ros::Subscriber race_state_listener = communication_handle_obj.subscribe("/race_state", 10, update_race_state_callback);


	//program_start_time = std::chrono::high_resolution_clock::now();//时，万物之始也

	init_load_map();//加载地图
	init_load_wall_line();//加载墙壁信息
	init_load_erode_wall_line();//加载腐蚀之后的地图信息

	namedWindow("Simulation_Environment@HEUsjh");//创建仿真环境窗口
	setMouseCallback("Simulation_Environment@HEUsjh", mouse_handle);//挂载鼠标事件回调函数

    //init_robot_position(1);

	need_replan_path = 0;
	img = img_plain_save.clone(); 
	img_final_path = img_plain_save.clone();
	
	auto t_start = std::chrono::high_resolution_clock::now();
	bool first_flag = 0;
	bool zone_start_change = 0;

	//进入工作
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
			delete_now_path(); //需要注意的是final_path的删除时机
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

//返回每一个加成惩罚区的位置
//需要注意的是这里虽然返回的是cv::Point
//但是这只是我为了传参数用的数据类型
//不代表它就是opencv Mat坐标系下的坐标。实际仍然是我们自己的地图坐标系下的坐标
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
	
	//第一象限
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 1;

	//第二象限
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 1;

	//第三象限
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 1;

	//第四象限
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 1;

	//设置动态禁行区成功
	return;
}

void delete_cant_go(cv::Point center, int aa)
{
	int x = center.x;
	int y = center.y;
	
	//第一象限
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 0;

	//第二象限
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j <= y + aa / 2; j++)
			a[i][j] = 0;

	//第三象限
	for (int i = x; i >= x - aa / 2; i--)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 0;

	//第四象限
	for (int i = x; i <= x + aa / 2; i++)
		for (int j = y; j >= y - aa / 2; j--)
			a[i][j] = 0;

	//设置动态禁行区成功
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
		//有加子弹的buff就直接吃
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

		//有加血的buff，等掉血了再吃
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

		//惩罚区添加a数组
		if (zone[i] == 5){
			//add_cant_go(get_zone_locate(i), 30);
		}

		//惩罚区添加a数组
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

void mouse_handle(int event, int x, int y, int flags, void* param)
{
	//注意cv::Mat坐标系和我们的地图坐标系xy轴是反过来的
	switch (event)
	{
	//左键单击发布终点
	case EVENT_LBUTTONDOWN:
		
		goalx = y;//y
		goaly = x;//x
		keep_xy_safe(goalx, goaly);
		cout<<"need replan path "<< x << "   "<<y<<endl;
		need_replan_path = 1;
		break;
	//右键单击发布敌人坐标
	case EVENT_RBUTTONDOWN:
		enemy1.x = y;
		enemy1.y = x;
		break;
	}
}

//保证(x,y)不在墙里面，以免路径规划炸了
//因为是引用，直接传参就可以了
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