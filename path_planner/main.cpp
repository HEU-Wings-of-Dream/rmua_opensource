#include <iostream>
#include <fstream>
#include <queue>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

Mat gggmap(240, 420, CV_8UC1, Scalar(0));
int map_x = 228, map_y = 408, memory_pool_i = 0, memory_pool_optimizer_i = 0;
int goalx, goaly, min_dist_goal_x, min_dist_goal_y;
int a[228][408]; //地图
double enemy_fire_cost[228][408];//敌方火力代价地图
bool flag[228][408], flag_optimizer[228][408];//A*标记数组
int corner_array[93024]; int number_of_corner = 0;//这里面放的是拐点在path_array里面的下标，方便同时调用，两个都是从1开始存的
cv::Mat img, img_plain_save, img_erode_save, simulation_map, path_map(226,406,CV_8UC1,Scalar(255)), mask, img_final_path;//定义地图啥的
bool need_replan_path = 1, first_in_simulation_function = 0;
bool delete_path_flag = 0;//为1时代表当前路径规划信息已经使用完毕，可以归还系统空间
std::vector<cv::Point>corner_vector;
//时间参数变量
std::chrono::steady_clock::time_point program_start_time;
double total_start_time, last_simulation_time;

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

class self_robot
{
//现令：恒定各项vmax为1.8(m/s), vrmax = 1.5π(rad/s)
//官车的性能非常好，不妨认为速度都是恒定的，没有加速时间、减速时间，以此简化计算
//这样，90度的转弯半径也只有0.18m，而且我觉得行进速度挺合适
public:
	double x;
	double y;
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
	int next_corner; //下一个拐角
	int next_state_change_node; //下一个接敌状态改变点
	navigation_point( ){ }
	navigation_point(int xx, int yy)
	{
		this->x = xx;
		this->y = yy;
	}
	navigation_point(int xx, int yy, int _next_corner)
	{
		this->x = xx;
		this->y = yy;
		this->next_corner = _next_corner;
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
	img_plain_save = cv::imread("map2.png", -1);
	//腐蚀操作取结构元所指定的领域内值的最小值作为该位置的输出灰度值
	Mat structureElement = getStructuringElement(MORPH_RECT, Size(25, 25), Point(-1, -1));
	//结构元到底是多大有待讨论，目前是25 * 2cm = 50cm，约等于底盘半径
	erode(img_plain_save, img_erode_save, structureElement);
	cv::imwrite("erode_map.jpg",img_erode_save);
	//imshow("222", img_erode_save);
	for (int i = 0; i < img_erode_save.rows; i++)
	{
		for (int j = 0; j < img_erode_save.cols; j++)
		{
			if (img_erode_save.at<uchar>(i, j) == 255) {
				a[i][j] = 0; continue;
			}
			else a[i][j] = -1;
		}
	}
	cout << "read map complete" << endl << endl;
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
	return  dis + 1000 * enemy_fire_cost[x][y];
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
//		cout << step << endl;
//		return;
//	}
//	//cout << '=';
//	my_point father_point = *(memory_pool[now.father]);
//	path_recall(father_point, step + 1);
//	//cout << "Callback " << step << " : " << now.x << ' ' << now.y << endl;
//}

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
	//			//cout << "Error: " << (vec1y / vec1x) << ' ' << (vec2y / vec2x) << ' '<<endl;
	//			//cout << path_array[i]->x << ' ' << path_array[i]->y << ' ' << path_array[i - 1]->x << ' ' << path_array[i - 1]->y << endl;
	//			//cout << path_array[i]->x << ' ' << path_array[i]->y << ' ' << path_array[i + 1]->x << ' ' << path_array[i + 1]->y << endl;
	//			number_of_corner++;
	//			corner_array[number_of_corner] = i;
	//		}
	//	}	
	//}
	//for (int i = 1; i <= number_of_corner; i++) {
	//	//cout << path_array[corner_array[i]]->x << ' ' << path_array[corner_array[i]]->y << endl;
	//	circle(img, cv::Point(path_array[corner_array[i]]->y, path_array[corner_array[i]]->x), 2, Scalar(0), -1);
	//}
	Mat temp_map = path_map.clone();
	cv::goodFeaturesToTrack(path_map, corner_vector, 100, 0.04, 8, mask, 5, true, 0.04);
	
	for (const auto& corner : corner_vector)
	{
		circle(path_map, cv::Point(corner.x, corner.y), 2, Scalar(0), -1);
	}
	imshow("15435", path_map);
	imshow("093039", temp_map);
	waitKey(1);
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

void draw_path()
{
	img_final_path = img_plain_save.clone();
	for (int i = 1; i <= final_path_i; i++)
	{
		img_final_path.at<uchar>(final_path[i].x, final_path[i].y) = 0;
	}
}

void path_optimizer()
{
	cout << "Running path_optimizer..." << endl;

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
			//cout << "need :  " << path_array[i]->y << ' ' << path_array[i]->x << endl;
			circle(img_final_path, cv::Point(path_array[i]->y, path_array[i]->x), 2, Scalar(0), -1);
		}

		last_state = now_state;
	}
	//imshow("123", img);
	//waitKey(1);
	path_optimizer_array_size++;
	path_optimizer_array[path_optimizer_array_size] = path_array[path_long];//存储最后一个点

	cout << "Path_optimizer: " << "Plan to optimize " << path_optimizer_array_size << " part path. "<<endl;
	//对于每两个接敌点，进行第二次A*最短路优化，以去除拐点
	for (int i = 1; i <= path_optimizer_array_size - 1; i++)
	{
		//cout << "Optimize " << i << " to " << i + 1 << endl;
		bool end_flag = 0, now_meet_state = is_met(enemy1, path_optimizer_array[i]->x, path_optimizer_array[i]->y);
		//cout << now_meet_state << endl;
		min_dist_goal_x = path_optimizer_array[i + 1]->x;
		min_dist_goal_y = path_optimizer_array[i + 1]->y;
		memory_pool_optimizer[memory_pool_optimizer_i] = new my_point(0, memory_pool_optimizer_i, path_optimizer_array[i]->x, path_optimizer_array[i]->y);
		Q.push(memory_pool_optimizer[memory_pool_optimizer_i]);
		//cout << Q.size() << endl;
		while (end_flag == 0)
		{
			my_point* now = Q.top();
			Q.pop();
			flag_optimizer[now->x][now->y] = 1;
			//cout << now->x << ' ' << now->y << ' ' << now_meet_state << ' '<< path_optimizer_array[i + 1]->x<<' '<< path_optimizer_array[i + 1]->y<< ' ' << is_met(enemy1, now->x, now->y)<<endl;
			//cout << is_met(enemy1, now->x + 1, now->y) << ' ' << is_met(enemy1, now->x, now->y + 1) << ' ' << is_met(enemy1, now->x, now->y - 1) << ' ' << is_met(enemy1, now->x - 1, now->y) << endl << endl;
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

				find_corner();

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
	//while (1) {
		//refresh_simulation_environment();
		if (need_replan_path == 1) {
			cout << "Running path_planner..." << endl;
			img = img_plain_save.clone();
			bool end_flag = 0;
			Refresh_enemy_fire_cost();//刷新火力地图
			//构造路径起点，并且令其父节点为自己，便于终止回溯
			memory_pool[memory_pool_i] = new my_point(0, memory_pool_i, robot1.x, robot1.y);
			search_queue.push(memory_pool[memory_pool_i]);
			auto t1 = std::chrono::high_resolution_clock::now();
			while (end_flag == 0)
			{
				//取队首
				my_point* now = search_queue.top();
				search_queue.pop();
				flag[now->x][now->y] = 1;

				//此段代码用于打印当前处于哪一个节点
				//cout << '(' << now->x << ',' << now->y << ")    h(x,y) = " << h(now->x, now->y) << endl;
				//此段代码用于打印当前搜索队列中的所有节点，不过现阶段显然不需要了，只在编写初期用到过
				//就是把search_queue复制了一遍然后全部取出来看一遍。优先队列不能够使用迭代器遍历
				//因为没有提供object.begin(),object.end()成员函数
				//if (search_queue.size()!=0)test = search_queue;
				//while (test.size() != 0)
				//{
				//	my_point* now2 = test.top();
				//	cout << '(' << now2->x << ',' << now2->y << ")    h(x,y) = " << h(now2->x, now2->y)  << endl;
				//	test.pop();
				//}
				//cout << endl << endl;

				//搜索终止
				if ((now->x == goalx) && (now->y == goaly))
				{
					cout << "search complete" << endl;
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
					cout << "Use point = " << memory_pool_i << endl;
					cout << "Queue size = " << search_queue.size() << endl;
					cout << "Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
					cout << "Total step = " << now_step << endl;
					cout << endl;

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
						cout << path_array[i]->x << ' ' << path_array[i]->y << endl;*/
					/*for (int i = 0; i < img.rows; i++) {
						for (int j = 0; j < img.cols; j++)
							cout << enemy_fire_cost[i][j] << ' ';
						cout << endl;
					}*/
					//cout << endl << " is_met :   " << is_met(enemy1, 187, 39) << endl;
					//cout << endl << "is cross:   " << is_cross(enemy1.x, enemy1.y, 187, 39, 186, 37, 190, 37);
					circle(img, cv::Point(enemy1.y, enemy1.x), 2, Scalar(0), -1);
					//imwrite("path_out.png", img);
					//imshow("path_img", path_map);
					robot1.x = goalx;
					robot1.y = goaly;
					//重新载入地图
					//img = imread("map2.png", -1);
					//simulation_map = img;
					end_flag = 1;
					need_replan_path = 0;
				}

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
			//cout << "out while";
		}
		//cout << "out if ";
	//}
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
	cout << "Cleaner Function:  Delete " << test-1 << " node" << endl;
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
	//final_path_i = 1;
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
		refresh_simulation_environment();
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

int main()
{	
	program_start_time = std::chrono::high_resolution_clock::now();//时，万物之始也

	init_load_map();//加载地图
	init_load_wall_line();//加载墙壁信息
	init_load_erode_wall_line();//加载腐蚀之后的地图信息

	namedWindow("Simulation_Environment@HEUsjh");//创建仿真环境窗口
	setMouseCallback("Simulation_Environment@HEUsjh", mouse_handle);//挂载鼠标事件回调函数

	robot1.x = 45;
	robot1.y = 45;

	need_replan_path = 0;
	img = img_plain_save.clone(); 

	/*for (int i=1;i<=img.rows;i++)
		for (int j = 1; j <= img.cols; j++)
		{
			if (is_met(enemy1, i, j) == 1)gggmap.at<uchar>(i, j) = 255;
			else gggmap.at<uchar>(i, j) = 0;
		}
	imshow("124134", gggmap);
	waitKey(10000);*/

	//进入工作
	while (1)
	{
		if (need_replan_path == 1) { 
			auto t1 = std::chrono::high_resolution_clock::now();
			path_planner(); 
			path_optimizer(); 
			delete_now_path(); //需要注意的是final_path的删除时机
			auto t2 = std::chrono::high_resolution_clock::now();
			cout << "Memory pool Clear!" << endl; 
			cout << "Total Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
		}
		//Navigation();
		draw_path();
		imshow("Simulation_Environment@HEUsjh", img);
		imshow("Simulation_Environment2@HEUsjh", img_final_path);
		waitKey(1);
		
	}
}

//注意，这里的xy坐标系是和opencv Mat坐标系是反过来的，也就是行是x，列是y
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
	//注意Mat坐标系和我们的地图坐标系xy轴是反过来的
	switch (event)
	{
	//左键单击发布终点
	case EVENT_LBUTTONDOWN:
		goalx = y;
		goaly = x;
		need_replan_path = 1;
		break;
	//右键单击发布敌人坐标
	case EVENT_RBUTTONDOWN:
		enemy1.x = y;
		enemy1.y = x;
		break;
	}
}