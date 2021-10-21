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

int map_x = 228;
int map_y = 408;
int goalx, goaly, start_x, start_y;
int a[228][408];
double enemy_fire_cost[228][408];
bool flag[228][408];
int memory_pool_i = 0;
cv::Mat img, img2, simulation_map;

class my_point
{
public:
	int x;
	int y;
	int father;
	int self;
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

class wall_line
{
public:
	int c1x;
	int c1y;
	int c2x;
	int c2y;
	wall_line(){}
	wall_line(int c1x_, int c1y_, int c2x_, int c2y_)
	{
		c1x = c1x_;
		c1y = c1y_;
		c2x = c2x_;
		c2y = c2y_;
	}
};

my_point* memory_pool[18118044];
std::vector <my_point*> path_vector;
std::vector <my_point*> corner_vector;
enemy enemy1(187, 36, 2000);
wall_line all_wall_line[21];

void init_load_wall_line();

//判断线段AB,CD是否相交,A(x1,y1),B(x2,y2),C(x3,y3),D(x4,y4)
//判断AB×AC 与AB×AD是否异号或有一方为零，若确实，则相交，相交返回True
//向量yyds (^_^)
bool is_cross(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
{
	int ab_ac = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
	int ab_ad = (x2 - x1) * (y4 - y1) - (x4 - x1) * (y2 - y1);
	if (ab_ac * ab_ad <= 0) return 1;//相交了
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

void Refresh_enemy_fire_cost()
{
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			//A = 1
			//sigma(x) = 50;
			//sigma(y) = 50;
			enemy_fire_cost[i][j] = is_met(enemy1, i, j) * (50000.0 * exp(-(((i - enemy1.x) * (i - enemy1.x)) / (2 * 50 * 50) + ((j - enemy1.y) * (j - enemy1.y)) / (2 * 50 * 50))));
			//此处应为enemy2，待填充
		}
	}
}

//刷新敌方火力威胁代价地图计算函数，注意，这里是纯粹的只考虑敌方火力威胁，只与敌方位置有关
//void Situation_Analyst()
//{
//	for (int i = 0; i < img.rows; i++)
//	{
//		for (int j = 0; j < img.cols; j++)
//		{
//			//cost[i][j] = is_met(enemy1, i, j) * calculate(enemy1, i, j) + is_met(enemy2, i, j) * calculate(enemy2, i, j);
//			enemy_fire_cost[i][j] = is_met(enemy1, i, j) * enemy_fire_cost[i][j];
//		}
//	}
//}

//对任意一点的估值函数
double h(int x, int y)
{
	double dis = sqrt((x - goalx) * (x - goalx) + (y - goaly) * (y - goaly));
	return  dis + 1000 * enemy_fire_cost[x][y];
}

//自定义优先队列排序方法
struct cmp
{
	bool operator () (my_point* a, my_point* b)
	{
		return h(a->x, a->y) > h(b->x, b->y);
	}
};

priority_queue <my_point*, std::vector<my_point*>, cmp> search_queue;
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
	int count = 0;
	my_point* front_point;
	my_point* middle_point;
	my_point* next_point;
	for (const auto& it : path_vector)
	{
		count++;
		//第一个点没有父节点，最后一个点没有子节点，都不要了
		if (count == 1) { front_point = it; continue; }
		if (count == path_vector.size()) continue;
		middle_point = it;
		next_point = memory_pool[it->father];
		double vec1x = middle_point->x - front_point->x;
		double vec1y = middle_point->y - front_point->y;
		double vec2x = middle_point->x - next_point->x;
		double vec2y = middle_point->y - next_point->y;
		//斜率是否相等，不相等那肯定是拐点了
		//向量yyds!!
		//if ((vec1y / vec1x) != (vec2y / vec2x))
			//Bezier_curve_optimizer(front_point->x, front_point->y, middle_point->x, middle_point->y, next_point->x, next_point->y);
	}
}

//使用贝塞尔曲线优化路径的函数
void Bezier_curve_optimizer(int x1, int y1, int x2, int y2, int x3, int y3)
{

}

void simulation_environment()
{
	imshow("Simulation_Environment@HEUsjh", simulation_map);
	waitKey(1);
}

//仿真环境鼠标时间回调函数
void mouse_handle(int event, int x, int y, int flags, void* param);

int main()
{
	//build_game_map();
	//加载地图
	img = cv::imread("map2.png", -1);

	auto program_start_time = std::chrono::high_resolution_clock::now();
	//初始化仿真环境
	namedWindow("Simulation_Environment@HEUsjh");
	setMouseCallback("Simulation_Environment@HEUsjh", mouse_handle);
	simulation_map = img;
	init_load_wall_line();
	//腐蚀操作取结构元所指定的领域内值的最小值作为该位置的输出灰度值
	Mat structureElement = getStructuringElement(MORPH_RECT, Size(25, 25), Point(-1, -1));
	//结构元到底是多大有待讨论，目前是25 * 2cm = 50cm，约等于底盘半径
	erode(img, img2, structureElement);
	imshow("222", img2);
	for (int i = 0; i < img2.rows; i++)
	{
		for (int j = 0; j < img2.cols; j++)
		{
			if (img2.at<uchar>(i, j) == 255) {
				a[i][j] = 0; continue;
			}
			else a[i][j] = -1;
		}
	} 
	cout << "read map complete" << endl;
	/*初期自行输入代价地图时候的产物
	for (int i = 0; i <= map_x-1; i++)
		for (int j = 0; j <= map_y-1; j++)
		{
			cin >> cost[i][j];
		}
	*/
	start_x = 45;
	start_y = 45;
	goalx = (4480 - 400)/20;
	goaly = (8080 - 400)/20;
	bool end_flag = 0;
	Refresh_enemy_fire_cost();//刷新火力地图
	//构造路径起点，并且令其父节点为自己，便于终止回溯
	memory_pool[memory_pool_i] = new my_point(0, memory_pool_i, start_x, start_y);
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
			
			my_point now_point = *now;
			int now_step = 0;
			while (now_point.father != now_point.self)
			{
				img.at<uchar>(now_point.x, now_point.y) = 0;
				my_point father_point = *(memory_pool[now_point.father]);
				now_point = father_point;
				path_vector.push_back(memory_pool[now_point.self]);
				now_step++;
			}
			//find_corner();

			//调用回溯函数打印路径
			//path_recall(*now, 1);
			//删除所有点
			/*while (search_queue.empty() == 0)
			{
				delete(search_queue.top());
				search_queue.pop();
			}*/
			cout << "Use point = " << memory_pool_i << endl;
			cout << "Queue size = " << search_queue.size() << endl;
			while (memory_pool_i != -1)
			{
				delete(memory_pool[memory_pool_i]);
				memory_pool_i = memory_pool_i - 1;
			}
			cout << "Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms";
			imwrite("path_out.png", img);
			
			cout << endl << "Total step = " << now_step << endl;
			
			
			/*for (int i = 0; i < img.rows; i++) {
				for (int j = 0; j < img.cols; j++)
					cout << enemy_fire_cost[i][j] << ' ';
				cout << endl;
			}*/
			//cout << endl << " is_met :   " << is_met(enemy1, 187, 39) << endl;
			//cout << endl << "is cross:   " << is_cross(enemy1.x, enemy1.y, 187, 39, 186, 37, 190, 37);
			circle(img, cv::Point(enemy1.y, enemy1.x), 2, Scalar(0), -1);
			//circle(img, cv::Point(39, 187), 2, Scalar(0), -1);
			imshow("111", img);
			waitKey(10000000);
			break;
		}

		//向下搜索，三个条件分别是：没有越过地图边界，没有走过，可以走上去（不是墙）
		//其实越界这个可以删掉的，因为我画地图的时候就画了边界
		//不过加了更放心（心理作用罢了）
		//x+1  下
		if (now->x + 1 <= map_x - 1 && (flag[now->x + 1][now->y] == 0) && (a[now->x + 1][now->y] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x + 1][now->y] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		//y+1  右
		if (now->y + 1 <= map_y - 1 && (flag[now->x][now->y + 1] == 0) && (a[now->x][now->y + 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x][now->y + 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y + 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		//y-1 左
		if (now->y - 1 >= 0 && (flag[now->x][now->y - 1] == 0) && (a[now->x][now->y - 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x][now->y - 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y - 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		//x-1  上
		if (now->x - 1 >= 0 && (flag[now->x - 1][now->y] == 0) && (a[now->x - 1][now->y] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x - 1][now->y] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		//左上
		if ((now->x - 1 >= 0) && (now->y - 1 >= 0) && (flag[now->x - 1][now->y - 1] == 0) && (a[now->x - 1][now->y - 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x - 1][now->y - 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y - 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		//
		if ((now->x - 1 >= 0) && (now->y + 1 <= map_y - 1) && (flag[now->x - 1][now->y + 1] == 0) && (a[now->x - 1][now->y + 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x - 1][now->y + 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y + 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		//
		if ((now->x + 1 <= map_x - 1) && (now->y - 1 >= 0) && (flag[now->x + 1][now->y - 1] == 0) && (a[now->x + 1][now->y - 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x + 1][now->y - 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y - 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		//
		if ((now->x + 1 <= map_x - 1) && (now->y + 1 <= map_y - 1) && (flag[now->x + 1][now->y + 1] == 0) && (a[now->x + 1][now->y + 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x + 1][now->y + 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y + 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}

		//cout << search_queue.size() << endl;
	}
}

void init_load_wall_line()
{
	//上围挡
	all_wall_line[1] = wall_line(46, 177, 46, 227);
	all_wall_line[2] = wall_line(46, 227, 56, 227);
	all_wall_line[3] = wall_line(56, 227, 56, 177);
	all_wall_line[4] = wall_line(56, 177, 46, 177);
	//左围挡
	all_wall_line[5] = wall_line(107, 75, 107, 115);
	all_wall_line[6] = wall_line(107, 115, 117, 115);
	all_wall_line[7] = wall_line(117, 115, 117, 75);
	all_wall_line[8] = wall_line(117, 75, 107, 75);

	all_wall_line[9] = wall_line(167, 177, 167, 227);
	all_wall_line[10] = wall_line(167, 227, 177, 227);
	all_wall_line[11] = wall_line(177, 227, 177, 177);
	all_wall_line[12] = wall_line(177, 177, 167, 177);

	all_wall_line[13] = wall_line(107, 289, 107, 329);
	all_wall_line[14] = wall_line(107, 329, 177, 329);
	all_wall_line[15] = wall_line(177, 329, 177, 289);
	all_wall_line[16] = wall_line(177, 289, 107, 289);

	all_wall_line[17] = wall_line(118, 202, 112, 195);
	all_wall_line[18] = wall_line(122, 195, 105, 202);
	all_wall_line[19] = wall_line(105, 202, 112, 208);
	all_wall_line[20] = wall_line(112, 208, 118, 202);
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
		break;
	//右键单击发布敌人坐标
	case EVENT_RBUTTONDOWN:
		enemy1.x = y;
		enemy1.y = x;
		break;
	}
}