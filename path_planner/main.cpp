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

int map_x = 228, map_y = 408, memory_pool_i = 0;
int goalx, goaly, start_x, start_y;
int a[228][408]; //��ͼ
double enemy_fire_cost[228][408];//�з��������۵�ͼ
bool flag[228][408];//A*�������
int corner_array[93024]; int number_of_corner = 0;//������ŵ��ǹյ���path_array������±꣬����ͬʱ���ã��������Ǵ�1��ʼ���
cv::Mat img, img2, simulation_map;//�����ͼɶ��
bool need_replan_path = 1, first_in_simulation_function = 0;
bool delete_path_flag = 0;//Ϊ1ʱ����ǰ·���滮��Ϣ�Ѿ�ʹ����ϣ����Թ黹ϵͳ�ռ�
//ʱ���������
std::chrono::steady_clock::time_point program_start_time;
double total_start_time, last_simulation_time;

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

class self_robot
{
//����㶨����vmaxΪ1.8(m/s), vrmax = 1.5��(rad/s)
//�������ܷǳ��ã�������Ϊ�ٶȶ��Ǻ㶨�ģ�Ҳ�򻯼���
//������90�ȵ�ת��뾶Ҳֻ��0.18m�������Ҿ����н��ٶ�ͦ����
public:
	double x;
	double y;
	double now_position;//˵������ǰλ���ǵ���������������ˮƽ�߼н�
	int HP;
	int MP;    
	double vx;
	double vy;
	double vr;
	self_robot() {}
	self_robot(double xx, double yy, int HP_ = 2000, int MP_ = 240, double vx_ = 0, double vy_ = 0, double vr_ = 0, double now_position_ = 0){
		x = xx; y = yy; HP = HP_; MP = MP_;
		vx = vx_; vy = vy_; vr = vr_; now_position = now_position_;
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

my_point* memory_pool[93024];
my_point* path_array[93024]; int path_long;
enemy enemy1(187, 36, 2000);
self_robot robot1(0, 0, 2000, 240, 0, 0, 0, 0);
wall_line all_wall_line[21];

//��ʼ������ǽ��ĺ���
void init_load_wall_line();

//�ж��߶�AB,CD�Ƿ��ཻ,A(x1,y1),B(x2,y2),C(x3,y3),D(x4,y4)
//�ж�AB��AC ��AB��AD�Ƿ���Ż���һ��Ϊ�㣬��ȷʵ�����ཻ���ཻ����True
//����yyds (^_^)
bool is_cross(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
{
	int ab_ac = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
	int ab_ad = (x2 - x1) * (y4 - y1) - (x4 - x1) * (y2 - y1);
	if (ab_ac * ab_ad <= 0) return 1;//�ཻ��
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
	return  dis + 1000 * enemy_fire_cost[x][y];
}

//�Զ������ȶ������򷽷�
struct cmp
{
	bool operator () (my_point* a, my_point* b)
	{
		return h(a->x, a->y) > h(b->x, b->y);
	}
};

priority_queue <my_point*, std::vector<my_point*>, cmp> search_queue;
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
//		cout << step << endl;
//		return;
//	}
//	//cout << '=';
//	my_point father_point = *(memory_pool[now.father]);
//	path_recall(father_point, step + 1);
//	//cout << "Callback " << step << " : " << now.x << ' ' << now.y << endl;
//}

//�ҹյ�ĺ���
void find_corner()
{
	int count = 0;
	my_point* front_point;
	my_point* middle_point;
	my_point* next_point;
	for (int i = 2; i <= path_long - 1; i++)
	{
		double vec1x = path_array[i]->x - path_array[i - 1]->x;
		double vec1y = path_array[i]->y - path_array[i - 1]->y;
		double vec2x = path_array[i]->x - path_array[i + 1]->x;
		double vec2y = path_array[i]->y - path_array[i + 1]->y;
		//б���Ƿ���ȣ�������ǿ϶��ǹյ���
		if ((vec1x == 0) && (vec2x == 0)) continue;
		else
		{
			if ((vec1y / vec1x) != (vec2y / vec2x))
			{
				cout << "Error: " << (vec1y / vec1x) << ' ' << (vec2y / vec2x) << ' '<<endl;
				cout << path_array[i]->x << ' ' << path_array[i]->y << ' ' << path_array[i - 1]->x << ' ' << path_array[i - 1]->y << endl;
				cout << path_array[i]->x << ' ' << path_array[i]->y << ' ' << path_array[i + 1]->x << ' ' << path_array[i + 1]->y << endl;
				number_of_corner++;
				corner_array[number_of_corner] = i;
			}
		}
			
	}
	for (int i = 1; i <= number_of_corner; i++) {
		//cout << path_array[corner_array[i]]->x << ' ' << path_array[corner_array[i]]->y << endl;
		circle(img, cv::Point(path_array[corner_array[i]]->y, path_array[corner_array[i]]->x), 2, Scalar(0), -1);
	}
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
	robot1.now_position += robot1.now_position * delta_time;
	cv::circle(simulation_map, cv::Point(int(robot1.y), int(robot1.x)), 2, Scalar(0), -1);
	imshow("Simulation_Environment@HEUsjh", simulation_map);
	last_simulation_time = total_start_time;//ʱ�����
	waitKey(1);
}

//���滷�����ʱ��ص�����
void mouse_handle(int event, int x, int y, int flags, void* param);

void path_planner()
{
	while (1) {
		refresh_simulation_environment();
		if (need_replan_path == 1) {
			img = imread("map2.png", -1);
			bool end_flag = 0;
			Refresh_enemy_fire_cost();//ˢ�»�����ͼ
			//����·����㣬�������丸�ڵ�Ϊ�Լ���������ֹ����
			memory_pool[memory_pool_i] = new my_point(0, memory_pool_i, robot1.x, robot1.y);
			search_queue.push(memory_pool[memory_pool_i]);
			auto t1 = std::chrono::high_resolution_clock::now();
			while (end_flag == 0)
			{
				//ȡ����
				my_point* now = search_queue.top();
				search_queue.pop();
				flag[now->x][now->y] = 1;

				//�˶δ������ڴ�ӡ��ǰ������һ���ڵ�
				//cout << '(' << now->x << ',' << now->y << ")    h(x,y) = " << h(now->x, now->y) << endl;
				//�˶δ������ڴ�ӡ��ǰ���������е����нڵ㣬�����ֽ׶���Ȼ����Ҫ�ˣ�ֻ�ڱ�д�����õ���
				//���ǰ�search_queue������һ��Ȼ��ȫ��ȡ������һ�顣���ȶ��в��ܹ�ʹ�õ���������
				//��Ϊû���ṩobject.begin(),object.end()��Ա����
				//if (search_queue.size()!=0)test = search_queue;
				//while (test.size() != 0)
				//{
				//	my_point* now2 = test.top();
				//	cout << '(' << now2->x << ',' << now2->y << ")    h(x,y) = " << h(now2->x, now2->y)  << endl;
				//	test.pop();
				//}
				//cout << endl << endl;

				//������ֹ
				if ((now->x == goalx) && (now->y == goaly))
				{
					cout << "search complete" << endl;
					end_flag = 1;
					auto t2 = std::chrono::high_resolution_clock::now();

					my_point now_point = *now;
					int now_step = 0;
					need_replan_path = 0;
					path_long = 1;
					while (now_point.father != now_point.self)
					{
						img.at<uchar>(now_point.x, now_point.y) = 0;
						my_point father_point = *(memory_pool[now_point.father]);
						now_point = father_point;
						path_array[path_long] = memory_pool[now_point.self];
						path_long++;
						now_step++;
					}
					path_long--;
					//���û��ݺ�����ӡ·��
					//path_recall(*now, 1);
					cout << "Use point = " << memory_pool_i << endl;
					cout << "Queue size = " << search_queue.size() << endl;
					cout << "Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms";
					cout << endl << "Total step = " << now_step << endl;
					cout << endl << "Path_long = " << path_long << endl;
					
					for (int i = 1; i <= path_long; i++)
					{
						if (path_long % 2 == 0) {
							if (i == (path_long) / 2 + 1) break;
						}
						else if (i == (path_long + 1) / 2) break;
						my_point* temp = path_array[i];
						path_array[i] = path_array[path_long - i + 1];
						path_array[path_long - i + 1] = temp;
					}
					find_corner();
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
					imwrite("path_out.png", img);
					imshow("111", img);
					waitKey(1);
					//���������ͼ
					//img = imread("map2.png", -1);
					//simulation_map = img;
					end_flag = 1;
					need_replan_path = 0;
					break;
				}

				//�������������������ֱ��ǣ�û��Խ����ͼ�߽磬û���߹�����������ȥ������ǽ��
				//��ʵԽ���������ɾ���ģ���Ϊ�һ���ͼ��ʱ��ͻ��˱߽�
				//�������˸����ģ��������ð��ˣ�
				//x+1  ��
				if (now->x + 1 <= map_x - 1 && (flag[now->x + 1][now->y] == 0) && (a[now->x + 1][now->y] == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x + 1][now->y] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//y+1  ��
				if (now->y + 1 <= map_y - 1 && (flag[now->x][now->y + 1] == 0) && (a[now->x][now->y + 1] == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x][now->y + 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y + 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//y-1 ��
				if (now->y - 1 >= 0 && (flag[now->x][now->y - 1] == 0) && (a[now->x][now->y - 1] == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x][now->y - 1] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y - 1);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//x-1  ��
				if (now->x - 1 >= 0 && (flag[now->x - 1][now->y] == 0) && (a[now->x - 1][now->y] == 0))
				{
					memory_pool_i = memory_pool_i + 1;
					flag[now->x - 1][now->y] = 1;
					memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y);
					search_queue.push(memory_pool[memory_pool_i]);
				}
				//����
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
	}
}

void delete_now_path()
{
	//����ڴ��
	while (memory_pool_i != -1)
	{
		delete(memory_pool[memory_pool_i]);
		memory_pool_i = memory_pool_i - 1;
	}
	//���A*������飬��Ȼ�´��Ѳ�����
	memset(flag, 0, sizeof(flag));
	//���·����������
	memset(path_array, 0, sizeof(path_array));
	path_long = 0;
	//��չյ�����
	memset(corner_array, 0, sizeof(corner_array));
	number_of_corner = 0;
	//��Ҫ�滮·����־λ����
	need_replan_path = 0;
	return;
}

void init_load_map()
{
	img = cv::imread("map2.png", -1);
	//��ʴ����ȡ�ṹԪ��ָ����������ֵ����Сֵ��Ϊ��λ�õ�����Ҷ�ֵ
	Mat structureElement = getStructuringElement(MORPH_RECT, Size(25, 25), Point(-1, -1));
	//�ṹԪ�����Ƕ���д����ۣ�Ŀǰ��25 * 2cm = 50cm��Լ���ڵ��̰뾶
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
}

void Navigation()
{
	while (true)
	{
		refresh_simulation_environment();
	}
}

int main()
{	
	program_start_time = std::chrono::high_resolution_clock::now();//ʱ������֮ʼҲ
	init_load_map();//���ص�ͼ
	init_load_wall_line();//����ǽ����Ϣ
	namedWindow("Simulation_Environment@HEUsjh");//�������滷������
	setMouseCallback("Simulation_Environment@HEUsjh", mouse_handle);//��������¼��ص�����

	robot1.x = 45;
	robot1.y = 45;
	goalx = (4480 - 400)/20;
	goaly = (8080 - 400)/20;

	//���빤��
	while (1)
	{
		if (need_replan_path == 1) path_planner();
		Navigation();
	}
}

void init_load_wall_line()
{
	//��Χ��
	all_wall_line[1] = wall_line(46, 177, 46, 227);
	all_wall_line[2] = wall_line(46, 227, 56, 227);
	all_wall_line[3] = wall_line(56, 227, 56, 177);
	all_wall_line[4] = wall_line(56, 177, 46, 177);
	//��Χ��
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
	//ע��Mat����ϵ�����ǵĵ�ͼ����ϵxy���Ƿ�������
	switch (event)
	{
	//������������յ�
	case EVENT_LBUTTONDOWN:
		goalx = y;
		goaly = x;
		need_replan_path = 1;
		break;
	//�Ҽ�����������������
	case EVENT_RBUTTONDOWN:
		enemy1.x = y;
		enemy1.y = x;
		break;
	}
}