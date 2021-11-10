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
int a[228][408]; //��ͼ
double enemy_fire_cost[228][408];//�з��������۵�ͼ
bool flag[228][408], flag_optimizer[228][408];//A*�������
int corner_array[93024]; int number_of_corner = 0;//������ŵ��ǹյ���path_array������±꣬����ͬʱ���ã��������Ǵ�1��ʼ���
cv::Mat img, img_plain_save, img_erode_save, simulation_map, path_map(226,406,CV_8UC1,Scalar(255)), mask, img_final_path;//�����ͼɶ��
bool need_replan_path = 1, first_in_simulation_function = 0;
bool delete_path_flag = 0;//Ϊ1ʱ����ǰ·���滮��Ϣ�Ѿ�ʹ����ϣ����Թ黹ϵͳ�ռ�
std::vector<cv::Point>corner_vector;
//ʱ���������
std::chrono::steady_clock::time_point program_start_time;
double total_start_time, last_simulation_time;

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

class self_robot
{
//����㶨����vmaxΪ1.8(m/s), vrmax = 1.5��(rad/s)
//�ٳ������ܷǳ��ã�������Ϊ�ٶȶ��Ǻ㶨�ģ�û�м���ʱ�䡢����ʱ�䣬�Դ˼򻯼���
//������90�ȵ�ת��뾶Ҳֻ��0.18m�������Ҿ����н��ٶ�ͦ����
public:
	double x;
	double y;
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
	int next_corner; //��һ���ս�
	int next_state_change_node; //��һ���ӵ�״̬�ı��
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
	img_plain_save = cv::imread("map2.png", -1);
	//��ʴ����ȡ�ṹԪ��ָ����������ֵ����Сֵ��Ϊ��λ�õ�����Ҷ�ֵ
	Mat structureElement = getStructuringElement(MORPH_RECT, Size(25, 25), Point(-1, -1));
	//�ṹԪ�����Ƕ���д����ۣ�Ŀǰ��25 * 2cm = 50cm��Լ���ڵ��̰뾶
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
			//cout << "need :  " << path_array[i]->y << ' ' << path_array[i]->x << endl;
			circle(img_final_path, cv::Point(path_array[i]->y, path_array[i]->x), 2, Scalar(0), -1);
		}

		last_state = now_state;
	}
	//imshow("123", img);
	//waitKey(1);
	path_optimizer_array_size++;
	path_optimizer_array[path_optimizer_array_size] = path_array[path_long];//�洢���һ����

	cout << "Path_optimizer: " << "Plan to optimize " << path_optimizer_array_size << " part path. "<<endl;
	//����ÿ�����ӵе㣬���еڶ���A*���·�Ż�����ȥ���յ�
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

				find_corner();

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
	//while (1) {
		//refresh_simulation_environment();
		if (need_replan_path == 1) {
			cout << "Running path_planner..." << endl;
			img = img_plain_save.clone();
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
					cout << "Use point = " << memory_pool_i << endl;
					cout << "Queue size = " << search_queue.size() << endl;
					cout << "Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
					cout << "Total step = " << now_step << endl;
					cout << endl;

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
					//���������ͼ
					//img = imread("map2.png", -1);
					//simulation_map = img;
					end_flag = 1;
					need_replan_path = 0;
				}

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
			//cout << "out while";
		}
		//cout << "out if ";
	//}
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
	cout << "Cleaner Function:  Delete " << test-1 << " node" << endl;
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
	//final_path_i = 1;
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
		refresh_simulation_environment();
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

int main()
{	
	program_start_time = std::chrono::high_resolution_clock::now();//ʱ������֮ʼҲ

	init_load_map();//���ص�ͼ
	init_load_wall_line();//����ǽ����Ϣ
	init_load_erode_wall_line();//���ظ�ʴ֮��ĵ�ͼ��Ϣ

	namedWindow("Simulation_Environment@HEUsjh");//�������滷������
	setMouseCallback("Simulation_Environment@HEUsjh", mouse_handle);//��������¼��ص�����

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

	//���빤��
	while (1)
	{
		if (need_replan_path == 1) { 
			auto t1 = std::chrono::high_resolution_clock::now();
			path_planner(); 
			path_optimizer(); 
			delete_now_path(); //��Ҫע�����final_path��ɾ��ʱ��
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

//ע�⣬�����xy����ϵ�Ǻ�opencv Mat����ϵ�Ƿ������ģ�Ҳ��������x������y
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