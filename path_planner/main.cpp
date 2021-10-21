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
			//A = 1
			//sigma(x) = 50;
			//sigma(y) = 50;
			enemy_fire_cost[i][j] = is_met(enemy1, i, j) * (50000.0 * exp(-(((i - enemy1.x) * (i - enemy1.x)) / (2 * 50 * 50) + ((j - enemy1.y) * (j - enemy1.y)) / (2 * 50 * 50))));
			//�˴�ӦΪenemy2�������
		}
	}
}

//ˢ�µз�������в���۵�ͼ���㺯����ע�⣬�����Ǵ����ֻ���ǵз�������в��ֻ��з�λ���й�
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
	for (const auto& it : path_vector)
	{
		count++;
		//��һ����û�и��ڵ㣬���һ����û���ӽڵ㣬����Ҫ��
		if (count == 1) { front_point = it; continue; }
		if (count == path_vector.size()) continue;
		middle_point = it;
		next_point = memory_pool[it->father];
		double vec1x = middle_point->x - front_point->x;
		double vec1y = middle_point->y - front_point->y;
		double vec2x = middle_point->x - next_point->x;
		double vec2y = middle_point->y - next_point->y;
		//б���Ƿ���ȣ�������ǿ϶��ǹյ���
		//����yyds!!
		//if ((vec1y / vec1x) != (vec2y / vec2x))
			//Bezier_curve_optimizer(front_point->x, front_point->y, middle_point->x, middle_point->y, next_point->x, next_point->y);
	}
}

//ʹ�ñ����������Ż�·���ĺ���
void Bezier_curve_optimizer(int x1, int y1, int x2, int y2, int x3, int y3)
{

}

void simulation_environment()
{
	imshow("Simulation_Environment@HEUsjh", simulation_map);
	waitKey(1);
}

//���滷�����ʱ��ص�����
void mouse_handle(int event, int x, int y, int flags, void* param);

int main()
{
	//build_game_map();
	//���ص�ͼ
	img = cv::imread("map2.png", -1);

	auto program_start_time = std::chrono::high_resolution_clock::now();
	//��ʼ�����滷��
	namedWindow("Simulation_Environment@HEUsjh");
	setMouseCallback("Simulation_Environment@HEUsjh", mouse_handle);
	simulation_map = img;
	init_load_wall_line();
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
	/*��������������۵�ͼʱ��Ĳ���
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
	Refresh_enemy_fire_cost();//ˢ�»�����ͼ
	//����·����㣬�������丸�ڵ�Ϊ�Լ���������ֹ����
	memory_pool[memory_pool_i] = new my_point(0, memory_pool_i, start_x, start_y);
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
			while (now_point.father != now_point.self)
			{
				img.at<uchar>(now_point.x, now_point.y) = 0;
				my_point father_point = *(memory_pool[now_point.father]);
				now_point = father_point;
				path_vector.push_back(memory_pool[now_point.self]);
				now_step++;
			}
			//find_corner();

			//���û��ݺ�����ӡ·��
			//path_recall(*now, 1);
			//ɾ�����е�
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
		break;
	//�Ҽ�����������������
	case EVENT_RBUTTONDOWN:
		enemy1.x = y;
		enemy1.y = x;
		break;
	}
}