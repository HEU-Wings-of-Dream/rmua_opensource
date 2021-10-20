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

int map_x = 4483;
int map_y = 8083;
int goalx, goaly, start_x, start_y;
int a[4483][8083];
//int cost[4483][8083];
bool flag[4483][8083];

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

my_point* memory_pool[18118044];
int memory_pool_i = 0;
std::vector <my_point> now_path;

//������һ��Ĺ�ֵ����
double h(int x, int y)
{
	double dis = sqrt((x - goalx) * (x - goalx) + (y - goaly) * (y - goaly));
	//return dis + cost[x][y];
	return dis;
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

int main()
{
	Mat img,img2;
	//build_game_map();
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
			//���û��ݺ�����ӡ·��
			my_point now_point = *now;
			int now_step = 0;
			while (now_point.father != now_point.self)
			{
				img.at<uchar>(now_point.x, now_point.y) = 0;
				my_point father_point = *(memory_pool[now_point.father]);
				now_point = father_point;
				now_step++;
			}
			//path_recall(*now, 1);
			//ɾ�����е�
			while (search_queue.empty() == 0)
			{
				delete(search_queue.top());
				search_queue.pop();
			}
			cout << "Time cost : " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms";
			imwrite("111.png", img);
			
			cout << endl << "Total step = " << now_step << endl;
			//cout << endl << endl;
			imshow("111", img);
			waitKey(10000);
			break;
		}

		//��������
		if (now->x + 1 <= map_x - 1 && (flag[now->x + 1][now->y] == 0) && (a[now->x + 1][now->y] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x + 1][now->y] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x + 1, now->y);
			search_queue.push(memory_pool[memory_pool_i]);
		}

		if (now->y + 1 <= map_y - 1 && (flag[now->x][now->y + 1] == 0) && (a[now->x][now->y + 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x][now->y + 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y + 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}
		
		if (now->y - 1 >= 0 && (flag[now->x][now->y - 1] == 0) && (a[now->x][now->y - 1] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x][now->y - 1] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x, now->y - 1);
			search_queue.push(memory_pool[memory_pool_i]);
		}

		if (now->x - 1 >= 0 && (flag[now->x - 1][now->y] == 0) && (a[now->x - 1][now->y] == 0))
		{
			memory_pool_i = memory_pool_i + 1;
			flag[now->x - 1][now->y] = 1;
			memory_pool[memory_pool_i] = new my_point(now->self, memory_pool_i, now->x - 1, now->y);
			search_queue.push(memory_pool[memory_pool_i]);
		}

		//cout << search_queue.size() << endl;
	}
}