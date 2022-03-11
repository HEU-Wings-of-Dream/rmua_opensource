#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int hang = 4480;
int lie = 8080;
//ע��Point����x����y������x�Ǻ��������ᣬҲ����˵(�У���)
//resolution: 0.01 (m/pix) == 1(cm/pix)
Mat my_map(4482, 8082, CV_8UC1, Scalar(255));

void work(int start_x, int start_y, int end_x, int end_y)
{
	line(my_map, Point(start_x, start_y), Point(end_x, end_y), Scalar(0), 1);
	return;
}

void fun(int x, int y, int cols, int rows)
{
	line(my_map, Point(x, y), Point(x + cols, y), Scalar(0), 1);
	line(my_map, Point(x + cols, y), Point(x + cols, y + rows), Scalar(0), 1);
	line(my_map, Point(x + cols, y + rows), Point(x, y + rows), Scalar(0), 1);
	line(my_map, Point(x, y + rows), Point(x, y), Scalar(0), 1);
}

int main()
{	
	//ʹ�ú�ɫ���Ƶ�ͼ�߽�
	work(0, 0, lie + 1, 0);
	work(lie + 1, 0, lie + 1, hang + 1);
	work(lie + 1, hang + 1, 0, hang + 1);
	work(0, hang + 1, 0, 0);
	//����������Χ��
	//����
	work(0, 1000, 1000, 1000);
	work(1000, 1000, 1000, 1200);
	work(1000, 1200, 0, 1200);
	work(0, 1200, 0, 1000);
	//����
	work(1500, 3480, 1700, 3480);
	work(1700, 3480, 1700, 4480);
	work(1700, 4480, 1500, 4480);
	work(1500, 4480, 1500, 3480);
	//����
	work(6580, 0, 6780, 0);
	work(6780, 0, 6780, 1000);
	work(6780, 1000, 6580, 1000);
	work(6580, 1000, 6580, 0);
	//����
	work(7080, 3280, 8080, 3280);
	work(8080, 3280, 8080, 3480);
	work(8080, 3480, 7080, 3480);
	work(7080, 3480, 7080, 3280);
	//��Χ��
	fun(1500, 2140, 800, 200);
	//��Χ��
	fun(5780, 2140, 800, 200);
	//��Χ��
	fun(3540, 935, 1000, 200);
	//��Χ��
	fun(3540, 3345, 1000, 200);
	//���Ŀ�
	line(my_map, Point(4040 ,2240 - 125), Point(4040 + 125, 2240), Scalar(0), 1);
	line(my_map, Point(4040 + 125, 2240), Point(4040, 2240 + 125), Scalar(0), 1);
	line(my_map, Point(4040, 2240 + 125), Point(4040 - 125, 2240), Scalar(0), 1);
	line(my_map, Point(4040 - 125, 2240), Point(4040, 2240 - 125), Scalar(0), 1);
	

	//��һ��
	/*uchar* imgdata = (uchar*) map.data;
	for (int i = 0; i < hang * lie; i++) {
		*imgdata /= 255;
		imgdata++;
	}*/
	imwrite("map.png", my_map);
	imshow("111", my_map);
	waitKey(100000);
	return 0;
}