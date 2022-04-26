#pragma once
#include <thread>
#include <chrono>
#include <iostream>
#include <algorithm>
#include "attack.hpp"

bool my_team;				//��һ������0��1

class robot
{
public:
	int HP, bullet_left;
	int armor_num;
	bool color;				//��һ������0��1
	bool light_bar_color;	//������ɫ
	bool is_ally;			//�Ƿ������ѣ�0��1��
	bool is_punish;			//���ڿ����ò��������ǽ����
	bool x, y;
	
	//�Ƿ�������״̬�������ǰ״̬���ѣ��򷵻�1���������ֵ�ᵼ���Ժ����ʱ���ٿ���ʹ����������ˡ�
	bool is_weak() {
		if (this->bullet_left <= 10 || this->HP <= 100)
			return 1;
		return 0;
	}

};
robot r[5];//ע�⣬�������Ǵ�1��ʼ�ŵģ�1��2��3��4

double nowtime, nowtimeleft;

/*---�涨----
* ���Ѫ�� 1
* ����Ѫ�� 2
* �첹���� 3
* �������� 4
* ��ֹ��� 5
* ��ֹ�ƶ� 6
*/
int radom_area[10];

//publisher�Ķ���opencv Mat����ϵ�µ����꣬ ��Ҫ�����
void goalpoint_publisher(int who, int x, int y)
{

}

void command_publisher(int who, int command)
{
	/*
	* ������ߣ�˫��ʹ��Ԥ��Լ�����������
	* who
	* 0:��ͼ����
	* 1,2:��Ӧ��
	*/
	
}

int get_best_robot();
int get_goal_enemy();
void Waitting();
void Courage();
bool cmp(cv::Point point1, cv::Point point2);

//�ǻ�������Ҫ��Ʒ�ʡ����������������Ƿ���ֱ���������������������
void Wisdom()
{
	while (true)
	{
		//buff���˶�����
		get_bullet_buff();
		get_HP_buff();
		
		for (int i = 1; i <= 4; i++)
		{
			//�ӵ����㣬���ǲ���ˢbuff�ˣ�����Ѫ��������һ�����ӵ���գ����������˺����ж�ʤ��
			if ((r[i].is_ally == 1) && (r[i].bullet_left >= 0) && (nowtimeleft <= 120))
				Courage();
			//����ˢbuff����������״̬���ѣ�������ˢbuff��������ս
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].is_weak() == 1))
				Waitting();
			//����ˢbuff��������״̬�ã��ƾ�����
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].bullet_left >= 20))
				Courage();
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(700));  //����700ms
	}
}

//��ͷ���ƴ�����Ժ���
//����������Ʒ�ʡ���Ϊ�����������˲���������ĺڰ�������������
void Courage()
{
	int goal_enemy = get_goal_enemy(); //�ҵ�һ������
	pick_strike_area(r[goal_enemy].x, r[goal_enemy].y);//�Ե��˵�������Ѱ���λ��
	if (strike_point_vector.empty() == 0)
	{
		if (strike_point_vector.size() == 1)
			goalpoint_publisher(get_best_robot(), strike_point_vector[0].x, strike_point_vector[0].y);
		
		if (strike_point_vector.size() == 2) {
			goalpoint_publisher(get_best_robot(), strike_point_vector[0].x, strike_point_vector[0].y);
			//����һ��������ս�����ĳ�ȥ��һ�����λ
			for (int i = 1; i <= 4; i++)
				if (r[i].is_ally && i != get_best_robot() && r[i].is_weak() == 0)
					goalpoint_publisher(i, strike_point_vector[0].x, strike_point_vector[0].y);
		}

		//��������Ԥѡ���λ�õ�ʱ��ѡ�������������Ĵ����
		//��ʵ���Ҫ�򱩻��Ļ�����ȫ����ѡ�����н����ĵ㣬�����Ǻ����߸���������
		if (strike_point_vector.size() > 2) {
			std::sort(strike_point_vector.begin(), strike_point_vector.end(), cmp);
			goalpoint_publisher(1, strike_point_vector[0].x, strike_point_vector[0].y);
			goalpoint_publisher(2, strike_point_vector[1].x, strike_point_vector[1].y);
		}
	}
}

void Waitting()
{
	for (int i = 1; i <= 4; i++) {
		if (r[i].is_ally && r[i].is_weak()) {
			if (r[i].armor_num == 1 && my_team == 0) goalpoint_publisher();
			if (r[i].armor_num == 2 && my_team == 0) goalpoint_publisher();
			if (r[i].armor_num == 1 && my_team == 1) goalpoint_publisher();
			if (r[i].armor_num == 2 && my_team == 1) goalpoint_publisher();
		}
	}
}

int get_best_robot()
{
	//Ŀǰ�ǰ����ӵ�ʣ����ѡ��
	int my_max = -999, ans;
	for (int i = 1; i <= 4; i++)
	{
		if (r[i].is_ally)
			if (r[i].bullet_left > my_max) ans = i;
	}
	return ans;
}

int get_goal_enemy()
{
	int my_min = 99999; int ans;
	for (int i = 1; i <= 4; i++) {
		if (!r[i].is_ally) {
			if (r[i].bullet_left < my_min) {
				my_min = r[i].bullet_left;  ans = i;
			}
		}
	}
	return ans;
}

inline void get_HP_buff()
{
	//��Ѫbuff:���������ҷ�������ʧ��Ѫ���������������ʧ������150������ǰ����buff�����򽫸ô���Ϊ�ϰ����������
	int Deducted_HP = 0;
	for (int i = 1; i <= 6; i++)//����buff��
	{
		//�Ǻ췽��Ѫ��
		if (radom_area[i] == 1 && my_team == 0) {
			for (int i = 1; i <= 4; i++)
				if (r[i].is_ally == 1)
					Deducted_HP += 2000 - r[i].HP;

			if (Deducted_HP >= 150) {
				getbuff(1, i);//��1�ų�ǰȥ��buff
				Deducted_HP = 0;
			}
		}

		//��������Ѫ��
		if (radom_area[i] == 2 && my_team == 1) {
			for (int i = 1; i <= 4; i++)
				if (r[i].is_ally == 1)
					Deducted_HP += 2000 - r[i].HP;

			if (Deducted_HP >= 150) {
				getbuff(1, i);
				Deducted_HP = 0;
			}
		}
	}
	return;
}

inline void get_bullet_buff()
{
	for (int i = 1; i <= 6; i++)
	{
		if (radom_area[i] == 3 && my_team == 1) {
			getbuff(1, i);
		}

		if (radom_area[i] == 4 && my_team == 2) {
			getbuff(1, i);
		}
	}
	return;
}

void getbuff(int who, int which)
{
	//publisher�Ķ���opencv Mat����ϵ�µ����꣬ ��Ҫ�����
	//buff����540����480(mm)
	if (which == 1) goalpoint_publisher(who, 379, 140);
	if (which == 2) goalpoint_publisher(who, 309, 83);
	if (which == 3) goalpoint_publisher(who, 202, 201);
	if (which == 4) goalpoint_publisher(who, 202, 23);
	if (which == 5) goalpoint_publisher(who, 95, 142);
	if (which == 6) goalpoint_publisher(who, 25, 83);
}

bool cmp(cv::Point point1, cv::Point point2)
{
	if (my_team == 0) 
		return sqrt((point1.x - 0) * (point1.x - 0) + (point1.y - 202) * (point1.y - 202)) < sqrt((point2.x - 0) * (point2.x - 0) + (point2.y - 202) * (point2.y - 202));
	if (my_team == 1)
		return sqrt((point1.x - 0) * (point1.x - 0) + (point1.y - 202) * (point1.y - 202)) < sqrt((point2.x - 0) * (point2.x - 0) + (point2.y - 202) * (point2.y - 202));
}