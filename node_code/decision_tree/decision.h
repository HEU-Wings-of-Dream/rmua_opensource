#pragma once
#include <thread>
#include <chrono>
#include <iostream>
#include <algorithm>
#include "attack.hpp"

bool my_team;				//哪一方，红0蓝1

class robot
{
public:
	int HP, bullet_left;
	int armor_num;
	bool color;				//哪一方，红0蓝1
	bool light_bar_color;	//灯条颜色
	bool is_ally;			//是否是盟友，0否1是
	bool is_punish;			//现在可能用不到，但是今后呢
	bool x, y;
	
	//是否处于虚弱状态，如果当前状态不佳，则返回1，这个返回值会导致以后决策时不再考虑使用这个机器人。
	bool is_weak() {
		if (this->bullet_left <= 10 || this->HP <= 100)
			return 1;
		return 0;
	}

};
robot r[5];//注意，这玩意是从1开始放的，1，2，3，4

double nowtime, nowtimeleft;

/*---规定----
* 红回血区 1
* 蓝回血区 2
* 红补弹区 3
* 蓝补弹区 4
* 禁止射击 5
* 禁止移动 6
*/
int radom_area[10];

//publisher的都是opencv Mat坐标系下的坐标， 不要搞混了
void goalpoint_publisher(int who, int x, int y)
{

}

void command_publisher(int who, int command)
{
	/*
	* 命令发布者，双方使用预先约定的命令代号
	* who
	* 0:地图更改
	* 1,2:对应车
	*/
	
}

int get_best_robot();
int get_goal_enemy();
void Waitting();
void Courage();
bool cmp(cv::Point point1, cv::Point point2);

//智慧是最重要的品质。它可以让你明辨是非曲直，引领你向光明而求索。
void Wisdom()
{
	while (true)
	{
		//buff区运动策略
		get_bullet_buff();
		get_HP_buff();
		
		for (int i = 1; i <= 4; i++)
		{
			//子弹充足，但是不会刷buff了，不管血量，殊死一搏，子弹打空，反正是以伤害量判定胜负
			if ((r[i].is_ally == 1) && (r[i].bullet_left >= 0) && (nowtimeleft <= 120))
				Courage();
			//还会刷buff，但是现在状态不佳，不妨等刷buff，休整再战
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].is_weak() == 1))
				Waitting();
			//还会刷buff，并且我状态好，淦就完了
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].bullet_left >= 20))
				Courage();
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(700));  //休眠700ms
	}
}

//从头定制打击策略函数
//勇气是最宝贵的品质。因为有了勇气，人才向内心里的黑暗与欲望斗争。
void Courage()
{
	int goal_enemy = get_goal_enemy(); //找到一个敌人
	pick_strike_area(r[goal_enemy].x, r[goal_enemy].y);//对敌人的坐标搜寻打击位置
	if (strike_point_vector.empty() == 0)
	{
		if (strike_point_vector.size() == 1)
			goalpoint_publisher(get_best_robot(), strike_point_vector[0].x, strike_point_vector[0].y);
		
		if (strike_point_vector.size() == 2) {
			goalpoint_publisher(get_best_robot(), strike_point_vector[0].x, strike_point_vector[0].y);
			//令另一辆具有作战能力的车去另一个打击位
			for (int i = 1; i <= 4; i++)
				if (r[i].is_ally && i != get_best_robot() && r[i].is_weak() == 0)
					goalpoint_publisher(i, strike_point_vector[0].x, strike_point_vector[0].y);
		}

		//多于两个预选打击位置的时候，选择两个离家最近的打击点
		//其实如果要打暴击的话，完全可以选两个夹角最大的点，但这是后来者该做的事了
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
	//目前是按照子弹剩余量选择
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
	//加血buff:计算所有我方车辆损失的血量总数，如果总损失量大于150则立即前往吃buff，否则将该处认为障碍，以免误吃
	int Deducted_HP = 0;
	for (int i = 1; i <= 6; i++)//遍历buff区
	{
		//是红方回血区
		if (radom_area[i] == 1 && my_team == 0) {
			for (int i = 1; i <= 4; i++)
				if (r[i].is_ally == 1)
					Deducted_HP += 2000 - r[i].HP;

			if (Deducted_HP >= 150) {
				getbuff(1, i);//派1号车前去吃buff
				Deducted_HP = 0;
			}
		}

		//是蓝方回血区
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
	//publisher的都是opencv Mat坐标系下的坐标， 不要搞混了
	//buff区长540，宽480(mm)
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