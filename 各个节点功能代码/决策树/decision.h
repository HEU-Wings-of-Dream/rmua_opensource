#pragma once
#include <thread>
#include <chrono>
#include <iostream>

bool my_team;				//哪一方，红0蓝1

class robot
{
public:
	int HP, bullet_left;
	bool color;				//哪一方，红0蓝1
	bool light_bar_color;	//灯条颜色
	bool is_ally;			//是否是盟友
	bool is_punish;			//现在可能用不到，但是今后呢
	bool x, y;
};
robot r[5];

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

void publisher(int who, int x, int y)
{

}

void DecisionTree()
{
	
	int Deducted_HP = 0;
	while (true)
	{
		//buff区运动策略
		for (int i = 1; i <= 6; i++)//遍历buff区
		{
			if (radom_area[i] == 1 && my_team == 0){
				for (int i = 1; i <= 4; i++)
					if (r[i].is_ally == 1)
						Deducted_HP += 2000 - r[i].HP;
				if (Deducted_HP >= 150){
					publisher(1, 0, 0);
					Deducted_HP = 0;
				}
			}

			if (radom_area[i] == 2 && my_team == 1) {
				for (int i = 1; i <= 4; i++)
					if (r[i].is_ally == 1)
						Deducted_HP += 2000 - r[i].HP;
				if (Deducted_HP >= 150) {
					publisher(1, 0, 0);
					Deducted_HP = 0;
				}
			}

			if (radom_area[i] == 3 && my_team == 1) {
				publisher(1, 0, 0);
			}

			if (radom_area[i] == 4 && my_team == 2) {
				publisher(1, 0, 0);
			} 
		}
		
		for (int i = 1; i <= 4; i++)
		{
			//子弹充足，但是不会刷buff了，不管血量，殊死一搏，子弹打空，反正是以伤害量判定胜负
			if ((r[i].is_ally == 1) && (r[i].bullet_left >= 0) && (nowtimeleft <= 120))
			{
				Attack();
				publisher();
			}
			//还会刷buff，但是现在状态不佳，不妨等刷buff，休整再战
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].bullet_left <= 20))
			{
				waitting();
			}
			//还会刷buff，并且我状态好，淦就完了
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].bullet_left >= 20))
			{
				Attack();
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(700));  //休眠700ms
	}
}