#pragma once
#include <thread>
#include <chrono>
#include <iostream>

bool my_team;				//��һ������0��1

class robot
{
public:
	int HP, bullet_left;
	bool color;				//��һ������0��1
	bool light_bar_color;	//������ɫ
	bool is_ally;			//�Ƿ�������
	bool is_punish;			//���ڿ����ò��������ǽ����
	bool x, y;
};
robot r[5];

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

void publisher(int who, int x, int y)
{

}

void DecisionTree()
{
	
	int Deducted_HP = 0;
	while (true)
	{
		//buff���˶�����
		for (int i = 1; i <= 6; i++)//����buff��
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
			//�ӵ����㣬���ǲ���ˢbuff�ˣ�����Ѫ��������һ�����ӵ���գ����������˺����ж�ʤ��
			if ((r[i].is_ally == 1) && (r[i].bullet_left >= 0) && (nowtimeleft <= 120))
			{
				Attack();
				publisher();
			}
			//����ˢbuff����������״̬���ѣ�������ˢbuff��������ս
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].bullet_left <= 20))
			{
				waitting();
			}
			//����ˢbuff��������״̬�ã��ƾ�����
			if ((r[i].is_ally == 1) && (nowtimeleft > 120) && (r[i].bullet_left >= 20))
			{
				Attack();
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(700));  //����700ms
	}
}