#include "attack.hpp"
#include <thread>
#include "TCP.h"

#define USE_TCP 0

#if USE_TCP

//因为接收频率的问题，有的时候会一下收到多个串粘在一起的情况，这样的话应该查找这个串中有没有我们需要的子串，进而提取并且处理
std::string check_have_need_msg(std::string msg_recv)
{
	int RECV_BUFFER_SIZE = 8;
	int str_len = msg_recv.length();
	//遍历每一个子串
	std::string temp_string;
	for (int i = 1; i <= (str_len + 1) / RECV_BUFFER_SIZE; i++) {
		temp_string = msg_recv.substr(RECV_BUFFER_SIZE * (i - 1), RECV_BUFFER_SIZE);
		if (temp_string[RECV_BUFFER_SIZE - 1] == 'E')
			return temp_string;
	}
	printf("NULL\n");
	return std::string("NULL");
}

void TCP_callback(void)
{
	int int_temp_x = 0, int_temp_y = 0;
	while (1) {
		if (msg_recv_queue.size() != 0) {
			std::string temp_str1 = msg_recv_queue.front();
			msg_recv_queue.pop();
			std::string temp_str = check_have_need_msg(temp_str1);
			if (temp_str == "NULL")
				continue;
			char temp_x[4], temp_y[4];
			temp_x[0] = temp_str[0]; temp_x[1] = temp_str[1]; temp_x[2] = temp_str[2]; //012 123
			temp_y[0] = temp_str[4]; temp_y[1] = temp_str[5]; temp_y[2] = temp_str[6];
			//std::cout << atoi(temp_x) << ' ' << atoi(temp_y) << std::endl;
			if (atoi(temp_y) >= 226 || atoi(temp_y) <= 0) { printf("out of map\n");  continue; }
			if (atoi(temp_x) >= 405 || atoi(temp_y) <= 0) { printf("out of map\n");  continue; }
			int_temp_x = 405 - atoi(temp_x);
			int_temp_y = 226 - atoi(temp_y);
			refresh_window(int_temp_x, int_temp_y);
			//std::cout << atoi(temp_x) << ' ' << atoi(temp_y) << std::endl;
		}
		/*else {
			if (int_temp_x != 0 && int_temp_y != 0) {
				printf("%d   %d", int_temp_x, int_temp_y);
				refresh_window(int_temp_y, int_temp_x);
			}
		}*/
	}
}
#endif

int main()
{
#if USE_TCP
	TCP_init();
	printf("tcp init success\n\n");

	std::thread TCPlistenThread(my_recv);//负责目前该干什么、该去哪、抢占buff的决定
	TCPlistenThread.detach();
	printf("tcp listener thread joined!\n\n");

	std::thread TCP_callback_thread(TCP_callback);
	TCP_callback_thread.detach();
	printf("TCP callback joined succed!!!\n\n");
#endif

#if USE_TCP == 0
	cv::namedWindow(WINDOW_NAME);
	cv::setMouseCallback(WINDOW_NAME, mouse_handle);
#endif // USETCP

	init();

	return 0;
}


//std::thread ListenerThread();//下位机接收函数
//std::thread YoloThread();//负责实时与YOLO进程通讯


//system_init();

//ros::init(argc, argv, "DecisionTree");
//ros::NodeHandle communication_handle_obj;

//ros::Subscriber locate_listener1 = communication_handle_obj.subscribe("/locate_from_Apriltag", 10, Update_Apriltag_Callback);
//ros::Subscriber locate_listener2 = communication_handle_obj.subscribe("/locate_from_Radar", 10, Update_Radar_Callback);
//ros::Subscriber locate_listener3 = communication_handle_obj.subscribe("/locate_from_Odom", 10, Update_Odom_Callback);
//ros::Subscriber locate_listener4 = communication_handle_obj.subscribe("/locate_from_UltrasonicRanging", 10, Update_UltrasonicRanging_Callback);