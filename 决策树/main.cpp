#include <attack.hpp>
#include <thread>

int main()
{
	cv::namedWindow(WINDOW_NAME);
	cv::setMouseCallback(WINDOW_NAME, mouse_handle);
	//std::thread DecisionThread();//����Ŀǰ�ø�ʲô����ȥ�ġ���ռbuff�ľ���
	//std::thread ListenerThread();//��λ�����պ���
	//std::thread YoloThread();//����ʵʱ��YOLO����ͨѶ

	//system_init();
	init();
	//ros::init(argc, argv, "DecisionTree");
	//ros::NodeHandle communication_handle_obj;

	//ros::Subscriber locate_listener1 = communication_handle_obj.subscribe("/locate_from_Apriltag", 10, Update_Apriltag_Callback);
	//ros::Subscriber locate_listener2 = communication_handle_obj.subscribe("/locate_from_Radar", 10, Update_Radar_Callback);
	//ros::Subscriber locate_listener3 = communication_handle_obj.subscribe("/locate_from_Odom", 10, Update_Odom_Callback);
	//ros::Subscriber locate_listener4 = communication_handle_obj.subscribe("/locate_from_UltrasonicRanging", 10, Update_UltrasonicRanging_Callback);
	return 0;
}





