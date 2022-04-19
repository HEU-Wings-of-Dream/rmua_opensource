#include <ros/ros.h>
#include"zimiaoshiyan/Run/run.h"
#include<thread>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point32.h>
#include"zimiaoshiyan/define.h"

using namespace rm;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "zimiaoshiyan");//初始化节点
    ros::NodeHandle node_obj;//创建通信句柄
    rm::Run _run;  
    _run.init();
    ros::Publisher auto_aim_publisher = node_obj.advertise <geometry_msgs::Accel>("/auto_aim_task", 10);
    
    //_run.auto_aim_publisher.publish(_run.my_new_controlframe);

    std::thread producerThread(&rm::Run::producer, &_run);
    std::thread consumerThread(&rm::Run::consumer, &_run);
    // printf("&&&&&&&&&&&&&&&&&&&&&&\n");
    // std::thread new_publishThread(&rm::Run::auto_aim_publisher_thread, &_run);
    // printf("##################\n");
    // //std::thread listenerThread(&rm::Run::listener,&_run);
    //std::thread publisherThread(&rm::Run::publisher,&_run);

#if (RECORD)
    std::thread recordThread(&rm::Run::recorder, &_run);
    recordThread.join();
#endif

    //new_publishThread.join();
    producerThread.detach();
    consumerThread.detach();
    

    //geometry_msgs::Accel test_msgs;
    //memset(&test_msgs, 0, sizeof(test_msgs));
    //auto_aim_publisher.publish(test_msgs);
    while(ros::ok())
            {
                if (_run.need_to_publish == 1)
                {
                    printf("control_data == %f   %f   %f \n", _run.my_new_controlframe.linear.x, _run.my_new_controlframe.linear.y, _run.my_new_controlframe.linear.z);
                    //auto_aim_publisher.publish(_run.my_new_controlframe);
                    _run.need_to_publish = 0;
                    ros::spinOnce();
                }
                _run.my_rate.sleep();
            }
    //listenerThread.join();
    //publisherThread.join();

    return 0;
}
