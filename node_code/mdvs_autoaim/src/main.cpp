#include "mdvs_autoaim/Run/run.h"
#include <thread>
#include "mdvs_autoaim/define.h"
#include <ros/ros.h>
#include <geometry_msgs/Accel.h>

//#include "glog/logging.h"   // glog 头文件
//#include "glog/raw_logging.h"
using namespace rm;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "zimiaoshiyan");//初始化节点
    ros::NodeHandle node_obj;//创建通信句柄
    ros::Publisher auto_aim_publisher = node_obj.advertise <geometry_msgs::Accel>("/auto_aim_task", 10);

    //   google::InitGoogleLogging(argv[0]);
    //   FLAGS_log_dir = "../HEU_Vision_2020_MDVS/logger/";
    //   FLAGS_logbufsecs = 0;//设置可以缓冲日志的最大秒数，0指实时输出
    rm::Run _run;
    if (_run.init() == 0)
        return 0;
    // cv::namedWindow("as");

    std::thread producerThread(&rm::Run::producer, &_run);
    std::thread consumerThread(&rm::Run::consumer, &_run);
    //std::thread listenerThread(&rm::Run::listener, &_run);
    //std::thread publisherThread(&rm::Run::publisher, &_run);

#if (RECORD)
    std::thread recordThread(&rm::Run::recorder, &_run);
    recordThread.join();
#endif

    producerThread.detach();
    consumerThread.detach();
    //listenerThread.join();
    //publisherThread.join();

    while(ros::ok())
    {
        if (_run.need_to_publish == 1)
        {
            printf("control_data == %f   %f   %f %f\n", _run.my_new_controlframe.linear.x, _run.my_new_controlframe.linear.y, _run.my_new_controlframe.linear.z, _run.my_new_controlframe.angular.z);
            auto_aim_publisher.publish(_run.my_new_controlframe);
            _run.need_to_publish = 0;
            ros::spinOnce();
        }
        _run.my_rate.sleep();
    }

    return 0;
}
