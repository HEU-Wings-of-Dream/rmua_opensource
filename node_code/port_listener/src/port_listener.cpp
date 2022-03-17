#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include "odom/data_frame.h"
#include "odom/Serial.h"
#include <odom/Posemsg.h>
#include <odom/velocity.h>

#include "head.h"

#define LINE_10 "----------"
#define LINE120 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10 LINE_10
#define LINE_IND LINE120
#define LINE_TIP_FMT		"|%-10s|%-10s|%-48s|%-48s|\n"
#define LINE_CONTENT_FMT	"|%-10d|%-10s|%-48s|%-48s|\n"
#define PRINT_ALL_COM 1

void autoaim_callback(const odom:: velocity :: ConstPtr& msg_receive)
{
    msg.x = msg_receive->x;
    msg.y = msg_receive->y;
    msg.z = msg_receive->z;
    ROS_INFO("x = %f y = %f z = %f ", msg_receive->x, msg_receive->y, msg_receive->z);
}

int main()
{

#if PRINT_ALL_COM
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    // 选择串口
    // 顶住左边开始, 每30个字符为一个显示字段
    printf("%s\n", LINE_IND);
    printf(LINE_TIP_FMT, "port_sn", "port_name", "port_desc", "port_hd_id");
    printf("%s\n", LINE_IND);

    int dev_port = 0;
    while (iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        printf(LINE_CONTENT_FMT, dev_port++, device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
        printf("%s\n", LINE_IND);
    }
#endif // !USE

    rm::my_sleep(1000);

#ifdef _WIN32
    serial::Serial my_serial("COM6", 460800, serial::Timeout::simpleTimeout(1000));
#else
    serial::Serial my_serial("/dev/ttyUSB0", 460800, serial::Timeout::simpleTimeout(1000));
#endif

    //建立roscore连接
    ros::init(argc, argv, "portlistener");
    ros::NodeHandle communication_handle_obj;

    //向里程计节点发布底盘速度信息
    ros::Publisher velocity_publisher = 
            communication_handle_obj.advertise <odom :: velocity> ("/velocity",10);

    //向自瞄节点发布底盘回传信息
    ros::Publisher autoaim_publisher = 
            communication_handle_obj.advertise <odom :: velocity> ("/velocity",10);
    
    //监听自瞄节点发布的三维目标点
    ros::Subscriber autoaim_listener = communication_handle_obj.subscribe("/velocity", 10, autoaim_callback);
    
    //打印调试信息
    serial::Serial* sp = &my_serial;
    if (my_serial.isOpen())cout << "Serial open succed!" << endl;
    //cout << my_serial.getStopbits() << endl;
    cout << "Now Baudrate is:  " << my_serial.getBaudrate() << endl;
    //rm::openport(my_serial);
    
    //进入监听阶段
    int which_bag = 0;
    rm::test_struct data;
    while (1)
    {
        which_bag = rm::try_read(my_serial, sp, &data);

        if (which_bag == 1){
            rm::move_control_struct my_msg;
            memcpy(&my_msg, data, sizeof(rm::test_struct));

        }

        if (which_bag == 2){
            rm::race_state_struct my_msg;
            memcpy(&my_msg, data, sizeof(rm::test_struct));

        }

        if (which_bag == 3){
            rm::autoaim_feedback_struct my_msg;
            memcpy(&my_msg, data, sizeof(rm::test_struct));

        }
        ros::spinOnce();
        
    }

    return 0;
}
