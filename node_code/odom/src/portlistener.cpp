#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include "odom/data_frame.h"
#include "odom/Serial.h"
#include <odom/Posemsg.h>
#include <odom/velocity.h>
#include <odom/my_control_frame.h>

rm::Serial::ControlFrame msg_rev;
odom :: velocity msg;
rm::Serial _Serial;
bool STM32_Online = false;
uint32_t ErrorExist = false;
bool need_to_send_flag = 0;

void update_velocity_callback(const odom ::my_control_frame ::ConstPtr &msg_receive)
{
   msg_rev.vx = (int)((msg_receive->vx)*100);
    //msg_rev.vx = (int)(50);
    msg_rev.vy = (int)((msg_receive->vy)*100);
    //msg_rev.vy = (int)(0);

    msg_rev.angle = (int)-(msg_receive->angle);
    //msg_rev.angle= 0;
    //msg_rev.angle = (int)(0);
    ROS_INFO("SEND vx: %d; vy: %d;  angle: %d",msg_rev.vx,msg_rev.vy,msg_rev.angle);
    need_to_send_flag = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "portlistener");
    ros::NodeHandle communication_handle_obj;
    ros::Publisher velocity_publisher = 
            communication_handle_obj.advertise <odom :: velocity> ("/velocity",10);
    ros::Subscriber move_control_subscriber = communication_handle_obj.subscribe("/robot_control", 10, update_velocity_callback);


    while (_Serial.openPort() != rm::Serial::SUCCESS)
    {
        ROS_INFO("Serial open failed!!");
    }
    _Serial.setDebug(true);
    rm::Serial::FeedBackFrame FDB;
    uint32_t lastPCt, LastSTMt;
    static uint8_t lastTeam;

    //rm::Bag_Frame b;
    while (ros::ok())
    {
        if (ErrorExist > 100)
            ROS_INFO("portlistener error exist!!");
        
        if (need_to_send_flag == 1)
        {
            while (_Serial.tryControl(msg_rev, std::chrono::milliseconds(3)) != rm::Serial::SUCCESS)
            {
            }
            need_to_send_flag = 0;
            ROS_INFO("Send vx = %d, angle = %d;  sizeof:%d    SUCCESS!", msg_rev.vx, msg_rev.angle,sizeof(msg_rev));
        }

        if (_Serial.tryFeedBack(FDB, std::chrono::milliseconds(3)) == rm::Serial::SUCCESS)
        {
            if (FDB.EOF != 0x88)
                continue;

            msg.vx = (float)FDB.vx;
            msg.vy = (float)FDB.vy;
            msg.omiga = FDB.omiga;
            //FDB.omiga = 0;
            msg.nowtime = ros::Time::now();

            velocity_publisher.publish(msg);
            ROS_INFO("vx = %lf, vy = %lf, omiga = %lf",msg.vx, msg.vy, msg.omiga);
            
        }
        else
            ROS_INFO("receive failed");
        ros::spinOnce();
    }
}
