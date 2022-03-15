#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include "odom/data_frame.h"
#include "odom/Serial.h"
#include <odom/Posemsg.h>
#include <odom/velocity.h>

odom :: velocity msg;
rm::Serial _Serial;
bool STM32_Online = false;
uint32_t ErrorExist = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "portlistener");
    ros::NodeHandle communication_handle_obj;
    ros::Publisher velocity_publisher = 
            communication_handle_obj.advertise <odom :: velocity> ("/velocity",10);


    while (_Serial.openPort() != rm::Serial::SUCCESS)
    {
        ROS_INFO("Serial open failed!!");
    }
    _Serial.setDebug(true);
    rm::Serial::FeedBackFrame FDB;
    uint32_t lastPCt, LastSTMt;
    static uint8_t lastTeam;

    rm::Bag_Frame b;
    while (ros::ok())
    {
        if (ErrorExist > 100)
            ROS_INFO("portlistener error exist!!");
        
        if (_Serial.tryFeedBack(FDB, std::chrono::milliseconds(3)) == rm::Serial::SUCCESS)
        {
            if (FDB.EOF != 0x88)
                continue;

            msg.vx = (double)FDB.vx/1.0000000001 ;
            msg.vy = (double)FDB.vy/1.0000000001 ;
            msg.omiga = FDB.omiga;
            msg.nowtime = ros::Time::now();

            velocity_publisher.publish(msg);
            ROS_INFO("vx = %lf, vy = %lf, omiga = %lf",msg.vx, msg.vy, msg.omiga);
            
        }
        else
            ROS_INFO("receive failed");
        ros::spinOnce();
    }
}
