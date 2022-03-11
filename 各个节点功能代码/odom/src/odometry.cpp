#include <chrono>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <odom/Posemsg.h>
#include <geometry_msgs/TransformStamped.h>
#include <odom/velocity.h>


class Pose
{
public:
    double x;
    double y;
    double angle;
    void now_pose () {x = 0.0; y = 0.0; angle = 0.0;}
}pose;
odom :: velocity msg;
odom :: Posemsg posemsg;
ros::Time lasttime;
int count = 0;

void update_velocity_callback(const odom:: velocity :: ConstPtr& msg_receive)
{
    msg.vx = msg_receive->vx;
    msg.vy = msg_receive->vy;
    msg.omiga = msg_receive->omiga;
    msg.nowtime = msg_receive->nowtime;
    ROS_INFO("vx = %f vy = %f omiga = %f ", msg_receive->vx, msg_receive->vy, msg_receive->omiga);
}

int main(int argc, char **argv)
{
    //初始化里程计节点
    ros::init(argc, argv, "odometry");
    //创建通信句柄
    ros::NodeHandle communication_handle_obj;
    //订阅消息
    ros::Subscriber velocity_listener = communication_handle_obj.subscribe("/velocity", 10, update_velocity_callback);
    //ros::Publisher odom_publisher = communication_handle_obj.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Publisher world_publisher = communication_handle_obj.advertise<odom::Posemsg>("/send_to_server_now_pose", 10);
    tf::TransformBroadcaster odom_broadcaster;
    auto _startTime = std::chrono::high_resolution_clock::now();
    while(ros::ok())
    {
        ros::spinOnce(); 
        double dt = (msg.nowtime - lasttime).toSec();
        lasttime = msg.nowtime;
        //这里有一个问题，是先更新angle还是先更新vx&&vy
        pose.angle += msg.omiga * dt;
        pose.x += (msg.vx * cos(pose.angle) - msg.vy * sin(pose.angle)) * dt;
        pose.y += (msg.vx * sin(pose.angle) + msg.vy * cos(pose.angle)) * dt;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.angle);
        //向哨岗发送位置信息
        posemsg.time = ros::Time::now();
        posemsg.x = pose.x;
        posemsg.y = pose.y;
        posemsg.angle = pose.angle;
        //准备发送tf变换消息
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = pose.x;
        odom_trans.transform.translation.y = pose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        if ((static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now()- _startTime)).count() >= 10)
        {
            //ROS_INFO("%d", count);
            count = 0;
            world_publisher.publish(posemsg);
            _startTime = std::chrono::high_resolution_clock::now();
        }
        count++;
        odom_broadcaster.sendTransform(odom_trans);
        ROS_INFO("x = %f,y = %f, angle = %f",pose.x, pose.y, pose.angle);
    }    
}
