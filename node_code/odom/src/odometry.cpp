#include <chrono>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <odom/Posemsg.h>
#include <geometry_msgs/TransformStamped.h>
#include <odom/velocity.h>
#include <tf/transform_listener.h>


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
//int count = 0;

auto last_receive_time = std::chrono::high_resolution_clock::now();

void update_velocity_callback(const odom:: velocity :: ConstPtr& msg_receive);

int main(int argc, char **argv)
{
    //初始化里程计节点
    ros::init(argc, argv, "odom");
    //创建通信句柄
    ros::NodeHandle communication_handle_obj;
    //订阅消息
    ros::Subscriber velocity_listener = communication_handle_obj.subscribe("/velocity", 10, update_velocity_callback);
    ros::Publisher odom_publisher = communication_handle_obj.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Publisher world_publisher = communication_handle_obj.advertise<odom::Posemsg>("/send_to_server_now_pose", 30);
    tf::TransformBroadcaster odom_broadcaster;
    auto _startTime = std::chrono::high_resolution_clock::now();
    //lasttime = ros::Time::now();
    pose.x = 0;
    pose.y = 0;
    pose.angle = 0;
    ros::Rate rate(150);
    int flag = 0;

    //tf::TransformListener listener;
    while(ros::ok())
    {
        ros::spinOnce();
        if (flag == 0)
        {
            lasttime = msg.nowtime;
            flag = 1;
            continue;
        }
        double dt = (msg.nowtime - lasttime).toSec();
        lasttime = msg.nowtime;
        //ROS_INFO("%lf   %lf  %lf  %lf %lf",msg.omiga, pose.angle,lasttime.toSec(),msg.nowtime.toSec(),dt);
        //这里有一个问题，是先更新angle还是先更新vx&&vy
        pose.angle = msg.omiga;
        //pose.angle = 0;
        
        // pose.x += (msg.vx * cos(pose.angle/180*3.1415926) - msg.vy * sin(pose.angle /180*3.1415926)) * dt;
        // pose.y += (msg.vx * sin(pose.angle) + msg.vy * cos(pose.angle /180*3.1415926)) * dt;
         geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( - posemsg.angle/180*3.1415926);
        //向哨岗发送位置信息
        posemsg.time = ros::Time::now();
        world_publisher.publish(posemsg);
        
        
        // posemsg.time = ros::Time::now();
        // posemsg.x = pose.x;
        // posemsg.y = pose.y;
        // posemsg.angle = pose.angle;
        //printf("111\n");
               // printf("111\n");
        
        //准备发送tf变换消息
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = posemsg.x / 100.0;
        odom_trans.transform.translation.y = -posemsg.y / 100.0;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        printf("nowx = %f,   nowy = %f,   now_angle = %f\n", odom_trans.transform.translation.x, odom_trans.transform.translation.y, posemsg.angle);

        // if ((static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now()- _startTime)).count() >= 10)
        // {
        //     //ROS_INFO("%d", count);
        //     //count = 0;
        //     world_publisher.publish(posemsg);
        //     _startTime = std::chrono::high_resolution_clock::now();
        // }
        //count++;
        odom_broadcaster.sendTransform(odom_trans);
        ros::spinOnce();
        /*-----------Send normal ROS odometry msgs-------------------------*/
        // nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

        // // Header
        // odom->header.stamp = ros::Time::now();
        // odom->header.frame_id = "odom_frame";
        // odom->child_frame_id = "base_frame";

        // // Position
        // odom->pose.pose.position.x = pose.x;
        // odom->pose.pose.position.y = pose.y;
        // odom->pose.pose.position.z = 0.0;
        // odom->pose.pose.orientation = odom_quat;

        // // Velocity
        // odom->twist.twist.linear.x = (msg.vx * cos(pose.angle) - msg.vy * sin(pose.angle));
        // odom->twist.twist.linear.y = (msg.vx * sin(pose.angle) + msg.vy * cos(pose.angle));
        // odom->twist.twist.linear.z = 0;
        // odom->twist.twist.angular.x = 0;
        // odom->twist.twist.angular.y = 0;
        // odom->twist.twist.angular.z = msg.omiga;

        // //odom->pose.covariance[0] = 0.1;                           // x的协方差
        // //odom->pose.covariance[7] = 0.1;                           // y的协方差
        // //odom->pose.covariance[35] = 0.2; //theta的协方差

        // //odom->pose.covariance[14] = 1e10; // set a non-zero covariance on unused    theta x axis
        // //odom->pose.covariance[21] = 1e10; // dimensions (z, pitch and roll); this          theta y  axis
        // //odom->pose.covariance[28] = 1e10; // is a requirement of robot_pose_ekf        theta z axis
        // odom_publisher.publish(odom);
        rate.sleep();
        //ROS_INFO("x = %f,y = %f, angle = %f",pose.y / 100.0 , pose.x / 100.0, pose.angle);
    }    
}

void update_velocity_callback(const odom:: velocity :: ConstPtr& msg_receive)
{
    posemsg.x = msg_receive->vx ;
    posemsg.y = msg_receive->vy ;
    posemsg.angle = msg_receive -> omiga;
    // msg.vx = msg_receive->vx;
    // msg.vy = -msg_receive->vy;
    // msg.omiga = msg_receive->omiga;
    // msg.nowtime = msg_receive->nowtime;
    // //printf("vx = %f     vy = %f \n", msg.vx, msg.vy);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // auto dt = ((static_cast<std::chrono::duration<float, std::milli>>(t1- last_receive_time)).count()) / (double)(1000.0);
    // last_receive_time = t1;
    // //printf("dt = %f     df1000 = %f \n", dt, dt*1000.0);
    // if (abs(msg_receive->omiga) > 500) return;
    // posemsg.angle = msg_receive->omiga;
    // if (std::abs((msg_receive->vx * std::cos(posemsg.angle / 180.0 * 3.14159265354) - msg_receive->vy * std::sin(posemsg.angle / 180.0 * 3.14159265354)) * (double)(dt / 1000.0))>10000) return;
    // if (std::abs((msg_receive->vx * std::sin(posemsg.angle / 180.0 * 3.14159265354) + msg_receive->vy * std::cos(posemsg.angle / 180.0 * 3.14159265354)) * (double)(dt / 1000.0))>10000) return;
    // posemsg.x += (((msg_receive->vx * std::cos(posemsg.angle / 180.0 * 3.14159265354) + msg.vy * std::sin(posemsg.angle / 180.0 * 3.14159265354)) )* (double)(dt )) / 2;
    // posemsg.y += (((msg.vy * std::cos(posemsg.angle / 180.0 * 3.14159265354) - msg_receive->vx * std::sin(posemsg.angle / 180.0 * 3.14159265354)) )* (double)(dt )) / 2;

    //printf("vx = %f     vy = %f     omiga = %f \n", msg_receive->vx, msg_receive->vy, msg_receive->omiga);
}