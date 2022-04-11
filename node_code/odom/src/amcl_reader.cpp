#include <ros/ros.h>
//#include <tf/transformlistener.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"amcl_listener");       //³õÊ¼»¯ROS½ÚµãÓë½ÚµãÃû³Æ
    ros::NodeHandle n;                         //´´½¨½ÚµãµÄ¾ä±ú
    ros::Rate loop_rate(20);                   //¿ØÖÆ½ÚµãÔËÐÐµÄÆµÂÊ,Óëloop.sleep¹²Í¬Ê¹ÓÃ

    ros::Publisher pub = n.advertise<geometry_msgs::Point>("/now_pos", 10);

    tf::StampedTransform transform;
    tf::TransformListener listener; 

    geometry_msgs::PointStamped point1;
    point1.header.stamp=ros::Time();
    point1.header.frame_id="base_link";
    point1.point.x=0;
    point1.point.y=0;
    point1.point.z=0;
    geometry_msgs::PointStamped point2;
    geometry_msgs::Point send_buffer;
    
    while (ros::ok()){
        try
        {
            listener.transformPoint("map",point1,point2);
            send_buffer.x = -point2.point.y;
            send_buffer.y = point2.point.x;
            send_buffer.z = point2.point.z;
            pub.publish(send_buffer);
            printf("x = %.2f, y = %.2f, z = %.2f\n", send_buffer.x, send_buffer.y, send_buffer.z);
            
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            loop_rate.sleep();
        }
    }

    while (n.ok())
    {

        
        try{
            listener.lookupTransform("/map", "/base_link",
                ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        float w = transform.getRotation().getW();
        float x = transform.getRotation().getX();
        float y = transform.getRotation().getY();
        float z = transform.getRotation().getZ();

        ROS_INFO("x = %.2f, y = %.2f, z = %.2f\n", x, y, z);
        
        loop_rate.sleep();
    }
}
