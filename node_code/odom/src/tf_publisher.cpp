#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_tf_publisher");
    ros::NodeHandle nodehandle_obj;

    tf::TransformBroadcaster transformbroadcaster;

    tf::TransformBroadcaster transformbroadcaster2;

    ros::Rate rate(100);//Attention!! The dimension is Hz!!

    while (ros::ok())
    {
//        transformbroadcaster.sendTransform(
//            tf::StampedTransform(
//                tf::Transform(tf::Quaternion(1,0,0,0), tf::Vector3(0,0,0)),
//                ros::Time::now(), "laser", "base_link")
//        );
        transformbroadcaster2.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(1,0,0,0), tf::Vector3(0,0,0)),
                ros::Time::now(), "base_link", "laser")
        );
        rate.sleep();

        ROS_INFO("Sent tf!");
    }
}

/*
odometry.cpp publish tf: odom -> base_link
tf_publishr.cpp publish tf: base_laser -> base_link
                            base_link -> base_laser
*/
