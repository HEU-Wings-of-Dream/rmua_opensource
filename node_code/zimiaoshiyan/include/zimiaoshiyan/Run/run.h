#pragma once

#include <opencv2/opencv.hpp>

#include <chrono>
#include <mutex>
#include <memory>
#include <thread>

#include "zimiaoshiyan/Others/Camera/RMVideoCapture.hpp"
#include "zimiaoshiyan/Armor/armor.h"
#include "zimiaoshiyan/Rune/Wind.hpp"
#include "zimiaoshiyan/Others/DataFrame/data_frame.h"
#include "zimiaoshiyan/Others/Trajectory/trajectory.h"
#include "zimiaoshiyan/Serials/Serial.h"

#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <ros/publisher.h>

namespace rm
{
    
    class Run
    {
    public:
        Run();
        ~Run() {}

        bool init();
        void producer();
        void consumer();
        void recorder();
        void listener();
        void publisher();
        int32_t float_rounding(float raw);
        uint32_t unsigned_float_rounding(float raw);

       // ros::Publisher auto_aim_publisher;

        bool need_to_publish = 0;

        geometry_msgs::Accel my_new_controlframe;

        ros::Rate my_rate = ros::Rate(100);

        //void auto_aim_publisher_thread();

        Frame imgF;


        Mode _MODE = AUTO_AIM; //AUTO_AIM HIT_RUNE_MIN HIT_RUNE_MAX;
        int _EXPO = 0;
        int _FPS = 330;

        float _transmit_time = 0;
        Serial::ControlFrame _Contraldata;
        bool _sendflg = false;
        bool STM32_Online = false;

        uint32_t ErrorExist = false;

        float FrameTimeStamp;
        float camTimeStamp;

    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;

        std::timed_mutex _mutex;
        std::timed_mutex _mutex_Log;

        FrameBuffer _buffer;
        Bag_FrameBuffer _bag_buffer;
        //std::vector <FeedBackFrame> _FeedBackData_Buffer;

        RMVideoCapture _RMVideoCapture;
        cv::VideoCapture _Test;

        Armor _Armor;
        Wind _Wind;
        GimbalContrl _Bullet;
        Serial _Serial;
    };

} // namespace rm
