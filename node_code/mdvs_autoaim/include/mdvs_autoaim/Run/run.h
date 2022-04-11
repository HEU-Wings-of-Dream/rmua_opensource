#pragma once

#include <opencv2/opencv.hpp>

#include <chrono>
#include <mutex>
#include <memory>
#include <thread>

#include "CameraApi.h" //相机SDK的API头文件
//#include "mdvs_autoaim/Others/Camera/RMVideoCapture.hpp"
#include "mdvs_autoaim/Armor/armor.h"
#include "mdvs_autoaim/Rune/Wind.hpp"
#include "mdvs_autoaim/Others/DataFrame/data_frame.h"
#include "mdvs_autoaim/Others/Trajectory/trajectory.h"
#include "mdvs_autoaim/Serials/Serial.h"
#include <ros/ros.h>
#include <geometry_msgs/Accel.h>

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
        int8_t CamInit(void);
        int32_t float_rounding(float raw);
        uint32_t unsigned_float_rounding(float raw);

        bool need_to_publish = 0;
        ros::Rate my_rate = ros::Rate(100);
        geometry_msgs::Accel my_new_controlframe;

        Frame imgF;

        Mode _MODE = AUTO_AIM; // AUTO_AIM HIT_RUNE_MIN HIT_RUNE_MAX;
        int _EXPO = 0;
        int _FPS = 330;
        int _WIDTH = 640;
        int _HEIGHT = 480;

        float _transmit_time = 0;
        Serial::ControlFrame _Contraldata;
        bool _sendflg = false;
        bool STM32_Online = false;

        uint32_t ErrorExist = 0;

        float FrameTimeStamp;
        float camTimeStamp;

        int iCameraCounts = 1;
        int iStatus = -1;
        tSdkCameraDevInfo tCameraEnumList;
        int hCamera;
        tSdkCameraCapbility tCapability; //设备描述信息
        tSdkFrameHead sFrameInfo;
        BYTE *pbyBuffer;
        int iDisplayFrames = 10000;
        IplImage *iplImage = NULL;
        int channel = 3;
        unsigned char *g_pRgbBuffer; //处理后数据缓存区

    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;

        std::timed_mutex _mutex;
        std::timed_mutex _mutex_Log;

        FrameBuffer _buffer;
        Bag_FrameBuffer _bag_buffer;
        // std::vector <FeedBackFrame> _FeedBackData_Buffer;
        Result_Show RS;
        //RMVideoCapture _RMVideoCapture;
        cv::VideoCapture _Test;

        Armor _Armor;
        Wind _Wind;
        GimbalContrl _Bullet;
        Serial _Serial;
    };

} // namespace rm
