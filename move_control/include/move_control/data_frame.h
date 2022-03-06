#pragma once

#include<opencv2/opencv.hpp>
#include "Serial.h"

#include<chrono>
#include<mutex>
#include<memory>


namespace rm
{
    struct Frame
    {
        bool FrameValid = false;
        cv::Mat img;
        float timeStamp;	//time in ms, from initialization to now
        double CAM_timeStamp;
    };

    class FrameBuffer
    {
    public:
        FrameBuffer(size_t size);

        ~FrameBuffer() = default;

        bool push(const Frame& frame);

        bool getLatest(Frame& frame,bool getvid=false);

    private:
        std::vector<Frame> _frames;

        std::vector<std::timed_mutex> _mutexs;
        std::vector<std::mutex> _mutexs1;

        size_t _tailIdx;
        size_t _headIdx;

        double _lastGetTimeStamp;
    };


    struct Bag_Frame
    {
        float    pitch=0;
        float    yaw=0;
        float    time_stamp=0;

    };
    class Bag_FrameBuffer
    {
    public:
        Bag_FrameBuffer(size_t size);

        ~Bag_FrameBuffer() = default;

        bool push(const Bag_Frame& frame);

        bool getLatest(Bag_Frame& frame,bool getvid=false);

        bool getClosest(Bag_Frame& frame,float t);
    private:

        std::vector<Bag_Frame> _Bag_Frames;
        size_t _size;
    };






    enum ColorChannels
    {
        INVALID_COLOR = 0,
        BLUE = 1,
        RED = 2
    };

    enum ObjectType
    {
        UNKNOWN_ARMOR = 0,
        SMALL_ARMOR = 1,
        BIG_ARMOR = 2,
        RUNE_WING = 3,
        RUNE_CENTRE = 4
    };

    enum Mode
    {
        SUSPEND = 0,
        AUTO_AIM,
        HIT_RUNE_MIN,
        HIT_RUNE_MAX
    };


}
