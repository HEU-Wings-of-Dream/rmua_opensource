#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <mutex>
#include <chrono>
#include <thread>

/* Serial frame's EOF may conflict with somewhere else's EOF */
#undef EOF

namespace rm
{

    /*
 * @Brief:  Serial communication protocol and inplement
 */
    class Serial
    {
    public:
        /*
     * @Brief: 控制战车帧结构体
     */
        struct ControlFrame
        {
            uint8_t SOF;
            uint8_t flg;
            int16_t x; //mm
            uint16_t y;
            int16_t z;
            uint32_t time_stamp;
            short reserve1;     // LightBar len ratio
            uint8_t reserve2;   // Target type
            uint8_t EOF;
        };

        struct DebugFrame
        {
            uint8_t data[100];
        };

        /*
     * @Brief: 战车回传数据帧结构体
     */
        struct FeedBackFrame
        {
            uint8_t SOF;
            uint8_t myteam; //  EE DD
            short pitch;    //  10k*rad
            short yaw;
            uint16_t bullet_speed; //  m/s
            uint32_t time_stamp;   //ms
            short reserve;
            uint8_t mode;
            uint8_t EOF;
        };

        /*
     * @Brief: 比赛红蓝方
     */
        enum TeamName
        {
            TEAM_BLUE = (uint8_t)0xDD,
            TEAM_RED = (uint8_t)0xEE
        };

        /*
     * @Breif:所需控制模式
     */
        enum TaskMode
        {
            NO_TASK = (uint8_t)(0x00),    //手动控制
            SMALL_BUFF = (uint8_t)(0x01), //小符模式
            BIG_BUFF = (uint8_t)(0x02),   //大符模式
            AUTO_SHOOT = (uint8_t)(0x03)  //自动射击
        };

        enum ErrorCode
        {
            SYSTEM_ERROR = 1,
            SUCCESS = 0,
            PORT_OCCUPIED = -1,
            READ_WRITE_ERROR = -2,
            CORRUPTED_FRAME = -3,
            TIME_OUT = -4
        };

    public:
        Serial();
        Serial(const Serial &right) = delete;
        Serial(Serial &&) = delete;
        ~Serial();
        /* @See: 'enum ErrorCode' */
        int _errorCode = SUCCESS;

        /*
     * @Brief: Open serial port and config options
     */
        int openPort();

        /*
     * @Brief:  close serial port
     */
        int closePort();

        /*
     * @Brief
     */
        bool isOpened() const;

        /*
     * @Brief: std::cout detailed states and information when en_debug is on.
     */
        void setDebug(const bool en_debug);

        /*
     * @Brief:  Set up communication with STM
     * @Output: self_team: BLUE_TEAM or RED_TEAM
     */
        int setup(int &self_color);

        /*
     * @Brief:  控制机器人
     * @Input:  time_duration:  try for how long to send control data
     */
        template <typename _Rep, typename _Period>
        int tryControl(const ControlFrame &controlData, const std::chrono::duration<_Rep, _Period> &time_duration)
        {
            //        std::unique_lock<std::mutex> lockGuard(_mutex1/*, time_duration*/);
            std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
            if (lockGuard.owns_lock())
            {
                control(controlData);
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
                return _errorCode;
            }
            else
            {
                if (_en_debug)
                    std::cout<<"PORT_OCCUPIED"<<std::endl;
                return PORT_OCCUPIED;
            }
        }

        /*
     * @Brief:  控制机器人
     * @Input:  time_duration:  try for how long to send control data
     */
        int tryDebug(char* data, uint32_t size);

        /*
     * @Brief:  从机器人接收信息
     * @Input:  time_duration:  try for how long to receive feedback data
     */
        template <typename _Rep, typename _Period>
        int tryFeedBack(FeedBackFrame &feedBackData, const std::chrono::duration<_Rep, _Period> &time_duration)
        {
            //        std::unique_lock<std::mutex> lockGuard(_mutex1/*, time_duration*/);
            std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
            if (lockGuard.owns_lock())
            {
                feedBack(feedBackData);
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
                return _errorCode;
            }
            else
            {
                return PORT_OCCUPIED;
            }
        }

        /*
     * @Brief:  DO NOT call this right after the record, control or feedback because
     *          '_errorCode' will not be thread safe after they return, which means
     *          that you may not get the exact error code representing the operation state.
     *          Call this function in the case which does not worry about thread safety.
     */
        int getErrorCode() const;

    private:
        /*
     * @Brief: 帧头帧尾
     */
        enum
        {
            CommSOF = (uint8_t)0x66,
            CommEOF = (uint8_t)0x88
        };

    private:
        /* -1 if serial port not opened */
        int _serialFd;

        /* 输出详细的记录至控制台 if true */
        bool _en_debug;

        ControlFrame _ControlData;
        FeedBackFrame _FeedBackData;

        //    std::mutex _mutex1;
        std::timed_mutex _mutex;

        //    std::ostream& operator<<(ostream& os, const ControlFrame& cf);
        //    void print(const ControlFrame& ct);
        //    void print(const FeedBackFrame& fb);

        int control(const ControlFrame &controlData);
        int feedBack(FeedBackFrame &feedBackData);

        int send();
        int receive();
    };

} // namespace rm
