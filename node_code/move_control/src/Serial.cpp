
/**************************************************************
MIT License
Copyright (c) 2018 SEU-SuperNova-CVRA
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Authors:    Binyan Hu
**************************************************************/
#include "move_control/Serial.h"

#include <fcntl.h>
#ifdef WIN32
 #include <windows.h>
 #else
 #include <unistd.h>
 #endif
#include <termios.h>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <stdexcept>
#include <exception>

using namespace std;

namespace rm
{
    Serial::Serial() : _serialFd(-1),
                       _errorCode(SUCCESS),
                       _en_debug(false)
    {
        //static_assert(sizeof(ControlFrame) == 16, "Size of backdata is not 16");
        //static_assert(sizeof(FeedBackFrame) == , "Size of backdata is not be 16");

        // 初始化通讯帧结构体通用项
        _ControlData.SOF = CommSOF;
        _ControlData.EOF = CommEOF;

        //    if(ob[i].track.size()<20)
        //    ob[i].track.push_back(rect.center);
        //    else
        //    {
        //        ob[i].track.erase(ob[i].track.begin());
        //        ob[i].track.push_back(rect.center);
        //    }
    }

    Serial::~Serial()
    {
        tcflush(_serialFd, TCIOFLUSH);
        if (-1 == close(_serialFd))
        {
            _errorCode = SYSTEM_ERROR;
            cout << "Serial closing  failed." << endl;
        }
        else
        {
            _errorCode = SUCCESS;
        }
    }

    int Serial::openPort()
    {
        _serialFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (_serialFd == -1)
        {
            cout << "Open serial port failed." << endl;
            return _errorCode = SYSTEM_ERROR;
            ;
        }

        termios tOption;                // 串口配置结构体
        tcgetattr(_serialFd, &tOption); //获取当前设置
        cfmakeraw(&tOption);
        cfsetispeed(&tOption, B460800); // 接收波特率
        cfsetospeed(&tOption, B460800); // 发送波特率
        tcsetattr(_serialFd, TCSANOW, &tOption);
        tOption.c_cflag &= ~PARENB;
        tOption.c_cflag &= ~CSTOPB;
        tOption.c_cflag &= ~CSIZE;
        tOption.c_cflag |= CS8;
        tOption.c_cflag &= ~INPCK;
        tOption.c_cflag |= (B460800 | CLOCAL | CREAD); // 设置波特率，本地连接，接收使能
        tOption.c_cflag &= ~(INLCR | ICRNL);
        tOption.c_cflag &= ~(IXON);
        tOption.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tOption.c_oflag &= ~OPOST;
        tOption.c_oflag &= ~(ONLCR | OCRNL);
        tOption.c_iflag &= ~(ICRNL | INLCR);
        tOption.c_iflag &= ~(IXON | IXOFF | IXANY);
        tOption.c_cc[VTIME] = 1; //只有设置为阻塞时这两个参数才有效
        tOption.c_cc[VMIN] = 1;
        tcflush(_serialFd, TCIOFLUSH); //TCIOFLUSH刷新输入、输出队列。

        cout << "Serial preparation complete." << endl;
        return _errorCode = SUCCESS;
    }

    int Serial::closePort()
    {
        tcflush(_serialFd, TCIOFLUSH);
        if (-1 == close(_serialFd))
        {
            _errorCode = SYSTEM_ERROR;
            cout << "Serial closing failed." << endl;
        }
        else
        {
            _errorCode = SUCCESS;
        }
        return _errorCode;
    }

    bool Serial::isOpened() const
    {
        return (_serialFd != -1);
    }

    void Serial::setDebug(bool en_debug)
    {
        _en_debug = en_debug;
    }

    //int Serial::setup(int& self_color)
    //{
    //    if(_en_debug)
    //    {
    //        cout << "[setup]\n";
    //    }

    //    ControlFrame Cdata;
    //    Cdata.reserve1=1;
    //    Cdata.time_stamp=
    //    if(tryControl(Cdata,std::chrono::milliseconds(3))==Serial::SUCCESS) ;
    //    else
    //    {
    //        return TIME_OUT;
    //    }

    ////            Serial::FeedBackFrame Fdata;
    ////            for(;;){

    ////                if(_Serial.tryFeedBack(Fdata,std::chrono::milliseconds(3))==Serial::SUCCESS)
    ////                {

    //     TeamName self_team;
    //     if(receive() == SUCCESS)
    //     {

    //     }

    ////    //_controlFrame.shoot_mode = SET_UP;
    ////    //if(send() == SUCCESS)
    ////    {
    ////        if(receive() == SUCCESS)
    ////        {
    ////            //cout<<"receive"<<_feedBackFrame.myteam<<endl;
    ////            tmp_self_team = _feedBackFrame.myteam;
    ////            if(tmp_self_team != BLUE_TEAM && tmp_self_team != RED_TEAM)
    ////            {
    ////                return _errorCode = CORRUPTED_FRAME;
    ////            }

    ////            //_controlFrame.shoot_mode = tmp_self_team;
    ////            if(send() == SUCCESS)
    ////            {
    ////                if(tmp_self_team == BLUE_TEAM)
    ////                {
    ////                    //self_color = rm::BLUE;
    ////                }
    ////                else if(tmp_self_team == RED_TEAM)
    ////                {
    ////                    //self_color = rm::RED;
    ////                }
    ////            }
    ////        }
    ////    }
    //    return _errorCode;
    //}

    int Serial::control(const ControlFrame &controlData)
    {
        if (_en_debug)
        {
            cout << "[control]\n";
        }
        _ControlData = controlData;
        _ControlData.SOF = CommSOF;
        _ControlData.EOF = CommEOF;
        return send();
    }

    int Serial::feedBack(FeedBackFrame &feedBackData)
    {
        if (_en_debug)
        {
            cout << "[request]\n";
        }

        if (receive() == SUCCESS)
        {
            feedBackData = _FeedBackData;
        }
        return _errorCode;
    }

    int Serial::getErrorCode() const
    {
        return _errorCode;
    }

    //void Serial::print(const ControlFrame &ct)
    //{
    //    cout<<hex<<(unsigned int)ct.SOF<<endl;
    //    cout<<dec<<(unsigned int)ct.frame_seq<<endl;
    //    cout<<hex<<(unsigned int)ct.shoot_mode<<endl;
    //    cout<<dec<<              ct.pitch_dev<<endl;
    //    cout<<dec<<              ct.yaw_dev<<endl;
    //    cout<<dec<<         (int)ct.rail_speed<<endl;
    //    cout<<hex<<(unsigned int)ct.gimbal_mode<<endl;
    //    cout<<hex<<(unsigned int)ct.EOF<<endl;
    //}

    //void Serial::print(const FeedBackFrame &fb)
    //{
    //    _mutex.lock();
    //    Log(10,"SOF"<<fb.SOF);
    //    Log(11,"myteam"<<fb.myteam);
    //    Log(12,"pitch"<<fb.pitch);
    //    Log(13,"yaw"<<fb.yaw);
    //    Log(14,"bullet_speed"<<fb.bullet_speed);
    //    Log(15,"time_stamp"<<fb.time_stamp);
    //    Log(16,"mode"<<fb.mode);
    //    Log(17,"EOF"<<fb.EOF);
    //    _mutex.unlock();
    //}

    int Serial::send()
    {
        tcflush(_serialFd, TCOFLUSH);

        int sendCount;
        try
        {
            sendCount = write(_serialFd, &_ControlData, sizeof(ControlFrame));
        }
        catch (exception e)
        {
            cout << e.what() << endl;
            return _errorCode = SYSTEM_ERROR;
        }

        if (sendCount == -1)
        {
            if (_en_debug)
            {
                cout << "\tSerial sending failed. Frame sequence: " << endl;
            }
            _errorCode = READ_WRITE_ERROR;
        }
        else if (sendCount < static_cast<int>(sizeof(ControlFrame)))
        {
            if (_en_debug)
            {
                cout << "\tSerial sending failed. " << sizeof(ControlFrame) - sendCount << " bytes unsent. Frame sequence: " << endl;
            }
            _errorCode = READ_WRITE_ERROR;
        }
        else
        {
            if (_en_debug)
            {
                cout << "\tSerial sending succeeded. "
                     << "Frame sequence: " << endl;
            }
            _errorCode = SUCCESS;
        }

        return _errorCode;
    }

    int Serial::receive()
    {
        memset(&_FeedBackData, 0, sizeof(FeedBackFrame)); //clean

        int readCount = 0;
        const auto t1 = std::chrono::high_resolution_clock::now();
        while (readCount < int(sizeof(FeedBackFrame)))
        {

            auto t2 = std::chrono::high_resolution_clock::now();

            if ((chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)).count() > 10) // Time limit is 10ms
            {
                if (_en_debug)
                {
                    cout << "\tReceiving time out. " << sizeof(FeedBackFrame) - readCount
                         << " bytes not received." << endl;
                }
                return _errorCode = TIME_OUT;
            }

            int onceReadCount;
            try
            {
                onceReadCount = read(_serialFd, ((unsigned char *)(&_FeedBackData)) + readCount, sizeof(FeedBackFrame) - readCount);
            }
            catch (exception e)
            {
                cout << e.what() << endl;
                return _errorCode = SYSTEM_ERROR;
            }

            if (onceReadCount == -1)
            {
                if (errno == EAGAIN)
                {
                    continue;
                }

                if (_en_debug)
                {
                    cout << "\tRead data from serial failed." << endl;
                }
                return _errorCode = READ_WRITE_ERROR;
            }

            readCount += onceReadCount;
        }

        tcflush(_serialFd, TCIFLUSH);

        if (_FeedBackData.SOF != CommSOF || _FeedBackData.EOF != CommEOF)
        {
            if (_en_debug)
            {
                cout << "\tFeed back frame SOF or EOF is not correct. SOF: " << (int)_FeedBackData.SOF << " ,EOF: " << (int)_FeedBackData.EOF << endl;
            }
            return _errorCode = CORRUPTED_FRAME;
        }
        else
        {
            if (_en_debug)
            {
                cout << "\tSerial receiving succeeded. " << endl;
            }
            return _errorCode = SUCCESS;
        }
    }

} // namespace rm
