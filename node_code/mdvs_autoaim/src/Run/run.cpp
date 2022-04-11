#include "mdvs_autoaim/Run/run.h"
#include "mdvs_autoaim/define.h"
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point32.h>
#include "CameraApi.h" //相机SDK的API头文件

namespace rm
{
    Run::Run() : _buffer(10),
                 _bag_buffer(1000)
    {
        _startTime = std::chrono::high_resolution_clock::now();
        _Armor._startTime = std::chrono::high_resolution_clock::now();
    }

    int8_t Run::CamInit(void)
    {
        CameraSdkInit(1);

        //枚举设备，并建立设备列表
        iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);

        //没有连接设备
        if (iCameraCounts == 0)
        {
            printf("count = %d\n", iCameraCounts);
            return -1;
        }

        //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
        iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

        if (iStatus != CAMERA_STATUS_SUCCESS)
        {
            //初始化失败
            printf("state = %d\n", iStatus);
            return -1;
        }

        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera, &tCapability);

#if USE_SOFT_TRIGGER
        CameraSetTriggerMode(hCamera, 1); // 0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式
#else
        CameraSetTriggerMode(hCamera, 0);
#endif
        CameraSetTriggerCount(hCamera, 1);

        CameraSetAeState(hCamera, false);
        if (_Armor._enemy_color == RED)
        {
            CameraSetExposureTime(hCamera, 2000);
            CameraSetAnalogGain(hCamera, 64);
        }
        else
        {
            CameraSetExposureTime(hCamera, 300);
            CameraSetAnalogGain(hCamera, 64);
        }

        CameraSetGamma(hCamera, 100);
        CameraSetContrast(hCamera, 100);
        int CameraGainR = 160, CameraGainG = 130, CameraGainB = 100;
        CameraSetGain(hCamera, CameraGainR, CameraGainG, CameraGainB);

        //
        g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
        // g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

        /*让SDK进入工作模式，开始接收来自相机发送的图像
                数据。如果当前相机是触发模式，则需要接收到
                触发帧以后才会更新图像。    */
        CameraPlay(hCamera);

        /*其他的相机参数设置
                例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
                     CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
                     CameraSetGamma、CameraSetContrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
                     更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
                */

        if (tCapability.sIspCapacity.bMonoSensor)
        {
            channel = 1;
            CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
        }
        else
        {
            channel = 3;
            CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
        }
        return true;
    }

    bool Run::init()
    {
#ifdef TEST_VIDEO
        std::string filenamev = TEST_VIDEO;
        _Test.open(filenamev);
        std::cout << filenamev;
#else
        if (CamInit() != true)
            return 0;
#endif
        // camera ID: C-A&L1
        _Wind.init(3);
        _Armor.init(3);
        _Bullet.Init(0, 0, 0, 0, 0, 15, 0);
        //_Serial.openPort();

        //        while (_Serial.openPort() != Serial::SUCCESS)
        //            cv::waitKey(500);
        //_Serial.setDebug(true);
        // LOG(INFO)<<"run init success";
        return true;
    }

    void Run::producer()
    {
#ifdef TEST_VIDEO

#else
        float t1, t2, t3, t4, t5, t6;
        float time, camStartTime;
        uint8_t firstGrab = 1;

        for (;;)
        {
            if (ErrorExist > 100)
            {
                //    LOG_FIRST_N(INFO, 1)<<"produce error";
                return;
            }
#if !USE_SOFT_TRIGGER
            t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
#endif
            if (CameraGetImageBufferPriority(hCamera, &sFrameInfo, &pbyBuffer, 20, CAMERA_GET_IMAGE_PRIORITY_NEWEST) == CAMERA_STATUS_SUCCESS)
            {
#if USE_SOFT_TRIGGER
                // use SoftTrigger
                CameraSoftTrigger(hCamera);
                t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
#endif

                if (firstGrab)
                    camStartTime = sFrameInfo.uiTimeStamp / 10.0f;
                firstGrab = 0;
                // Log(4, "CAM dt: " << ((float)sFrameInfo.uiTimeStamp) / 10.0f - time << " ms       ");
                // std::cout<< ((float)sFrameInfo.uiTimeStamp) / 10.0f - time << std::endl;
                time = sFrameInfo.uiTimeStamp / 10.0f;
                t2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
                t3 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                cv::Mat matImage(
                    cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer);
                t4 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

                _mutex.lock();
                imgF.FrameValid = true;
                imgF.img = matImage;
                imgF.timeStamp = t1;
                imgF.CAM_timeStamp = sFrameInfo.uiTimeStamp / 10.0f - camStartTime;
                _mutex.unlock();

                //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
                //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
                CameraReleaseImageBuffer(hCamera, pbyBuffer);
                t5 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            }
            else
            {
#if USE_SOFT_TRIGGER
                // use SoftTrigger
                CameraSoftTrigger(hCamera);
#endif
            }

            /*do
            {
                g_pRgbBuffer = CameraGetImageBufferPriorityEx(hCamera,&sFrameInfo.iWidth,&sFrameInfo.iHeight,1000,CAMERA_GET_IMAGE_PRIORITY_NEWEST);
            }while(g_pRgbBuffer == 0);
            t2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            t3 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            cv::Mat matImage(
                        cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),
                        sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                        g_pRgbBuffer
                        );
            t4 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            imgF.FrameValid = true;
            imgF.img = matImage;
            imgF.timeStamp = t4;
            t5 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            */

#if (LOGG)
            _mutex_Log.lock();
            // std::cout<<1000.0f/(static_cast<std::chrono::duration<double, std::milli>>(t5 - t1)).count()<<std::endl;
            Log(1, "ImgRead cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t5 - t1)).count() << " ms   ");
            Log(2, "FPS: " << 1000.0f / ((static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() - t6) << " ms");
            t6 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            _mutex_Log.unlock();
            continue;
            Log(1, "CameraGetImageBuffer period: " << t2 - t1 << " ms");
            //      Log(2, "CameraImageProcess period: " << t3 - t2 << " ms");
            //   Log(3, "matImage period: " << t4 - t3 << " ms");
            //     Log(4, "ReleaseImageBuffer period: " << t5 - t4 << " ms");
            Log(5, "FPS: " << 1000.0f / (static_cast<std::chrono::duration<double, std::milli>>(t5 - t1)).count() << " ms");
#endif
        }
#endif
    }

    void Run::consumer()
    {
        cv::Point3f Point;
        cv::Point2f P2D;
        Bag_Frame Bagdata;
        float LastImgTimestamp;
        float totalCost;
        float t_start, t_end;
        float t_startGrab, t_endGrab;
        uint64_t count = 0;
        uint64_t loopCount = 0;
        uint8_t flaggg = 0;
        for (;;)
        {
            if (ErrorExist > 100)
            {
                // LOG_FIRST_N(INFO, 1)<<"consumer error";
                return;
            }
#ifdef TEST_VIDEO
            int keyASCII;
            keyASCII = cv::waitKey(1);
            if (keyASCII == -1 && flaggg == 1)
            {
                continue;
            }
            flaggg = 1;
            float t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            double rate = _Test.get(CV_CAP_PROP_FPS);
            imgF.FrameValid = true;
            _Test >> imgF.img;
            if (imgF.img.empty())
            { //如果某帧为空则退出循环
                std::cout << "vedio empty" << std::endl;
                continue;
            }
            imgF.CAM_timeStamp = loopCount;
            imgF.timeStamp = t1;
            loopCount++;

            if (keyASCII >= 49 && keyASCII <= 57)
            {
                count++;
                for (int i = 0; i < rate * 60 * (keyASCII - 48); i++)
                {
                    _Test >> imgF.img;
                }
                std::cout << "jump" << keyASCII - 48 << "mins" << count << std::endl;
            }

#endif
            t_start = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            if (imgF.FrameValid == true && imgF.img.cols != 0)
            {
                _mutex.lock();
                imgF.FrameValid = false;
                FrameTimeStamp = imgF.timeStamp;
                camTimeStamp = imgF.CAM_timeStamp;
                _mutex.unlock();
                // cv::waitKey(5);
            }
            else
            {
                continue;
            }

            if (_MODE == AUTO_AIM)
            {
                _Armor.ImageFrame.timeStamp = FrameTimeStamp;
                _Armor.ImageFrame.CAM_timeStamp = camTimeStamp;
                _mutex.lock();
                float t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                _Armor.ImageFrame.img = imgF.img.clone();
                float t2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                // Log(8,"clone cost:"<<t2-t1<<"         ");
                _mutex.unlock();
                if (_Armor.feed(_Armor.ImageFrame, 0, 0, Point))
                {
                    _Contraldata.flg = 1;
#if USE_LIGHTBAR_P4P_POINTS
                    _Contraldata.x = float_rounding(Point.x);
                    _Contraldata.y = float_rounding(Point.y);
                    _Contraldata.z = float_rounding(Point.z);
                    _Contraldata.reserve1 = float_rounding(_Armor._solve.LightBarLenRatio * 1000.0f);
                    _Contraldata.reserve2 = _Armor._solve.AromrType;

#else
 float now_time = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                
                    my_new_controlframe.linear.x = float_rounding(Point.x);
                    my_new_controlframe.linear.y = float_rounding(Point.y);
                    my_new_controlframe.linear.z = float_rounding(Point.z);
                    my_new_controlframe.angular.x = float_rounding(_Armor._solve.LightBarLenRatio * 1000.0f);
                    my_new_controlframe.angular.y = _Armor._solve.AromrType;
                    my_new_controlframe.angular.z = (now_time - _Armor.ImageFrame.timeStamp);
                    need_to_publish = 1;
                    //printf("need to publish = 1");
                    _Contraldata.x = float_rounding(Point.x);
                    _Contraldata.y = float_rounding(Point.y);
                    _Contraldata.z = float_rounding(Point.z);
                    _Contraldata.reserve1 = float_rounding(_Armor._solve.LightBarLenRatio * 1000.0f);
                    _Contraldata.reserve2 = _Armor._solve.AromrType;
#endif
#if (LOGG)
                    _mutex_Log.lock();

                    Log(4, "Position x: " << _Contraldata.x << "     "); // right
                    Log(5, "Position y: " << _Contraldata.y << "     "); // front
                    Log(6, "Position z: " << _Contraldata.z << "     "); // up

                    _mutex_Log.unlock();
#endif
                    //_Contraldata.time_stamp = float_rounding(imgF.timeStamp * 10);
                    _sendflg = true;
                }
                else
                {
                    _Contraldata.flg = 0;
                    _sendflg = true;
                    if (_Armor.sequence > 0)
                    {
                    }
                }
            }
            else if (_MODE != SUSPEND)
            {
                cv::Point3f Tgt;
                if (_Wind.wind(_Armor.ImageFrame, 7 / _Bullet.init_v_, Tgt, _Armor._enemy_color, _MODE))
                {
                    _Contraldata.flg = 2;
                    _Contraldata.x = Tgt.x;
                    _Contraldata.y = Tgt.y;
                    _Contraldata.z = Tgt.z;
                    totalCost = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() - _Armor.ImageFrame.timeStamp;
                    //_Contraldata.time_stamp = float_rounding(imgF.timeStamp * 10);
                    _Contraldata.time_stamp = unsigned_float_rounding(totalCost * 10);
                    _sendflg = true;
                }
                else
                {
                    _Contraldata.flg = 0;
                    _sendflg = true;
                }
            }
            t_end = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

#if (LOGG)
            _mutex_Log.lock();
            // Log(1, "ImgRead cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t_startConsume - t_startGrab)).count() << " ms   ");
            // Log(2, "Consume cost: " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t_startConsume)).count() << " ms   ");
            Log(3, "thread period: " << t_end - t_start << " ms   ");
            _mutex_Log.unlock();
#endif
            LastImgTimestamp = FrameTimeStamp;
        }
    }

    void Run::recorder()
    {
        time_t nowtime;
        time(&nowtime);
        struct tm *t = localtime(&nowtime);
        cv::VideoWriter writer;
        // Frame frame;
        bool isRecording = false;
        std::string datatime = to_string(t->tm_mon + 1) + "-" + to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + ":" + to_string(t->tm_min) + ":" + to_string(t->tm_sec);
        const std::string fileName = "../HEU_Vision_2020_MDVS/video/" + datatime + "expo-" + std::to_string(_EXPO) + ".avi";
        while (imgF.img.empty())
        {
            cv::waitKey(100);
        }
        //'D', 'I', 'V', 'X' 也比较小
        writer.open(fileName, cv::CAP_ANY, CV_FOURCC('M', 'P', '4', '2'), 5, imgF.img.size(), true);
        std::cout << "Start capture. " + fileName + " created." << std::endl;

        auto t1 = std::chrono::high_resolution_clock::now();

        for (;;)
        {
            if (ErrorExist > 100)
            {
                // LOG_FIRST_N(INFO, 1)<<"recorder error";
                return;
            }

            //            if (!_buffer.getLatest(frame, true))
            //            {
            //                continue;
            //            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            if (isRecording)
            {
                _mutex_Log.lock();
                float tx = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                writer << imgF.img;
                float t9 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                Log(5, t9 - tx << "      ");
                _mutex_Log.unlock();
            }
            if (!writer.isOpened())
            {
                std::cout << "Capture failed." << std::endl;
                continue;
            }
            isRecording = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            auto t2 = std::chrono::high_resolution_clock::now();
#if (LOGG)
            _mutex_Log.lock();
            Log(3, "recorder period: " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms");
            _mutex_Log.unlock();
#endif
            t1 = t2;
        }
    }

    void Run::listener()
    {
        Serial::FeedBackFrame FDB;
        Serial::ControlFrame CT;
        uint32_t lastPCt, LastSTMt;
        uint32_t ExpCount = 0;
        static uint8_t lastTeam = INVALID_COLOR;

        Bag_Frame b;
        for (;;)
        {
            if (ErrorExist > 100)
            {
                // LOG_FIRST_N(INFO, 1)<<"listener error";
                return;
            }

            if (_Armor.TrackingTarget == 1 && STM32_Online)
                continue;
            // Log(9, "T=" << (uint32_t)(static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() << "  " << FDB.reserve);

            if (_Serial.tryFeedBack(FDB, std::chrono::milliseconds(3)) == Serial::SUCCESS)
            {
                if (FDB.EOF != 0x88)
                    continue;

                STM32_Online = true;

                Log(10, "recive data" << (uint32_t)(static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count());
                //_mutex.lock();
                b.pitch = (float)FDB.pitch / 10000;
                b.yaw = (float)FDB.yaw / 10000;
                b.time_stamp = FDB.time_stamp;
                _bag_buffer.push(b);
                Log(15, "myteam" << FDB.myteam);
                if (FDB.myteam == BLUE && FDB.myteam != INVALID_COLOR)
                {
                    _Armor._enemy_color = RED;
                    Log(7, "HIT RED          ");
                }
                else
                {
                    _Armor._enemy_color = BLUE;
                    Log(7, "HIT BLUE          ");
                }
                lastTeam = FDB.myteam;

                _MODE = (rm::Mode)FDB.mode;

                if (FDB.reserve == 6)
                {
                    while (FDB.reserve != 4)
                    {
                        CT.reserve1 = 1;
                        CT.time_stamp = unsigned_float_rounding((static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count());
                        _Serial.tryControl(CT, std::chrono::milliseconds(3));

                        if (_Serial.tryFeedBack(FDB, std::chrono::milliseconds(3)) != Serial::SUCCESS)
                            continue;
                        CT.time_stamp = unsigned_float_rounding((static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count());
                        CT.reserve1 = 3;
                        if (FDB.reserve == 2)
                        {
                            _Serial.tryControl(CT, std::chrono::milliseconds(3));
                        }
                        else
                            continue;

                        if (_Serial.tryFeedBack(FDB, std::chrono::milliseconds(3)) != Serial::SUCCESS)
                            continue;
                        if (FDB.reserve == 4)
                        {
                            _transmit_time = FDB.time_stamp;
                            break;
                        }
                    }
                }
            }
            else
            {
                // LOG_EVERY_N(INFO, 1000)<<"receive failed"<< google::COUNTER <<"times";
                Log(18, "receive failed" << (uint32_t)(static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void Run::publisher()
    {
        char debugData[100];
        uint32_t size = 0;
        for (;;)
        {
            if (ErrorExist > 100)
            {
                // LOG_FIRST_N(INFO, 1)<<"publisher error";
                return;
            }
#if DEBUG_PLOT
            if (_sendflg == true)
            {
                if (_Contraldata.z > 8000)
                    _Contraldata.z = 8000;
                memset(debugData, 0, 100);
                sprintf(debugData, "a=%d,b=%d,c=%d\n", _Contraldata.x, _Contraldata.y, _Contraldata.z);
                for (int i = 99; i >= 0; i--)
                {
                    if (debugData[i] == '\n')
                    {
                        size = i + 1;
                        break;
                    }
                }
                _Serial.tryDebug(debugData, size);
                _sendflg = false;
            }
#else
            if (_sendflg == true && STM32_Online)
            {
                _Contraldata.time_stamp = float_rounding(((static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() - FrameTimeStamp) * 10);
                // std::cout<<"dt: "<<_Contraldata.time_stamp<<std::endl;
                // std::cout<<"serial send period " << (static_cast<std::chrono::duration<double, std::milli>>(t1 - t2)).count()<<std::endl;
                while (_Serial.tryControl(_Contraldata, std::chrono::milliseconds(3)) != Serial::SUCCESS)
                {
                }
                _sendflg = false;
            }
#endif
        }
    }

    int32_t Run::float_rounding(float raw)
    {
        static int integer;
        static float decimal;
        integer = (int)raw;
        decimal = raw - integer;
        if (decimal > 0.5f)
            integer++;
        return integer;
    }

    uint32_t Run::unsigned_float_rounding(float raw)
    {
        static uint32_t integer;
        static float decimal;
        if (raw < 0)
            return 0;
        integer = (uint32_t)raw;
        decimal = raw - integer;
        if (decimal > 0.5f)
            integer++;
        return integer;
    }

} // namespace rm
