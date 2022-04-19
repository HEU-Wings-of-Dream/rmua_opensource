#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point32.h>
#include "zimiaoshiyan/Run/run.h"
#include "zimiaoshiyan/define.h"

namespace rm
{
    Run::Run() : _buffer(10),
                 _bag_buffer(1000)
    {
        _startTime = std::chrono::high_resolution_clock::now();
        _Armor._startTime = std::chrono::high_resolution_clock::now();
    }

    bool Run::init()
    {

#ifdef TEST_VIDEO
        std::string filenamev = TEST_VIDEO;
        _Test.open(filenamev);
#else
        while (_RMVideoCapture.open(0, 2) != true)
        {
            cv::waitKey(1000);
        }
        _RMVideoCapture.setVideoFormat(WIDTH, HEIGHT, true);
        _RMVideoCapture.setExposureTime(50);   // blue 800    red 1000
        _RMVideoCapture.setGamma(100);
        _RMVideoCapture.startStream();
#if (LOG)
        _RMVideoCapture.info();
        _RMVideoCapture.getResolution();
#endif

#endif
        //ros::init();
        _Wind.init(2);//std::cout<<"4444444444444444444444444444"<<std::endl;
        _Armor.init(2);//std::cout<<"5555555555555555555555555"<<std::endl;
        _Bullet.Init(0, 0, 0, 0, 0, 15, 0);//std::cout<<"666666666666666666666"<<std::endl;

        //while (_Serial.openPort() != Serial::SUCCESS)
        //    ;
        _Serial.setDebug(true);
        std::cout<<"init complete!!!"<<std::endl;
        return true;
    }

    void Run::producer()
    {
        cv::Mat newImg;
        float t1, t2, t3, t4, t5, t6;
        double cam_timeStamp;
        for (;;)
        {
            //cout<<"111111111111111111111111111"<<endl;
            if (ErrorExist > 100)
                return;

            t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            if (!_RMVideoCapture.grab(&cam_timeStamp))
            {
                cout << "grab Error";
                //ErrorExist += 101;
                continue;
            }

            t2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            if (!_RMVideoCapture.retrieve(newImg))
            {
                cout << "retrieve Error";
               // std::cout<<"---=-=-=-=  "<<newImg.rows<<' '<<newImg.cols<<std::endl;
                //ErrorExist += 101;
                continue;
            }
            else
            {
                float timeStamp = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
                _mutex.lock();
                imgF.FrameValid = true;
                imgF.img = newImg;
            //    std::cout<<"---=-=-=))))  "<<newImg.rows<<' '<<newImg.cols<<std::endl;
                imgF.timeStamp = t1;
                imgF.CAM_timeStamp = cam_timeStamp;
                _mutex.unlock();
            }

            t3 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

#if (LOG)
            _mutex_Log.lock();
            Log(1, "ImgRead period: " << t3 - t1 << " ms  ");
            Log(2, "FPS: " << 1000.0f/((static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() - t4) << " ms");
            t4 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            _mutex_Log.unlock();
#endif
        }
    }

    void Run::consumer()
    {
        cv::Mat newImg;
        cv::Point3f Point;
        cv::Point2f P2D;
        Bag_Frame Bagdata;
        float LastImgTimestamp;
        float totalCost;
        float t_start, t_end, t_startConsume;

        for (;;)
        {
            if (ErrorExist > 100)
                return;

            t_start = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            if (imgF.FrameValid == true && imgF.img.cols != 0 && imgF.img.rows != 0)
            {
                _mutex.lock();
                imgF.FrameValid = false;
                FrameTimeStamp = imgF.timeStamp;
                camTimeStamp = imgF.CAM_timeStamp;
                _mutex.unlock();
            }
            else
            {
                continue;
            }
//            while (imgF.FrameValid != true)
//            ;
//            _mutex.lock();
//            imgF.FrameValid = false;
//            FrameTimeStamp = imgF.timeStamp;
//            _mutex.unlock();

            if (FrameTimeStamp - LastImgTimestamp > 3000)
            {
                ErrorExist += 101;
            }

            t_startConsume = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            if (_MODE == AUTO_AIM)
            {
                _Armor.ImageFrame.timeStamp = FrameTimeStamp;
                _Armor.ImageFrame.CAM_timeStamp = camTimeStamp;
                _mutex.lock();
                _Armor.ImageFrame.img = imgF.img.clone();
                _mutex.unlock();
                // if (_Armor.ImageFrame.img.empty() == 0){
                //     transpose(_Armor.ImageFrame.img, _Armor.ImageFrame.img);
                //     flip(_Armor.ImageFrame.img, _Armor.ImageFrame.img, 1);
                // }
                if (_Armor.feed(_Armor.ImageFrame, 0, 0, Point))
                {
                    _Contraldata.flg = 1;
#if USE_WRONG_P4P_POINTS
                    _Contraldata.x = float_rounding(Point.x / ARMOR_SIZE_SCALE);
                    _Contraldata.y = float_rounding(Point.y / ARMOR_SIZE_SCALE);
                    _Contraldata.z = float_rounding(Point.z / ARMOR_SIZE_SCALE);
                    _Contraldata.reserve1 = float_rounding(_Armor._solve.LightBarLenRatio * 1000.0f);
                    _Contraldata.reserve2 = _Armor._solve.AromrType;

#else
                    Log(4, "Position x: " << Point.x << "     "); //right
                    Log(5, "Position y: " << Point.y << "     "); //front
                    Log(6, "Position z: " << Point.z << "     "); //up

                    my_new_controlframe.linear.x = float_rounding(Point.x);
                    my_new_controlframe.linear.y = float_rounding(Point.y);
                    my_new_controlframe.linear.z = float_rounding(Point.z);
                    my_new_controlframe.angular.x = float_rounding(_Armor._solve.LightBarLenRatio * 1000.0f);
                    my_new_controlframe.angular.y = _Armor._solve.AromrType;
                    my_new_controlframe.angular.z = _Armor.ImageFrame.timeStamp;
                    // my_new_controlframe.reserve1 = float_rounding(_Armor._solve.LightBarLenRatio * 1000.0f);
                    // my_new_controlframe.reserve2 = _Armor._solve.AromrType;
                    need_to_publish = 1;
                    // _Contraldata.x = float_rounding(Point.x);
                    // _Contraldata.y = float_rounding(Point.y);
                    // _Contraldata.z = float_rounding(Point.z);
                    FrameTimeStamp = _Armor.ImageFrame.timeStamp;
                    // _Contraldata.reserve1 = float_rounding(_Armor._solve.LightBarLenRatio * 1000.0f);
                    // _Contraldata.reserve2 = _Armor._solve.AromrType;
#endif
#if (LOG)
                    _mutex_Log.lock();

                    // Log(4, "Position x: " << my_new_controlframe.linear.x << "     "); //right
                    // Log(5, "Position y: " << my_new_controlframe.linear.y << "     "); //front
                    // Log(6, "Position z: " << my_new_controlframe.linear.z << "     "); //up
                    // Log(7, "CAM Frame Period: " << _Armor.CAM_ValidImgPeriod << "     "); //up

                    
                    Log(7, "CAM Frame Period: " << _Armor.CAM_ValidImgPeriod << "     "); //up

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
                _Armor.ImageFrame.timeStamp = FrameTimeStamp;
                _Armor.ImageFrame.CAM_timeStamp = camTimeStamp;
                _mutex.lock();
                _Armor.ImageFrame.img = imgF.img.clone();
                _mutex.unlock();
                float temp_timestemp=FrameTimeStamp;
                float last_timestemp;
                cv::Point3f Tgt;
                if (_Wind.wind(_Armor.ImageFrame, 0, 7 / _Bullet.init_v_, Tgt, _Armor._enemy_color, _MODE, temp_timestemp, LastImgTimestamp))
                {
                    _Contraldata.flg = 2;
                    _Contraldata.x = Tgt.x;
                    _Contraldata.y = Tgt.y;
                    _Contraldata.z = Tgt.z;
                    totalCost = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() - _Armor.ImageFrame.timeStamp;
                    //_Contraldata.time_stamp = float_rounding(imgF.timeStamp * 10);
                    _Contraldata.time_stamp = unsigned_float_rounding(totalCost * 10);
                    _sendflg = true;
                    //last_timestemp = temp_timestemp;
                }
                else
                {
                    //last_timestemp = temp_timestemp;
                    _Contraldata.flg = 0;
                    _sendflg = true;
                }
            }
            t_end = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
#if (LOG)
            _mutex_Log.lock();
            //Log(1, "ImgRead cost: " << t_startConsume - t_start << " ms   ");
            Log(3, "Consume cost: " << t_end - t_startConsume << " ms   ");
            //Log(3, "thread period: " << t_end - t_start << " ms   ");
            _mutex_Log.unlock();
#endif

            LastImgTimestamp = FrameTimeStamp;
        }
    }

    void Run::recorder()
    {
        //        time_t t;
        //        time(&t);
        //        cv::VideoWriter writer;
        Frame frame;
        //        bool isRecording = false;
        //        const std::string fileName = "../HEU_Vision_2020/Data/" + std::to_string(t) +"_expo_"+std::to_string(_EXPO)+ ".avi";
        //        writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(_WIDTH, _HEIGHT));
        //        std::cout << "Start capture. " + fileName +" created." << std::endl;

        auto t1 = std::chrono::high_resolution_clock::now();

        for (;;)
        {
            if (ErrorExist)
                return;

            if (!_buffer.getLatest(frame, true))
            {
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            //            if(isRecording)
            //            {
            //                writer << frame.img;

            //            }
            //            if(!writer.isOpened())
            //            {
            //                std::cout << "Capture failed." << std::endl;
            //                continue;
            //            }
            //            isRecording = true;
            //std::this_thread::sleep_for(std::chrono::milliseconds(3));
            auto t2 = std::chrono::high_resolution_clock::now();
#if (LOG)
            //_mutex.lock();
            Log(3, "recorder period: " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms");
            //_mutex.unlock();
#endif
            t1 = t2;
        }
    }

    void Run::listener()
    {
        Serial::FeedBackFrame FDB;
        Serial::ControlFrame CT;
        uint32_t lastPCt, LastSTMt;
        static uint8_t lastTeam;

        Bag_Frame b;
        for (;;)
        {
            if (ErrorExist > 100)
                return;

            if (_Armor.TrackingTarget == 1 && STM32_Online)
                continue;
            //Log(9, "T=" << (uint32_t)(static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() << "  " << FDB.reserve);

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
                if (FDB.myteam == BLUE && FDB.myteam != INVALID_COLOR)
                {
                    if (lastTeam != FDB.myteam)
                        _RMVideoCapture.setExposureTime(1000);
                    _Armor._enemy_color = RED;
                    Log(14, "HIT RED     ");
                }
                else
                {
                    if (lastTeam != FDB.myteam)
                        _RMVideoCapture.setExposureTime(800);
                    _Armor._enemy_color = BLUE;
                    Log(14, "HIT BLUE     ");
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
                Log(18, "receive failed");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void Run::publisher()
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t2 = std::chrono::high_resolution_clock::now();

        for (;;)
        {
            if (ErrorExist > 100)
                return;

            if (_sendflg == true && STM32_Online)
            {
                t1 = std::chrono::high_resolution_clock::now();
                _Contraldata.time_stamp = float_rounding(((static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() - FrameTimeStamp)*10);
                //std::cout<<"dt: "<<_Contraldata.time_stamp<<std::endl;
                //std::cout<<"serial send period " << (static_cast<std::chrono::duration<double, std::milli>>(t1 - t2)).count()<<std::endl;
                while (_Serial.tryControl(_Contraldata, std::chrono::milliseconds(3)) != Serial::SUCCESS)
                {
                }
                _sendflg = false;
                t2 = t1;
            }
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

    // void Run::auto_aim_publisher_thread()
    // {
            
    //         while(ros::ok())
    //         {
    //             if (rm::Run::need_to_publish == 1)
    //             {
    //                 printf("control_data == %f   %f   %f \n", my_new_controlframe.linear.x, my_new_controlframe.linear.y, my_new_controlframe.linear.z);

    //                 auto_aim_publisher.publish(my_new_controlframe);
    //                 need_to_publish = 0;
    //                 ros::spinOnce();
    //             }
    //             Run::my_rate.sleep();
    //         }
    //     }
} // namespace rm
