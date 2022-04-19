#include "zimiaoshiyan/Armor/armor.h"

namespace rm
{

    Armor::Armor()
    {
    }

    bool Armor::feed(Frame imgFrame, float pitch, float yaw, cv::Point3f &Output)
    {
        float t1, t2, t3, t4, t5, t6;
        bool retval;
        ValidImgPeriod = imgFrame.timeStamp - LastImgTimestamp;
        LastImgTimestamp = imgFrame.timeStamp;

        CAM_ValidImgPeriod = imgFrame.CAM_timeStamp - CAM_LastImgTimestamp;
        CAM_LastImgTimestamp = imgFrame.CAM_timeStamp;
        if (CAM_ValidImgPeriod < 0.5)
            CAM_ValidImgPeriod = ValidImgPeriod;
        t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        std::vector<cv::RotatedRect> Bars;
        Bars = find_light_bar(imgFrame.img);
        t2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

        std::vector<std::vector<cv::Point2f>> rects;

        rects = rectangle_match(Bars);
        t3 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

        if (rects.size() > 0 && Bars.size() > 1)
        {
            std::vector<cv::Point2f> TargetRect = rects[0];

            _solve.Solve_Armor(TargetRect);
            t4 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            if (TrackingTarget == 0)
                ClearTargetState(TargetRect);
            else
                GetTargetState(TargetRect);

#if (TARGET_SHOW && DEBUG)
            cv::Mat _img;
            _img = imgFrame.img.clone();
            cv::circle(_img, Last2DPosition, 3, cv::Scalar(255, 255, 255), 2);
            cv::namedWindow("rect_center", 0);
            cv::resizeWindow("rect_center", 640, 480);
            cv::imshow("rect_center", _img);
            cv::waitKey(1);
#endif
  

            cv::Point3f P = _solve.get_real_Point3f(0, 0);

            Output = P;
            TrackingCount++;
            LostFlag = 0;
            LostCount = 0;
            if (TrackingCount >= 2)
                TrackingTarget = 1;
            else
                TrackingTarget = 0;

            retval = true;
        }
        else
        {
            /*
            if (TrackingTarget == 1)
            {
                TrackingTarget = 0;
                LostFlag = 1;
                goto RE_DETECT;
            }*/
            TrackingTarget = 0;
            TrackingCount = 0;
            LostFlag = 1;
            LostCount++;
            if (LostCount > LightBarBlinkCount)
                _solve.ArmorTypeCount = 0;
            retval = false;
        }
        t5 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        //        Log(9, "find_light_bar cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t2 - t1)).count() << " ms   ");
        //        Log(10, "rectangle_match cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t3 - t2)).count() << " ms   ");
        //        Log(11, "Solve_Armor cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t4 - t3)).count() << " ms   ");
        //        Log(12, "GetTargetState cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t5 - t4)).count() << " ms   ");
        //        Log(13, "total cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t5 - t1)).count() << " ms   ");
        return retval;
    }

    void Armor::init(const int id)
    {
        KF.init();
        // _CAM_MATRIX.at<double>(0,0) = 1025.1617329202392739;
        // _CAM_MATRIX.at<double>(0,1) =0.0000000000000000;
        // _CAM_MATRIX.at<double>(0,2) =341.7363058175941433;
        // _CAM_MATRIX.at<double>(1,0) =0.0000000000000000;
        // _CAM_MATRIX.at<double>(1,1) =1025.2181222457525109;
        // _CAM_MATRIX.at<double>(1,2) =184.0896934865698142;
        // _CAM_MATRIX.at<double>(2,0) =0.0000000000000000;
        // _CAM_MATRIX.at<double>(2,1) =0.0000000000000000;
        // _CAM_MATRIX.at<double>(2,2) =1.0000000000000000;

        // _DISTORTION_COEFF.at<double>(0,0) = -0.5601348119216157;
        // _DISTORTION_COEFF.at<double>(1,0) = -0.1880765590404777;
        // _DISTORTION_COEFF.at<double>(2,0) = 0.0000000000000000;
        // _DISTORTION_COEFF.at<double>(3,0) = 0.0000000000000000;
        // _DISTORTION_COEFF.at<double>(4,0) = 2.8447383368907211;

        _CAM_MATRIX.at<double>(0,0) = 1091.8;
        _CAM_MATRIX.at<double>(0,1) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(0,2) = 622.1768;
        _CAM_MATRIX.at<double>(1,0) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(1,1) = 1092;
        _CAM_MATRIX.at<double>(1,2) = 328.7523;
        _CAM_MATRIX.at<double>(2,0) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(2,1) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(2,2) = 1.0000000000000000;

        _DISTORTION_COEFF.at<double>(0,0) = -0.4516;
        _DISTORTION_COEFF.at<double>(1,0) = 0.2120;
        _DISTORTION_COEFF.at<double>(2,0) = 0.0000000000000000;
        _DISTORTION_COEFF.at<double>(3,0) = 0.0000000000000000;
        _DISTORTION_COEFF.at<double>(4,0) = 0.0;
        
        cv::FileStorage fsread("/home/ubuntu/catkin/src/zimiaoshiyan/src/Others/SolvePosition/angle_solver_params.xml", cv::FileStorage::READ);
        if (!fsread.isOpened())
        {
            std::cerr << "failed to open xml" << std::endl;
            return;
        }

        switch (id)
        {
        case 0:
        {
            fsread["CAMERA_MARTRIX_0"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_0"] >> _DISTORTION_COEFF;
            break;
        }
        case 1:
        {
            fsread["CAMERA_MARTRIX_1"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_1"] >> _DISTORTION_COEFF;
            break;
        }
        case 2:
        {
            fsread["CAMERA_MARTRIX_2"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_2"] >> _DISTORTION_COEFF;
            break;
        }
        case 3:
        {
            fsread["CAMERA_MARTRIX_3"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_3"] >> _DISTORTION_COEFF;
            break;
        }
        case 4:
        {
            fsread["CAMERA_MARTRIX_4"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_4"] >> _DISTORTION_COEFF;
            break;
        }
        case 5:
        {
            fsread["CAMERA_MARTRIX_5"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_5"] >> _DISTORTION_COEFF;
            break;
        }
        case 6:
        {
            fsread["CAMERA_MARTRIX_6"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_6"] >> _DISTORTION_COEFF;
            break;
        }
        case 7:
        {
            fsread["CAMERA_MARTRIX_7"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_7"] >> _DISTORTION_COEFF;
            break;
        }
        case 8:
        {
            fsread["CAMERA_MARTRIX_8"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_8"] >> _DISTORTION_COEFF;
            break;
        }
        case 9:
        {
            fsread["CAMERA_MARTRIX_9"] >> _CAM_MATRIX;
            fsread["DISTORTION_COEFF_9"] >> _DISTORTION_COEFF;
            break;
        }
        default:
            std::cout << "wrong cam number given." << std::endl;
            break;
        }
		
        _solve.init(_CAM_MATRIX, _DISTORTION_COEFF);
    }

} // namespace rm
