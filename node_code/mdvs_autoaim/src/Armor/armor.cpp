#include "mdvs_autoaim/Armor/armor.h"

namespace rm
{

    Armor::Armor()
    {
    }

    bool Armor::feed(Frame imgFrame, float pitch, float yaw, cv::Point3f &Output)
    {
        float t1, t2, t3, t4, t5, t6;
        float cloneStart, cloneEnd;
        bool retval;
#if (TARGET_SHOW && DEBUG)
        cv::Mat _img;
        _img = imgFrame.img.clone();
#endif

        //        cloneStart = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

        //        cloneEnd = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        // Log(5, "CLone cost: "<< cloneEnd - cloneStart<<"ms   ");
        ValidImgPeriod = imgFrame.timeStamp - LastImgTimestamp;
        LastImgTimestamp = imgFrame.timeStamp;

        CAM_ValidImgPeriod = imgFrame.CAM_timeStamp - CAM_LastImgTimestamp;
        CAM_LastImgTimestamp = imgFrame.CAM_timeStamp;

        t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        std::vector<cv::RotatedRect> Bars;
        Bars = find_light_bar(imgFrame.img, imgFrame.CAM_timeStamp);

        t2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

        std::vector<std::vector<cv::Point2f>> rects;

        rects = rectangle_match(Bars);
        t3 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

        if (rects.size() > 0 && Bars.size() > 1)
        {
            std::vector<cv::Point2f> TargetRect = rects[0];

            _solve.Solve_Armor(TargetRect);
            t4 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            if (TrackingCount == 0)
                ClearTargetState(TargetRect);
            else
                GetTargetState(TargetRect);

#if (TARGET_SHOW && DEBUG)
            drawFrameAxes(_img, _solve._CAM_MATRIX, _solve._DISTORTION_COEFF, _solve._rVec, _solve._tVec, 50);
            //            for(int i = 0; i < 4; i++)
            //                cv::circle(_img, rects[0][i], 5, cv::Scalar(255, 0, 255), 3);
            //            cv::circle(_img, Last2DPosition, 5, cv::Scalar(255, 255, 255), 3);
            RS.to_show_img(_img, "rect_center", Size(WIDTH / 2, HEIGHT / 2));
#endif

            cv::Point3f P = _solve.get_real_Point3f(0, 0);

            Output = P;
            TrackingCount++;
            LostFlag = 0;
            LostCount = 0;
            if (TrackingCount > 2)
                TrackingTarget = 1;
            else
                TrackingTarget = 0;

            retval = true;
        }
        else
        {
            if (TrackingTarget == 1)
                TargetLostTimeStamp = imgFrame.CAM_timeStamp;
            TrackingCount = 0;
            LostFlag = 1;
            LostCount++;
            TrackingTarget = 0;
            if (LostCount > LightBarBlinkCount)
                _solve.ArmorTypeCount = 0;
            retval = false;
        }
        t5 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        /*
        Log(9, "find_light_bar cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t2 - t1)).count() << " ms   ");
        Log(10, "rectangle_match cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t3 - t2)).count() << " ms   ");
        Log(11, "Solve_Armor cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t4 - t3)).count() << " ms   ");
        Log(12, "GetTargetState cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t5 - t4)).count() << " ms   ");
        Log(13, "total cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t5 - t1)).count() << " ms   ");
        */

        return retval;
    }

    cv::Mat Armor::ArmorSubtractThresold(cv::Mat src, rm::ColorChannels _enemy_color, uint8_t min)
    {
        cv::Mat binary_dst = cv::Mat::zeros(src.size(), CV_8UC1);

        uchar *idata = (uchar *)src.data;
        uchar *odata = (uchar *)binary_dst.data;

        int srcData = src.cols * src.rows;

        if (_enemy_color == RED)
        {
            for (int i = 0; i < srcData; i++)
            {
                if (*(idata + 2) - *idata > min)
                    *odata = 255;

                idata += 3;
                odata++;
            }
        }
        else
        {
            for (int i = 0; i < srcData; i++)
            {
                if (*idata - *(idata + 2) > min)
                    *odata = 255;

                idata += 3;
                odata++;
            }
        }

        return binary_dst;
    }

    void Armor::init(const int id)
    {
        // KF.init();
#if USE_SVM
        svm_obj.init_model();
#endif
        cv::FileStorage fsread("/home/ubuntu/catkin/src/mdvs_autoaim/include/mdvs_autoaim/Others/SolvePosition/angle_solver_params.xml", cv::FileStorage::READ);
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
