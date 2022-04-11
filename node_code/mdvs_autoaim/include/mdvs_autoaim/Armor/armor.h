#pragma once

#include <opencv2/opencv.hpp>
#include "mdvs_autoaim/Others/DataFrame/data_frame.h"
#include "mdvs_autoaim/Others/SolvePosition/solve_position.h"
#include "mdvs_autoaim/Armor/ArmorPredict/armor_predict.h"
#include "mdvs_autoaim/Others/General/opencv_owns.hpp"
#include "mdvs_autoaim/Armor/number_detector/svm.h"
#include "mdvs_autoaim/define.h"

namespace rm
{

    class Armor
    {
        struct Params_Detect
        {
            double CONTOUR_H_W_RATIO_MIN = 2.5;
            double CONTOUR_H_W_RATIO_MAX = 13.0;
            double CONTOUR_H_W_RATIO_MIN_LONG;

            int CONTOUR_AREA_MIN = 15;
            int CONTOUR_AREA_MAX = 15000;
            double CONTOUR_FIT_DEGREE_MIN = 0.37;
            double LENTH = 20;

        } D_Params;

        struct Params_Match
        {
#if USE_MIX_ROTATED_RECT
            double ANGLE_DIFF_MAX = 4;
#else
            double ANGLE_DIFF_MAX = 8;
#endif
            double LENGTH_RATIO = 0.75;
            double MEAN_LENGTH_DIST_RATIO_MIN = 1.8;
            double MEAN_LENGTH_DIST_RATIO_MAX = 5;
            double MEAN_LENGTH_DIST_RATIO_MAX_LONG;
            double ANGLE_DIFF_MAX_LONG;
            double LENTH = 20;
            double BLUE_LENGTH_SCALE = 1.15+1*0;
            double RED_LENGTH_SCALE = 1.22+1*0;

        } M_Params;

    public:
        Armor();
        ~Armor() {}

        bool feed(Frame imgFrame, float pitch, float yaw, cv::Point3f &Output);
        void init(const int id);

        std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;

        Frame ImageFrame;

        Solve _solve;
        Result_Show RS;

        cv::Rect TrackingROI;
        std::vector<cv::Point2f> LastTargetRect;
        cv::Point2f Last2DPosition;
        cv::Point2f Last2DVelocity;
        cv::Point2f Last2DAccel;

        float TargetLostTimeStamp;
        uint8_t TrackingTarget = 0;
        uint32_t TrackingCount = 0;
        uint8_t LostFlag = 0;
        uint32_t LostCount = 0;

        uint8_t LightBarBlinkCount = 16;

        ColorChannels _enemy_color = BLUE;

        float ValidImgPeriod;
        float LastImgTimestamp;
        double CAM_ValidImgPeriod;
        double CAM_LastImgTimestamp;

        unsigned long sequence = 0;
        unsigned long fake_sequence = 0;

        float timeCost;
        std::vector<cv::Mat> channels;
        rm::svmdetector svm_obj;

    private:
        cv::Mat _CAM_MATRIX;       // Camera Matrix
        cv::Mat _DISTORTION_COEFF; // Distortion matrix

        kf KF;

        void ClearTargetState(std::vector<cv::Point2f> TargetRect);
        void SpreadTargetState(std::vector<cv::Point2f> TargetRect);
        void GetTargetState(std::vector<cv::Point2f> TargetRect);
        void SetTrackingROI(float imgTimeStamp);
        cv::Mat ArmorSubtractThresold(cv::Mat src, rm::ColorChannels _enemy_color, uint8_t min);
        std::vector<cv::RotatedRect> find_light_bar(cv::Mat &img, float imgTimeStamp);
        std::vector<std::vector<cv::Point2f>> rectangle_match(std::vector<cv::RotatedRect> &vrect);
        int picnum = 1;
        int count = 1;
        uchar *data;
    };

} // namespace rm
