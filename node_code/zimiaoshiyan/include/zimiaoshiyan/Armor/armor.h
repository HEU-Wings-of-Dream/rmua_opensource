#pragma once

#include <opencv2/opencv.hpp>
#include "zimiaoshiyan/Others/DataFrame/data_frame.h"
#include "zimiaoshiyan/Others/SolvePosition/solve_position.h"
#include "zimiaoshiyan/Armor/ArmorPredict/armor_predict.h"
#include "zimiaoshiyan/Others/General/opencv_owns.hpp"
#include "zimiaoshiyan/define.h"

namespace rm
{

    class Armor
    {
        struct Params_Detect
        {
            double CONTOUR_H_W_RATIO_MIN = 2.5;
            double CONTOUR_H_W_RATIO_MAX = 15.0;
            double CONTOUR_H_W_RATIO_MIN_LONG;

            int CONTOUR_AREA_MIN = 25;
            int CONTOUR_AREA_MAX = 4250;
            double CONTOUR_FIT_DEGREE_MIN = 0.5;
            double LENTH = 20;

        } D_Params;

        struct Params_Match
        {
            double ANGLE_DIFF_MAX = 6;
            double LENGTH_RATIO = 0.5;
            double MEAN_LENGTH_DIST_RATIO_MIN = 1.25;
            double MEAN_LENGTH_DIST_RATIO_MAX = 5;
            double MEAN_LENGTH_DIST_RATIO_MAX_LONG;
            double ANGLE_DIFF_MAX_LONG;
            double LENTH = 20;

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

        uint8_t TrackingTarget = 0;
        uint32_t TrackingCount = 0;
        uint8_t LostFlag = 0;
        uint32_t LostCount = 0;

        uint8_t LightBarBlinkCount = 10;

        ColorChannels _enemy_color = RED;

        float AimAssistUpdatePeriod;
        float ValidImgPeriod;
        float LastImgTimestamp;
        double CAM_ValidImgPeriod;
        double CAM_LastImgTimestamp;

        unsigned long sequence = 0;
        unsigned long fake_sequence = 0;
		bool ArmorNumMatch(cv::Mat &ROI, bool is_use_gamma, double gamma, double valueThresold, ObjectType &armorType, int &armorNum);
		void GammaCorrection(cv::Mat &src, cv::Mat &dst, double gamma);
		void ClearNumMatchParams(ObjectType &armorType, int &armorNum);
    private:
        cv::Mat _img;

        cv::Mat _CAM_MATRIX = cv::Mat::zeros(3,3,CV_32FC1);       //Camera Matrix
        cv::Mat _DISTORTION_COEFF = cv::Mat::zeros(5,1,CV_32FC1); //Distortion matrix

        kf KF;

        void ClearTargetState(std::vector<cv::Point2f> TargetRect);
        void GetTargetState(std::vector<cv::Point2f> TargetRect);
        void SetTrackingROI(void);
        std::vector<cv::RotatedRect> find_light_bar(cv::Mat &img);
        std::vector<std::vector<cv::Point2f>> rectangle_match(std::vector<cv::RotatedRect> &vrect);

        void ArmorSplit(const cv::Mat &mtx, vector<Mat> &mv);
    };

} // namespace rm
