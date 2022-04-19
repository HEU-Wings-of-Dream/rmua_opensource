#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "zimiaoshiyan/Others/DataFrame/data_frame.h"
#include "zimiaoshiyan/Others/SolvePosition/solve_position.h"
#include "zimiaoshiyan/Others/General/opencv_owns.hpp"
#include "zimiaoshiyan/Rune/WindPredict/wind_predict.hpp"
#include "zimiaoshiyan/define.h"
#include <vector>
#define WIND_KF 1
namespace rm
{
    class Wind
    {
    public:
        Wind();
        ~Wind() {}

        struct Params_Detect
        {
            int ARMOR_AREA_MIN = 200; // (640X480  100)
            int ARMOR_AREA_MAX = 1000;

            double ARMOR_W_H_RATIO_MIN = 1.3;
            double ARMOR_W_H_RATIO_MAX = 2.7;
            double WING_FIT_DEGREE_MAX = 0.58;

            int SEQ_MODE_MIN = 1;
            int SEQ_MODE_MAX = 1;

            double CENTER_VECTOR_RATIO = 3.2135;

            double WING_LEAF_FIT_MOMENTS_VALUE_MAX = 0.028*1.5;

            int NOSIE_AREA_MIN = 400;
            // int CENTER_AREA_MAX = 750;
            // double CENTER_W_H_RATIO_MIN = 0.9;
            // double CENTER_W_H_RATIO_MAX = 1.1;
            // double CENTER_FIT_DEGREE_MIN = 0.75;
        } Params;

        cv::Mat CAM_MATRIX = cv::Mat::zeros(3,3,CV_32FC1);       //Camera Matrix
        cv::Mat DISTORTION_COEFF = cv::Mat::zeros(5,1,CV_32FC1); //Distortion matrix

        cv::Mat draw_show;
        Result_Show RS;

        windKF windkf;
        bool is_hit_success;
        std::vector <cv::Mat> temple_vec;
        Solve windSolve;
        void read_temple();
        enum Turn_Direction
        {
            CLOCK_NONE = 0,
            CLOCK_WISE, //顺时针
            CLOCK_NWISE //逆时针
        } DIRECTION;

        //std::vector<pair<float, float>> Theta_TimeStamp;
        cv::Point2f circle_center;             //orign -> perspective
        std::vector<cv::Point2f> wing_corners; //orign -> perspective

        cv::RotatedRect last_armor;int kkkk=0;
        cv::Point2f last_circle_center;
        std::vector<double> armor_angle;
        long long DirectionCount;

        int recorder = 0;
        std::vector<float> times; //处理时间 s
        float LastImgTimestamp;
        std::ofstream stream;
        cv::RotatedRect armor_origin;
        cv::Point2f warpCenter;

        std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;

        std::vector<cv::Point> Leaf_Contour_Moments;
        cv::Mat Leaf_Contour_Moments_Img;
        int i = 0;int gggg=0;
        bool first_frame = 0;

    public:
        /***
        @brief Initialize wind plate carmera params, kalman-Filter ,angle solving
        @param id camera_id
        @return None
        ***/
        void init(int id);

        /***
        @brief Hit Wind
        @param img Real-time incoming image;
        @param start_time the time of attacking aim,before shooting unit:s
        @param time the wasting time of shooting  unit:s
        @param aim the 3d aim center by solving and predicting
        @param enemy the color of enemy
        @param mode MAX or MIN
        @return whether capturing any wing, aim point by solving and predicting
        ***/
        bool wind(rm::Frame img, float start_time, float time, cv::Point3f &aim, rm::ColorChannels enemy, rm::Mode mode, float timestemp, float last_timestemp);

    private:
        /***
        @brief  spilting and thresolding
        @param img Real-time incoming image;
        @return Mat
        ***/
        cv::Mat to_thresold_wing(cv::Mat img, rm::ColorChannels enemy);

        /***
        @brief get target armor's rotated rect
        @param img an image spilting and thresolding
        @return
        ***/
        bool to_wing_center(cv::Mat img, cv::RotatedRect &armor);
        bool to_wing_center(cv::Mat img, cv::RotatedRect &armor, bool is_using_tradition);

        /***
        @brief Determine whether the speed limit is reached
        @return
        ***/
        bool to_wing_mode_max(float time, float &theta);

        /***
        @brief perspective transformation
        @param rect taeget armor's rotated rect
        @param warpMatrix rotated matrix
        @return warpCenter
        ***/
        cv::Point2f to_perspective(cv::RotatedRect rect, cv::Mat &warpMatrix);

        /***
        @brief perspective inverse transformation
        @param pre_points prediction points after perspective transformation
        @param warpMatrix rotated matrix
        @return
        ***/
        void to_perspective(std::vector<cv::Point2f> &pre_points, cv::Mat warpMatrix);

        /***
        @brief polar coordinates prediction
        @param center
        @param theta positive
        @return
        ***/
        std::vector<cv::Point2f> to_turn_points(cv::Point2f center, float theta);

        double to_time_angle(float start_time, float time);

        void clear()
        {
            //recorder = 0;
            std::vector<double>().swap(armor_angle);
            std::vector<float>().swap(times);
            //first_frame = 0;
        }

        cv::Mat WindSubtractThresold(cv::Mat src, rm::ColorChannels _enemy_color)
        {
            cv::Mat binary_dst = cv::Mat::zeros(src.size(), CV_8UC1);

            uchar *idata = (uchar *)src.data;
            uchar *odata = (uchar *)binary_dst.data;

            int srcData = src.cols * src.rows;

            if (_enemy_color == BLUE)
            {
                for (int i = 0; i < srcData; i++)
                {
                    if (*(idata + 2) - *idata > 60)
                        *odata = 255;

                    idata += 3;
                    odata++;
                }
            }
            else
            {
                for (int i = 0; i < srcData; i++)
                {
                    if (*idata - *(idata + 2) > 60)
                        *odata = 255;

                    idata += 3;
                    odata++;
                }
            }

            return binary_dst;
        }
    };

} // namespace rm
