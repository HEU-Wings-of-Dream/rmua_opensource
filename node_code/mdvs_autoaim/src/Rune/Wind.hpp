#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "Others/DataFrame/data_frame.h"
#include "Others/SolvePosition/solve_position.h"
#include "Others/General/opencv_owns.hpp"
#include "define.h"

namespace rm
{
    class Wind
    {
        struct Params_Detect
        {
            int ARMOR_AREA_MIN = 200; // (640X480  100)
            int ARMOR_AREA_MAX = 1000;
            double ARMOR_W_H_RATIO_MIN = 1.3;
            double ARMOR_W_H_RATIO_MAX = 2.7;
            double WING_FIT_DEGREE_MAX = 0.65;

            int SEQ_MODE_MIN = 1;
            int SEQ_MODE_MAX = 10;

            double CENTER_VECTOR_RATIO = 5.213;

            int NOSIE_AREA_MIN = 400;
            int CENTER_AREA_MAX = 750;
            double CENTER_W_H_RATIO_MIN = 0.9;
            double CENTER_W_H_RATIO_MAX = 1.1;
            double CENTER_FIT_DEGREE_MIN = 0.75;
        } Params;

        cv::Mat CAM_MATRIX;       //Camera Matrix
        cv::Mat DISTORTION_COEFF; //Distortion matrix

        cv::Mat draw_show;
        Result_Show RS;

        Solve windSolve;

        enum Turn_Direction
        {
            CLOCK_NONE = 0,
            CLOCK_WISE, //顺时针
            CLOCK_NWISE //逆时针
        } DIRECTION;

        cv::Point2f circle_center;             //orign -> perspective
        std::vector<cv::Point2f> wing_corners; //orign -> perspective

        cv::RotatedRect last_armor;
        std::vector<double> armor_angle;
        long long DirectionCount;

        int recorder = 0;
        std::vector<float> times; //处理时间 s
        float LastImgTimestamp;

        cv::RotatedRect armor_origin;
        cv::Point2f warpCenter;

    public:
        /***
        @brief Initialize wind plate carmera params, kalman-Filter ,angle solving
        @param id camera_id
        @return None
        ***/
        void init(int id);
        std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;

        /***
        @brief Hit Wind
        @param img Real-time incoming image;
        @param start the time of attacking aim,before shooting unit:s
        @param time the wasting time of shooting  unit:s
        @param aim the 3d aim center by solving and predicting
        @param enemy the color of enemy
        @param mode MAX or MIN
        @return whether capturing any wing, aim point by solving and predicting
        ***/
        bool wind(rm::Frame img, float time, cv::Point3f &aim, rm::ColorChannels enemy, rm::Mode mode);

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

        void clear()
        {
            //recorder = 0;
            std::vector<double>().swap(armor_angle);
            std::vector<float>().swap(times);
        }
    };

}; // namespace rm
