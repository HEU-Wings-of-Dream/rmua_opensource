#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

namespace rm
{
    class Solve
    {

    public:
        static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_BIG;
        static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_SMALL;

        static std::vector<cv::Point3f> POINT_3D_OF_RUNE_CENTER;
        static std::vector<cv::Point3f> POINT_3D_OF_RUNE_BIG;
        static std::vector<cv::Point3f> POINT_3D_OF_RUNE_WING;

        //        std::vector<cv::Point3f> POINT_3D_OF_ARMOR_BIG=std::vector<cv::Point3f>
        //        {
        //            #define ARMOR_SMALL_WIDTH 50
        //            #define ARMOR_SMALL_HEIGHT 55//68.75
        //            cv::Point3f(-ARMOR_SMALL_WIDTH, -ARMOR_SMALL_HEIGHT, 0),	//tl
        //            cv::Point3f(ARMOR_SMALL_WIDTH, -ARMOR_SMALL_HEIGHT, 0),     //tr
        //            cv::Point3f(ARMOR_SMALL_WIDTH, ARMOR_SMALL_HEIGHT, 0),		//br
        //            cv::Point3f(-ARMOR_SMALL_WIDTH, ARMOR_SMALL_HEIGHT, 0)		//bl
        //        };
        //        std::vector<cv::Point3f> POINT_3D_OF_ARMOR_SMALL= std::vector<cv::Point3f>
        //        {
        //            #define ARMOR_SMALL_WIDTH 50
        //            #define ARMOR_SMALL_HEIGHT 55//68.75
        //            cv::Point3f(-ARMOR_SMALL_WIDTH, -ARMOR_SMALL_HEIGHT, 0),	//tl
        //            cv::Point3f(ARMOR_SMALL_WIDTH, -ARMOR_SMALL_HEIGHT, 0),     //tr
        //            cv::Point3f(ARMOR_SMALL_WIDTH, ARMOR_SMALL_HEIGHT, 0),		//br
        //            cv::Point3f(-ARMOR_SMALL_WIDTH, ARMOR_SMALL_HEIGHT, 0)		//bl
        //        };

        //        std::vector<cv::Point3f> POINT_3D_OF_RUNE_CENTER = std::vector<cv::Point3f>
        //        {
        //            #define UNE_CENTER 1000
        //            cv::Point3f(-UNE_CENTER, -UNE_CENTER, 0),	//tl
        //            cv::Point3f(UNE_CENTER, -UNE_CENTER, 0),     //tr
        //            cv::Point3f(UNE_CENTER, UNE_CENTER, 0),		//br
        //            cv::Point3f(-UNE_CENTER, UNE_CENTER, 0)		//bl
        //        };

        //        std::vector<cv::Point3f> POINT_3D_OF_RUNE_WING = std::vector<cv::Point3f>
        //        {
        //        //#define UNE_CENTER 50
        //        cv::Point3f(-UNE_CENTER, -UNE_CENTER, 0),	//tl
        //        cv::Point3f(UNE_CENTER, -UNE_CENTER, 0),     //tr
        //        cv::Point3f(UNE_CENTER, UNE_CENTER, 0),		//br
        //        cv::Point3f(-UNE_CENTER, UNE_CENTER, 0)		//bl
        //        };

        void init(cv::Mat &CAM_MATRIX, cv::Mat &DISTORTION_COEFF);
        void solve(std::vector<cv::Point3f> OBJECT_3D, std::vector<cv::Point2f> OBJECT_2D);
        void Solve_Armor(std::vector<cv::Point2f> OBJECT_2D);

        cv::Point3f get_real_Point3f(double pitch, double yaw);
        cv::Point2f get_Point2f(cv::Point3f input, float pitch, float yaw);
        cv::Mat get_rVec();
        cv::Mat get_tVec();

        int32_t ArmorTypeCount = 0;
        uint8_t AromrType;
        float LightBarLenRatio;
        float w_hRatio;

    private:
        cv::Mat _CAM_MATRIX;       //Camera Matrix
        cv::Mat _DISTORTION_COEFF; //Distortion matrix

        /*
        *    z  y
        *    | /
        *    |/______x
        *    O
        */

        //the unit of vector is mm

        cv::Mat _rVec = cv::Mat::zeros(3, 1, CV_32FC1); //init rvec
        cv::Mat _tVec = cv::Mat::zeros(3, 1, CV_32FC1); //init tvec
        cv::Mat _rMat = cv::Mat::zeros(3, 1, CV_32FC1);
    };

} // namespace rm
