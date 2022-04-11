#include "mdvs_autoaim/Rune/Wind.hpp"
#include "mdvs_autoaim/define.h"

namespace rm
{
    void Wind::init(int id)
    {
        _startTime = std::chrono::high_resolution_clock::now();
        //cv::FileStorage fsread("Others/General/angle_solver_params.xml", cv::FileStorage::READ);
        cv::FileStorage fsread("/home/ubuntu/catkin/src/mdvs_autoaim/include/mdvs_autoaim/Others/SolvePosition/angle_solver_params.xml", cv::FileStorage::READ);
        if (!fsread.isOpened())
        {
            //LOG(INFO)<<"failed to open angle_solver_params.xml";
            std::cout << "failed to open angle_solver_params.xml" << std::endl;
            return;
        }
        if (0 <= id && id <= 9)
        {
            fsread["CAMERA_MARTRIX_" + std::to_string(id)] >> CAM_MATRIX;
            fsread["DISTORTION_COEFF_" + std::to_string(id)] >> DISTORTION_COEFF;
        }
        else
            std::cout << "wrong camera serial number " << std::endl;

        windSolve.init(CAM_MATRIX, DISTORTION_COEFF);

        DirectionCount = 0;

        //windKf.init(2, 2);
    }

    bool Wind::wind(rm::Frame img, float time, cv::Point3f &aim, rm::ColorChannels enemy, rm::Mode mode)
    {
        bool OUT = false;

        recorder += 1;
        draw_show = img.img.clone();
        //RS.to_show_img(draw_show, "Wind_IMG_SHOW", cv::Size(640, 480));

        float process_time;
        if (recorder != 1)
        {
            process_time = (img.timeStamp - LastImgTimestamp) / 1000; //s
            times.push_back(process_time);
        }



        cv::Mat img1 = to_thresold_wing(img.img, enemy);

        cv::RotatedRect armor;
        bool flag;
        //Log(18,"WIND start"<< (static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() );
        flag = to_wing_center(img1, armor);
        //Log(19,"WIND end"<< (static_cast<std::chrono::duration<double, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() );

        if (flag)
        {
            armor_angle.push_back(armor.angle);
            Log(9, "angle error = " << fabsf(armor.angle - last_armor.angle) * 180 / CV_PI << "                   ");

            if (fabsf(armor.angle - last_armor.angle) * 180 / CV_PI < 36)
            {
                if (armor.angle > last_armor.angle)
                {
                    DirectionCount++;
                }
                else
                {
                    DirectionCount--;
                }
            }
            if (DirectionCount > 0)
            {
                DIRECTION = CLOCK_NWISE;
                Log(10, "CLOCK_NWISE             ");
            }
            else if (DirectionCount < 0)
            {
                DIRECTION = CLOCK_WISE;
                Log(10, "CLOCK_WISE                ");
            }
            else
            {
                DIRECTION = CLOCK_NONE;
            }
            Log(12, "DirectionCount = " << DirectionCount << "                   ");

            if (mode == Mode::HIT_RUNE_MIN)
            {
                if (recorder % Params.SEQ_MODE_MIN == 0)
                {
                    float theta = time * CV_PI / 3; //time:s  20*pi /60

#if (ANGLE_PERSPECTIVE_COMPARE && DEBUG)
                    /**************************Angle No Perspective**********************/

                    cv::Point2f armor_points[4];
                    std::vector<cv::Point2f> warp_points;
                    armor_origin.points(armor_points);

                    for (const auto &point : armor_points)
                    {
                        cv::Point2f temp;
                        float temp_theta;

                        if (DIRECTION == CLOCK_WISE)
                        {
                            temp_theta = theta;
                        }
                        else
                        {
                            temp_theta = -theta;
                        }
                        Log(11, "theta =" << temp_theta);

                        double x = point.x - circle_center.x;
                        double y = point.y - circle_center.y;

                        temp.x = x * cos(temp_theta) + y * sin(temp_theta) + circle_center.x;
                        temp.y = -x * sin(temp_theta) + y * cos(temp_theta) + circle_center.y;

                        warp_points.push_back(temp);
                        cv::circle(draw_show,temp,4,RS.COLOR_RED,-1);
                        cv::circle(draw_show,point,4,RS.COLOR_RED,-1);

                    }
//                    cv::RotatedRect warp_armor_rect = cv::RotatedRect(warp_points[0],warp_points[1],warp_points[2]);
//                    RS.to_draw_RotatedRect(warp_armor_rect,draw_show,RS.COLOR_RED);
                    cv::circle(draw_show,circle_center,3,RS.COLOR_PURPLE,-1);
#endif

                    cv::Mat warpMatrix;
                    warpCenter = to_perspective(armor, warpMatrix);

#if (ANGLE_PERSPECTIVE_COMPARE && DEBUG)
                    /***************************Angle Perspective************************/
                    cv::RotatedRect warpArmorRect = cv::RotatedRect(wing_corners[0], wing_corners[1], wing_corners[2]);
                    double warpRadius = sqrt(pow((warpArmorRect.center.x - warpCenter.x), 2) + pow((warpArmorRect.center.y - warpCenter.y), 2));
                    float warpAngle = asin((warpArmorRect.center.y - warpCenter.y) / warpRadius) * 180 / CV_PI; // Radian System

                    std::ofstream AngleFile("../angle_perspective_compare.txt", std::ios_base::out);
                    //AngleFile << (recorder - 1) << "\t\t\t" << Angle << "\t\t\t" << warpAngle << endl;
                    AngleFile.close();
#endif

                    std::vector<cv::Point2f> pre_points = to_turn_points(warpCenter, theta);
                    to_perspective(pre_points, warpMatrix);

#if WIND_TARGET_PREDICT && DEBUG
                    for (size_t i = 0; i < pre_points.size(); i++)
                        cv::line(draw_show, pre_points[i], pre_points[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
                    RS.to_show_img(draw_show, "Wind_Target_Predict", cv::Size(640, 480));
#endif
                    windSolve.solve(windSolve.POINT_3D_OF_RUNE_WING, pre_points);
                    cv::Point3f XYZ = windSolve.get_real_Point3f(0, 0);
                    aim = XYZ;

                    clear();
                    OUT = true;
                }
            }
            if (mode == Mode::HIT_RUNE_MAX)
            {
                if (recorder % Params.SEQ_MODE_MAX == 0) //start != 0)
                {
                    float theta1 = 0;
                    if (to_wing_mode_max(time, theta1))
                    {
                        // w = 0.785 * sin(1.884*t) + 1.305    rad/s
                        // float theta2 = 0;
                        // float delta_time = time / 50000;
                        // for (float i = start; i <= time + start; i += delta_time)
                        // {
                        //     float w = (0.785 * sin(1.884 * (i)) + 1.305);
                        //     theta2 += w * delta_time;
                        // }
                        cv::Mat warpMatrix;
                        warpCenter = to_perspective(armor, warpMatrix);
                        std::vector<cv::Point2f> pre_points = to_turn_points(warpCenter, theta1);
                        to_perspective(pre_points, warpMatrix);

#if WIND_TARGET_PREDICT && DEBUG
                        for (size_t i = 0; i < pre_points.size(); i++)
                            cv::line(draw_show, pre_points[i], pre_points[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
                        RS.to_show_img(draw_show, "Wind_Target_Predict", cv::Size(640, 480));
#endif
                        windSolve.solve(windSolve.POINT_3D_OF_RUNE_WING, pre_points);
                        cv::Point3f XYZ = windSolve.get_real_Point3f(0, 0);
                        aim = XYZ;

                        clear();
                        OUT = true;
                    }
                }
            }

            last_armor = armor;
            std::vector<cv::Point2f>().swap(wing_corners);
        }
        else
            OUT = false;

        LastImgTimestamp = img.timeStamp;

        return OUT;
    }

    cv::Mat Wind::to_thresold_wing(cv::Mat img, rm::ColorChannels enemy)
    {
        cv::Mat img1, img2, channels[3];
        cv::split(img, channels);
        img1 = enemy == rm::ColorChannels::RED ? channels[2] - channels[0] : channels[0] - channels[2];

        cv::threshold(img1, img1, 60, 255, cv::THRESH_BINARY);
        int elementSize = 2;
        cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1), cv::Size(elementSize, elementSize));
        cv::dilate(img1, img1, element);
        //        cv::floodFill(img1, cv::Point(0, 0), cv::Scalar(0));
        //        elementSize = 1;
        //        element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1), cv::Size(elementSize, elementSize));
        //        cv::morphologyEx(img1, img2, cv::MORPH_CLOSE, element);
        cv::erode(img1, img2, element);

#if (WIND_THRESOLD_FINAL && DEBUG)
        RS.to_show_img(img2, "Wind_Thresold_Final", cv::Size(640, 480));
        cv::waitKey(1);
#endif

        return img2;
    }

    bool Wind::to_wing_center(cv::Mat img, cv::RotatedRect &armor)
    {
        bool OUT = false;

        std::vector<std::vector<cv::Point>> Contours;
        std::vector<cv::Vec4i> Hierarchy;
        cv::findContours(img, Contours, Hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        cv::Point2f points[4];
        std::vector<std::pair<cv::Point2f, double>> center_distance;

        for (size_t i = 0; i < Contours.size(); i++)
        {
            if (Hierarchy[i][2] < 0 && Contours[i].size() > 5)
            {
                cv::RotatedRect rect = fitEllipse(Contours[i]);
                center_distance.push_back(std::make_pair(rect.center, 0));
            }
            if (Contours[i].size() < 6 || Hierarchy[i][2] > 0)
                continue;

            if (cv::contourArea(Contours[i]) < Params.ARMOR_AREA_MIN ||
                cv::contourArea(Contours[i]) > Params.ARMOR_AREA_MAX)
                continue;

            cv::RotatedRect armor_rect = cv::fitEllipse(Contours[i]);
            double width = armor_rect.size.width;
            double height = armor_rect.size.height;
            double ratio = height / width;

            if (armor_rect.size.area() > 1000)
                continue;

            if (ratio < Params.ARMOR_W_H_RATIO_MIN || ratio > Params.ARMOR_W_H_RATIO_MAX)
                continue;

            if (Contours[Hierarchy[i][3]].size() < 6 || Hierarchy[i][3] < 0)
                continue;

            cv::RotatedRect leaf_rect = cv::fitEllipse(Contours[Hierarchy[i][3]]);

            if (leaf_rect.size.area() > 5000)
                continue;

            if (cv::contourArea(Contours[Hierarchy[i][3]]) / leaf_rect.size.area() > Params.WING_FIT_DEGREE_MAX)
                continue;

            armor_rect.points(points);
            armor = armor_rect;
            armor_origin = armor_rect;

            circle_center = (1 - Params.CENTER_VECTOR_RATIO) * armor_rect.center + Params.CENTER_VECTOR_RATIO * leaf_rect.center;

#if (WIND_TARGET_CIRCLE_CENTER && DEBUG)
            RS.to_draw_RotatedRect(armor_rect, draw_show, cv::Scalar(0, 255, 255));
            RS.to_draw_RotatedRect(leaf_rect, draw_show, cv::Scalar(0, 255, 255));

            cv::circle(draw_show, armor_rect.center, 2, cv::Scalar(0, 255, 255), 4);
            cv::circle(draw_show, leaf_rect.center, 2, cv::Scalar(0, 255, 255), 4);
#endif
        }


        if (circle_center.x > 0 && circle_center.y > 0 && center_distance.size() > 0)
        {
            OUT = true;
            for (size_t i = 0; i < center_distance.size(); i++)
            {
                double distance = sqrt((pow(circle_center.x - center_distance[i].first.x, 2)) + (pow(circle_center.y - center_distance[i].first.y, 2)));
                center_distance[i].second = distance;
            }

            std::sort(center_distance.begin(), center_distance.end(), [](const std::pair<cv::Point2f, double> &ld1, const std::pair<cv::Point2f, double> &ld2) {
                return ld1.second < ld2.second;
            });
            circle_center = center_distance[0].first;

            double radius = sqrt(pow((armor.center.x - circle_center.x), 2) +
                                 pow((armor.center.y - circle_center.y), 2));

            if (armor.center.x - circle_center.x > 0)
                armor.angle = asin((armor.center.y - circle_center.y) / radius);
            else
                armor.angle = asin((circle_center.  y - armor.center.y) / radius);

#if (WIND_TARGET_CIRCLE_CENTER && DEBUG)
            cv::circle(draw_show, circle_center, 2, cv::Scalar(0, 255, 255), 4);
            RS.to_show_img(draw_show, "Wind_Target_Circle_Center", cv::Size(640, 480));
#endif
        }
        if (OUT)
        {
            for (const auto &point : points)
                wing_corners.push_back(point);
        }

        return OUT;
    }

    bool Wind::to_wing_mode_max(float time, float &theta)
    {
        bool OUT = false;

        std::vector<float> V; //角速度

        // w = 0.785 * sin(1.884*t) + 1.305    rad/s
        // T = 3.3

        for (size_t i = 1; i < recorder; i++)
        {
            float s = armor_angle[i] - armor_angle[i - 1];
            float v = s / times[i];
            V.push_back(v);

            if (V.size() > 3)
            {
                for (size_t i = 1; i < V.size(); i++)
                {
                    if (V[i - 1] < V[i] && V[i + 1] < V[i])
                    {
                        float delta_time = time / 50000;
                        for (float i = 0; i <= time; i += delta_time)
                        {
                            float w = (0.785 * sin(1.884 * i + CV_PI / 2) + 1.305);
                            theta += w * delta_time;
                        }
                        OUT = true;
                        return OUT;
                    }
                    else if (V[i - 1] > V[i] && V[i + 1] > V[i])
                    {
                        float delta_time = time / 50000;
                        for (float i = 0; i <= time; i += delta_time)
                        {
                            float w = (0.785 * sin(1.884 * i - CV_PI / 2) + 1.305);
                            theta += w * delta_time;
                        }
                        OUT = true;
                        return OUT;
                    }
                }
            }
        }

        return OUT;
    }

    cv::Point2f Wind::to_perspective(cv::RotatedRect rect, cv::Mat &warpMatrix)
    {
        cv::Point2f src[4], dst[4];
        for (size_t i = 0; i < wing_corners.size(); i++){
            src[i] = wing_corners[i];
            cv::circle(draw_show,src[i],4,RS.COLOR_ORANGE,-1);
        }
        dst[0] = cv::Point2f(0, 0);
        dst[1] = cv::Point2f(230, 0);
        dst[2] = cv::Point2f(230, 127);
        dst[3] = cv::Point2f(0, 127);
//        dst[0] = cv::Point2f(0, 0);
//        dst[1] = cv::Point2f(1*10, 0);
//        dst[2] = cv::Point2f(1*10, 230/127*10);
//        dst[3] = cv::Point2f(0, 230/127*10);
//        dst[0] = cv::Point2f(0, rect.size.height);
//        dst[1] = cv::Point2f(0, 0);
//        dst[2] = cv::Point2f(rect.size.width, 0);
//        dst[3] = cv::Point2f(rect.size.height, rect.size.height);

        warpMatrix = cv::getPerspectiveTransform(src, dst);

        cv::Mat_<double> mat_pt(3, 1);
        mat_pt(0, 0) = circle_center.x;
        mat_pt(1, 0) = circle_center.y;
        mat_pt(2, 0) = 1;

        cv::Mat mat_pt_view = warpMatrix * mat_pt ;
        double a1 = mat_pt_view.at<double>(0, 0);
        double a2 = mat_pt_view.at<double>(1, 0);
        double a3 = mat_pt_view.at<double>(2, 0);

        for (size_t i = 0; i < wing_corners.size(); i++){
            wing_corners[i] = dst[i];
            cv::circle(draw_show,wing_corners[i],4,RS.COLOR_BLUE,-1);
        }
        return cv::Point2f(a1 / a3, a2 / a3);
    }

    void Wind::to_perspective(std::vector<cv::Point2f> &pre_points, cv::Mat warpMatrix)
    {
        cv::invert(warpMatrix, warpMatrix, cv::DECOMP_LU);
        for (size_t i = 0; i < pre_points.size(); i++)
        {
            cv::Point2f point = pre_points[i];
            cv::Mat_<double> mat_pt(3, 1);
            mat_pt(0, 0) = point.x;
            mat_pt(1, 0) = point.y;
            mat_pt(2, 0) = 1;

            cv::Mat mat_pt_view = warpMatrix * mat_pt;
            double a1 = mat_pt_view.at<double>(0, 0);
            double a2 = mat_pt_view.at<double>(1, 0);
            double a3 = mat_pt_view.at<double>(2, 0);

            pre_points[i] = cv::Point2f(a1 / a3, a2 / a3);
        }
    }

    std::vector<cv::Point2f> Wind::to_turn_points(cv::Point2f center, float theta)
    {
        std::vector<cv::Point2f> OUT;
        float temp_theta;
        for (const auto &point : wing_corners)
        {
            cv::Point2f temp;

            if (DIRECTION == CLOCK_WISE)
            {
                temp_theta = theta;
            }
            else
            {
                temp_theta = -theta;
            }
            Log(11, "theta =" << temp_theta);

            double x = point.x - center.x;
            double y = point.y - center.y;

            temp.x = x * cos(temp_theta) + y * sin(temp_theta) + center.x;
            temp.y = -x * sin(temp_theta) + y * cos(temp_theta) + center.y;

            OUT.push_back(temp);
        }
        return OUT;
    }
}
