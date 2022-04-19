#include "zimiaoshiyan/Rune/Wind.hpp"
#include "zimiaoshiyan/define.h"
#include "fstream"
#include <stdio.h>
#include <iostream>

namespace rm
{
    Wind::Wind()
    {
    }

    void Wind::init(int id)
    {
		CAM_MATRIX.at<double>(0,0) = 1094.9;
        CAM_MATRIX.at<double>(0,1) =0.0000000000000000;
        CAM_MATRIX.at<double>(0,2) =623.8025;
        CAM_MATRIX.at<double>(1,0) =0.0000000000000000;
        CAM_MATRIX.at<double>(1,1) = 1095;
        CAM_MATRIX.at<double>(1,2) = 330.2489;
        CAM_MATRIX.at<double>(2,0) =0.0000000000000000;
        CAM_MATRIX.at<double>(2,1) =0.0000000000000000;
        CAM_MATRIX.at<double>(2,2) =1.0000000000000000;

        DISTORTION_COEFF.at<double>(0,0) = -0.4510;
        DISTORTION_COEFF.at<double>(1,0) = 0.2074;
        DISTORTION_COEFF.at<double>(2,0) = 0.0000000000000000;
        DISTORTION_COEFF.at<double>(3,0) = 0.0000000000000000;
        DISTORTION_COEFF.at<double>(4,0) = 0.0;
        //stream.open("../heu_-vision_2020/omiga.txt");
        _startTime = std::chrono::high_resolution_clock::now();
        //cv::FileStorage fsread("Others/General/angle_solver_params.xml", cv::FileStorage::READ);
        /*
        cv::FileStorage fsread("../src/Others/SolvePosition/angle_solver_params.xml", cv::FileStorage::READ);
        if (!fsread.isOpened())
        {
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
            */
        //read_temple();
        //windkf.init();

        //windSolve.init(CAM_MATRIX, DISTORTION_COEFF);

        DirectionCount = 0;

        //Leaf_Contour_Moments_Img = imread("Leaf_Contour_Moments_Img.jpg", -1);

        //resize(Leaf_Contour_Moments_Img, Leaf_Contour_Moments_Img, Size(30, 30));

        //RS.to_show_img(Leaf_Contour_Moments_Img,"Leaf_Contour_Moments_Img",Size(30,30));

        //windKf.init(2, 2);
    }

    bool Wind::wind(rm::Frame img, float start_time, float time, cv::Point3f &aim, rm::ColorChannels enemy, rm::Mode mode, float timestemp, float last_timestemp)
    {
        bool OUT = false;
        recorder += 1;//cout<<"0000000000000000000"<<endl;
        float last_delta_angle=0x3f3f3f3f;
        draw_show = img.img.clone();//cout<<"5555555555555555555"<<endl;


        auto t1 = std::chrono::high_resolution_clock::now();//cout<<"6666666666666666666666"<<endl;

        cv::Mat img1 = to_thresold_wing(img.img, enemy);//


        cv::RotatedRect armor;
        bool flag;
        flag = to_wing_center(img1, armor);//cout<<"5555555555555555555"<<endl;
Log(14,"center:   "<<gggg);
         //   if (flag)
        if (flag)
        {
            armor_angle.push_back(armor.angle);

            if (armor.angle > last_armor.angle)
                DirectionCount++;
            else
                DirectionCount--;

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
                DIRECTION = CLOCK_NONE;

            float delta_angle = fabs(armor.angle - last_armor.angle);

            stream << armor.angle <<endl;

            if (delta_angle < 2 * CV_PI / 7 || delta_angle > 2 * CV_PI - 2 * CV_PI / 10)
                is_hit_success = false;
            else
            {
                is_hit_success = true;
                windkf.clear();
                i++;
            }

            Log(16, "delta angle : " << delta_angle / CV_PI << "     ");
            Log(17, "is_hit_success : " << i << "     ");

            std::vector<cv::Point2f> pre_points;
            if (mode == Mode::HIT_RUNE_MIN)
            {
                if (recorder % Params.SEQ_MODE_MIN == 0 && circle_center != cv::Point2f(0, 0))
                {
                    float theta = time * CV_PI / 3; //time:s  20*pi /60

                    //cv::Mat warpMatrix;
                    //warpCenter = to_perspective(armor, warpMatrix);

                    pre_points = to_turn_points(circle_center, theta);
                    //to_perspective(pre_points, warpMatrix);

                    windSolve.solve(windSolve.POINT_3D_OF_RUNE_BIG, pre_points);
                    aim = windSolve.get_real_Point3f(0, 0);

                    clear();
                    OUT = true;
                }
#if WIND_TARGET_PREDICT && DEBUG
                for (size_t i = 0; i < pre_points.size(); i++)
                    cv::line(draw_show, pre_points[i], pre_points[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
                RS.to_show_img(draw_show, "Wind_Target_Predict", cv::Size(640, 480));
#endif
            }

            if (mode == Mode::HIT_RUNE_MAX)
            {
                if (recorder % Params.SEQ_MODE_MAX == 0) //start != 0)
                {
                    float theta = 0;
                    if (circle_center != cv::Point2f(0, 0))
                    {
#if !WIND_KF
                        theta = to_time_angle(start_time, time);
#endif
                        cv::Mat warpMatrix;
                        warpCenter = to_perspective(armor, warpMatrix);//zhe li hui bao zhan
#if WIND_KF
                        if (!is_hit_success)
                        {

                            float round_theta = armor.angle;
                            windkf.loop_limit(round_theta);
                            windkf.Measurement = round_theta;
                            windkf.TimeStamp = img.timeStamp;
                            //if (first_frame == 0) {return false; first_frame = 1;}
                            windkf.estimate_phi();
                            windkf.predict(theta, img.timeStamp + time * 1000);
                            Log(18, "ESEIMATE_PHI_SUCCEED! :" << windkf.phi_hat << "          ");

                            if (theta < (-0.785 + 1.305) * time && theta > (0.785 + 1.305) * time)
                                theta = 1.305 * time;

                        }

#endif
                        if (theta != 0)
                        {
                            //std::vector<cv::Point2f>
                            pre_points = to_turn_points(warpCenter, theta);
                            to_perspective(pre_points, warpMatrix);

                            windSolve.solve(windSolve.POINT_3D_OF_RUNE_BIG, pre_points);
                            aim = windSolve.get_real_Point3f(0, 0);

                            //for (size_t i = 0; i < pre_points.size(); i++)
                            //    cv::line(draw_show, pre_points[i], pre_points[(i + 1) % 4], cv::Scalar(0, 0, 255), 3);
                            clear();
                            OUT = true;
                        }
                        else
                            OUT = false;

#if WIND_TARGET_PREDICT && DEBUG
                        Log(19,pre_points.size());
                        for (size_t i = 0; i < pre_points.size(); i++){

                            cv::line(draw_show, pre_points[i], pre_points[(i + 1) % 4], cv::Scalar(0, 255, 255), 3);
                        }
                        RS.to_show_img(draw_show, "Wind_Target_Predict_Max", cv::Size(640, 480));
#endif
                    }
                }
            }
            last_delta_angle = delta_angle;
            last_armor = armor;
            last_circle_center = circle_center;
            std::vector<cv::Point2f>().swap(wing_corners);
        }
        else{
            OUT = false;//gggg++;
}
        LastImgTimestamp = img.timeStamp;

        auto t2 = std::chrono::high_resolution_clock::now();

        // Log(6, "Capture period: " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms");

        return OUT;
    }

    cv::Mat Wind::to_thresold_wing(cv::Mat img, rm::ColorChannels enemy)
    {
       //cout<<img.rows<<"+111111111+"<<img.rows<<"11111111111111111 "<<endl;
        cv::Mat img1, img2, channels[3];
        cv::split(img, channels);
        img1 = enemy == rm::ColorChannels::BLUE ? channels[2] - channels[0] : channels[0] - 0*channels[2];

        cv::threshold(img1, img1, 90, 255, cv::THRESH_BINARY);
//cout<<"999999999999999&"<<endl;
        //        img1 = WindSubtractThresold(img,enemy);

        int elementSize = 2;
        cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1));//
        cv::dilate(img1, img1, element);//cout<<"--------------------&"<<endl;
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

    bool Wind::to_wing_center(cv::Mat img, cv::RotatedRect &armor, bool is_using_tradition)
    {
        std::vector<std::vector<cv::Point>> Contours;
        std::vector<cv::Vec4i> Hierarchy;
        cv::findContours(img, Contours, Hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        cv::Point2f points[4];
        std::vector<std::pair<cv::Point2f, double>> center_distance;
        std::vector<std::pair<int, double>> idx_value;

        bool is_find_armor = false;

        cv::RotatedRect armor_rect, leaf_rect, leaf_rect_temp, armor_rect_temp;

        for (size_t i = 0; i < Contours.size(); i++)
        {
            if (Hierarchy[i][3] < 0)
            {
                cv::RotatedRect rect = minAreaRect(Contours[i]);
                center_distance.push_back(std::make_pair(rect.center, 0));
            }
        }

        for (size_t i = 0; i < Contours.size(); i++)
        {

            //            if (cv::contourArea(Contours[i]) < Params.ARMOR_AREA_MIN ||
            //                cv::contourArea(Contours[i]) > Params.ARMOR_AREA_MAX)
            //                continue;

            if (Contours[i].size() < 6 || Hierarchy[i][3] < 0 || Contours[Hierarchy[i][3]].size() < 6)
                continue;

            armor_rect_temp = cv::fitEllipse(Contours[i]);
            double width = armor_rect_temp.size.width;
            double height = armor_rect_temp.size.height;
            double ratio = height / width;

            //              if (armor_rect.size.area() > 1000)
            //                  continue;

            if (ratio < Params.ARMOR_W_H_RATIO_MIN || ratio > Params.ARMOR_W_H_RATIO_MAX)
                continue;

            leaf_rect_temp = cv::minAreaRect(Contours[Hierarchy[i][3]]);

            //              if (leaf_rect.size.area() > 5000)
            //                  continue;

            if (cv::contourArea(Contours[Hierarchy[i][3]]) / leaf_rect_temp.size.area() > Params.WING_FIT_DEGREE_MAX)
                continue;

            armor_rect = armor_rect_temp;
            leaf_rect = leaf_rect_temp;

            is_find_armor = true;
            break;
        }

        if (center_distance.size() == 0 || !is_find_armor)
            return false;

        circle_center = (1 - Params.CENTER_VECTOR_RATIO) * armor_rect.center + Params.CENTER_VECTOR_RATIO * leaf_rect.center;

#if (WIND_TARGET_CIRCLE_CENTER && DEBUG)
        RS.to_draw_RotatedRect(armor_rect, draw_show, cv::Scalar(0, 0, 255));
        RS.to_draw_RotatedRect(leaf_rect, draw_show, cv::Scalar(0, 255, 255));

        cv::circle(draw_show, armor_rect.center, 2, cv::Scalar(0, 255, 255), 4);
        cv::circle(draw_show, leaf_rect.center, 2, cv::Scalar(0, 255, 255), 4);
#endif

        if (circle_center != cv::Point2f(0, 0))
        {
            for (size_t i = 0; i < center_distance.size(); i++)
            {
                double distance = DIST(circle_center, center_distance[i].first);
                center_distance[i].second = distance;
            }

            std::sort(center_distance.begin(), center_distance.end(), [](const std::pair<cv::Point2f, double> &ld1, const std::pair<cv::Point2f, double> &ld2)
                      { return ld1.second < ld2.second; });
            circle_center = center_distance[0].first;

            double radius = sqrt(pow((armor.center.x - circle_center.x), 2) +
                                 pow((armor.center.y - circle_center.y), 2));

            if (armor.center.y < circle_center.y && armor.center.x > circle_center.x)
                armor.angle = asin((circle_center.y - armor.center.y) / radius);
            else if (armor.center.y < circle_center.y && armor.center.x < circle_center.x)
                armor.angle = -asin((circle_center.y - armor.center.y) / radius) + CV_PI;
            else if (armor.center.y > circle_center.y && armor.center.x > circle_center.x)
                armor.angle = asin((circle_center.y - armor.center.y) / radius) + CV_PI * 2;
            else if (armor.center.y > circle_center.y && armor.center.x < circle_center.x)
                armor.angle = -asin((circle_center.y - armor.center.y) / radius) + CV_PI;

#if (WIND_TARGET_CIRCLE_CENTER && DEBUG)
            cv::circle(draw_show, circle_center, 5, cv::Scalar(0, 0, 255), -1);
            RS.to_show_img(draw_show, "Wind_Target_Circle_Center", cv::Size(640, 480));
#endif

            for (const auto &point : points)
                wing_corners.push_back(point);
            return true;
        }
        else
            return false;
    }

    void Wind::read_temple()
    {


        Mat temp;
        for (int i=1;i<=5;i++){
            string filename = "/home/ubuntu/ros/src/zimiaoshiyan/src/Rune/temple";
            filename += to_string(i) + ".jpg";
            temp = imread(filename,-1);
            if (temp.empty() == 1)std::cout<<"temple empty!!!"<<std::endl;
            temple_vec.push_back(temp);
        }
        std::cout<<"read temple complete"<<std::endl;
        return;
    }

    bool Wind::to_wing_center(cv::Mat img, cv::RotatedRect &armor)
    {
        std::vector<std::vector<cv::Point>> Contours;
        std::vector<cv::Vec4i> Hierarchy;
        cv::findContours(img, Contours, Hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        cv::Point2f points[4];
        std::vector<std::pair<cv::Point2f, double>> center_distance;
        std::vector<std::pair<int, double>> idx_value;

        cv::RotatedRect armor_rect, leaf_rect, leaf_rect_2;

        for (size_t i = 0; i < Contours.size(); i++)
        {
            if (Hierarchy[i][3] < 0)
            {
                cv::RotatedRect rect = minAreaRect(Contours[i]);
                Rect yyy = rect.boundingRect();
                //rectangle(draw_show,yyy,Scalar(0,0,255),2);
                center_distance.push_back(std::make_pair(rect.center, 0));
            }
        }
        for (size_t i = 0; i < Contours.size(); i++)
        {
            //cout<<"dsfdsfsdfsdsfd"<<Contours.size();
            bool flag2 = Hierarchy[i][3] < 0 && Hierarchy[i][2] > 0 && Contours[Hierarchy[i][2]].size() > 5;
            if (flag2)
            {
                leaf_rect_2 = cv::minAreaRect(Contours[i]);
                //leaf_rect_2 = cv::fitEllipse(Contours[i]);
                //cout<<"***"<<leaf_rect_2.size<<endl;
                if (cv::contourArea(Contours[i]) / leaf_rect_2.size.area() < Params.WING_FIT_DEGREE_MAX)
                {
                    //cout<<"////////"<<endl;
                    Rect ttemp = leaf_rect_2.boundingRect();
                    RS.makeRectSafe(ttemp, cv::Size(WIDTH, HEIGHT));

                    //cout<<ttemp.size()<<"?????"<<endl;
                    cv::Mat temp_leaf = img(ttemp);
                    //cout<<endl<<temp_leaf.cols<<"   ====  "<<temp_leaf.rows<<endl<<endl;
                    resize(temp_leaf, temp_leaf, Size(30, 30));

                    double value = 0;
                    for (int j=0;j<temple_vec.size();j++){
                        Mat resized_temple;
                        //cout<< "-----+++----"<<temple_vec[j].size()<<endl<<endl;
                        resize(temple_vec[j], resized_temple,Size(30,30));
                        value += cv::matchShapes(resized_temple, temp_leaf, 1, 0);
                    }
                    value = value / double(5.00);

                    Log(11,"match value++++++:   "<< value);
                    if (value < Params.WING_LEAF_FIT_MOMENTS_VALUE_MAX)
                    {
                        leaf_rect = cv::minAreaRect(Contours[i]);
                        armor = cv::fitEllipse(Contours[Hierarchy[i][2]]);
                        armor_origin = armor;
                        armor_rect = armor;
                        //Rect uuu = armor_rect.boundingRect();
                        //rectangle(draw_show,uuu,Scalar(0,255,0),2);
                        armor.points(points);

                        break;
                    }
                }
            }
        }

        if (center_distance.size() == 0)
            return false;

        circle_center = (1 - Params.CENTER_VECTOR_RATIO) * armor_rect.center + Params.CENTER_VECTOR_RATIO * leaf_rect.center;
//cout<<"++++++++33----3++++++++"<<endl;
#if (WIND_TARGET_CIRCLE_CENTER && DEBUG)
        //RS.to_draw_RotatedRect(armor_rect, draw_show, cv::Scalar(0, 0, 255));
        //RS.to_draw_RotatedRect(leaf_rect, draw_show, cv::Scalar(0, 255, 255));
        //        Rect ooo = leaf_rect.boundingRect();
        //                Mat oooo = img(ooo);
        //                kkkk++;
        //                Log(9,"kkkk : "<< kkkk);
        //                string namee = "../heu_-vision_2020/shapes/ "+to_string(kkkk) + ".jpg";
        //                //imwrite(namee, oooo);
        //cv::circle(draw_show, armor_rect.center, 2, cv::Scalar(0, 255, 255), 4);
        //cv::circle(draw_show, leaf_rect.center, 2, cv::Scalar(0, 255, 255), 4);
#endif

        bool flag = 0;
        Log(12,"center size:    "<<center_distance.size());
        if (circle_center != cv::Point2f(0, 0))
        {
            for (size_t i = 0; i < center_distance.size(); i++)
            {
                double distance = DIST(circle_center, center_distance[i].first);
                center_distance[i].second = distance;
            }

            std::sort(center_distance.begin(), center_distance.end(), [](const std::pair<cv::Point2f, double> &ld1, const std::pair<cv::Point2f, double> &ld2)
                      { return ld1.second < ld2.second; });
            circle_center = center_distance[0].first;

            double radius = sqrt(pow((armor.center.x - circle_center.x), 2) +
                                 pow((armor.center.y - circle_center.y), 2));

            if (armor.center.y < circle_center.y && armor.center.x > circle_center.x)
                armor.angle = asin((circle_center.y - armor.center.y) / radius);
            else if (armor.center.y < circle_center.y && armor.center.x < circle_center.x)
                armor.angle = -asin((circle_center.y - armor.center.y) / radius) + CV_PI;
            else if (armor.center.y > circle_center.y && armor.center.x > circle_center.x)
                armor.angle = asin((circle_center.y - armor.center.y) / radius) + CV_PI * 2;
            else if (armor.center.y > circle_center.y && armor.center.x < circle_center.x)
                armor.angle = -asin((circle_center.y - armor.center.y) / radius) + CV_PI;

            for (const auto &point : points)
                wing_corners.push_back(point);

            flag = true;
        }
        else
            flag = false;

#if (WIND_TARGET_CIRCLE_CENTER && DEBUG)
        cv::circle(draw_show, circle_center, 5, cv::Scalar(0, 0, 255), -1);
        RS.to_show_img(draw_show, "Wind_Target_Circle_Center", cv::Size(640, 480));
#endif
        return flag;
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
        cv::Point2f src[5], dst[5];
        for (size_t i = 0; i < wing_corners.size(); i++)
        {
            src[i] = wing_corners[i];
            cv::circle(draw_show, src[i], 4, RS.COLOR_ORANGE, -1);
        }
        src[4]=circle_center;

        dst[0] = cv::Point2f(0, 0);
        dst[1] = cv::Point2f(230, 0);
        dst[2] = cv::Point2f(230, 127);
        dst[3] = cv::Point2f(0, 127);
        dst[4] = cv::Point2f(115,763.5);

        warpMatrix = cv::getPerspectiveTransform(src, dst);

        cv::Mat_<double> mat_pt(3, 1);
        mat_pt(0, 0) = circle_center.x;
        mat_pt(1, 0) = circle_center.y;
        mat_pt(2, 0) = 1;

        cv::Mat mat_pt_view = warpMatrix * mat_pt;
        double a1 = mat_pt_view.at<double>(0, 0);
        double a2 = mat_pt_view.at<double>(1, 0);
        double a3 = mat_pt_view.at<double>(2, 0);

        for (size_t i = 0; i < wing_corners.size(); i++)
        {
            wing_corners[i] = dst[i];
            //cv::circle(draw_show, wing_corners[i], 4, RS.COLOR_BLUE, -1);
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
                temp_theta = -theta;
            }
            else
            {
                temp_theta = +theta;
            }

            double x = point.x - center.x;
            double y = point.y - center.y;

            temp.x = x * cos(temp_theta) + y * sin(temp_theta) + center.x;
            temp.y = -x * sin(temp_theta) + y * cos(temp_theta) + center.y;

            OUT.push_back(temp);
        }
        return OUT;
    }

    double Wind::to_time_angle(float start_time, float time)
    {
        //w = 0.785 * sin(1.884 * t) + 1.305 rad / s float theta2 = 0;
        float delta_time = time / 500000;
        float theta;
        for (float i = start_time; i <= time + start_time; i += delta_time)
        {
            float w = (0.785 * sin(1.884 * (i)) + 1.305);
            theta += w * delta_time;
        }

        return theta;
    }

}
