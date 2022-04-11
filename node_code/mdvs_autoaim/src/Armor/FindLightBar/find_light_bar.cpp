#include "mdvs_autoaim/Armor/armor.h"
#include <math.h>

namespace rm
{
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
    void DrawContoursParameter(cv::Mat &img, cv::Rect &TrackingROI, std::vector<cv::Point> Contour, cv::Scalar ContourColor,
                               cv::Point2f TextLeftCenter, std::string text, float parameter, Scalar TextColor)
    {
        for (int i = 0; i < Contour.size(); i++)
        {
            Contour[i].x += TrackingROI.x;
            Contour[i].y += TrackingROI.y;
        }
        static std::vector<std::vector<cv::Point>> wrongContours;
        char p[8];
        std::string paramtext;
        wrongContours.push_back(Contour);
        drawContours(img, wrongContours, -1, ContourColor, 1); //输入的轮廓组,每组轮廓必须由vector点集构成
        if (parameter != -1.0)
        {
            sprintf(p, "%.2f", parameter);
            paramtext = p;
            text += paramtext;
        }
        putText(img, text, Point(TextLeftCenter.x + TrackingROI.x - 40, TextLeftCenter.y + TrackingROI.y - 5),
                FONT_HERSHEY_SIMPLEX, 1, TextColor, 1, 4);
        wrongContours.pop_back();
    }
#endif

    std::vector<cv::RotatedRect> Armor::find_light_bar(cv::Mat &img, float imgTimeStamp)
    {
        std::vector<cv::RotatedRect> R;
        cv::Mat grayImg;
        cv::Mat grayImg1;
        float t1, t2, t3, t4, t5, t6;
#if (LIGHT_BAR_RECT && DEBUG)
        cv::Mat img_rect = img.clone();
#endif
#if (LIGHT_BAR_CONTOUR && DEBUG)
        cv::Mat img_contour = img.clone();
#endif

        t1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        SetTrackingROI(imgTimeStamp);
        img = img(TrackingROI);

        t2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        cv::split(img, channels); //分离色彩通道
        if (_enemy_color == RED)
        {
            //grayImg = channels.at(2) + channels.at(1) * 0.0 - 0.4 * channels.at(0);
            grayImg = channels.at(2) + channels.at(1) * 0.5 - 0.7 * channels.at(0);
//                         RS.to_show_img(channels.at(2) + channels.at(1)*0.5, "RED+GREEN", Size(WIDTH , HEIGHT ));
//                         RS.to_show_img(grayImg, "MIX", Size(WIDTH , HEIGHT ));
            cv::threshold(grayImg, grayImg, 160, 255, cv::THRESH_BINARY);
        }
        else
        {
            // grayImg = channels.at(0); //vediotest
            // grayImg = channels.at(0) + channels.at(1) * 0.0 - 0.0 * channels.at(2);
            grayImg = channels.at(0) + channels.at(1) * 0.75 - 0.0 * channels.at(2);
//                        RS.to_show_img(channels.at(0) + channels.at(1) * 0.0, "BLUE+GREEN", Size(WIDTH , HEIGHT ));
//                        RS.to_show_img(grayImg, "MIX", Size(WIDTH , HEIGHT ));
            cv::threshold(grayImg, grayImg, 235, 255, cv::THRESH_BINARY);
        }
        t3 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        // 色彩图膨胀腐蚀
        if (_enemy_color == RED)
        {
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            dilate(grayImg, grayImg, element);
            erode(grayImg, grayImg, element);
        }
#if LIGHT_BAR_BINARY && DEBUG
        RS.to_show_img(grayImg, "LIGHT_BAR_BINARY", Size(grayImg.rows, grayImg.cols));
#endif

        t4 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        std::vector<std::vector<cv::Point>> lightContours;
        cv::findContours(grayImg, lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        t5 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
#if LIGHT_BAR_SPLIT_ERODE && DEBUG
        RS.to_show_img(grayImg, "LIGHT_BAR_SPLIT_ERODE", Size(grayImg.rows, grayImg.cols));
#endif
        for (const auto &contour : lightContours)
        {
            float contour_Area = contourArea(contour);
            // 筛选轮廓
            if (contour.size() <= 5)
            {
                //#if(LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                //                DrawContoursParameter(img_contour,TrackingROI, contour, Scalar(144,238,144),
                //                                contour[0], "6points",-1.0, Scalar(255, 255, 255));
                //#endif
                continue;
            }
            if (contour_Area < D_Params.CONTOUR_AREA_MIN)
            {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                      contour[0], "sz", contour_Area, Scalar(255, 255, 255));
#endif
                continue;
            }
            if (contour_Area > D_Params.CONTOUR_AREA_MAX)
            {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR && 0)
                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                      contour[0], "SZ", contour_Area, Scalar(255, 255, 255));
#endif
                continue;
            }

#if USE_MIX_ROTATED_RECT
            cv::RotatedRect lightRec_temp = cv::minAreaRect(contour);
            cv::RotatedRect lightRecEllipse = cv::fitEllipse(contour);

            cv::Point2f *temp_point = new cv::Point2f[4];
            lightRec_temp.points(temp_point);
            float dist01 = (temp_point[0].x - temp_point[1].x) * (temp_point[0].x - temp_point[1].x) +
                           (temp_point[0].y - temp_point[1].y) * (temp_point[0].y - temp_point[1].y);
            float dist12 = (temp_point[1].x - temp_point[2].x) * (temp_point[1].x - temp_point[2].x) +
                           (temp_point[1].y - temp_point[2].y) * (temp_point[1].y - temp_point[2].y);

            // cv::RotatedRect lightRec(lightRec_temp.center, lightRec_temp.size, lightRecEllipse.angle);
            cv::RotatedRect lightRec(lightRec_temp.center, lightRec_temp.size, lightRecEllipse.angle);
            bool position_flag= 0;
            for(int j=0;j<3;j++){
                if(grayImg.cols/2 - fabsf(grayImg.cols / 2 - temp_point[j].x) < 3||grayImg.rows/2 - fabsf(grayImg.rows / 2 - temp_point[j].y) < 3){
                    position_flag = 1;
                    break;
                }
            }
            if (position_flag)
            {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                      lightRec.center, "side", -1.0, Scalar(255, 255, 255));
#endif
                continue;
            }
            //            if (LightBarRect.size.height / LightBarRect.size.width < D_Params.CONTOUR_H_W_RATIO_MIN_LONG ||
            //                LightBarRect.size.height / LightBarRect.size.width > D_Params.CONTOUR_H_W_RATIO_MAX)
            //                continue;

            if (contour_Area / lightRec.size.area() < D_Params.CONTOUR_FIT_DEGREE_MIN * (1 - 0.3 * (_enemy_color == RED)))
            {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                      lightRec.center, "rect", contour_Area / lightRec.size.area(), Scalar(255, 255, 255));
#endif
                continue;
            }

            if (lightRec.angle < 140 && lightRec.angle > 40)
            {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                      lightRec.center, "angle", lightRec.angle, Scalar(255, 255, 255));
#endif
                continue;
            }

            if (lightRec.angle > 90)
                lightRec.angle -= 180;

            if (dist01 > dist12)
            {
                cv::RotatedRect LightBarRect((cv::Point2f)TrackingROI.tl() + lightRec.center, lightRec.size, lightRec.angle);

                if (LightBarRect.size.height < D_Params.LENTH)
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN - 0.075 * (D_Params.LENTH - lightRec.size.height);
                else
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN;

                if (LightBarRect.size.height / LightBarRect.size.width < D_Params.CONTOUR_H_W_RATIO_MIN_LONG ||
                    LightBarRect.size.height / LightBarRect.size.width > D_Params.CONTOUR_H_W_RATIO_MAX)
                {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                    DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                          lightRec.center, "HW", LightBarRect.size.height / LightBarRect.size.width,
                                          Scalar(255, 255, 255));
#endif
                    continue;
                }
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                //                DrawContoursParameter(img_contour,TrackingROI, contour, Scalar(255, 0, 255),
                //                                lightRec.center,"",LightBarRect.size.height,
                //                                  Scalar(255, 0, 255));

                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(255, 0, 255),
                                      lightRec.center, "", LightBarRect.size.height / LightBarRect.size.width,
                                      Scalar(255, 0, 255));
#endif

                R.push_back(LightBarRect);
            }
            else
            {
                lightRec.angle += 90;
                cv::Point2f *p = new cv::Point2f[4];
                lightRec.points(p);
                cv::RotatedRect LightBarRect((cv::Point2f)TrackingROI.tl() + p[1],
                                             (cv::Point2f)TrackingROI.tl() + p[2],
                                             (cv::Point2f)TrackingROI.tl() + p[3]);

                if (LightBarRect.size.height < D_Params.LENTH)
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN - 0.075 * (D_Params.LENTH - lightRec.size.height);
                else
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN;

                if (LightBarRect.size.height / LightBarRect.size.width < D_Params.CONTOUR_H_W_RATIO_MIN_LONG ||
                    LightBarRect.size.height / LightBarRect.size.width > D_Params.CONTOUR_H_W_RATIO_MAX)
                {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                    DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                          lightRec.center, "HW", LightBarRect.size.height / LightBarRect.size.width,
                                          Scalar(255, 255, 255));
#endif
                    continue;
                }
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                //                DrawContoursParameter(img_contour,TrackingROI, contour, Scalar(255, 0, 255),
                //                                lightRec.center,"",LightBarRect.size.height,
                //                                  Scalar(255, 0, 255));

                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(255, 0, 255),
                                      lightRec.center, "", LightBarRect.size.height / LightBarRect.size.width,
                                      Scalar(255, 0, 255));
#endif

                R.push_back(LightBarRect);
            }
#else
            cv::RotatedRect lightRec = cv::minAreaRect(contour);

            if (fabsf(grayImg.cols / 2 - lightRec.center.x) / (grayImg.cols / 2) > 0.95f)
            {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                      lightRec.center, "side", -1.0, Scalar(255, 255, 255));
#endif
                continue;
            }

            if (contour_Area / lightRec.size.area() < D_Params.CONTOUR_FIT_DEGREE_MIN * (1 - 0.3 * (_enemy_color == RED)))
            {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                      lightRec.center, "rect", contour_Area / lightRec.size.area(), Scalar(255, 255, 255));
#endif
                continue;
            }

            if (lightRec.size.height > lightRec.size.width)
            {
                if (-lightRec.angle > 40)
                {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                    DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                          lightRec.center, "angle 1:", lightRec.angle, Scalar(255, 255, 255));
#endif
                    continue;
                }

                if (lightRec.size.height < D_Params.LENTH)
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN - 0.075 * (D_Params.LENTH - lightRec.size.height);
                else
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN;

                if (lightRec.size.height / lightRec.size.width < D_Params.CONTOUR_H_W_RATIO_MIN_LONG ||
                    lightRec.size.height / lightRec.size.width > D_Params.CONTOUR_H_W_RATIO_MAX)
                {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                    DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                          lightRec.center, "HW", lightRec.size.height / lightRec.size.width,
                                          Scalar(255, 255, 255));
#endif
                    continue;
                }
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                //                DrawContoursParameter(img_contour,TrackingROI, contour, Scalar(255, 0, 255),
                //                                lightRec.center,"",LightBarRect.size.height,
                //                                  Scalar(255, 0, 255));

//                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
//                                      lightRec.center, "angle1:", lightRec.angle, Scalar(255, 255, 255));
#endif
                cv::RotatedRect LightBarRect((cv::Point2f)TrackingROI.tl() + lightRec.center, lightRec.size, lightRec.angle);
                R.push_back(LightBarRect);
            }
            else
            {
                if (-lightRec.angle < 50)
                {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                    DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                          lightRec.center, "angle 2:", lightRec.angle, Scalar(255, 255, 255));
#endif
                    continue;
                }

                if (lightRec.size.width < D_Params.LENTH)
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN - 0.075 * (D_Params.LENTH - lightRec.size.height);
                else
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN;

                if (lightRec.size.width / lightRec.size.height < D_Params.CONTOUR_H_W_RATIO_MIN_LONG ||
                    lightRec.size.width / lightRec.size.height > D_Params.CONTOUR_H_W_RATIO_MAX)
                {
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                    DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
                                          lightRec.center, "HW", lightRec.size.width / lightRec.size.height,
                                          Scalar(255, 255, 255));
#endif
                    continue;
                }
#if (LIGHT_BAR_CONTOUR && DEBUG && WRONG_LIGHTBAR)
                //                DrawContoursParameter(img_contour,TrackingROI, contour, Scalar(255, 0, 255),
                //                                lightRec.center,"",LightBarRect.size.height,
                //                                  Scalar(255, 0, 255));

//                DrawContoursParameter(img_contour, TrackingROI, contour, Scalar(144, 238, 144),
//                                      lightRec.center, "angle2:", lightRec.angle, Scalar(255, 255, 255));
#endif
                float tempWidth = lightRec.size.width;
                lightRec.size.width = lightRec.size.height;
                lightRec.size.height = tempWidth;
                cv::RotatedRect LightBarRect((cv::Point2f)TrackingROI.tl() + lightRec.center, lightRec.size, lightRec.angle + 90);
                R.push_back(LightBarRect);
            }
#endif

#if (LIGHT_BAR_RECT && DEBUG)
            cv::Point2f *p = new cv::Point2f[4];
            cv::RotatedRect &tempLightBarRect = R[R.size() - 1];
            tempLightBarRect.points(p);
            for (int i = 0; i < 4; i++)
            {
                cv::line(img_rect, p[i], p[(i + 1) % 4], cv::Scalar(255, 0, 255), 2, 0);
            }
#endif
#if (LIGHT_BAR_CONTOUR && DEBUG)
            for (int i = 0; i < contour.size(); i++)
            {
                cv::Point2f P;
                P.x = contour[i].x + TrackingROI.x;
                P.y = contour[i].y + TrackingROI.y;
                cv::circle(img_contour, P, 1, cv::Scalar(255, 0, 255));
            }

#endif
        }
        t6 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
/*
        Log(12, "SetTrackingROI cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t2 - t1)).count() << " ms   ");
        Log(13, "split cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t3 - t2)).count() << " ms   ");
        Log(14, "dilate cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t4 - t3)).count() << " ms   ");
        Log(15, "findContours cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t5 - t4)).count() << " ms   ");
        Log(16, "fit cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t6 - t5)).count() << " ms   ");
        Log(17, "total cost: " << (static_cast<std::chrono::duration<float, std::milli>>(t6 - t1)).count() << " ms   ");
*/
#if (LIGHT_BAR_RECT && DEBUG)
        RS.to_show_img(img_rect, "Light_Bar_Rect", RS.RSIZE);
#endif
#if (LIGHT_BAR_CONTOUR && DEBUG)
        cv::rectangle(img_contour, TrackingROI, cv::Scalar(255, 255, 255), 3, 0);
        RS.to_show_img(img_contour, "Light_Bar_Contour", Size(WIDTH, HEIGHT));
#endif

        sort(R.begin(), R.end(), [](const cv::RotatedRect &ld1, const cv::RotatedRect &ld2) { // Lambda函数,作为sort的cmp函数
            return ld1.center.x < ld2.center.x;
        });
        return R;
    }

} // namespace rm
