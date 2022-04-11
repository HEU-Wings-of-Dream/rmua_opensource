#include "../armor.h"
#include "math.h"

namespace rm
{
#if (RECT_MATCH && DEBUG && WRONG_RECT)
    void DrawRectParameter(cv::Mat &img, cv::RotatedRect leftLight, cv::RotatedRect rightLight, cv::Scalar RectColor,
                           cv::Point2f TextLeftCenter, std::string text, float parameter, Scalar TextColor)
    {
        std::vector<cv::Point2f> rect;
        char p[10];
        std::string paramtext;
        rect.push_back(cv::Point2f(leftLight.center.x - cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height, leftLight.center.y - sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height));
        rect.push_back(cv::Point2f(rightLight.center.x - cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height, rightLight.center.y - sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height));
        rect.push_back(cv::Point2f(rightLight.center.x + cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height, rightLight.center.y + sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height));
        rect.push_back(cv::Point2f(leftLight.center.x + cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height, leftLight.center.y + sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height));
        for (int j = 0; j < 4; j++)
        {
            cv::line(img, rect[j], rect[(j + 1) % 4], RectColor, 2, 0);
        }
        cv::circle(img, rect[0], 2, RectColor, 2, 0);
        cv::circle(img, rect[1], 5, RectColor, 2, 0);
        cv::circle(img, rect[2], 8, RectColor, 2, 0);
        cv::circle(img, rect[3], 11, RectColor, 2, 0);
        if (parameter != -1.0)
        {
            if (parameter > 1)
            {
                sprintf(p, "%.2f", parameter);
                paramtext = p;
                text += parameter;
            }
            else
            {
                paramtext = std::to_string(parameter);
                text += paramtext;
            }
        }
        putText(img, text, Point(TextLeftCenter.x - 40, TextLeftCenter.y - 5),
                FONT_HERSHEY_SIMPLEX, 1, TextColor, 1, 4);
    }
#endif
    std::vector<std::vector<cv::Point2f>> Armor::rectangle_match(std::vector<cv::RotatedRect> &vrect)
    {
        std::vector<std::vector<cv::Point2f>> R;
#if (RECT_MATCH && DEBUG)
        cv::Mat rect_match = ImageFrame.img.clone();
#endif
        for (size_t i = 0; (i + 1) < vrect.size(); i++)
        {
            for (size_t j = 0; i + 1 + j <= vrect.size(); j++)
            {
                if (i + 1 + j == vrect.size())
                    break;
                const cv::RotatedRect &leftLight = vrect[i];
                const cv::RotatedRect &rightLight = vrect[i + 1 + j];

                // The order is bottomLeft, topLeft, topRight, bottomRight
                cv::Point2f leftLightPoints[4], rightLightPoints[4];
                leftLight.points(leftLightPoints);
                rightLight.points(rightLightPoints);

                cv::Vec2f v0(leftLightPoints[0].x - leftLightPoints[1].x, leftLightPoints[0].y - leftLightPoints[1].y);
                cv::Vec2f v1(rightLight.center.x - leftLight.center.x, rightLight.center.y - leftLight.center.y);
                cv::Vec2f v0Normalize = cv::normalize(v0);
                cv::Vec2f v1Normalize = cv::normalize(v1);

                if (fabsf(v0Normalize.dot(v1Normalize)) > 0.3)
                {
                    //  std::cout<<"111"<<fabsf(v0Normalize.dot(v1Normalize))<<std::endl;
#if (RECT_MATCH && DEBUG && WRONG_RECT) //巨多奇怪形状
                    DrawRectParameter(rect_match, leftLight, rightLight, Scalar(144, 238, 144),
                                      leftLightPoints[1], "acute", fabsf(v0Normalize.dot(v1Normalize)), Scalar(255, 255, 255));
#endif
                    continue; // Eliminate acute angle
                }

                float LeftLightLen = sqrt(powf((leftLightPoints[0].x - leftLightPoints[1].x), 2) + powf((leftLightPoints[0].y - leftLightPoints[1].y), 2));
                float RightLightLen = sqrt(powf((rightLightPoints[0].x - rightLightPoints[1].x), 2) + powf((rightLightPoints[0].y - rightLightPoints[1].y), 2));

                float angleDiff_ = fabs(leftLight.angle - rightLight.angle);
                // real长度比
                float LenRatio = cv::min(LeftLightLen, RightLightLen) / cv::max(LeftLightLen, RightLightLen);

                //左右灯条长度的平均值
                float meanLen = (LeftLightLen + RightLightLen) / 2;
                //            Log(17, "lenth: " << meanLen );
                //            Log(18, "ANGEL: " << angleDiff_ );

                if (meanLen < M_Params.LENTH)
                    M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX + 0 * (M_Params.LENTH - meanLen);
                else
                {
                    M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX;
                }
                //筛选
                if ((angleDiff_ > M_Params.ANGLE_DIFF_MAX_LONG * (1 + 0.6 * (_enemy_color == RED)) || LenRatio < M_Params.LENGTH_RATIO))
                {

#if (RECT_MATCH && DEBUG && WRONG_RECT)
                    DrawRectParameter(rect_match, leftLight, rightLight, Scalar(255, 255, 240),
                                      leftLightPoints[1], "LenRatio", LenRatio, Scalar(255, 255, 255));
                    char p[10];
                    sprintf(p, "%.2f", angleDiff_);
                    std::string angleDiff_show = p;
                    putText(rect_match, "angleDiff" + angleDiff_show, Point(leftLightPoints[1].x - 40, leftLightPoints[1].y + 30),
                            FONT_HERSHEY_PLAIN, 4, Scalar(255, 255, 255), 0.5, 4);
#endif
                    continue;
                }
                //筛选

                //左右灯条相距距离
                float dist = sqrt(powf((leftLight.center.x - rightLight.center.x), 2) + powf((leftLight.center.y - rightLight.center.y), 2));

                //相距距离与灯条长度比值
                float ratio = dist / cv::min(LeftLightLen, RightLightLen);

                if (meanLen < M_Params.LENTH)
                    M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX + 0.05 * (M_Params.LENTH - meanLen);
                else
                {
                    M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX;
                }

                if (ratio > M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG || ratio < M_Params.MEAN_LENGTH_DIST_RATIO_MIN)
                {
                    // 大装甲正视比例为 4.09
#if (RECT_MATCH && DEBUG && WRONG_RECT)
                    DrawRectParameter(rect_match, leftLight, rightLight, Scalar(107, 125, 139),
                                      leftLightPoints[1], "ratio", ratio, Scalar(255, 255, 255));
#endif
                    continue;
                }

                //                 if (j == 1)
                //                 {
                //                     // second check
                //                     const cv::RotatedRect &templeftLight = vrect[i + j];
                //                     uint8_t matchLeft = 1;
                //                     // The order is bottomLeft, topLeft, topRight, bottomRight
                //                     cv::Point2f templeftLightPoints[4], temprightLightPoints[4];
                //                     templeftLight.points(templeftLightPoints);
                //                     rightLight.points(temprightLightPoints);

                //                     cv::Vec2f tempv0(templeftLightPoints[0].x - templeftLightPoints[1].x, templeftLightPoints[0].y - templeftLightPoints[1].y);
                //                     cv::Vec2f tempv1(rightLight.center.x - templeftLight.center.x, rightLight.center.y - templeftLight.center.y);
                //                     cv::Vec2f tempv0Normalize = cv::normalize(tempv0);
                //                     cv::Vec2f tempv1Normalize = cv::normalize(tempv1);

                //                     if (fabsf(tempv0Normalize.dot(tempv1Normalize)) > 0.3)
                //                         matchLeft = 0; // Eliminate acute angle

                //                     float tempLeftLightLen = sqrt(powf((templeftLightPoints[0].x - templeftLightPoints[1].x), 2) + powf((templeftLightPoints[0].y - templeftLightPoints[1].y), 2));
                //                     float tempRightLightLen = sqrt(powf((temprightLightPoints[0].x - temprightLightPoints[1].x), 2) + powf((temprightLightPoints[0].y - temprightLightPoints[1].y), 2));

                //                     float tempangleDiff_ = fabs(templeftLight.angle - rightLight.angle);
                //                     // real长度比
                //                     float tempLenRatio = cv::min(tempLeftLightLen, tempRightLightLen) / cv::max(tempLeftLightLen, tempRightLightLen);

                //                     //左右灯条长度的平均值
                //                     float tempmeanLen = (tempLeftLightLen + tempRightLightLen) / 2;

                //                     if (tempmeanLen < M_Params.LENTH)
                //                         M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX + 0.1 * (M_Params.LENTH - tempmeanLen);
                //                     else
                //                     {
                //                         M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX;
                //                     }
                //                     //筛选
                //                     if ((tempangleDiff_ > M_Params.ANGLE_DIFF_MAX_LONG * (1 + 0.6 * (_enemy_color == RED)) || tempLenRatio < M_Params.LENGTH_RATIO))
                //                         matchLeft = 0;
                //                     //筛选

                //                     //左右灯条相距距离
                //                     float tempdist = sqrt(powf((templeftLight.center.x - rightLight.center.x), 2) + powf((templeftLight.center.y - rightLight.center.y), 2));

                //                     //相距距离与灯条长度比值
                //                     float tempratio = tempdist / cv::min(tempLeftLightLen, tempRightLightLen);

                //                     if (tempmeanLen < M_Params.LENTH)
                //                         M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX + 1 * (M_Params.LENTH - tempmeanLen);
                //                     else
                //                     {
                //                         M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX;
                //                     }
                //                     if (tempratio > M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG || ratio < M_Params.MEAN_LENGTH_DIST_RATIO_MIN)
                //                         // 大装甲正视比例为 4.09
                //                         matchLeft = 0;
                //                     if (matchLeft == 1)
                //                     {
                //                         std::cout << "444" << std::endl;
                // #if (RECT_MATCH && DEBUG && WRONG_RECT)
                //                         DrawRectParameter(rect_match, leftLight, rightLight, Scalar(0, 255, 255),
                //                                           leftLightPoints[1], "match", tempratio, Scalar(255, 255, 255));
                // #endif
                //                         continue;
                //                     }
                //                 }

                std::vector<cv::Point2f> rect;
                float lengthScale;
                if (_enemy_color == RED)
                    lengthScale = M_Params.RED_LENGTH_SCALE;
                else
                    lengthScale = M_Params.BLUE_LENGTH_SCALE;
                    // The order is topLeft, topRight, bottomRight, bottomLeft
#if USE_WRONG_P4P_POINTS
                rect.push_back(cv::Point2f(leftLight.center.x - cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height, leftLight.center.y - sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height));
                rect.push_back(cv::Point2f(rightLight.center.x - cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height, rightLight.center.y - sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height));
                rect.push_back(cv::Point2f(rightLight.center.x + cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height, rightLight.center.y + sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height));
                rect.push_back(cv::Point2f(leftLight.center.x + cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height, leftLight.center.y + sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height));

#else
                rect.push_back(cv::Point2f(leftLight.center.x - cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height * lengthScale, leftLight.center.y - sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height * lengthScale));
                rect.push_back(cv::Point2f(rightLight.center.x - cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height * lengthScale, rightLight.center.y - sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height * lengthScale));
                rect.push_back(cv::Point2f(rightLight.center.x + cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height * lengthScale, rightLight.center.y + sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height * lengthScale));
                rect.push_back(cv::Point2f(leftLight.center.x + cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height * lengthScale, leftLight.center.y + sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height * lengthScale));
#endif
                R.push_back(rect);
                break;
            }
        }

#if (RECT_MATCH && DEBUG)

        for (int i = 0; i < R.size(); i++)
        {

            for (int j = 0; j < 4; j++)
            {
                cv::line(rect_match, R[i][j], R[i][(j + 1) % 4], cv::Scalar(0, 0, 255), 1, 0);
            }
            cv::circle(rect_match, R[i][0], 2, cv::Scalar(255, 255, 255), 2, 0);
            cv::circle(rect_match, R[i][1], 5, cv::Scalar(255, 255, 255), 2, 0);
            cv::circle(rect_match, R[i][2], 8, cv::Scalar(255, 255, 255), 2, 0);
            cv::circle(rect_match, R[i][3], 11, cv::Scalar(255, 255, 255), 2, 0);
        }
        cv::rectangle(rect_match, TrackingROI, cv::Scalar(255, 255, 255), 3, 0);

        RS.to_show_img(rect_match, "rect_match", cv::Size(WIDTH, HEIGHT));
        //  RS.to_show_img(rect_match(TrackingROI), "rect_match_ROI", cv::Size(WIDTH, HEIGHT));
#endif

#if USE_SVM
        auto svmt1 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
        if (R.size() > 0)
        {
            cv::Point2f src[4], dst[4];
            for (int i = 0; i <= 3; i++)
                src[i] = R[0][i] - cv::Point2f(TrackingROI.x, TrackingROI.y);
            dst[0] = cv::Point2f(0, 0);
            dst[1] = cv::Point2f(110, 0);
            dst[2] = cv::Point2f(110, 110);
            dst[3] = cv::Point2f(0, 110);

            //  std::cout << "-6:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt1 << std::endl;
            cv::Mat rot_mat, RotatedMat;
            rot_mat = cv::getPerspectiveTransform(src, dst);
            cv::warpPerspective(channels[1], RotatedMat, rot_mat, Size(110, 110));
            //    std::cout << "warpPerspective_cost" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt1 << std::endl;

            cv::Mat ROI = RotatedMat;
            auto svmt2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            int i = 10;
            // 65 20
            int lightbar_thresold = 50, lightbar_Max = 20;
            while (!(ROI.at<uchar>(i, 0) > lightbar_thresold))
            {
                i++;
            }
            // cout<<i;
            for (; ROI.at<uchar>(i, 0) > lightbar_thresold && i < 110; i++)
            {
                data = ROI.ptr<uchar>(i);
                for (int j = 0; data[j] > lightbar_thresold && j < 110; j++)
                {
                    // cout<<data[j]<<"  ";
                    data[j] = lightbar_Max;
                }
                for (int k = 109; data[k] > lightbar_thresold && k > 0; k--)
                {
                    data[k] = lightbar_Max;
                }
            }
            // std::cout << "lightbar-solve:::" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt2 << std::endl;
            auto svmt3 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();
            cv::medianBlur(ROI, ROI, 3);
            // std::cout << "medianBlur:::" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt3 << std::endl;
            auto svmt4 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            //    std::cout << "lightbar-pre-solve" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt4 << std::endl;

            //         auto svmt2 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            // svm_obj.GammaCorrection(ROI,ROI,0.4);
            //   std::cout << "GammaCorrection" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt2 << std::endl;

#if SVM_IMG
            RS.to_show_img(ROI, "ROI origin", Size(ROI.rows, ROI.cols));
#endif
            // cv::medianBlur(ROI,ROI,3);
            // std::cout << "-4:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            // cvtColor(ROI,ROI,COLOR_BGR2GRAY);

            // std::cout << "-3:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            //         int histSize = 256;
            //         float range[] = { 0,256 };
            //         const float* ranges = { range };
            //         cv::Mat hist;
            //         calcHist(&ROI, 1, 0, Mat(), hist, 1, &histSize, &ranges, true, false);
            //         normalize(hist, hist, 0, 1, NORM_MINMAX, -1, Mat());
            //         int left = 0, right = 0;
            //         int interal =80;
            //         for (int i = 5; i < 60; i++) {
            //            if (hist.at<float>(i) > hist.at<float>(left)) {left = i;}
            //            if (hist.at<float>(i+interal) > hist.at<float>(right+interal)) {right = i;}
            //         }
            //         right = right + interal;
            //         float mean = ((float)(right) - float(left)) *0.6 + (float)left;

            threshold(ROI, ROI, 0, 255, cv::THRESH_OTSU);
            // std::cout << "otsu_cost:::" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt4 << std::endl;
            //  std::cout << "img_solve_cost:::" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt1 << std::endl;

            //存数据集用
            //         if(!ROI.empty() && picnum %20 ==0){
            //            std::string mypath = "/home/star/numberpic/afternoon//1 ("+std::to_string(count)+").jpg";
            //            std::cout<<"1111wwweee";
            //            while(!cv::imwrite(mypath,ROI));
            //            count++;
            //          }
            //            picnum++;

#if SVM_IMG
            RS.to_show_img(ROI, "ROI", Size(ROI.rows, ROI.cols));
#endif
            //         Mat only_number(Scalar(255));
            //         Log(19,"cols: "<<only_number.cols);
            //         Log(21,"rows:"<<only_number.rows);

            //         std::vector < std::vector <cv::Point> > RotatedMat_contours;

            //         std::cout << "-2:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            //         cv::findContours(ROI,RotatedMat_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
            //         std::cout << "-1:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            //         std::vector<std::pair<int,int>> Area_index;
            //         for (size_t i = 0;i < RotatedMat_contours.size();i++){

            //             RotatedRect roi_rect = minAreaRect(RotatedMat_contours[i]);
            //             double place_ = (abs(roi_rect.center.x - ROI.cols/2.0)/(ROI.cols/2));
            //                 if (place_ > 0.25)continue;
            //                 if (roi_rect.size.area()/ROI.cols/ROI.rows < 0.1)
            //                     continue;
            //             Area_index.push_back(make_pair(contourArea(RotatedMat_contours[i]),i));

            //         }
            //         std::cout << "0:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            //         Log(19, "Area_index.size(): " << Area_index.size());
            //         if(Area_index.size() > 0)
            //         {
            //         std::sort(Area_index.begin(),Area_index.end(),[=](const std::pair<int,int>&p1,const std::pair<int,int>&p2)
            //         {
            //             return p1.first > p2.first;
            //         });
            //         std::cout << "1:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            //        Rect roi_rect = boundingRect(RotatedMat_contours[Area_index[0].second]);
            //        std::cout << "2:" << (uint32_t)(static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            //        RS.makeRectSafe(roi_rect,ROI.size());
            //        only_number = ROI(roi_rect);
            //        RS.to_show_img(only_number,"only_number",RS.RSIZE);
            //        std::cout << "3:" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count()-svmt1 << std::endl;
            // std::cout<<only_number.cols<<"    "<<only_number.rows<<"aaaaaaaaaaaaa"<<std::endl;
            auto svmt5 = (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count();

            if (ROI.cols != 1)
            {
                svm_obj.predict_number(ROI);
            }
            // std::cout << "predict-number" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt5 << std::endl;
            // std::cout << "TOTALCOST::" << (static_cast<std::chrono::duration<float, std::milli>>(std::chrono::high_resolution_clock::now() - _startTime)).count() -svmt1 << std::endl;
        }
#endif
        sort(R.begin(), R.end(),
             [&](const std::vector<cv::Point2f> &rect1, const std::vector<cv::Point2f> &rect2)
             {
                 // Lambda函数,作为sort的cmp函数
                 cv::Vec2f LeftHeightVec1(rect1[3].x - rect1[0].x, rect1[3].y - rect1[0].y);
                 cv::Vec2f LeftHeightVec2(rect2[3].x - rect2[0].x, rect2[3].y - rect2[0].y);
                 cv::Vec2f RightHeightVec1(rect1[2].x - rect1[1].x, rect1[2].y - rect1[1].y);
                 cv::Vec2f RightHeightVec2(rect2[2].x - rect2[1].x, rect2[2].y - rect2[1].y);

                 LeftHeightVec1 = cv::normalize(LeftHeightVec1);
                 LeftHeightVec2 = cv::normalize(LeftHeightVec2);
                 RightHeightVec1 = cv::normalize(RightHeightVec1);
                 RightHeightVec2 = cv::normalize(RightHeightVec2);

                 float AngleDiff1 = LeftHeightVec1.dot(RightHeightVec1);
                 float AngleDiff2 = LeftHeightVec2.dot(RightHeightVec2);
                 return AngleDiff1 > AngleDiff2;

                 float Rect1Area = contourArea(rect1);
                 float Rect2Area = contourArea(rect2);
                 return Rect1Area > Rect2Area;
             });
        return R;
    }

} // namespace rm
