#include "zimiaoshiyan/Armor/armor.h"
#include "math.h"

namespace rm
{
    std::vector<std::vector<cv::Point2f>> Armor::rectangle_match(std::vector<cv::RotatedRect> &vrect)
    {
        std::vector<std::vector<cv::Point2f>> R;

        for (size_t i = 0; (i + 1) < vrect.size(); i++)
        {
            for (size_t j = 0; j < 2; j++)
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

                if (fabsf(v0Normalize.dot(v1Normalize)) > 0.35)
                    continue; // Eliminate acute angle

                float LeftLightLen = sqrt(powf((leftLightPoints[0].x - leftLightPoints[1].x), 2) + powf((leftLightPoints[0].y - leftLightPoints[1].y), 2));
                float RightLightLen = sqrt(powf((rightLightPoints[0].x - rightLightPoints[1].x), 2) + powf((rightLightPoints[0].y - rightLightPoints[1].y), 2));

                float angleDiff_ = fabs(leftLight.angle - rightLight.angle);
                //real长度比
                float LenRatio = cv::min(LeftLightLen, RightLightLen) / cv::max(LeftLightLen, RightLightLen);

                //左右灯条长度的平均值
                float meanLen = (LeftLightLen + RightLightLen) / 2;
                //            Log(17, "lenth: " << meanLen );
                //            Log(18, "ANGEL: " << angleDiff_ );

                if (meanLen < M_Params.LENTH)
                    M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX + 0.1 * (M_Params.LENTH - meanLen);
                else
                {
                    M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX;
                }
                //筛选
                if ((angleDiff_ > M_Params.ANGLE_DIFF_MAX_LONG || LenRatio < M_Params.LENGTH_RATIO))
                {
                    Log(10, "LenRatio:" << LenRatio << "       ");
                    Log(11, "angleDiff_:" << angleDiff_ << "       ");
                    continue;
                }
                //筛选

                //左右灯条相距距离
                float dist = sqrt(powf((leftLight.center.x - rightLight.center.x), 2) + powf((leftLight.center.y - rightLight.center.y), 2));

                //相距距离与灯条长度比值
                float ratio = dist / cv::min(LeftLightLen, RightLightLen);

                if (meanLen < M_Params.LENTH)
                    M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX + 1 * (M_Params.LENTH - meanLen);
                else
                {
                    M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX;
                }
                //Log(12,"meanLen:"<<meanLen<<"       ");
                //Log(13,"MEAN_LENGTH_DIST_RATIO_MAX_LONG:"<<M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG<<"       ");
                if (ratio > M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG || ratio < M_Params.MEAN_LENGTH_DIST_RATIO_MIN)
                {
                    // 大装甲正视比例为 4.09
                    Log(12, "ratio:" << ratio << "       ");
                    continue;
                }

                if (j == 1)
                {
                    // second check
                    const cv::RotatedRect &templeftLight = vrect[i + j];
                    uint8_t matchLeft = 1;
                    // The order is bottomLeft, topLeft, topRight, bottomRight
                    cv::Point2f templeftLightPoints[4], temprightLightPoints[4];
                    templeftLight.points(templeftLightPoints);
                    rightLight.points(temprightLightPoints);

                    cv::Vec2f tempv0(templeftLightPoints[0].x - templeftLightPoints[1].x, templeftLightPoints[0].y - templeftLightPoints[1].y);
                    cv::Vec2f tempv1(rightLight.center.x - templeftLight.center.x, rightLight.center.y - templeftLight.center.y);
                    cv::Vec2f tempv0Normalize = cv::normalize(tempv0);
                    cv::Vec2f tempv1Normalize = cv::normalize(tempv1);

                    if (fabsf(tempv0Normalize.dot(tempv1Normalize)) > 0.35)
                        matchLeft = 0; // Eliminate acute angle

                    float tempLeftLightLen = sqrt(powf((templeftLightPoints[0].x - templeftLightPoints[1].x), 2) + powf((templeftLightPoints[0].y - templeftLightPoints[1].y), 2));
                    float tempRightLightLen = sqrt(powf((temprightLightPoints[0].x - temprightLightPoints[1].x), 2) + powf((temprightLightPoints[0].y - temprightLightPoints[1].y), 2));

                    float tempangleDiff_ = fabs(templeftLight.angle - rightLight.angle);
                    //real长度比
                    float tempLenRatio = cv::min(tempLeftLightLen, tempRightLightLen) / cv::max(tempLeftLightLen, tempRightLightLen);

                    //左右灯条长度的平均值
                    float tempmeanLen = (tempLeftLightLen + tempRightLightLen) / 2;

                    if (tempmeanLen < M_Params.LENTH)
                        M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX + 0.1 * (M_Params.LENTH - tempmeanLen);
                    else
                    {
                        M_Params.ANGLE_DIFF_MAX_LONG = M_Params.ANGLE_DIFF_MAX;
                    }
                    //筛选
                    if ((tempangleDiff_ > M_Params.ANGLE_DIFF_MAX_LONG || tempLenRatio < M_Params.LENGTH_RATIO))
                        matchLeft = 0;
                    //筛选

                    //左右灯条相距距离
                    float tempdist = sqrt(powf((templeftLight.center.x - rightLight.center.x), 2) + powf((templeftLight.center.y - rightLight.center.y), 2));

                    //相距距离与灯条长度比值
                    float tempratio = tempdist / cv::min(tempLeftLightLen, tempRightLightLen);

                    if (tempmeanLen < M_Params.LENTH)
                        M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX + 1 * (M_Params.LENTH - tempmeanLen);
                    else
                    {
                        M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG = M_Params.MEAN_LENGTH_DIST_RATIO_MAX;
                    }
                    if (tempratio > M_Params.MEAN_LENGTH_DIST_RATIO_MAX_LONG || ratio < M_Params.MEAN_LENGTH_DIST_RATIO_MIN)
                        // 大装甲正视比例为 4.09
                        matchLeft = 0;
                    if (matchLeft == 1)
                        continue;
                }

                std::vector<cv::Point2f> rect;
                // The order is topLeft, topRight, bottomRight, bottomLeft
#if USE_WRONG_P4P_POINTS
                rect.push_back(cv::Point2f(leftLight.center.x - cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height, leftLight.center.y - sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height));
                rect.push_back(cv::Point2f(rightLight.center.x - cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height, rightLight.center.y - sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height));
                rect.push_back(cv::Point2f(rightLight.center.x + cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height, rightLight.center.y + sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height));
                rect.push_back(cv::Point2f(leftLight.center.x + cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height, leftLight.center.y + sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height));

#else
                rect.push_back(cv::Point2f(leftLight.center.x - cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height*1.2, leftLight.center.y - sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height*1.2));
                rect.push_back(cv::Point2f(rightLight.center.x - cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height*1.2, rightLight.center.y - sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height*1.2));
                rect.push_back(cv::Point2f(rightLight.center.x + cos(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height*1.2, rightLight.center.y + sin(CV_PI * (rightLight.angle + 90) / 180) * rightLight.size.height*1.2));
                rect.push_back(cv::Point2f(leftLight.center.x + cos(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height*1.2, leftLight.center.y + sin(CV_PI * (leftLight.angle + 90) / 180) * leftLight.size.height*1.2));
#endif
                R.push_back(rect);
                break;
            }
        }

#if (RECT_MATCH && DEBUG)
        cv::Mat rect_match;
        rect_match = ImageFrame.img.clone();

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
        cv::rectangle(rect_match, TrackingROI, cv::Scalar(255, 255, 255), 1, 0);

        cv::namedWindow("rect_match", 0);
        cv::resizeWindow("rect_match", 640, 480);
        cv::imshow("rect_match", rect_match);
        cv::waitKey(1);
#endif
        sort(R.begin(), R.end(),
             [=](const std::vector<cv::Point2f> &rect1, const std::vector<cv::Point2f> &rect2)
             {
                 //Lambda函数,作为sort的cmp函数
                 cv::Vec2f WidthVec1(rect1[1].x - rect1[0].x, rect1[1].y - rect1[0].y);
                 cv::Vec2f WidthVec2(rect2[1].x - rect2[0].x, rect2[1].y - rect2[0].y);
                 cv::Vec2f HeightVec1(rect1[3].x - rect1[0].x, rect1[3].y - rect1[0].y);
                 cv::Vec2f HeightVec2(rect2[3].x - rect2[0].x, rect2[3].y - rect2[0].y);

                 cv::Vec2f WidthVec1Normalize = cv::normalize(WidthVec1);
                 cv::Vec2f WidthVec2Normalize = cv::normalize(WidthVec2);
                 cv::Vec2f HeightVec1Normalize = cv::normalize(HeightVec1);
                 cv::Vec2f HeightVec2Normalize = cv::normalize(HeightVec2);

                 float x1 = (rect1[0].x + rect1[1].x + rect1[2].x + rect1[3].x) / 4;
                 float y1 = (rect1[0].y + rect1[1].y + rect1[2].y + rect1[3].y) / 4;
                 float x2 = (rect2[0].x + rect2[1].x + rect2[2].x + rect2[3].x) / 4;
                 float y2 = (rect2[0].y + rect2[1].y + rect2[2].y + rect2[3].y) / 4;

                 //  float dist1 = x1 * x1 + y1 * y1;
                 //  float dist2 = x2 * x2 + y2 * y2;
                 //  return dist1 < dist2;

                 float Rect1Area = contourArea(rect1);
                 float Rect2Area = contourArea(rect2);
                 return Rect1Area > Rect2Area;
             });
        return R;
    }

} // namespace rm
