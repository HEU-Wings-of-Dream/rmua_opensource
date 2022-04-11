#include "../armor.h"

namespace rm
{

    void Armor::ClearTargetState(std::vector<cv::Point2f> TargetRect)
    {
        float Target2DxPosition = (TargetRect[0].x + TargetRect[1].x + TargetRect[2].x + TargetRect[3].x) / 4;
        float Target2DyPosition = (TargetRect[0].y + TargetRect[1].y + TargetRect[2].y + TargetRect[3].y) / 4;
        Last2DAccel = cv::Point2f(0, 0);
        Last2DVelocity = cv::Point2f(0, 0);
        Last2DPosition = cv::Point2f(Target2DxPosition, Target2DyPosition);
        LastTargetRect = TargetRect;
    }
    void Armor::SpreadTargetState(std::vector<cv::Point2f> TargetRect)
    {
        std::vector<cv::Point2f> NowTargetRect = LastTargetRect;
    }
    void Armor::GetTargetState(std::vector<cv::Point2f> TargetRect)
    {
        static float maxV = 2;

        float Target2DxPosition = (TargetRect[0].x + TargetRect[1].x + TargetRect[2].x + TargetRect[3].x) / 4;
        float Target2DyPosition = (TargetRect[0].y + TargetRect[1].y + TargetRect[2].y + TargetRect[3].y) / 4;
        float Target2DxVelocity = (Target2DxPosition - Last2DPosition.x) / CAM_ValidImgPeriod;
        float Target2DyVelocity = (Target2DyPosition - Last2DPosition.y) / CAM_ValidImgPeriod;
        if (Target2DxVelocity > maxV)
            Target2DxVelocity = maxV;
        if (Target2DxVelocity < -maxV)
            Target2DxVelocity = -maxV;
        if (Target2DyVelocity > maxV)
            Target2DyVelocity = maxV;
        if (Target2DyVelocity < -maxV)
            Target2DyVelocity = -maxV;
        float Target2DxAccel = (Target2DxVelocity - Last2DVelocity.x) / CAM_ValidImgPeriod;
        float Target2DyAccel = (Target2DyVelocity - Last2DVelocity.y) / CAM_ValidImgPeriod;

        Last2DAccel = cv::Point2f(Target2DxAccel, Target2DyAccel);
        Last2DVelocity = cv::Point2f(Target2DxVelocity, Target2DyVelocity);
        Last2DPosition = cv::Point2f(Target2DxPosition, Target2DyPosition);
        LastTargetRect = TargetRect;
    }

    void Armor::SetTrackingROI(float imgTimeStamp)
    {
        Rodrigues(_solve._rVec, _solve._RMat);
        //        int c11 = _solve._RMat.at<double>(0, 0)*100;
        //        int c12 = _solve._RMat.at<double>(0, 1)*100;
        //        int c13 = _solve._RMat.at<double>(0, 2)*100;
        //        int c21 = _solve._RMat.at<double>(1, 0)*100;
        //        int c22 = _solve._RMat.at<double>(1, 1)*100;
        //        int c23 = _solve._RMat.at<double>(1, 2)*100;
        //        int c31 = _solve._RMat.at<double>(2, 0)*100;
        //        int c32 = _solve._RMat.at<double>(2, 1)*100;
        //        int c33 = _solve._RMat.at<double>(2, 2)*100;
        //        std::cout<<std::endl;
        //        std::cout<<std::endl;
        //        std::cout<<"x:"<<c11<<' '<<c12<<' '<<c13<<std::endl;
        //        std::cout<<"y:"<<c21<<' '<<c22<<' '<<c23<<std::endl;
        //        std::cout<<"z:"<<c31<<' '<<c32<<' '<<c33<<std::endl;
        //        std::cout<<"theta:"<<atan2(c31,c33)*180/CV_PI<<std::endl;
        if (TrackingCount > 0 && LastTargetRect.size() > 3) //&&0 TrackingCount > 0
        {
            // 获取跟踪ROI
            float LastTargetMax_x = 0;
            float LastTargetMax_y = 0;
            float LastTargetMin_x = LastTargetRect[0].x;
            float LastTargetMin_y = LastTargetRect[0].y;
            for (uint8_t i = 0; i < 4; i++)
            {
                if (LastTargetRect[i].x > LastTargetMax_x)
                    LastTargetMax_x = LastTargetRect[i].x;
                if (LastTargetRect[i].y > LastTargetMax_y)
                    LastTargetMax_y = LastTargetRect[i].y;
                if (LastTargetRect[i].x < LastTargetMin_x)
                    LastTargetMin_x = LastTargetRect[i].x;
                if (LastTargetRect[i].y < LastTargetMin_y)
                    LastTargetMin_y = LastTargetRect[i].y;
            }
            float Target_Width = LastTargetMax_x - LastTargetMin_x;
            float Target_Height = LastTargetMax_y - LastTargetMin_y;
            float ROI_Width = Target_Width * (1.3 + fabsf(Last2DAccel.x) * 1.25);
            float ROI_Height = Target_Height * (1.3 + fabsf(Last2DAccel.y) * 1.25);
            if (ROI_Height < 40)
            {
                ROI_Width *= 1.25;
                ROI_Height *= 1.25;
            }

            TrackingROI.x = Last2DPosition.x - ROI_Width + CAM_ValidImgPeriod * Last2DVelocity.x + 0.5 * CAM_ValidImgPeriod * CAM_ValidImgPeriod * Last2DAccel.x;
            TrackingROI.y = Last2DPosition.y - ROI_Height + CAM_ValidImgPeriod * Last2DVelocity.y + 0.5 * CAM_ValidImgPeriod * CAM_ValidImgPeriod * Last2DAccel.y;
            TrackingROI.width = ROI_Width * 2;
            TrackingROI.height = ROI_Height * 2;

            std::vector<cv::Point> ROI_Points;

            for (uint8_t i = 0; i < 4; i++)
                ROI_Points.push_back(LastTargetRect[i]);

            ROI_Points.push_back(TrackingROI.br());
            ROI_Points.push_back(TrackingROI.tl());

            RS.makeRectSafe(TrackingROI, cv::Size(WIDTH, HEIGHT));
        }
        //装甲板受击闪烁周期约为50ms
        else if (imgTimeStamp - TargetLostTimeStamp < 60)
        {
            if (atan2(_solve._RMat.at<double>(2, 0), _solve._RMat.at<double>(2, 2)) * 180 / CV_PI > 40 && LostCount == 1)
            {
                TrackingROI.x -= TrackingROI.width;
                TrackingROI.width += TrackingROI.width;
            }

            if (atan2(_solve._RMat.at<double>(2, 0), _solve._RMat.at<double>(2, 2)) * 180 / CV_PI < -40 && LostCount == 1)
            {
                TrackingROI.width += TrackingROI.width;
            }

            //  Solve._rVec;
            TrackingROI.x += CAM_ValidImgPeriod * Last2DVelocity.x + 0.5 * CAM_ValidImgPeriod * CAM_ValidImgPeriod * Last2DAccel.x;
            TrackingROI.y += CAM_ValidImgPeriod * Last2DVelocity.y + 0.5 * CAM_ValidImgPeriod * CAM_ValidImgPeriod * Last2DAccel.y;
            float k = 0.001;
            float delta_width = CAM_ValidImgPeriod * k * TrackingROI.width;
            float delta_height = CAM_ValidImgPeriod * k * TrackingROI.height;
            TrackingROI.x -= 0.5 * delta_width;
            TrackingROI.y -= 0.5 * delta_height;
            TrackingROI.width += delta_width;
            TrackingROI.height += delta_height;
            RS.makeRectSafe(TrackingROI, cv::Size(WIDTH, HEIGHT));
        }
        else
        {
            TrackingROI.x = 0;
            TrackingROI.y = 0;
            TrackingROI.width = WIDTH;
            TrackingROI.height = HEIGHT;
        }
    }
}
