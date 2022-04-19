#include "zimiaoshiyan/Armor/armor.h"

namespace rm
{
    std::vector<cv::RotatedRect> Armor::find_light_bar(cv::Mat &img)
    {
        std::vector<cv::RotatedRect> R;
        std::vector<cv::Mat> channels;
        cv::Mat grayImg;
        cv::Mat grayImg1;
#if (LIGHT_BAR_RECT && DEBUG)
        cv::Mat img_rect = img.clone();
#endif
#if (LIGHT_BAR_CONTOUR && DEBUG)
        cv::Mat img_contour = img.clone();
#endif

        SetTrackingROI();
        img = img(TrackingROI);

        //auto t1 = std::chrono::high_resolution_clock::now();
        cv::split(img, channels); //分离色彩通道
        //ArmorSplit(img, channels);
        //auto t2 = std::chrono::high_resolution_clock::now();
        //auto splitTime = (static_cast<std::chrono::duration<float, std::milli>>(t1 - t2)).count();

        //Log(20, "SplitTime: " << splitTime << "ms"
        //                      << "             ");

        if (_enemy_color == RED)
        {
            grayImg = channels.at(2) - 0.4*0 * channels.at(0); //- channels.at(1); //Get red-blue image;

#if LIGHT_BAR_SPLIT && DEBUG
            RS.to_show_img(grayImg, "LIGHT_BAR_SPLIT", RS.RSIZE);
            RS.to_show_img(channels.at(0), "LIGHT_BAR_SPLIT_TOWARDS", RS.RSIZE);
#endif
            cv::threshold(grayImg, grayImg, 150, 255, cv::THRESH_BINARY);
        }
        else
        {
            grayImg = channels.at(0) - 0.4*0 * channels.at(2); // - channels.at(1); //Get blue-red image
#if LIGHT_BAR_SPLIT && DEBUG
            RS.to_show_img(grayImg, "LIGHT_BAR_SPLIT", RS.RSIZE);
            RS.to_show_img(channels.at(2), "LIGHT_BAR_SPLIT_TOWARDS", RS.RSIZE);
#endif
            cv::threshold(grayImg, grayImg, 150, 255, cv::THRESH_BINARY);
        }
#if LIGHT_BAR_BINARY && DEBUG
        RS.to_show_img(grayImg, "LIGHT_BAR_BINARY", RS.RSIZE);
#endif
        // 色彩图膨胀腐蚀
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        dilate(grayImg, grayImg, element);
        erode(grayImg, grayImg, element);

#if LIGHT_BAR_SPLIT_ERODE && DEBUG
        RS.to_show_img(grayImg, "LIGHT_BAR_SPLIT_ERODE", RS.RSIZE);
#endif

        std::vector<std::vector<cv::Point>> lightContours;
        cv::findContours(grayImg, lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        for (const auto &contour : lightContours)
        {
            /*if (_enemy_color == RED)
            {
                uchar* pdata = channels.at(1).ptr<uchar>(contour.at(0).x);
                int data = (int)pdata[contour.at(0).y];
                //int data = (int)channels.at(0).at<uchar>(contour.at(0).x, contour.at(0).y);
                Log(4, "gray : " << data << "     ");

                if (data > 100)
                    continue;
            }
            else
            {
                uchar* pdata = channels.at(1).ptr<uchar>(contour.at(0).x);
                int data = (int)pdata[contour.at(0).y];
                //int data = (int)channels.at(2).at<uchar>(contour.at(0).x, contour.at(0).y);
                Log(4, "gray : " << data << "     ");

                if (data > 100)
                    continue;
            }*/
            float contour_Area = contourArea(contour);
            // 筛选轮廓
            if (contour.size() <= 5 ||
                contour_Area < D_Params.CONTOUR_AREA_MIN ||
                contour_Area > D_Params.CONTOUR_AREA_MAX)
                continue;
            cv::RotatedRect lightRec_temp = cv::minAreaRect(contour);
            cv::RotatedRect lightRecEllipse = cv::fitEllipse(contour);

            cv::Point2f *temp_point = new cv::Point2f[4];
            lightRec_temp.points(temp_point);
            float dist01 = (temp_point[0].x - temp_point[1].x) * (temp_point[0].x - temp_point[1].x) +
                           (temp_point[0].y - temp_point[1].y) * (temp_point[0].y - temp_point[1].y);
            float dist12 = (temp_point[1].x - temp_point[2].x) * (temp_point[1].x - temp_point[2].x) +
                           (temp_point[1].y - temp_point[2].y) * (temp_point[1].y - temp_point[2].y);

            cv::RotatedRect lightRec(lightRec_temp.center, lightRec_temp.size, lightRecEllipse.angle);

            if (fabsf(grayImg.cols/2 - lightRec_temp.center.x)/(grayImg.cols/2) > 0.85f)
                continue;


            if (contour_Area / lightRec.size.area() < D_Params.CONTOUR_FIT_DEGREE_MIN)
                continue;

            if (lightRec.angle < 110 && lightRec.angle > 70)
                continue;

            if (lightRec.angle > 90)
                lightRec.angle -= 180;

            if (dist01 > dist12)
            {
                cv::RotatedRect LightBarRect((cv::Point2f)TrackingROI.tl() + lightRec.center, lightRec.size, lightRec.angle);

                if (LightBarRect.size.height < D_Params.LENTH)
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN - 0*0.075 * (D_Params.LENTH - lightRec.size.height);
                else
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN;

                if (LightBarRect.size.height / LightBarRect.size.width < D_Params.CONTOUR_H_W_RATIO_MIN_LONG ||
                    LightBarRect.size.height / LightBarRect.size.width > D_Params.CONTOUR_H_W_RATIO_MAX)
                    continue;

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
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN - 0*0.075 * (D_Params.LENTH - lightRec.size.height);
                else
                    D_Params.CONTOUR_H_W_RATIO_MIN_LONG = D_Params.CONTOUR_H_W_RATIO_MIN;

                if (LightBarRect.size.height / LightBarRect.size.width < D_Params.CONTOUR_H_W_RATIO_MIN_LONG ||
                    LightBarRect.size.height / LightBarRect.size.width > D_Params.CONTOUR_H_W_RATIO_MAX)
                    continue;

                R.push_back(LightBarRect);
            }

#if (LIGHT_BAR_RECT && DEBUG)
            cv::Point2f *p = new cv::Point2f[4];
            cv::RotatedRect &tempLightBarRect = R[R.size() - 1];
            tempLightBarRect.points(p);
            for (int i = 0; i < 4; i++)
            {
                cv::line(img_rect, p[i], p[(i + 1) % 4], cv::Scalar(255, 0, 255), 4, 0);
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

#if (LIGHT_BAR_RECT && DEBUG)
        RS.to_show_img(img_rect, "Light_Bar_Rect", RS.RSIZE);
#endif
#if (LIGHT_BAR_CONTOUR && DEBUG)
        RS.to_show_img(img_contour, "Light_Bar_Contour", RS.RSIZE);
#endif

        sort(R.begin(), R.end(), [](const cv::RotatedRect &ld1, const cv::RotatedRect &ld2) { //Lambda函数,作为sort的cmp函数
            return ld1.center.x < ld2.center.x;
        });
        return R;
    }

    void Armor::ArmorSplit(const cv::Mat &mtx, vector<Mat> &mv)
    {
        cv::Mat blue_dst = cv::Mat::zeros(mtx.size(), CV_8UC1);
        cv::Mat green_dst = cv::Mat::zeros(mtx.size(), CV_8UC1);
        cv::Mat red_dst = cv::Mat::zeros(mtx.size(), CV_8UC1);

        uchar *src_data = (uchar *)mtx.data;
        uchar *blue_dst_data = (uchar *)blue_dst.data;
        uchar *green_dst_data = (uchar *)green_dst.data;
        uchar *red_dst_data = (uchar *)red_dst.data;

        for (int i = 0; i < mtx.cols * mtx.rows; i++)
        {

            *blue_dst_data = *(src_data);
            *green_dst_data = *(src_data + 1);
            *red_dst_data = *(src_data + 2);

            src_data += 3;
            blue_dst_data++;
            green_dst_data++;
            red_dst_data++;
        }

        mv = vector<Mat>{blue_dst, green_dst, red_dst};
    }

} // namespace rm
