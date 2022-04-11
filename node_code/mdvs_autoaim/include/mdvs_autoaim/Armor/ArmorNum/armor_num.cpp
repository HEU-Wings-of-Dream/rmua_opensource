#include "../armor.h"

namespace rm
{

    bool Armor::ArmorNumMatch(cv::Mat &ROI, bool is_use_gamma, double gamma, double valueThresold, ObjectType &armorType, int &armorNum)
    {

        //        cv::Point2f src[4], dst[4];
        //        RROI.points(src);

        //        dst[0] = cv::Point2f(0, RROI.size.width);
        //        dst[1] = cv::Point2f(0, 0);
        //        dst[2] = cv::Point2f(RROI.size.height, 0);
        //        dst[3] = cv::Point2f(RROI.size.height, RROI.size.width);

        //        cv::Mat rot_mat, RotatedMat, ROI;
        //        rot_mat = cv::getPerspectiveTransform(src, dst);
        //        cv::warpPerspective(num_img.clone(), RotatedMat, rot_mat, RROI.size);
        //        cv::Mat ROI;
        //        ROI = num_img(RROI.boundingRect());

        //        RS.to_show_img(ROI,"ROI",RS.RSIZE);

        //        if (is_use_gamma)
        //            GammaCorrection(ROI, ROI, 0.75);

        //        resize(ROI,ROI,Size(500,500));
        //        RS.to_show_img(ROI,"ROI_RESIZE",RS.RSIZE);

        cvtColor(ROI, ROI, COLOR_BGR2GRAY);

        threshold(ROI, ROI, 20, 255, cv::THRESH_BINARY);

        std::vector<std::pair<double, int>> value_label; //匹配值，数字

        for (unsigned int i = 0; i < Template_big.size(); i++)
        {
            cv::Mat small = Template_small[i];
            cv::Mat big = Template_big[i];

            cv::Mat result;
            cv::Point maxLoc, minLoc;
            double maxVal, minVal;

            cv::matchTemplate(ROI, small, result, cv::TemplateMatchModes::TM_CCOEFF_NORMED); // 0-1
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
            if (maxVal < valueThresold)
                continue;
            value_label.push_back(std::make_pair(maxVal, i + 1));

            cv::matchTemplate(ROI, big, result, cv::TemplateMatchModes::TM_CCOEFF_NORMED);
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
            if (maxVal < valueThresold)
                continue;
            value_label.push_back(std::make_pair(maxVal, (i + 1) * 11));
        }

        std::sort(value_label.begin(), value_label.end(), [](const std::pair<double, int> &ld1, const std::pair<double, int> &ld2)
                  { return ld1.first > ld2.first; });

        if (value_label.size() > 0)
        {
            if (value_label[0].second > 10)
            {
                armorType = BIG_ARMOR;
                armorNum = value_label[0].second / 11;
            }
            else
            {
                armorType = SMALL_ARMOR;
                armorNum = value_label[0].second;
            }

            return true;
        }
        else
            return false;
    }

    void Armor::GammaCorrection(cv::Mat &src, cv::Mat &dst, double gamma)
    {
        CV_Assert(src.data);
        CV_Assert(src.depth() != sizeof(uchar));

        unsigned char lut[256];
        for (int i = 0; i < 256; i++)
            lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), gamma) * 255.0f);

        dst = src.clone();
        const int channels = dst.channels();
        switch (channels)
        {
        case 1:
        {
            cv::MatIterator_<uchar> it, end;
            for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
                *it = lut[(*it)];
            break;
        }
        case 3:
        {
            cv::MatIterator_<cv::Vec3b> it, end;
            for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++)
            {
                (*it)[0] = lut[((*it)[0])];
                (*it)[1] = lut[((*it)[1])];
                (*it)[2] = lut[((*it)[2])];
            }
            break;
        }
        }
    }

    void Armor::ClearNumMatchParams(ObjectType &armorType, int &armorNum)
    {
        armorNum = 0;
        armorType = UNKNOWN_ARMOR;
    }
}
