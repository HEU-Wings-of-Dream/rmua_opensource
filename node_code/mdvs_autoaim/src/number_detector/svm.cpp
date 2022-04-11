#include "svm.h"
#include <iostream>
#include "define.h"

namespace rm
{

    void svmdetector::init_model()
    {
        svm = cv::ml::SVM::create();
        svm = cv::Algorithm::load<cv::ml::SVM>("../HEU_Vision_2020_MDVS/Others/number_detector/model.xml");
        if (svm.empty() == 1)
        {
            std::cout << "modle do not exsit!!!" << std::endl;
            return;
        }
        std::cout << "modle SUCCESS!!!" << std::endl;
        return;
    }

    int svmdetector::predict_number(cv::Mat img0)
    {
        //    Log(23,"div : " << (double)img0.rows/(double)img0.cols );
        //    if (double(img0.rows)/double(img0.cols) > 3.5)
        //    {
        //        Log(18,"ans :"<< 1<<"               ");
        //        return 1;
        //    }
        // std::cout<<"predict   number";
        cv::resize(img0, reshaped_img, cv::Size(40, 130));
        line_img = reshaped_img.reshape(0, 1);

        line_img.convertTo(predict_mat, CV_32FC1);
        ans = svm->predict(predict_mat);
        // ans2 = ans+2;
        Log(18, "predict :" << ans << "               ");
        // std::cout<<"result:"<<ans<<std::endl;
        return ans;
    }

    void svmdetector::GammaCorrection(cv::Mat &src, cv::Mat &dst, double gamma)
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
}
