#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>

namespace rm
{
    class svmdetector
    {
    public:
        // var define
        cv::Ptr<cv::ml::SVM> svm;
        cv::Mat reshaped_img, line_img, predict_mat;
        int ans;
        int ans2;
        cv::Mat num_img;

        // function define

        void init_model();
        int predict_number(cv::Mat img0);
        void GammaCorrection(cv::Mat &src, cv::Mat &dst, double gamma);
    };
}
