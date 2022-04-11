#pragma once

#include "iostream"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv2/video/tracking.hpp"

namespace rm
{
    class kf
    {
    public:
        void init();

        cv::Point3f predict(cv::Point3f &input);

    private:
        cv::Point3f pinput = cv::Point3f(0, 0, 0);
        int _dynamParams = 6;
        int _measureParams = 6;
        cv::KalmanFilter KF;
    };
} // namespace rm
