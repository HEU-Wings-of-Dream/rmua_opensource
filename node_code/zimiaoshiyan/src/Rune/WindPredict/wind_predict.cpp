#include "zimiaoshiyan/Rune/WindPredict/wind_predict.hpp"

namespace rm
{
    windKF::windKF(/* args */)
    {
    }

    void windKF::init()
    {
        stepNumber = 10000;
        P = 1000;
        Q = 10;
        R = 100;

        processNoise = cv::Mat(1.5 * stepNumber, 1, CV_32F);
        randn(processNoise, cv::Scalar::all(Params.NoiseExpect), cv::Scalar::all(Params.NoiseStandDev));
    }


    bool windKF::estimate_phi()
    {
        bool flag = false;

        P = P + Q;//Q=10

        float H = 0.785 * sin(1.884 * TimeStamp + last_predict_phi) + 1.305;

        float K = P * H / (H * P * H + R);//R=100

        float temp_delta_z = -0.41667 * cos(1.884 * TimeStamp + last_predict_phi) + 1.305 * TimeStamp;

        loop_limit(temp_delta_z);

        predict_phi = last_predict_phi + K * (Measurement - temp_delta_z);

        loop_limit(predict_phi);

        P = (1 - K * H) * P;

        flag = true;

        phi_hat = predict_phi;

        last_predict_phi = predict_phi;

        return flag;
    }

    void windKF::predict(float &whole_theta, float TimeStamp)
    {
        double T;
        whole_theta = -785 / 1884 * cos(1.884 * TimeStamp + phi_hat) + 1.305 * TimeStamp;
        loop_limit(whole_theta);
    }

    void windKF::clear()
    {
        Measurement = 0;
        TimeStamp = 0;
        predict_phi = 1.5;
        last_predict_phi = 1.5;
    }

    void windKF::loop_limit(float &t)
    {
        if (t > 2 * CV_PI)
        {
            t = fmod(t, 2 * CV_PI);
        }
        if (t < 0)
        {
            int u = -floor(t / (2 * CV_PI));
            t = u * (2 * CV_PI) + t;
        }
    }
}
