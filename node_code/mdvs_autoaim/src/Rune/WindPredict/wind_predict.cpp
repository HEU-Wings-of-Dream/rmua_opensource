#include "wind_predict.hpp"

namespace rm
{
    windKF::windKF(/* args */)
    {
    }

    void windKF::init()
    {
        stepNumber = 100;
        P = 1000;
        Q = 10;
        R = 100;

        processNoise = cv::Mat(1.5 * stepNumber, 1, CV_32F);
        randn(processNoise, cv::Scalar::all(Params.NoiseExpect), cv::Scalar::all(Params.NoiseStandDev));

        Measurement = cv::Mat(1.5 * stepNumber, 1, CV_32F);
        TimeStamp = cv::Mat(1.5 * stepNumber, 1, CV_32F);

        x_hat = cv::Mat::zeros(stepNumber, 1, CV_32F);
    }

    void windKF::getMeasurement(std::vector<pair<float, float>> Theta_TimeStamp)
    {
        temp_StepNumber = Theta_TimeStamp.size();

        for (unsigned int i = 0; i < Theta_TimeStamp.size() && temp_StepNumber == stepNumber; i++)
        {
            float noise_theta = Theta_TimeStamp[i].first + processNoise.at<float>(i);
            loop_limit(noise_theta);
            Measurement.at<float>(i) = float(noise_theta);

            TimeStamp.at<float>(i) = float(Theta_TimeStamp[i].second);
        }
    }

    bool windKF::estimate_phi()
    {
        bool flag = false;
        for (unsigned int i = 1; i < temp_StepNumber && temp_StepNumber == stepNumber; i++)
        {
            P = P + Q;

            float H = 0.785 * sin(1.884 * TimeStamp.at<float>(i) + x_hat.at<float>(i - 1)) + 1.305;

            float K = P * H / (H * P * H + R);

            float temp_delta_z = -0.41667 * cos(1.884 * TimeStamp.at<float>(i) + x_hat.at<float>(i - 1)) + 1.305 * TimeStamp.at<float>(i);

            loop_limit(temp_delta_z);

            x_hat.at<float>(i) = x_hat.at<float>(i - 1) + K * (Measurement.at<float>(i) - temp_delta_z);

            loop_limit(x_hat.at<float>(i));

            P = (1 - K * H) * P;

            flag = true;
        }

        phi_hat = x_hat.at<float>(temp_StepNumber);

        return flag;

        //panduan
    }

    void windKF::predict(float &whole_theta, float TimeStamp)
    {
        float delta_time = TimeStamp / 500000;

        for (float i = 0; i <= TimeStamp; i += delta_time)
        {
            float w = (0.785 * sin(1.884 * (i) + phi_hat) + 1.305);
            whole_theta += w * delta_time;
        }
    }

    void windKF::clear()
    {
        Measurement = cv::Mat::zeros(1.5 * stepNumber, 1, CV_32F);
        TimeStamp = cv::Mat::zeros(1.5 * stepNumber, 1, CV_32F);

        x_hat = cv::Mat::zeros(stepNumber, 1, CV_32F);
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
