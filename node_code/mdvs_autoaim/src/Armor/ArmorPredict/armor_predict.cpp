#include "mdvs_autoaim/Armor/ArmorPredict/armor_predict.h"

#define DT 10
#define Q 3.14 * 1e-4
#define R 1e-1

namespace rm
{

    void kf::init()
    {

        _dynamParams = 6;
        _measureParams = 6;

        KF.init(_dynamParams, _measureParams);
        cv::Mat state(_dynamParams, 1, CV_32FC1); //state(x,y,detaX,detaY)
        //Mat processNoise(stateNum, 1, CV_32F);

        //Mat measurement = Mat::zeros(measureParams, 1, CV_32F);	//measurement(x,y)

        randn(state, cv::Scalar::all(0), cv::Scalar::all(0.1)); //随机生成一个矩阵，期望是0，标准差为0.1;
        KF.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
                               0, 1, 0, 0, 1, 0,
                               0, 0, 1, 0, 0, 1,
                               0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 1); //元素导入矩阵，按行;

        //setIdentity: 缩放的单位对角矩阵;
        //!< measurement matrix (H) 观测模型
        setIdentity(KF.measurementMatrix);

        //!< process noise covariance matrix (Q)
        // wk 是过程噪声，并假定其符合均值为零，协方差矩阵为Qk(Q)的多元正态分布;
        setIdentity(KF.processNoiseCov, cv::Scalar::all(Q)); ///1e-6/////3.14*1e-4
        //setIdentity(KF.processNoiseCov, Scalar::all(1e-6));///1e-6/////3.14*1e-4

        //!< measurement noise covariance matrix (R)
        //vk 是观测噪声，其均值为零，协方差矩阵为Rk,且服从正态分布;
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(R)); ///1e-1

        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  A代表F: transitionMatrix
        //预测估计协方差矩阵;
        setIdentity(KF.errorCovPost, cv::Scalar::all(1));

        //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
        //initialize post state of kalman filter at random
        randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
    }

    cv::Point3f kf::predict(cv::Point3f &input)
    {

        cv::Mat measurement = cv::Mat::zeros(_measureParams, 1, CV_32F);

        //2.kalman prediction
        KF.predict();

        //3.update measurement
        measurement.at<float>(0) = (float)input.x;
        measurement.at<float>(1) = (float)input.y;
        measurement.at<float>(2) = (float)input.z;
        measurement.at<float>(3) = input.x - pinput.x;
        measurement.at<float>(4) = input.y - pinput.y;
        measurement.at<float>(5) = input.z - pinput.z;

        //                        cout<<"sssssssssssssssssssssssssssssss"<<xyz.x-pxyz.x<<endl;

        pinput = input;

        //4.update
        KF.correct(measurement);

        //2.kalman prediction
        //Mat prediction = KF.statePost;
        cv::Point3f output = cv::Point3f(
            KF.statePost.at<float>(0) + DT * KF.statePost.at<float>(3, 0),
            KF.statePost.at<float>(1) + DT * KF.statePost.at<float>(4, 0),
            KF.statePost.at<float>(2) + DT * KF.statePost.at<float>(5, 0));

        return output;
    }

} // namespace rm
