#include "opencv2/opencv.hpp"
#include "iostream"

using namespace std;
using namespace cv;

namespace rm
{
    class windKF
    {

    public:
        windKF();
        ~windKF() {}

    private:
        struct KFParams
        {
            const int dynamParams = 1;

            const double NoiseExpect = 0;
            const double NoiseStandDev = 0.02;
        } Params;

    public:
        void init();
        void getMeasurement(std::vector<pair<float, float>> Theta_TimeStamp);

        bool estimate_phi();

        void predict(float &whole_theta, float TimeStamp);

        void clear();
        void loop_limit(float &t);

        cv::Mat Measurement;
        cv::Mat processNoise;
        cv::Mat TimeStamp;

        cv::Mat x_hat;

        int stepNumber;
        int P;
        int Q;
        int R;

        float phi_hat;

        int temp_StepNumber = 0;
    };

}
