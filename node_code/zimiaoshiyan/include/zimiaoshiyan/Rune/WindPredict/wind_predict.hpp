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

        float Measurement;
        float TimeStamp;

        cv::Mat processNoise;
        //std::vector<float> x_hat;
        float predict_phi = 1.5;
        float last_predict_phi = 1.5;
        int stepNumber;
        int P;
        int Q;
        int R;

        float phi_hat;

        int temp_StepNumber = 0;
    };

}
