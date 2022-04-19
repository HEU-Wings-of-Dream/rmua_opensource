#include "zimiaoshiyan/Others/SolvePosition/solve_position.h"
#include "zimiaoshiyan/define.h"
#include "zimiaoshiyan/Run/run.h"

#include "opencv2/core/core_c.h"

#define _offset_x 0
#define _offset_y 0
#define _offset_z 0

namespace rm
{

    std::vector<cv::Point3f> Solve::POINT_3D_OF_ARMOR_BIG = std::vector<cv::Point3f>
    {
#if USE_WRONG_P4P_POINTS
#define ARMOR_BIG_WIDTH 225 / 2 * ARMOR_SIZE_SCALE
#define ARMOR_BIG_HEIGHT 55
#else
#define ARMOR_BIG_WIDTH 225 / 2
#define ARMOR_BIG_HEIGHT 55 //68.75
#endif
        cv::Point3f(-ARMOR_BIG_WIDTH, -ARMOR_BIG_HEIGHT, 0),    //tl
            cv::Point3f(ARMOR_BIG_WIDTH, -ARMOR_BIG_HEIGHT, 0), //tr
            cv::Point3f(ARMOR_BIG_WIDTH, ARMOR_BIG_HEIGHT, 0),  //br
            cv::Point3f(-ARMOR_BIG_WIDTH, ARMOR_BIG_HEIGHT, 0)  //bl
    };
    std::vector<cv::Point3f> Solve::POINT_3D_OF_ARMOR_SMALL = std::vector<cv::Point3f>
    {
#if USE_WRONG_P4P_POINTS
#define ARMOR_SMALL_WIDTH 132 / 2 * ARMOR_SIZE_SCALE
#define ARMOR_SMALL_HEIGHT 55 //68.75
#else
#define ARMOR_SMALL_WIDTH 132 / 2
#define ARMOR_SMALL_HEIGHT 55 //68.75
#endif
        cv::Point3f(-ARMOR_SMALL_WIDTH, -ARMOR_SMALL_HEIGHT, 0),    //tl
            cv::Point3f(ARMOR_SMALL_WIDTH, -ARMOR_SMALL_HEIGHT, 0), //tr
            cv::Point3f(ARMOR_SMALL_WIDTH, ARMOR_SMALL_HEIGHT, 0),  //br
            cv::Point3f(-ARMOR_SMALL_WIDTH, ARMOR_SMALL_HEIGHT, 0)  //bl
    };

    std::vector<cv::Point3f> Solve::POINT_3D_OF_RUNE_CENTER = std::vector<cv::Point3f>{
#define UNE_CENTER 125
        cv::Point3f(-UNE_CENTER, -UNE_CENTER, 0), //tl
        cv::Point3f(UNE_CENTER, -UNE_CENTER, 0),  //tr
        cv::Point3f(UNE_CENTER, UNE_CENTER, 0),   //br
        cv::Point3f(-UNE_CENTER, UNE_CENTER, 0)   //bl
    };

    std::vector<cv::Point3f> Solve::POINT_3D_OF_RUNE_WING = std::vector<cv::Point3f>{
#define RUNE_WING_WIDTH UNE_CENTER * 3
#define RUNE_WING_HEIGHT RUNE_WING_WIDTH / 2                 //68.75
        cv::Point3f(-RUNE_WING_WIDTH, -RUNE_WING_HEIGHT, 0), //tl
        cv::Point3f(RUNE_WING_WIDTH, -RUNE_WING_HEIGHT, 0),  //tr
        cv::Point3f(RUNE_WING_WIDTH, RUNE_WING_HEIGHT, 0),   //br
        cv::Point3f(-RUNE_WING_WIDTH, RUNE_WING_HEIGHT, 0)   //bl
    };

    std::vector<cv::Point3f> Solve::POINT_3D_OF_RUNE_BIG = std::vector<cv::Point3f>{
#define RUNE_ARMOR_BIG_WIDTH 230 / 2  // 225/2
#define RUNE_ARMOR_BIG_HEIGHT 127 / 2 //115 /2

        cv::Point3f(-RUNE_ARMOR_BIG_WIDTH, -RUNE_ARMOR_BIG_HEIGHT, 0), //tl
        cv::Point3f(RUNE_ARMOR_BIG_WIDTH, -RUNE_ARMOR_BIG_HEIGHT, 0),  //tr
        cv::Point3f(RUNE_ARMOR_BIG_WIDTH, RUNE_ARMOR_BIG_HEIGHT, 0),   //br
        cv::Point3f(-RUNE_ARMOR_BIG_WIDTH, RUNE_ARMOR_BIG_HEIGHT, 0)   //bl
    };

    void Solve::init(cv::Mat &CAM_MATRIX, cv::Mat &DISTORTION_COEFF)
    {
        _CAM_MATRIX = CAM_MATRIX.clone();
        _DISTORTION_COEFF = DISTORTION_COEFF.clone();
    }
    cv::Mat Solve::get_rVec()
    {
        return _rVec;
    }
    cv::Mat Solve::get_tVec()
    {
        return _tVec;
    }

    cv::Point3f Solve::get_real_Point3f(double pitch, double yaw)
    {
        //        pitch=3.1415926*pitch/180;
        //        yaw=3.1415926*yaw/180;

        float k[3][3] = {
            1, 0, 0,
            0, 0, 1,
            0, -1, 0};
        cv::Mat K(3, 3, CV_32FC1, k);

        float offset[3] = {_offset_x, _offset_y, _offset_z};
        cv::Mat OFFSET(3, 1, CV_32FC1, offset);

        float xyz[3] = {(float)_tVec.at<double>(0, 0), (float)_tVec.at<double>(1, 0), (float)_tVec.at<double>(2, 0)};
        cv::Mat XYZ(3, 1, CV_32FC1, xyz);

        float z_rotate[3][3] = {
            (float)cos(yaw), -(float)sin(yaw), 0,
            (float)sin(yaw), (float)cos(yaw), 0,
            0, 0, 1};
        cv::Mat Z_ROTATE(3, 3, CV_32FC1, z_rotate);

        //        float ychange[3][3]={
        //                             cos(yaw),0,sin(yaw),
        //                             0,1,0,
        //                             -sin(yaw),0,cos(yaw)
        //                            };
        //        cv::Mat yc(3,3,CV_32FC1,ychange);

        float x_rotate[3][3] = {
            1, 0, 0,
            0, (float)cos(pitch), -(float)sin(pitch),
            0, (float)sin(pitch), (float)cos(pitch)};
        cv::Mat X_ROTATE(3, 3, CV_32FC1, x_rotate);

        cv::Mat result = Z_ROTATE * X_ROTATE * (K * XYZ + OFFSET);

        return cv::Point3f(result.at<float>(0, 0), result.at<float>(1, 0), result.at<float>(2, 0));
    }

    cv::Point2f Solve::get_Point2f(cv::Point3f input, float pitch, float yaw)
    {

        cv::Mat p(3, 1, CV_64F, cv::Scalar(1));
        p.at<double>(0, 0) = input.x;
        p.at<double>(1, 0) = input.y;
        p.at<double>(2, 0) = input.z;

        double k[3][3] = {
            1, 0, 0,
            0, 0, 1,
            0, -1, 0};
        cv::Mat K(3, 3, CV_64F, k);

        double offset[3] = {_offset_x, _offset_y, _offset_z};
        cv::Mat OFFSET(3, 1, CV_64F, offset);

        double zchange[3][3] = {
            cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1};
        cv::Mat zc(3, 3, CV_64F, zchange);

        double xchange[3][3] = {
            1, 0, 0,
            0, cos(pitch), -sin(pitch),
            0, sin(pitch), cos(pitch)};
        cv::Mat xc(3, 3, CV_64F, xchange);

        cv::Mat _Point;
        _Point = K.inv() * (xc.inv() * zc.inv() * p - OFFSET);

        cv::Mat Point(3, 1, CV_64F, cv::Scalar(1));
        Point.at<double>(0, 0) = _Point.at<double>(0, 0);
        Point.at<double>(1, 0) = _Point.at<double>(1, 0);
        Point.at<double>(2, 0) = _Point.at<double>(2, 0);

        //            std::cout<<Point<<std::endl;

        //            cv::Mat out(3,3,CV_64F);
        //            CvMat sout=out;
        //            CvMat R=_rVec;
        //            cvRodrigues2(&R,&sout);
        //            cv::Mat RE=cv::cvarrToMat(&sout);

        //            cv::Mat kk(3,4,CV_64F,cv::Scalar(0));
        //            kk.at<double>(0,0)=1;
        //            kk.at<double>(1,1)=1;
        //            kk.at<double>(2,2)=1;

        //            cv::Mat RT(4,4,CV_64F,cv::Scalar(0));

        //            RT.at<double>(0,0)=RE.at<double>(0,0);
        //            RT.at<double>(0,1)=RE.at<double>(0,1);
        //            RT.at<double>(0,2)=RE.at<double>(0,2);
        //            RT.at<double>(0,3)=_tVec.at<double>(0,0);

        //            RT.at<double>(1,0)=RE.at<double>(1,0);
        //            RT.at<double>(1,1)=RE.at<double>(1,1);
        //            RT.at<double>(1,2)=RE.at<double>(1,2);
        //            RT.at<double>(1,3)=_tVec.at<double>(1,0);

        //            RT.at<double>(2,0)=RE.at<double>(2,0);
        //            RT.at<double>(2,1)=RE.at<double>(2,1);
        //            RT.at<double>(2,2)=RE.at<double>(2,2);
        //            RT.at<double>(2,3)=_tVec.at<double>(2,0);

        //            RT.at<double>(3,3)=1;

        //            cv::Mat output=_CAM_MATRIX*kk*(RT*Point);
        cv::Mat output = _CAM_MATRIX * Point;

        output.at<double>(0, 0) /= output.at<double>(2, 0);
        output.at<double>(1, 0) /= output.at<double>(2, 0);
        output.at<double>(2, 0) /= output.at<double>(2, 0);

        return cv::Point2f(output.at<double>(0, 0), output.at<double>(1, 0));
        //return cv::Point2f(Point.at<double>(0,0),Point.at<double>(1,0));
    }

    void Solve::solve(std::vector<cv::Point3f> OBJECT_3D, std::vector<cv::Point2f> OBJECT_2D)
    {
        solvePnP(OBJECT_3D, OBJECT_2D, _CAM_MATRIX, _DISTORTION_COEFF, _rVec, _tVec, false, CV_ITERATIVE);
    }
    void Solve::Solve_Armor(std::vector<cv::Point2f> OBJECT_2D)
    {
        /*cv::Mat _rVec_temp = cv::Mat::zeros(3, 1, CV_32FC1); //init rvec
        cv::Mat _tVec_temp = cv::Mat::zeros(3, 1, CV_32FC1); //init tvec
        cv::Mat _rMat_temp = cv::Mat::zeros(3, 3, CV_32FC1); //init rmat

        solvePnP(POINT_3D_OF_ARMOR_SMALL, OBJECT_2D, _CAM_MATRIX, _DISTORTION_COEFF, _rVec, _tVec, false, CV_ITERATIVE);
        solvePnP(POINT_3D_OF_ARMOR_BIG, OBJECT_2D, _CAM_MATRIX, _DISTORTION_COEFF, _rVec_temp, _tVec_temp, false, CV_ITERATIVE);

        cv::Rodrigues(_rVec, _rMat);
        cv::Rodrigues(_rVec_temp, _rMat_temp);
        cv::Vec3f CamZ(0.0, 0.0, 1.0);
        cv::Vec3f TgtZ1((float)_rMat.at<double>(0, 2), (float)_rMat.at<double>(1, 2), (float)_rMat.at<double>(2, 2));
        cv::Vec3f TgtZ2((float)_rMat_temp.at<double>(0, 2), (float)_rMat_temp.at<double>(1, 2), (float)_rMat_temp.at<double>(2, 2));
        cv::Vec3f Cross1, Cross2;
        TgtZ1 = cv::normalize(TgtZ1);
        TgtZ2 = cv::normalize(TgtZ2);
        Cross1 = CamZ.cross(TgtZ1);
        Cross2 = CamZ.cross(TgtZ2);*/

        float width1 = sqrt(powf((OBJECT_2D[0].x - OBJECT_2D[1].x), 2) + powf((OBJECT_2D[0].y - OBJECT_2D[1].y), 2));
        float height1 = sqrt(powf((OBJECT_2D[1].x - OBJECT_2D[2].x), 2) + powf((OBJECT_2D[1].y - OBJECT_2D[2].y), 2));
        float width2 = sqrt(powf((OBJECT_2D[3].x - OBJECT_2D[2].x), 2) + powf((OBJECT_2D[3].y - OBJECT_2D[2].y), 2));
        float height2 = sqrt(powf((OBJECT_2D[0].x - OBJECT_2D[3].x), 2) + powf((OBJECT_2D[0].y - OBJECT_2D[3].y), 2));

        w_hRatio = cv::max(width1, width2) / cv::min(height1, height2);
        LightBarLenRatio = cv::max(height1, height2) / cv::min(height1, height2);

#if USE_WRONG_P4P_POINTS
        if (w_hRatio < 1.5)
        {
            ArmorTypeCount++;
        }
        else
        {
            ArmorTypeCount--;
        }
        //Log(10, "w_hRatio:" << w_hRatio << "      ");
#else
        if (w_hRatio < 1.5)
        {
            ArmorTypeCount++;
        }
        else
        {
            ArmorTypeCount--;
        }
        /*if (cv::norm(Cross1) < cv::norm(Cross2))
        {
            ArmorTypeCount++;
        }
        else
        {
            ArmorTypeCount--;
        }*/
#endif
        Log(9, "ArmorCount : " << ArmorTypeCount << "        ");
        if (ArmorTypeCount > 0)
        {
            AromrType = SMALL_ARMOR;
            Log(8, "ArmorType : Small sure        ");
        }
        else
        {
            AromrType = BIG_ARMOR;
            Log(8, "ArmorType : Big sure          ");
        }

        /*if (ArmorTypeCount > 0 && w_hRatio < 1.5)
        {
            AromrType = SMALL_ARMOR;
            Log(8, "ArmorType : Small sure        ");
        }
        else if (ArmorTypeCount < 0 && w_hRatio > 1.5)
        {
            _rVec = _rVec_temp;
            _tVec = _tVec_temp;
            AromrType = BIG_ARMOR;
            Log(8, "ArmorType : Big sure          ");
        }
        else
        {
            if (abs(ArmorTypeCount) < 5)
                if (w_hRatio < 1.5)
                {
                    AromrType = SMALL_ARMOR;
                    Log(8, "ArmorType : Small lenRatio       ");
                }
                else
                {
                    _rVec = _rVec_temp;
                    _tVec = _tVec_temp;
                    AromrType = BIG_ARMOR;
                    Log(8, "ArmorType : Big lenRatio        ");
                }
            else if (ArmorTypeCount > 0)
            {
                AromrType = SMALL_ARMOR;
                Log(8, "ArmorType : Small Vec          ");
            }
            else if (w_hRatio > 1.5)
            {
                _rVec = _rVec_temp;
                _tVec = _tVec_temp;
                AromrType = BIG_ARMOR;
                Log(8, "ArmorType : Big Vec          ");
            }
        }*/

        _CAM_MATRIX.at<double>(0,0) = 722.7326;
        _CAM_MATRIX.at<double>(0,1) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(0,2) = 320.7117;
        _CAM_MATRIX.at<double>(1,0) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(1,1) = 723.0260;
        _CAM_MATRIX.at<double>(1,2) = 224.9253;
        _CAM_MATRIX.at<double>(2,0) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(2,1) = 0.0000000000000000;
        _CAM_MATRIX.at<double>(2,2) = 1.0000000000000000;

        _DISTORTION_COEFF.at<double>(0,0) = -0.4494;
        _DISTORTION_COEFF.at<double>(1,0) = 0.2189;
        _DISTORTION_COEFF.at<double>(2,0) = 0.0000000000000000;
        _DISTORTION_COEFF.at<double>(3,0) = 0.0000000000000000;
        _DISTORTION_COEFF.at<double>(4,0) = 0.0015;

        if (AromrType == SMALL_ARMOR)
            solvePnP(POINT_3D_OF_ARMOR_SMALL, OBJECT_2D, _CAM_MATRIX, _DISTORTION_COEFF, _rVec, _tVec, false, CV_ITERATIVE);
        else
            solvePnP(POINT_3D_OF_ARMOR_BIG, OBJECT_2D, _CAM_MATRIX, _DISTORTION_COEFF, _rVec, _tVec, false, CV_ITERATIVE);
        Log(13, OBJECT_2D[0].x << ' '<< OBJECT_2D[0].y <<"                         ");
        Log(14, OBJECT_2D[1].x << ' '<< OBJECT_2D[1].y <<"                         ");
        Log(15, OBJECT_2D[2].x << ' '<< OBJECT_2D[2].y <<"                         ");

    }

} // namespa