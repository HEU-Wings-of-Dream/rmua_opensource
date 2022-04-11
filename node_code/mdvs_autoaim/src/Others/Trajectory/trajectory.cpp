/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
#include <cmath>
#include <stdio.h>
#include <vector>

#include "mdvs_autoaim/Others/Trajectory/trajectory.h"

namespace rm {

void GimbalContrl::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k) {
    offset_.x = x;
    offset_.y = y;
    offset_.z = z;
    offset_pitch_ = pitch;
    offset_yaw_ = yaw;
    init_v_ = init_v;
    init_k_ = init_k;
}

//air friction is considered
float GimbalContrl::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t, y;
    t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return y;
}

//x:distance , y: height
float GimbalContrl::GetPitch(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++) {
        a = (float) atan2(y_temp, x);
        y_actual = BulletModel(x, v, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
    }
    return a;

}

void GimbalContrl::Transform(cv::Point3f &postion, float &pitch, float &yaw) {
    if(init_k_!=0) pitch = (GetPitch((postion.y + offset_.y) / 100, (postion.z + offset_.z) / 100, 15) + (float)(offset_pitch_ * 3.1415926535 / 180))*180/3.1415926535;
    else {pitch = (atan2(postion.z + offset_.z, postion.y + offset_.y))*180/3.1415926535;}

    //yaw positive direction :anticlockwise
    yaw = ((float) (atan2(postion.x + offset_.x, postion.y + offset_.y)) + (float)(offset_yaw_ * 3.1415926535 / 180))*180/3.1415926535;
}



} // roborts_detection



