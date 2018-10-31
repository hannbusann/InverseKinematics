//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
// TODO 这个类希望实现脚和身体的逆运动学

#ifndef PROJECT_INVERSEKINEMATICS_H
#define PROJECT_INVERSEKINEMATICS_H

#include <vector>
#include <Eigen/Dense>
#include "../Polynomial/Polynomial.hpp"

namespace dmotion {

    class InvKin {
    private:

    public:
        // 静态常成员变量便于调用,这些参数比较固定,机器人装好后一般绝对不会变,故没有写在参数文件中
        // 参数的单位是cm
        static const double upper_leg_length = 12.0;  //大腿的长度
        static const double lower_leg_length = 12.0;  //小腿的长度
        static const double ankle_from_ground = 3.5;  //脚踝距离地面的高度
        static const double half_hip_width = 4.5;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
        static const double hip_x_from_origin = 0;    //髋关节点相对于身体中心原点的x方向坐标
        static const double hip_z_from_origin = -8.0; //髋关节点相对于身体中心原点的z


        // 逆运动学,以身体中心为原点,前x左y上z的方向建立坐标系,输入左/右脚的位置(x,y,z)和姿态(r,p,y)
        // 为了方便理解和调参,这里的RPY变换根据以下描述进行变换
        // 起始状态脚坐标系B和此处的世界坐标系A坐标轴方向重合,先绕Z_a转yaw角,再绕Y_b转pitch角,最后绕X_b转roll角.
        // 输出从上到下的共6个舵机角度值(角度值).
        std::vector<double> & LegInvKin(std::vector<double> &foot_pose, bool isRight);




    };
}


#endif //PROJECT_INVERSEKINEMATICS_H
