//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
//

#ifndef PROJECT_INVERSEKINEMATICS_H
#define PROJECT_INVERSEKINEMATICS_H


#include <Eigen/Dense>
#include "../Polynomial/Polynomial.hpp"

namespace dmotion {

    class InvKin {
    private:

    public:
        static const double upper_leg_length = 12.0;  //大腿的长度,静态常成员变量便于调用,这个参数比较固定,一般绝对不会变,故没有写在参数文件中
        static const double lower_leg_length = 12.0;  //小腿的长度
        static const double ankle_from_ground = 3.5;  //脚踝距离地面的高度
        static const double half_hip_width = 4.5;     //两髋关节距离的一半
        static const double hip_x_from_origin = 0;    //原点到髋关节x方向上的距离
        //以上参数的单位都是cm
    };
}


#endif //PROJECT_INVERSEKINEMATICS_H
