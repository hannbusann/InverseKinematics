//
// Created by fw on 18/10/17.
// E-mail: zjufanwu@zju.edu.cn
//

#ifndef DMOTION_MATH_HPP
#define DMOTION_MATH_HPP

#include <vector>
#include <cmath>
#include <iostream>

namespace dmotion {


    /**
     * 把曲线斜率化为倾斜角（角度制）的函数
     */
    template<class T>
    double Slope_to_AngleDeg(T s) {
        return atan(s) * 57.3;
    }

    /**
     * 角度制到弧度值的转换
     */
    template<class T>
    inline T Deg2Rad(T deg) {
        return deg * M_PI / 180;
    }

    /**
     * 弧度值到角度值的转换
     */
    template<class T>
    inline T Rad2Deg(T rad) {
        return rad * 180 / M_PI;
    }

    /**
     * vector模版类的全局输出
     */
    template<class T>
    inline void PrintVector(const std::vector <T> &vectorOb) {
        for (unsigned i = 0; i < vectorOb.size(); i++)
            std::cout << " " << vectorOb[i];
        std::cout << std::endl;
    }

    /**
     * 判断输入参数的符号的模板函数
     */
    template<typename T>
    inline T sign(T val) {
        return val > 0 ? 1 : (val < 0 ? -1 : 0);
    }

}


#endif //DMOTION_MATH_HPP
