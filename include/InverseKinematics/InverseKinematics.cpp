//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
// TODO 站在地上的机器人逆运动学

#include "InverseKinematics.h"
#include <iostream>
#include <cmath>
#include <vector>
#include "dmotion_math.hpp"

namespace dmotion {

    const double InvKin::upper_leg_length = 12.0;  //大腿的长度
    const double InvKin::lower_leg_length = 12.0;  //小腿的长度
    const double InvKin::ankle_from_ground = 3.5;  //脚踝距离地面的高度
    const double InvKin::half_hip_width = 4.5;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
    const double InvKin::hip_x_from_origin = 0;    //髋关节点相对于身体中心原点的x方向坐标
    const double InvKin::hip_z_from_origin = 8.0;  //髋关节点相对于身体中心原点的z

    InvKin::InvKin(bool isRight) {
        isRight_ = isRight;
        hip_yaw_ = 0;
        hip_roll_ = 0;
        hip_pitch_ = 0;
        knee_pitch_ = 0;
        ankle_pitch_ = 0;
        ankle_roll_ = 0;
    }

    /**
     * 求解脚相对身体中心点的逆运行学
     * @param foot_pose 脚底的x y z r p y
     * @return
     */
    std::vector<double> InvKin::LegInvKin(std::vector<double> &foot_pose) {
        /** 下面三个坐标是脚踝点相对于髋关节点的坐标 **/
        foot_pose[3] = dmotion::Deg2Rad(foot_pose[3]);
        foot_pose[4] = dmotion::Deg2Rad(foot_pose[4]);
        foot_pose[5] = dmotion::Deg2Rad(foot_pose[5]);

        double foot_vertical_x = -(ankle_from_ground *
                                   (std::sin(foot_pose[3]) *
                                    std::sin(foot_pose[5]) +
                                    std::cos(foot_pose[3]) *
                                    std::cos(foot_pose[5]) *
                                    std::sin(foot_pose[4])));
        double foot_vertical_y = -(-ankle_from_ground *
                                   (std::cos(foot_pose[5]) *
                                    std::sin(foot_pose[3]) -
                                    std::cos(foot_pose[3]) *
                                    std::sin(foot_pose[4]) *
                                    std::sin(foot_pose[5])));
        double foot_vertical_z = -(ankle_from_ground * std::cos(foot_pose[4]) *
                                   std::cos(foot_pose[3]));

        double ankle_x_to_hip = foot_pose[0] - foot_vertical_x - hip_x_from_origin;
        double ankle_y_to_hip = foot_pose[1] - foot_vertical_y - (isRight_ ? (-half_hip_width) : half_hip_width);
        double ankle_z_to_hip = foot_pose[2] - foot_vertical_z + hip_z_from_origin;
        double ankle_norm = std::sqrt(
                ankle_x_to_hip * ankle_x_to_hip + ankle_y_to_hip * ankle_y_to_hip + ankle_z_to_hip * ankle_z_to_hip);
        /** 获得了knee_pitch的角度 **/
        knee_pitch_ = M_PI - CosineTheorem(upper_leg_length, lower_leg_length, ankle_norm);
        /** 这里计算脚踝roll关节的转轴的矢量 **/
        double ankle_axis_x = std::cos(foot_pose[4]) * std::cos(foot_pose[5]);
        double ankle_axis_y = std::cos(foot_pose[4]) * std::sin(foot_pose[5]);
        double ankle_axis_z = -std::sin(foot_pose[4]);
        /** 求ankle_axis和ankle_to_hip的公垂线向量,默认y方向是1**/
        double vertical_x = (ankle_z_to_hip * ankle_axis_y - ankle_y_to_hip * ankle_axis_z) /
                            (ankle_x_to_hip * ankle_axis_z - ankle_axis_x * ankle_z_to_hip);
        double vertical_z = (ankle_x_to_hip * ankle_axis_y - ankle_axis_x * ankle_y_to_hip) /
                            (ankle_z_to_hip * ankle_axis_x - ankle_x_to_hip * ankle_axis_z);
        /** 这里获得这个公垂线与水平面的夹角 **/
        double vertical_unitz = GetDelta(0, 0, 1, vertical_x, 1, vertical_z);
        /** 获得了hip_roll的角度 **/
        hip_roll_ = isRight_ ? (vertical_unitz - M_PI / 2) : (M_PI / 2 - vertical_unitz);
        /** 获得了hip_yaw的角度 **/
        double hip_yaw_delta = GetDelta(0, 1, vertical_x, 1);
        if (vertical_x > 0)
            hip_yaw_ = isRight_ ? hip_yaw_delta : (-hip_yaw_delta);
        else
            hip_yaw_ = isRight_ ? (-hip_yaw_delta) : hip_yaw_delta;

        double ankle_to_hip_yaw_roll_x;
        double ankle_to_hip_yaw_roll_z;
        /**分左右腿获得hip_pitch的角度**/
        if (!isRight_) {
            /** 先得到ankle_to_hip在ankle只经过yaw和roll变换之后的坐标系中的坐标 **/
            ankle_to_hip_yaw_roll_x = ankle_x_to_hip * std::cos(dmotion::Deg2Rad(hip_yaw_)) +
                                      ankle_y_to_hip * std::sin(dmotion::Deg2Rad(hip_yaw_));
            /** 因为估计此时的y分量应该为0，所以不算ankle_to_hip_yaw_roll_y **/
            /*
            double ankle_to_hip_yaw_roll_y = ankle_z_to_hip * std::sin( hip_roll_)) +
                                             ankle_y_to_hip * std::cos(dmotion::Deg2Rad(hip_roll_)) *
                                             std::cos(dmotion::Deg2Rad(hip_yaw_)) -
                                             ankle_x_to_hip * std::cos(dmotion::Deg2Rad(hip_roll_)) *
                                             std::sin(dmotion::Deg2Rad(hip_yaw_));
            */
            ankle_to_hip_yaw_roll_z = ankle_z_to_hip * std::cos(dmotion::Deg2Rad(hip_roll_)) -
                                      ankle_y_to_hip * std::cos(dmotion::Deg2Rad(hip_yaw_)) *
                                      std::sin(dmotion::Deg2Rad(hip_roll_)) +
                                      ankle_x_to_hip * std::sin(dmotion::Deg2Rad(hip_roll_)) *
                                      std::sin(dmotion::Deg2Rad(hip_yaw_));
        } else if (isRight_) {
            ankle_to_hip_yaw_roll_x = ankle_x_to_hip * std::cos(dmotion::Deg2Rad(-hip_yaw_)) +
                                      ankle_y_to_hip * std::sin(dmotion::Deg2Rad(-hip_yaw_));
            /** 因为估计此时的y分量应该为0，所以不算ankle_to_hip_yaw_roll_y **/
            /*
            double ankle_to_hip_yaw_roll_y = ankle_z_to_hip * std::sin(dmotion::Deg2Rad(hip_roll_)) +
                                             ankle_y_to_hip * std::cos(dmotion::Deg2Rad(hip_roll_)) *
                                             std::cos(dmotion::Deg2Rad(hip_yaw_)) -
                                             ankle_x_to_hip * std::cos(dmotion::Deg2Rad(hip_roll_)) *
                                             std::sin(dmotion::Deg2Rad(hip_yaw_));
            */
            ankle_to_hip_yaw_roll_z = ankle_z_to_hip * std::cos(dmotion::Deg2Rad(-hip_roll_)) -
                                      ankle_y_to_hip * std::cos(dmotion::Deg2Rad(-hip_yaw_)) *
                                      std::sin(dmotion::Deg2Rad(-hip_roll_)) +
                                      ankle_x_to_hip * std::sin(dmotion::Deg2Rad(-hip_roll_)) *
                                      std::sin(dmotion::Deg2Rad(-hip_yaw_));
        }
        /** 获得只经过yaw和roll变换后的-z_unit向量和上面这个向量的夹角
         * 获得hip_pitch_**/
        double hip_pitch_absolute = std::atan(ankle_to_hip_yaw_roll_x / (-ankle_to_hip_yaw_roll_z));
        hip_pitch_ = hip_pitch_absolute + CosineTheorem(upper_leg_length, ankle_norm, lower_leg_length);
        /** 这里获得脚底向下的向量在经过hip的ypr变换之后的坐标系中的坐标**/
        if (!isRight_) {
            double foot_hip_rpy_x = foot_vertical_x * std::cos(-hip_pitch_absolute) * std::cos(hip_yaw_) -
                                    foot_vertical_z * std::cos(hip_roll_) * std::sin(-hip_pitch_absolute) +
                                    foot_vertical_y * std::cos(-hip_pitch_absolute) * std::sin(hip_yaw_) +
                                    foot_vertical_y * std::cos(hip_yaw_) * std::sin(-hip_pitch_absolute) *
                                    std::sin(hip_roll_) -
                                    foot_vertical_x * std::sin(-hip_pitch_absolute) * std::sin(hip_roll_) *
                                    std::sin(hip_yaw_);
            double foot_hip_rpy_y =
                    foot_vertical_z * std::sin(hip_roll_) + foot_vertical_y * std::cos(hip_roll_) * std::cos(hip_yaw_) -
                    foot_vertical_x * std::cos(hip_roll_) * std::sin(hip_yaw_);
            double foot_hip_rpy_z = foot_vertical_z * std::cos(-hip_pitch_absolute) * std::cos(hip_roll_) +
                                    foot_vertical_x * std::cos(hip_yaw_) * std::sin(-hip_pitch_absolute) +
                                    foot_vertical_y * std::sin(-hip_pitch_absolute) * std::sin(hip_yaw_) -
                                    foot_vertical_y * std::cos(-hip_pitch_absolute) * std::cos(hip_yaw_) *
                                    std::sin(hip_roll_) +
                                    foot_vertical_x * std::cos(-hip_pitch_absolute) * std::sin(hip_roll_) *
                                    std::sin(hip_yaw_);
            double ankle_pitch_absolute = std::atan(-foot_hip_rpy_x / foot_hip_rpy_z);
            ankle_pitch_ = ankle_pitch_absolute + CosineTheorem(lower_leg_length, ankle_norm, upper_leg_length);
            ankle_roll_ = std::atan(
                    -foot_hip_rpy_y / std::sqrt(foot_hip_rpy_x * foot_hip_rpy_x + foot_hip_rpy_z * foot_hip_rpy_z));

        } else if (isRight_) {
            double foot_hip_rpy_x = foot_vertical_x * std::cos(-hip_pitch_absolute) * std::cos(-hip_yaw_) -
                                    foot_vertical_z * std::cos(-hip_roll_) * std::sin(-hip_pitch_absolute) +
                                    foot_vertical_y * std::cos(-hip_pitch_absolute) * std::sin(-hip_yaw_) +
                                    foot_vertical_y * std::cos(-hip_yaw_) * std::sin(-hip_pitch_absolute) *
                                    std::sin(-hip_roll_) -
                                    foot_vertical_x * std::sin(-hip_pitch_absolute) * std::sin(-hip_roll_) *
                                    std::sin(-hip_yaw_);
            double foot_hip_rpy_y =
                    foot_vertical_z * std::sin(-hip_roll_) + foot_vertical_y * std::cos(-hip_roll_) * std::cos(-hip_yaw_) -
                    foot_vertical_x * std::cos(-hip_roll_) * std::sin(-hip_yaw_);
            double foot_hip_rpy_z = foot_vertical_z * std::cos(-hip_pitch_absolute) * std::cos(-hip_roll_) +
                                    foot_vertical_x * std::cos(-hip_yaw_) * std::sin(-hip_pitch_absolute) +
                                    foot_vertical_y * std::sin(-hip_pitch_absolute) * std::sin(-hip_yaw_) -
                                    foot_vertical_y * std::cos(-hip_pitch_absolute) * std::cos(-hip_yaw_) *
                                    std::sin(-hip_roll_) +
                                    foot_vertical_x * std::cos(-hip_pitch_absolute) * std::sin(-hip_roll_) *
                                    std::sin(-hip_yaw_);
            double ankle_pitch_absolute = std::atan(-foot_hip_rpy_x / foot_hip_rpy_z);
            ankle_pitch_ = ankle_pitch_absolute + CosineTheorem(lower_leg_length, ankle_norm, upper_leg_length);
            ankle_roll_ = std::atan(
                    foot_hip_rpy_y / std::sqrt(foot_hip_rpy_x * foot_hip_rpy_x + foot_hip_rpy_z * foot_hip_rpy_z));
        }

        std::vector<double> finals = {hip_yaw_, hip_roll_, hip_pitch_, knee_pitch_, ankle_pitch_, ankle_roll_};
        dmotion::Rad2Deg(finals);
        return finals;

    }

    /**
     * 使用三角形的余弦定理求某个角度,高考必考
     * @param edge_1 临边1长度
     * @param edge_2 临边2长度
     * @param edge_opppsite 对边长度
     * @return 要求的角的角度值
     */
    double InvKin::CosineTheorem(const double &edge_1,
                                 const double &edge_2,
                                 const double &edge_opppsite) {
        double cosine = (edge_1 * edge_1 + edge_2 * edge_2 - edge_opppsite * edge_opppsite) / (2 * edge_1 * edge_2);
        return std::acos(cosine);
    }

    /**
     * 获得两个三维向量之间夹角的函数
     * @param x_1
     * @param y_1
     * @param z_1
     * @param x_2
     * @param y_2
     * @param z_2
     * @return
     */
    double InvKin::GetDelta(const double &x_1, const double &y_1, const double &z_1, const double &x_2,
                            const double &y_2, const double &z_2) {
        double tmpcos = (x_1 * x_2 + y_1 * y_2 + z_1 * z_2) /
                        sqrt((x_1 * x_1 + y_1 * y_1 + z_1 * z_1) * (x_2 * x_2 + y_2 * y_2 + z_2 * z_2));
        return std::acos(tmpcos);
    }

    /**
     * 获得两个二维向量之间夹角的函数
     * @param x_1
     * @param y_1
     * @param x_2
     * @param y_2
     * @return
     */
    double InvKin::GetDelta(const double &x_1, const double &y_1, const double &x_2, const double &y_2) {
        double tmpcos = (x_1 * x_2 + y_1 * y_2) / sqrt((x_1 * x_1 + y_1 * y_1) * (x_2 * x_2 + y_2 * y_2));
        return std::acos(tmpcos);
    }


}
