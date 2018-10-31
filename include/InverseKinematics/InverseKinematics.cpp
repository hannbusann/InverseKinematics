//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
//

#include "InverseKinematics.h"
#include <cmath>
#include "dmotion_math.hpp"

namespace dmotion {

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
    std::vector<double> &InvKin::LegInvKin(std::vector<double> &foot_pose) {
        // 下面三个坐标是脚踝点相对于髋关节点的坐标
        double ankle_x_to_hip = foot_pose[0] + ankle_from_ground *
                                               (std::sin(Deg2Rad(foot_pose[3])) * std::sin(Deg2Rad(foot_pose[5])) +
                                                std::cos(Deg2Rad(foot_pose[3])) *
                                                std::cos(Deg2Rad(foot_pose[5])) *
                                                std::sin(Deg2Rad(foot_pose[4]))) - hip_x_from_origin;
        double ankle_y_to_hip = foot_pose[1] - ankle_from_ground *
                                               (std::cos(Deg2Rad(foot_pose[5])) * std::sin(Deg2Rad(foot_pose[3])) -
                                                std::cos(Deg2Rad(foot_pose[3])) * std::sin(Deg2Rad(foot_pose[4])) *
                                                std::sin(Deg2Rad(foot_pose[5]))) -
                                (isRight ? (-half_hip_width) : half_hip_width);
        double ankle_z_to_hip =
                foot_pose[2] + ankle_from_ground * std::cos(Deg2Rad(foot_pose[4])) * std::cos(Deg2Rad(foot_pose[3])) -
                hip_z_from_origin;
        double ankle_norm = std::sqrt(
                ankle_x_to_hip * ankle_x_to_hip + ankle_y_to_hip * ankle_y_to_hip + ankle_z_to_hip * ankle_z_to_hip);
        /** 获得了knee_pitch的角度 **/
        knee_pitch_ = 180 - CosineTheorem(upper_leg_length, lower_leg_length, ankle_norm);
        /** 这里计算脚踝roll关节的转轴的矢量 **/
        double ankle_axis_x = std::cos(Deg2Rad(foot_pose[4])) * std::cos(Deg2Rad(foot_pose[5]));
        double ankle_axis_y = std::cos(Deg2Rad(foot_pose[4])) * std::sin(Deg2Rad(foot_pose[5]));
        double ankle_axis_z = -std::sin(Deg2Rad(foot_pose[4]));
        /** 求ankle_axis和ankle_to_hip的公垂线向量,默认y方向是1**/
        double vertical_x = (ankle_z_to_hip * ankle_axis_y - ankle_y_to_hip * ankle_axis_z) /
                            (ankle_x_to_hip * ankle_axis_z - ankle_axis_x * ankle_z_to_hip);
        double vertical_z = (ankle_x_to_hip * ankle_axis_y - ankle_axis_x * ankle_y_to_hip) /
                            (ankle_z_to_hip * ankle_axis_x - ankle_x_to_hip * ankle_axis_z);
        /** 这里获得这个公垂线与水平面的夹角 **/
        double vertical_unitz = GetDelta(0, 0, 1, vertical_x, 1, vertical_z);
        /** 获得了hip_roll的角度 **/
        hip_roll_ = isRight_ ? (vertical_unitz - 90) : (90 - vertical_unitz);
        /** 获得了hip_yaw的角度 **/
        double hip_yaw_delta = GetDelta(0, 1, vertical_x, 1);
        if (vertical_x > 0)
            hip_yaw_ = isRight_ ? hip_yaw_delta : (-hip_yaw_delta);
        else
            hip_yaw_ = isRight_ ? (-hip_yaw_delta) : hip_yaw_delta;
        
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
        return std::acos(cosine) * 57.296;
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
        return std::acos(tmpcos) * 57.296;
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
        return std::acos(tmpcos) * 57.296;
    }


}
