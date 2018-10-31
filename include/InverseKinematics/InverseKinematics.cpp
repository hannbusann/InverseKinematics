//
// Created by zjudancer on 18-10-29.
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

    std::vector<double> &InvKin::LegInvKin(std::vector<double> &foot_pose, bool isRight) {
        // 下面三个坐标是脚踝点相对于髋关节点的坐标
        double ankle_x_to_hip = foot_pose[0] + ankle_from_ground *
                                               (std::sin(Deg2Rad(foot_pose[3])) * std::sin(Deg2Rad(foot_pose[5])) +
                                                std::std::cos(Deg2Rad(foot_pose[3])) *
                                                std::std::cos(Deg2Rad(foot_pose[5])) *
                                                std::sin(Deg2Rad(foot_pose[4]))) - hip_x_from_origin;
        double ankle_y_to_hip = foot_pose[1] - ankle_from_ground *
                                               (std::cos(Deg2Rad(foot_pose[5])) * std::sin(Deg2Rad(foot_pose[3])) -
                                                std::cos(Deg2Rad(foot_pose[3])) * std::sin(Deg2Rad(foot_pose[4])) *
                                                std::sin(Deg2Rad(foot_pose[5]))) -
                                (isRight ? (-half_hip_width) : half_hip_width);
        double ankle_z_to_hip = foot_pose[2] + ankle_from_ground * cos(pp) * cos(rr) - hip_z_from_origin;
        double ankle_norm = std::sqrt(
                ankle_x_to_hip * ankle_x_to_hip + ankle_y_to_hip * ankle_y_to_hip + ankle_z_to_hip * ankle_z_to_hip);
        hip_pitch_ = CosineTheorem(upper_leg_length, ankle_norm, lower_leg_length);
        knee_pitch_ = 180 - CosineTheorem(upper_leg_length,lower_leg_length,ankle_norm);

    }

    double InvKin::CosineTheorem(const double &edge_1,
                                 const double &edge_2,
                                 const double &edge_opppsite) {
        double cosine = (edge_1 * edge_1 + edge_2 * edge_2 - edge_opppsite * edge_opppsite) / (2 * edge_1 * edge_2);
        return std::acos(cosine);
    }
}
