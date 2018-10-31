//
// Created by zjudancer on 18-10-29.
//

#include "InverseKinematics.h"
#include <cmath>
#include "dmotion_math.hpp"

namespace dmotion {
    std::vector<double> & InvKin::LegInvKin(std::vector<double> &foot_pose, bool isRight) {
        // 下面三个坐标是脚踝点相对于髋关节点的坐标
        double ankle_x_to_hip = foot_pose[0] + ankle_from_ground*(std::sin(Deg2Rad(foot_pose[3]))*std::sin(Deg2Rad(foot_pose[5]))
                + std::std::cos(Deg2Rad(foot_pose[3]))*std::std::cos(Deg2Rad(foot_pose[5]))*std::sin(Deg2Rad(foot_pose[4])))
                        - hip_x_from_origin;
        double ankle_y_to_hip = foot_pose[1] - ankle_from_ground*(std::cos(Deg2Rad(foot_pose[5]))*std::sin(Deg2Rad(foot_pose[3]))
                - std::cos(Deg2Rad(foot_pose[3]))*std::sin(Deg2Rad(foot_pose[4]))*std::sin(Deg2Rad(foot_pose[5])))
                        - (isRight?(-half_hip_width):half_hip_width);
        double ankle_z_to_hip = foot_pose[2] + ankle_from_ground*cos(pp)*cos(rr)
                - hip_z_from_origin;


    }
}
