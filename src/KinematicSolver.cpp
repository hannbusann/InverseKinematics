//
// Created by zjudancer on 17-11-13.
//

#include "KinematicSolver.h"

#include <cmath>

#include <Eigen/Geometry>

namespace Motion
{
    template <typename T>
    inline T sign(T val)
    {
        return val > 0 ? 1: (val < 0? -1: 0);
    }

    template <typename Scalar>
    KinematicSolver<Scalar>::KinematicSolver()
    {
        std::cout << parameters.motion.length_tibia << std::endl;
        std::cout << parameters.motion.length_thigh << std::endl;
        std::cout << parameters.motion.length_hip << std::endl;
    }

    template <typename Scalar>
    void KinematicSolver<Scalar>::LegInverseKinematic(const Isometry3<Scalar> &_T_com_ankle,
                                                      std::vector<Scalar>& joint_values_,
                                                      bool _isRightLeg)
    {
        Scalar tibia = parameters.motion.length_tibia;
        Scalar thigh = parameters.motion.length_thigh;
        Scalar half_hip_len = parameters.motion.length_hip/2;
        
        joint_values_.resize(6);

        // ankle pose
        Matrix3<Scalar> R7 = _T_com_ankle.rotation();
        Vector3<Scalar> p7 = _T_com_ankle.translation();

        // base link pose
        Matrix3<Scalar> R1 = Matrix3<Scalar>::Identity();
        Vector3<Scalar> p1(0, 0, 0);    // base_link pose

        Vector3<Scalar> p2;
        if (_isRightLeg)
            p2 = p1 - R1*Vector3<Scalar>(0, half_hip_len, 0);
        else
            p2 = p1 - R1*Vector3<Scalar>(0, -half_hip_len, 0);

        Vector3<Scalar> r = R7.transpose()*(p2-p7);
        Scalar C = r.norm();
        Scalar c5 = (C*C- thigh*thigh-tibia*tibia)/(2.0*thigh*tibia);
        Scalar q5 = std::acos( c5 );
        if (c5 > 1)
            q5 = 0;
        else if(c5 < -1)
            q5 = static_cast<Scalar>(M_PI);

        Scalar alpha = std::asin(thigh/C * std::sin(M_PI-q5));

        Scalar q7 = std::atan2(r[1], r[2]);
        Scalar q6 = -std::atan2(r[0], sign(r[2])*std::sqrt(r[1]*r[1]+r[2]*r[2]))-alpha;

        Matrix3<Scalar> R234 = R1.transpose()*R7*AngleAxis<Scalar>(-q7, Vector3<Scalar>::UnitX()).toRotationMatrix()
                               *AngleAxis<Scalar>(-q5-q6, Vector3<Scalar>::UnitY()).toRotationMatrix();

        Scalar q2 = std::atan2(-R234(0,1), R234(1,1));
        Scalar q3 = std::atan2(R234(2,1), -R234(0, 1)*std::sin(q2)+R234(1, 1)*std::cos(q2));
        Scalar q4 = std::atan2(-R234(2,0), R234(2, 2));
        // Eigen::Vector3d q234 = R234.eulerAngles(2, 0, 1);     // TODO maybe work?
        // double q2 = q234[0], q3 = q234[1], q4 = q234[2];

        joint_values_[0] = q2; joint_values_[1] = q3; joint_values_[2] = q4;
        joint_values_[3] = q5; joint_values_[4] = q6; joint_values_[5] = q7;
    }

    template <typename Scalar>
    void KinematicSolver<Scalar>::ComInverseKinematic(const Isometry3<Scalar> &_T_base_com,
                                              const Isometry3<Scalar> &_T_base_swing,
                                              std::vector<Scalar> &joint_values_,
                                              const BaseType _base_coordinate)
    {
        Isometry3<Scalar> _T_com_base = _T_base_com.inverse();
        Isometry3<Scalar> _T_com_swing = _T_com_base*_T_base_swing;

//        joint_values_.resize(12);

        std::vector<Scalar> joint_values_l, joint_values_r;
        if (_base_coordinate == COORDINATE_RIGHT)
        {
            LegInverseKinematic(_T_com_base, joint_values_r, true);
            LegInverseKinematic(_T_com_swing, joint_values_l, false);
        }
        else if(_base_coordinate == COORDINATE_LEFT)
        {
            LegInverseKinematic(_T_com_swing, joint_values_r, true);
            LegInverseKinematic(_T_com_base, joint_values_l, false);
        }

        joint_values_ = joint_values_r;
        joint_values_.insert(joint_values_.end(), joint_values_l.begin(), joint_values_l.end());
    }

    template <typename Scalar>
    void KinematicSolver<Scalar>::WholeBodyInverseKinematic(const WholeBodyTransform& transforms,
                                                            std::vector<float>& joint_values)
    {
        std::vector<Scalar> joint_values_l, joint_values_r;
        
        LegInverseKinematic(transforms.right_leg, joint_values_r, true);
        LegInverseKinematic(transforms.left_leg, joint_values_l, false);

        joint_values = std::move(joint_values_r);
        joint_values.insert(joint_values.end(), joint_values_l.begin(), joint_values_l.end());
    }

    // TODO(hhy)
    template <typename Scalar>
    void KinematicSolver<Scalar>::LegForwardKinematic(const std::vector<Scalar> &joint_values_,
                                              Isometry3<Scalar> &T,
                                              bool isRight)
    {
        /*Eigen::Isometry3d T_r = Eigen::Isometry3d::Identity();

        // to pelvis
        if (isRight)
            T_r.pretranslate(Eigen::Vector3d(0, -D, 0));
        else
            T_r.pretranslate(Eigen::Vector3d(0, D, 0));
        T_r.rotate(Motion::AngleAxisZ(joint_values_[0])
                   *Motion::AngleAxisX(joint_values_[1])
                   *Motion::AngleAxisY(joint_values_[2]));

        // to knee
        T_r.translate(Eigen::Vector3d(0, 0, -A));
        T_r.rotate(Motion::AngleAxisY(joint_values_[3]));

        // to ankle
        T_r.translate(Eigen::Vector3d(0, 0, -B));
        T_r.rotate(Motion::AngleAxisY(joint_values_[4])
                   *Motion::AngleAxisX(joint_values_[5]));

        T_base_ankle_ = T_r;*/
    }

    template class KinematicSolver<float>;
}

