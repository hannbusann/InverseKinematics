//
// Created by zjudancer on 17-11-13.
//

#pragma once

#include "Common/Type.h"
#include "Common/Parameters.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

// TODO avoid hard code!! using parameter class instead
namespace Motion
{
    template <typename T>
    inline T sign(T val);

    template <typename Scalar>
    class KinematicSolver {
    public:
        enum BaseType : unsigned short
        {
            COORDINATE_LEFT,
            COORDINATE_RIGHT
        };

        KinematicSolver();

        ~KinematicSolver() = default;

        /**
         * @brief   inverse kinematic function for solving leg joint angles
         *
         * @param   T_base_ankle_   transform from com to ankle(presented in com coordinate)
         * @param   isRightLeg      flag for determining the base coordinate
         * @return  six joint values of assigned leg
         */
         void LegInverseKinematic(const Isometry3<Scalar> &T_com_ankle,
                                  std::vector<Scalar>& joint_values,
                                  bool isRightLeg = true);

        /**
         * @brief forward kinematic function for calculating transform from com to leg
         *
         * @param   joint_values_   six joint values of one leg
         * @param   isRight         flag for detemining the base coordinate
         * @return  transform from com to ankle(represent in coordinate of com)
         */
        void LegForwardKinematic(const std::vector<Scalar> &joint_values_,
                                 Isometry3<Scalar>& T,
                                 bool isRight = true);

        /**
         * @brief inverse kinecmatic function that calculate joint values of two legs,
         * assuming that two feet are fixed on ground
         *
         * @param T_ar_com          input   transform from right_ankle to COM(base_link)
         * @param T_ar_al           input   transform from right_ankle to left ankle
         * @param joint_values_r    output  six joint values of right leg
         * @param joint_values_l    output  six joint values of left leg
         */
        void  ComInverseKinematic(const Isometry3<Scalar> &T_base_com,
                                  const Isometry3<Scalar> &T_base_swing,
                                  std::vector<Scalar>& joint_values,
                                  const BaseType base_coordinate);

        void WholeBodyInverseKinematic(const WholeBodyTransform& transforms,
                                       std::vector<float>& joint_values);

    private:
        // const MotionParameters& cfg;
    };

    /**
     * @description generate an Isometry represent a 3d frame from translation vector and eulers
     *
     * @param translation_  input   translation vector of the source coordinate
     * @param eulers_       input   euler angles of the source coordinate
     * @return a homogenous matrix of isometry representing transform to source coordinate
     */
    template <typename Scalar>
    Isometry3<Scalar> createFrame3d(const Vector3<Scalar> &translation_,
                                    const Vector3<Scalar> &eulers_) // TODO(hhy) avoid singularity of eulers
    {
        Isometry3<Scalar> T = Isometry3<Scalar>::Identity();
        T.rotate(AngleAxisZ<Scalar>(eulers_[2])*AngleAxisY<Scalar>(eulers_[1])*AngleAxisX<Scalar>(eulers_[0]));
        T.pretranslate(translation_);

        return std::move(T);
    }
}
