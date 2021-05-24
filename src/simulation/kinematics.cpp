#include "simulation/kinematics.h"

#include "Eigen/Dense"

#include "acclaim/bone.h"
#include "util/helper.h"

#define M_PI 3.1415

namespace kinematics {
Eigen::VectorXd pseudoInverseLinearSolver(const Eigen::Matrix4Xd& Jacobian, const Eigen::Vector4d& target) {
    // TODO
    // You need to return the solution (x) of the linear least squares system:
    //     i.e. find x which min(| Jacobian * x - target |)

    Eigen::Matrix3Xd newJacobian = Jacobian.topRows(3);
    Eigen::Vector3d newTarget = target.head<3>();

    Eigen::MatrixX3d pseudoInverseJacobian =
        newJacobian.transpose() * ((newJacobian * newJacobian.transpose())).inverse();

    Eigen::VectorXd solution = pseudoInverseJacobian * newTarget;

    return solution;
}

/**
 * @brief Perform inverse kinematics (IK)
 *
 * @param target_pos The position where `end_bone` will move to.
 * @param start_bone This bone is the last bone you can move while doing IK
 * @param end_bone This bone will try to reach `target_pos`
 * @param posture The original AMC motion's reference, you need to modify this
 *
 * @return True if IK is stable (HW3 bonus)
 */
bool inverseJacobianIKSolver(const Eigen::Vector4d& target_pos, acclaim::Bone* start_bone, acclaim::Bone* end_bone,
                             acclaim::Posture& posture) {
    constexpr int max_iteration = 1000;
    constexpr double epsilon = 1E-3;
    constexpr double step = 0.1;

    // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is root.
    acclaim::Bone* root_bone = start_bone - start_bone->idx;

    // TODO
    // Perform inverse kinematics (IK)
    // HINTs will tell you what should do in that area.
    // Of course you can ignore it (Any code below this line) and write your own code.

    acclaim::Bone* temp_bone = end_bone;
    size_t bone_num = 1;

    while (temp_bone != start_bone) {
        if (temp_bone == root_bone) {
            double targetStartingLength =
                sqrt((target_pos - root_bone->start_position).dot(target_pos - root_bone->start_position));
            break;
        }

        bone_num++;
        temp_bone = temp_bone->parent;
    }

    // HINT:
    // calculate number of bones need to move to perform IK, store in `bone_num`
    // a.k.a. how may bones from end_bone to its parent than to start_bone (include both side)

    for (int iter = 0; iter < max_iteration; iter++) {

        forwardSolver(posture, root_bone);

        Eigen::Vector4d desiredVector = target_pos - end_bone->end_position;
        if (desiredVector.norm() < epsilon) {
            return true;
        }

        // HINT:
        // Calculate Jacobian, store in `Jacobian`
        
        Eigen::Matrix4Xd Jacobian(4, 3 * bone_num);
        Jacobian.setZero();

        acclaim::Bone* temp_bone = end_bone;

        for (int i = 0; i < bone_num; i++) {

            int jacobianCount = i * 3;

            if (temp_bone->dofrx) {
                Eigen::Vector4d rotationAxisX = (temp_bone->rotation * Eigen::Vector4d(1, 0, 0, 0)).normalized();
                Jacobian.col(jacobianCount + 0) = rotationAxisX.cross3(target_pos - temp_bone->start_position);
            }
            if (temp_bone->dofry) {
                Eigen::Vector4d rotationAxisY = (temp_bone->rotation * Eigen::Vector4d(0, 1, 0, 0)).normalized();
                Jacobian.col(jacobianCount + 1) = rotationAxisY.cross3(target_pos - temp_bone->start_position);
            }
            if (temp_bone->dofrz) {
                Eigen::Vector4d rotationAxisZ = (temp_bone->rotation * Eigen::Vector4d(0, 0, 1, 0)).normalized();
                Jacobian.col(jacobianCount + 2) = rotationAxisZ.cross3(target_pos - temp_bone->start_position);
            }

            temp_bone = temp_bone->parent;
        }

        Eigen::VectorXd deltatheta = step * pseudoInverseLinearSolver(Jacobian, desiredVector);

        // HINT:
        // Change `posture.bone_rotation` based on deltatheta

        temp_bone = end_bone;
        
        for (int i = 0; i < bone_num; i++) {
            int deltaThetaNum = i * 3;
            int bone_idx = temp_bone->idx;

           Eigen::Vector4d deltaRotation(deltatheta[deltaThetaNum + 0] * (180 / M_PI),
                                          deltatheta[deltaThetaNum + 1] * (180 / M_PI),
                                          deltatheta[deltaThetaNum + 2] * (180 / M_PI), 0);
           
           posture.bone_rotations[bone_idx] += deltaRotation;
           temp_bone = temp_bone->parent;
        }
    }

    // TODO (Bonus)
    // Return IK is stable?
    // i.e. not swinging its hand in air

    return false;
}
}  // namespace kinematics
