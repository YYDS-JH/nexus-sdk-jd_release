#pragma once
/**
 * @file ik_solver_internal.hpp
 * @brief DLS / QP 等数值 IK 迭代器的内部共用工具。
 *        仅在 src/controllers/ 内部 include，不是公共 API。
 */

#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace teleop_app {
namespace controllers {
namespace ik_internal {

inline Eigen::Matrix4d computeEndEffectorPoseMatrix(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    int ee_frame_id) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (ee_frame_id >= 0) {
        const auto fid = static_cast<pinocchio::FrameIndex>(ee_frame_id);
        const auto& placement = data.oMf[fid];
        T.block<3, 3>(0, 0) = placement.rotation();
        T.block<3, 1>(0, 3) = placement.translation();
    } else {
        const auto& placement = data.oMi[model.njoints - 1];
        T.block<3, 3>(0, 0) = placement.rotation();
        T.block<3, 1>(0, 3) = placement.translation();
    }
    return T;
}

inline Eigen::VectorXd clampJointPositionsWithLimits(
    const Eigen::VectorXd& q,
    const std::vector<double>& lower,
    const std::vector<double>& upper,
    double margin) {
    Eigen::VectorXd out = q;
    if (lower.empty() || upper.empty() || lower.size() != upper.size()) {
        return out;
    }
    const int n = static_cast<int>(out.size());
    for (int i = 0; i < n && i < static_cast<int>(lower.size()); ++i) {
        double lo = lower[static_cast<size_t>(i)] + margin;
        double hi = upper[static_cast<size_t>(i)] - margin;
        if (lo > hi) {
            const double mid = 0.5 * (lower[static_cast<size_t>(i)] + upper[static_cast<size_t>(i)]);
            lo = mid - 1e-6;
            hi = mid + 1e-6;
        }
        out(i) = std::clamp(out(i), lo, hi);
    }
    return out;
}

}  // namespace ik_internal
}  // namespace controllers
}  // namespace teleop_app
