#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <memory>

namespace teleop_app {
namespace solvers {

/**
 * @brief 运动学解算结果
 */
struct KinematicsPoseResult {
    Eigen::Vector3d position;       // 位置 [x, y, z]
    Eigen::Quaterniond quaternion;  // 姿态四元数 [x, y, z, w]
    Eigen::Vector3d rpy;            // 欧拉角 [roll, pitch, yaw]
    Eigen::Matrix4d homogeneous_matrix; // 齐次变换矩阵 (4x4)
    bool valid;                     // 是否有效
    
    KinematicsPoseResult() : valid(false) {
        position.setZero();
        quaternion.setIdentity();
        rpy.setZero();
        homogeneous_matrix.setIdentity();
    }
};

/**
 * @brief 运动学解算器参数
 */
struct KinematicsSolverParams {
    std::string urdf_path;       // URDF模型路径
    std::string end_link_name;   // 末端link名称
    int joint_count;             // 关节数量
    // 可扩展其他参数
};

/**
 * @brief 运动学解算器状态
 */
enum class SolverStatus {
    UNINITIALIZED,  // 未初始化
    INITIALIZED,    // 已初始化
    ERROR          // 错误状态
};

/**
 * @brief 运动学解算器类（独立算法组件）
 * 
 * 设计理念：
 * - 纯算法组件，不包含ROS相关功能
 * - 使用Pimpl模式隐藏实现细节
 * - 提供FK和IK接口
 * - 后续可以扩展不同的算法实现（Pinocchio、KDL等）
 */
class KinematicsSolver {
public:
    KinematicsSolver();
    ~KinematicsSolver();

    /**
     * @brief 初始化解算器
     * @param params 解算器参数
     * @return true 初始化成功
     */
    bool initialize(const KinematicsSolverParams& params);

    /**
     * @brief 正向运动学计算
     * @param joint_positions 关节位置向量
     * @return 末端位姿结果
     */
    KinematicsPoseResult computeFK(const Eigen::VectorXd& joint_positions);

    /**
     * @brief 逆向运动学计算
     * @param target_position 目标位置
     * @param target_orientation 目标姿态（四元数）
     * @param initial_guess 初始猜测关节位置（可选）
     * @return 关节位置解（如果valid为false则求解失败）
     */
    KinematicsPoseResult computeIK(
        const Eigen::Vector3d& target_position,
        const Eigen::Quaterniond& target_orientation,
        const Eigen::VectorXd& initial_guess = Eigen::VectorXd());

    /**
     * @brief 获取关节数量
     */
    int getJointCount() const;

    /**
     * @brief 获取解算器状态
     */
    SolverStatus getStatus() const;

private:
    // Pimpl模式：隐藏实现细节
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace solvers
} // namespace teleop_app

