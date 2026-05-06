/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: human_data/include/human_data/solvers/kinematics_solver.hpp
 * @Description: 运动学解算器类，提供正向和逆向运动学计算
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef __KINEMATICS_SOLVER_HPP__
#define __KINEMATICS_SOLVER_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <memory>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace human_data {
namespace solvers {

/*******************************************************************************
 * Structure definition
 ******************************************************************************/

/**
  * @brief  运动学解算结果
  * @param  null
  * @retval null
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
  * @brief  运动学解算器参数
  * @param  null
  * @retval null
  */
struct KinematicsSolverParams {
    std::string urdf_path;          // URDF模型路径
    std::string base_link_name;     // 基座link名称（FK计算的起始点）
    std::string end_link_name;      // 末端link名称
    std::vector<int> joint_indices; // 使用的关节索引（在URDF模型中的位置）
};

/**
  * @brief  运动学解算器状态枚举
  * @param  null
  * @retval null
  */
enum class SolverStatus {
    UNINITIALIZED,  // 未初始化
    INITIALIZED,    // 已初始化
    ERROR          // 错误状态
};

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  运动学解算器类（独立算法组件）
  * @param  null
  * @retval null
  * @note   设计理念：
  *         - 纯算法组件，不包含ROS相关功能
  *         - 使用Pimpl模式隐藏实现细节
  *         - 提供FK和IK接口
  *         - 后续可以扩展不同的算法实现（Pinocchio、KDL等）
  */
class KinematicsSolver {
public:
    /**
      * @brief  构造函数
      * @param  null
      * @retval null
      * @usage
      */
    KinematicsSolver();

    /**
      * @brief  析构函数
      * @param  null
      * @retval null
      * @usage
      */
    ~KinematicsSolver();

    /**
      * @brief  初始化解算器
      * @param  params 解算器参数
      * @retval true 初始化成功，false 初始化失败
      * @usage
      */
    bool initialize(const KinematicsSolverParams& params);

    /**
      * @brief  正向运动学计算
      * @param  joint_positions 关节位置向量
      * @retval 末端位姿结果
      * @usage
      */
    KinematicsPoseResult computeFK(const Eigen::VectorXd& joint_positions);

    /**
      * @brief  逆向运动学计算
      * @param  target_position 目标位置
      * @param  target_orientation 目标姿态（四元数）
      * @param  initial_guess 初始猜测关节位置（可选）
      * @retval 关节位置解（如果valid为false则求解失败）
      * @usage
      */
    KinematicsPoseResult computeIK(
        const Eigen::Vector3d& target_position,
        const Eigen::Quaterniond& target_orientation,
        const Eigen::VectorXd& initial_guess = Eigen::VectorXd());

    /**
      * @brief  获取关节数量
      * @param  null
      * @retval 关节数量
      * @usage
      */
    int getJointCount() const;

    /**
      * @brief  获取解算器状态
      * @param  null
      * @retval 解算器状态
      * @usage
      */
    SolverStatus getStatus() const;

private:
    // Pimpl模式：隐藏实现细节
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace solvers
} // namespace human_data

#endif // __KINEMATICS_SOLVER_HPP__
