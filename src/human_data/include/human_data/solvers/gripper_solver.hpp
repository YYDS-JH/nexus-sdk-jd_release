/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: human_data/include/human_data/solvers/gripper_solver.hpp
 * @Description: 夹爪解算器类，提供夹爪状态计算和反向计算
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef __GRIPPER_SOLVER_HPP__
#define __GRIPPER_SOLVER_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <Eigen/Core>
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
  * @brief  夹爪解算结果
  * @param  null
  * @retval null
  */
struct GripperResult {
    double value;                  // 夹爪值（距离或角度，具体含义由robot_type决定）
    bool valid;                    // 是否有效
    
    GripperResult() : value(0.0), valid(false) {}
};

/**
  * @brief  夹爪解算器参数
  * @param  null
  * @retval null
  * @note   简化设计：
  *         - 只需要robot_type和joint_indices
  *         - 具体的夹爪解算逻辑由解算器内部根据robot_type实现
  */
struct GripperSolverParams {
    std::string robot_type;           // 机器人类型（如"imeta", "unitree"等）
    std::vector<int> joint_indices;   // 关节索引列表
    
    GripperSolverParams() 
        : robot_type("") {}
};

/**
  * @brief  夹爪解算器状态枚举
  * @param  null
  * @retval null
  */
enum class GripperSolverStatus {
    UNINITIALIZED,  // 未初始化
    INITIALIZED,    // 已初始化
    ERROR          // 错误状态
};

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  夹爪解算器类（独立算法组件）
  * @param  null
  * @retval null
  * @note   设计理念：
  *         - 纯算法组件，不包含ROS相关功能
  *         - 使用Pimpl模式隐藏实现细节
  *         - 输入：关节值，输出：夹爪值
  *         - 根据机器人类型内部选择不同的解算实现
  */
class GripperSolver {
public:
    /**
      * @brief  构造函数
      * @param  null
      * @retval null
      * @usage
      */
    GripperSolver();

    /**
      * @brief  析构函数
      * @param  null
      * @retval null
      * @usage
      */
    ~GripperSolver();

    /**
      * @brief  初始化解算器
      * @param  params 解算器参数（robot_type和joint_count）
      * @retval true 初始化成功，false 初始化失败
      * @usage
      */
    bool initialize(const GripperSolverParams& params);

    /**
      * @brief  计算夹爪状态（从关节位置计算夹爪值）
      * @param  joint_positions 关节位置向量
      * @retval 夹爪状态结果
      * @usage
      */
    GripperResult compute(const Eigen::VectorXd& joint_positions);

    /**
      * @brief  反向计算（从期望的夹爪值计算关节位置）
      * @param  desired_value 期望的夹爪值
      * @retval 关节位置向量
      * @usage
      */
    Eigen::VectorXd computeInverse(double desired_value);

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
    GripperSolverStatus getStatus() const;

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

#endif // __GRIPPER_SOLVER_HPP__
