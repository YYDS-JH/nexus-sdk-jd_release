/*
 * @FilePath: nexus_manage/include/nexus_manage/components/rotation_mapping.hpp
 * @Description: 主臂相对旋转到从臂旋转的非线性映射
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

#ifndef __ROTATION_MAPPING_HPP__
#define __ROTATION_MAPPING_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <Eigen/Dense>
#include <Eigen/Geometry>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace nexus_manage {
namespace components {

/**
  * @brief  主臂相对旋转 → 从臂旋转（内部非线性映射 + 缩放）
  * @param  R_delta            主臂相对旋转矩阵
  * @param  rot_scaling_factor 旋转 scaling 因子
  * @retval 映射后的旋转矩阵
  * @usage  仅 Scaling 模式下使用
  */
Eigen::Matrix3d mapRotationMasterToSlave(
    const Eigen::Matrix3d& R_delta,
    double rot_scaling_factor);

/**
  * @brief  限制旋转矩阵的 RPY 角度在指定范围内
  * @param  R           输入旋转矩阵 (3x3)
  * @param  rpy_limits   6维向量 [r_min, r_max, p_min, p_max, y_min, y_max] (rad)
  * @retval 限制后的旋转矩阵
  */
Eigen::Matrix3d clampRotationByRpyLimits(
    const Eigen::Matrix3d& R,
    const Eigen::VectorXd& rpy_limits);

}  // namespace components
}  // namespace nexus_manage

#endif
