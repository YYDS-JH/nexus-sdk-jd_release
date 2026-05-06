/*
 * @FilePath: nexus_manage/include/nexus_manage/components/rotation_orthogonalizer.hpp
 * @Description: 使用 SVD 对旋转矩阵进行正交化，确保满足 R^T*R=I 且 det(R)=1
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

#ifndef __ROTATION_ORTHOGONALIZER_HPP__
#define __ROTATION_ORTHOGONALIZER_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <Eigen/Dense>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace nexus_manage {
namespace components {

/**
  * @brief  使用 SVD 将 3x3 矩阵正交化为旋转矩阵（R^T*R=I, det(R)=1）
  * @param  R  输入矩阵，可能因数值误差而不严格正交
  * @retval 正交化后的旋转矩阵
  * @note   M = U*S*V^T，取 R' = U*V^T；若 det(R')=-1 则修正为 U*diag(1,1,-1)*V^T
  */
Eigen::Matrix3d orthogonalizeRotationMatrix(const Eigen::Matrix3d& R);

/**
  * @brief  检查 3x3 矩阵是否已近似正交（可选，用于跳过不必要的 SVD）
  * @param  R        待检查矩阵
  * @param  tolerance 容差，默认 1e-6
  * @retval true 若 ||R^T*R - I||_F < tolerance
  */
bool isRotationMatrixOrthogonal(const Eigen::Matrix3d& R, double tolerance = 1e-6);

}  // namespace components
}  // namespace nexus_manage

#endif
