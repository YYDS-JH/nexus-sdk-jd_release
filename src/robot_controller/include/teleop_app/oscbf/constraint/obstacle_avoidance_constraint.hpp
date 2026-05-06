#ifndef OBSTACLE_AVOIDANCE_CONSTRAINT_HPP
#define OBSTACLE_AVOIDANCE_CONSTRAINT_HPP

#include "base_constraint.hpp"
#include <Eigen/Dense>

namespace OSCBF {

// Forward declaration to avoid circular dependency
enum class ObstacleType;

/**
 * @brief Obstacle avoidance constraint for velocity-based control using CBF
 */
class ObstacleAvoidanceVelocityConstraint : public BaseVelocityCBFConstraint {
public:
    /**
     * @brief Initialize obstacle avoidance constraint for velocity control
     * 
     * @param name Constraint name
     * @param safety_distance Minimum safe distance from obstacle
     * @param obstacle_radius Radius of the obstacle
     * @param ee_radius Radius of the end-effector collision body (default: 0.0)
     * @param alpha First-order CBF coefficient
     * @param alpha2 Second-order CBF coefficient (deprecated)
     * @param enable_debug Whether to print debug information
     */
    ObstacleAvoidanceVelocityConstraint(
        const std::string& name = "obstacle_avoidance",
        double safety_distance = 0.05,
        double obstacle_radius = 0.06,
        double ee_radius = 0.0,
        double alpha = 50.0,
        double alpha2 = 50.0,
        bool enable_debug = false
    );

    /**
     * @brief Compute obstacle avoidance CBF for velocity control
     * 
     * 速度控制下，相对度为1的一阶CBF（根据论文公式 ż = u）：
     * - 系统动力学：ż = u，其中 z = q（关节位置），u = q̇（关节速度命令）
     * - f(z) = 0（没有漂移项），g(z) = I（单位矩阵）
     * 
     * CBF定义：
     * - h = distance - safety_distance - (obstacle_radius + ee_radius)  [对于SPHERE]
     * - h = distance - safety_distance - ee_radius  [对于PLANE]
     * - ḣ = direction_norm^T · (vel_ee - vel_obstacle)
     *      = direction_norm^T · J_ee · v - direction_norm^T · vel_obstacle
     * 
     * Lie导数：
     * - Lf_h = ∂h/∂q · f(z) = -direction_norm^T · vel_obstacle（障碍物速度导致的漂移）
     * - Lg_h = ∂h/∂q · g(z) = direction_norm^T · J_ee（控制输入梯度）
     * 
     * 约束：Lg_h @ v >= -Lf_h - αh
     * 即：direction_norm^T · J_ee · v >= direction_norm^T · vel_obstacle - αh
     * 
     * @param pos_ee End-effector position (3D)
     * @param pos_obstacle Obstacle position (3D) - 对于SPHERE是中心，对于PLANE是平面上一点
     * @param vel_ee End-effector velocity (3D) - 仅用于调试，不用于计算Lf_h
     * @param vel_obstacle Obstacle velocity (3D)
     * @param J_ee End-effector Jacobian (3x6)
     * @param obstacle_type Obstacle type (SPHERE or PLANE)
     * @param plane_normal Plane normal vector (仅用于PLANE类型，需要归一化)
     * @param plane_d Plane equation parameter d (仅用于PLANE类型)
     * @return CBFResult containing (h, Lg_h, cbf_rhs)
     */
    CBFResult compute_cbf(
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& pos_obstacle,
        const Eigen::Vector3d& vel_ee,
        const Eigen::Vector3d& vel_obstacle,
        const Eigen::MatrixXd& J_ee,
        ObstacleType obstacle_type,
        const Eigen::Vector3d& plane_normal,
        double plane_d
    );

    // Override pure virtual function (not used, but required)
    CBFResult compute_cbf() override {
        throw std::runtime_error("ObstacleAvoidanceVelocityConstraint::compute_cbf() requires parameters");
    }

    // Getters
    double getSafetyDistance() const { return safety_distance_; }
    double getObstacleRadius() const { return obstacle_radius_; }
    double getEeRadius() const { return ee_radius_; }
    bool isDebugEnabled() const { return enable_debug_; }

private:
    double safety_distance_;
    double obstacle_radius_;
    double ee_radius_;
    bool enable_debug_;
};

} // namespace OSCBF

#endif // OBSTACLE_AVOIDANCE_CONSTRAINT_HPP

