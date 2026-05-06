#ifndef OBSTACLE_AVOIDANCE_TORQUE_CONSTRAINT_HPP
#define OBSTACLE_AVOIDANCE_TORQUE_CONSTRAINT_HPP

#include "teleop_app/oscbf/torque_cbf/constraint/base_constraint.hpp"
#include <Eigen/Dense>

namespace OSCBF {
namespace torque_cbf {

// Forward declaration to avoid circular dependency
enum class ObstacleType;

/**
 * @brief Obstacle avoidance constraint using torque-based CBF
 * 
 * 基于力矩的障碍物避障约束，使用二阶CBF：
 * - h = distance - safety_distance - (obstacle_radius + ee_radius)
 * - h2 = Lf_h + alpha * h
 * - Lg_h2 = direction_norm^T @ J_ee @ M^{-1}
 * - Lf_h2 = direction_norm^T @ J_ee @ M^{-1} @ (c + G) + alpha * Lf_h
 * - 约束：Lg_h2 @ τ >= -alpha2 * h2 - Lf_h2
 */
class ObstacleAvoidanceTorqueConstraint : public BaseTorqueCBFConstraint {
public:
    /**
     * @brief Initialize obstacle avoidance torque constraint
     * 
     * @param name Constraint name
     * @param safety_distance Minimum safe distance from obstacle (m)
     * @param obstacle_radius Obstacle radius (m)
     * @param ee_radius End-effector radius (m)
     * @param alpha First-order CBF coefficient
     * @param alpha2 Second-order CBF coefficient
     * @param enable_debug Whether to print debug information
     */
    ObstacleAvoidanceTorqueConstraint(
        const std::string& name,
        double safety_distance = 0.05,
        double obstacle_radius = 0.06,
        double ee_radius = 0.03,
        double alpha = 50.0,
        double alpha2 = 50.0,
        bool enable_debug = false
    );

    /**
     * @brief Compute CBF (implements base class pure virtual function)
     * 
     * Note: This method requires additional information (pos_ee, pos_obstacle, vel_ee, vel_obstacle, J_ee)
     * which must be computed externally. This is a stub implementation that throws an error.
     * Use compute_cbf(pos_ee, pos_obstacle, vel_ee, vel_obstacle, J_ee, q, v, a, M, c, G) instead.
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @return CBFResult (throws if called directly)
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    ) override;

    /**
     * @brief Compute obstacle avoidance CBF with complete Lf_h2 calculation
     * 
     * @param pos_ee End-effector position (3D)
     * @param pos_obstacle Obstacle position (3D) - 对于SPHERE是中心，对于PLANE是平面上一点
     * @param vel_ee End-effector velocity (3D)
     * @param vel_obstacle Obstacle velocity (3D)
     * @param J_ee End-effector Jacobian (3x6)
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @param c Coriolis and centrifugal force vector (n_joints,)
     * @param G Gravity vector (n_joints,)
     * @param obstacle_type Obstacle type (SPHERE or PLANE)
     * @param plane_normal Plane normal vector (仅用于PLANE类型，需要归一化)
     * @param plane_d Plane equation parameter d (仅用于PLANE类型)
     * @return CBFResult
     */
    CBFResult compute_cbf(
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& pos_obstacle,
        const Eigen::Vector3d& vel_ee,
        const Eigen::Vector3d& vel_obstacle,
        const Eigen::MatrixXd& J_ee,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M,
        const Eigen::VectorXd& c,
        const Eigen::VectorXd& G,
        ObstacleType obstacle_type,
        const Eigen::Vector3d& plane_normal,
        double plane_d
    );

    // Getters
    double getSafetyDistance() const { return safety_distance_; }
    double getObstacleRadius() const { return obstacle_radius_; }
    double getEERadius() const { return ee_radius_; }

private:
    double safety_distance_;
    double obstacle_radius_;
    double ee_radius_;
    bool enable_debug_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // OBSTACLE_AVOIDANCE_TORQUE_CONSTRAINT_HPP
