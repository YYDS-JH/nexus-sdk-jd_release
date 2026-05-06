#ifndef SELF_COLLISION_TORQUE_CONSTRAINT_HPP
#define SELF_COLLISION_TORQUE_CONSTRAINT_HPP

#include "teleop_app/oscbf/torque_cbf/constraint/base_constraint.hpp"
#include "teleop_app/oscbf/utils/collision_pair_manager.hpp"
#include <memory>
#include <vector>
#include <string>

#ifdef USE_PINOCCHIO
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#endif

namespace OSCBF {
namespace torque_cbf {

/**
 * @brief Self-collision avoidance constraint using torque-based CBF
 * 
 * 基于力矩的自碰撞避障约束，使用二阶CBF：
 * - h = min_distance - safety_distance
 * - h2 = Lf_h + alpha * h
 * - Lg_h2 = direction_norm^T @ (J1 - J2) @ M^{-1}
 * - 约束：Lg_h2 @ τ >= -alpha2 * h2 - Lf_h2
 */
class SelfCollisionTorqueConstraint : public BaseTorqueCBFConstraint {
public:
    /**
     * @brief Initialize self-collision torque constraint
     * 
     * @param collision_manager Collision pair manager (shared ownership)
     * @param name Constraint name
     * @param safety_distance Minimum safe distance between links (m)
     * @param alpha First-order CBF coefficient
     * @param alpha2 Second-order CBF coefficient (torque control requires alpha2)
     * @param min_distance_epsilon Minimum distance threshold
     */
    SelfCollisionTorqueConstraint(
        std::shared_ptr<CollisionPairManager> collision_manager,
        const std::string& name = "self_collision",
        double safety_distance = 0.05,
        double alpha = 50.0,
        double alpha2 = 150.0,
        double min_distance_epsilon = 1e-6
    );

    /**
     * @brief Compute self-collision CBF for all collision pairs
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @return CBFResult for the minimum distance collision pair
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    ) override;

    /**
     * @brief Compute CBF for all collision pairs
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @return Vector of CBFResult for all collision pairs
     */
    std::vector<CBFResult> compute_all_self_collision_cbfs(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    );

    // Getters
    double getSafetyDistance() const { return safety_distance_; }
    std::shared_ptr<CollisionPairManager> getCollisionManager() const { return collision_manager_; }

private:
    /**
     * @brief Compute CBF for a single collision pair
     * 
     * @param pair_info Collision pair information
     * @param q Joint positions
     * @param v Joint velocities
     * @param a Joint accelerations
     * @param M Joint space inertia matrix
     * @return CBFResult for this collision pair
     */
    CBFResult compute_single_pair_cbf(
        const CollisionPairManager::CollisionPairInfo& pair_info,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    );

    /**
     * @brief Compute link Jacobians for two geometry objects
     * 
     * @param geom1_idx Index of first geometry object
     * @param geom2_idx Index of second geometry object
     * @param q Joint positions
     * @return Pair of (J_link1, J_link2) Jacobians (6x3 each)
     */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_link_jacobians(
        size_t geom1_idx,
        size_t geom2_idx,
        const Eigen::VectorXd& q
    );

    std::shared_ptr<CollisionPairManager> collision_manager_;
    double safety_distance_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // SELF_COLLISION_TORQUE_CONSTRAINT_HPP

