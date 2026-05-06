#ifndef SELF_COLLISION_CONSTRAINT_HPP
#define SELF_COLLISION_CONSTRAINT_HPP

#include "base_constraint.hpp"
#include "teleop_app/oscbf/utils/collision_pair_manager.hpp"
#include <Eigen/Dense>
#include <string>
#include <memory>

#ifdef USE_PINOCCHIO
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#endif

namespace OSCBF {

/**
 * @brief Self-collision computation mode (deprecated, only ALL_PAIRS is used)
 */
enum class SelfCollisionMode {
    ALL_PAIRS             // 计算所有碰撞对的CBF
};

/**
 * @brief Self-collision avoidance constraint for velocity-based control
 * 
 * 自碰撞避免CBF（Self-collision avoidance）：
 * 确保机器人自身的不同连杆之间不会发生碰撞。
 * 使用 CollisionPairManager 管理碰撞对，使用 Pinocchio 计算雅可比。
 */
class SelfCollisionVelocityConstraint : public BaseVelocityCBFConstraint {
public:
    /**
     * @brief Initialize self-collision avoidance constraint
     * 
     * @param collision_manager CollisionPairManager 实例（必须已加载 URDF 和设置碰撞对）
     * @param name Constraint name
     * @param safety_distance Minimum safe distance between links (m)
     * @param alpha CBF coefficient
     * @param enable_debug Whether to print debug information
     */
    SelfCollisionVelocityConstraint(
        std::shared_ptr<CollisionPairManager> collision_manager,
        const std::string& name = "self_collision",
        double safety_distance = 0.05,
        double alpha = 50.0,
        bool enable_debug = false
    );

    /**
     * @brief Compute self-collision avoidance CBF for velocity control
     * 
     * 速度控制下，相对度为1的一阶CBF（根据论文公式 ż = u）：
     * - 系统动力学：ż = u，其中 z = q（关节位置），u = q̇（关节速度命令）
     * - f(z) = 0（没有漂移项），g(z) = I（单位矩阵）
     * 
     * CBF定义：
     * - h = distance - safety_distance（两个连杆之间的最小距离）
     * - ḣ = direction_norm^T · (vel_link1 - vel_link2)
     *      = direction_norm^T · (J1 - J2) · v
     * 
     * Lie导数：
     * - Lf_h = ∂h/∂q · f(z) = 0（因为两个连杆都是机器人自身，没有外部速度）
     * - Lg_h = ∂h/∂q · g(z) = direction_norm^T · (J1 - J2)（控制输入梯度）
     * 
     * 约束：Lg_h @ v >= -Lf_h - αh
     * 即：direction_norm^T · (J1 - J2) · v >= -αh
     * 
     * @param q Joint positions
     * @param v Joint velocities（仅用于调试，不用于计算Lf_h）
     * @return CBFResult containing (h, Lg_h, cbf_rhs)
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v
    );

    // Override pure virtual function (not used, but required)
    CBFResult compute_cbf() override {
        throw std::runtime_error("SelfCollisionVelocityConstraint::compute_cbf() requires parameters");
    }

    /**
     * @brief Compute all self-collision CBF constraints for all collision pairs
     * 
     * @param q Joint positions
     * @param v Joint velocities（仅用于调试，不用于计算Lf_h）
     * @param distance_threshold Optional: only compute CBFs for pairs with distance < threshold (default: infinity = all pairs)
     * @return Vector of ConstraintInfo containing CBF results for each collision pair
     */
    struct ConstraintInfo {
        CBFResult result;
        std::string link1_name;
        std::string link2_name;
        double distance;
        size_t pair_idx;
    };
    std::vector<ConstraintInfo> compute_all_self_collision_cbfs(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        double distance_threshold = std::numeric_limits<double>::infinity()
    );

    // Getters
    double getSafetyDistance() const { return safety_distance_; }
    bool isDebugEnabled() const { return enable_debug_; }

private:
    std::shared_ptr<CollisionPairManager> collision_manager_;
    double safety_distance_;
    bool enable_debug_;

    /**
     * @brief Compute Jacobians for two links with robust error handling
     * 
     * @param q Joint positions
     * @param geom1_idx Geometry object 1 index (from CollisionPairInfo)
     * @param geom2_idx Geometry object 2 index (from CollisionPairInfo)
     * @return Pair of (J_link1, J_link2), each is 6xN (position + orientation)
     */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_link_jacobians(
        const Eigen::VectorXd& q,
        size_t geom1_idx,
        size_t geom2_idx
    );

    /**
     * @brief Compute CBF for a single collision pair (helper method)
     * 
     * @param q Joint positions
     * @param v Joint velocities
     * @param pair_info Collision pair information
     * @return CBFResult for the specified collision pair
     */
    CBFResult compute_single_pair_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const CollisionPairManager::CollisionPairInfo& pair_info
    );
};

} // namespace OSCBF

#endif // SELF_COLLISION_CONSTRAINT_HPP

