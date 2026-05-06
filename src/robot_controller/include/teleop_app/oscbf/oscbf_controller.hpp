#ifndef OSCBF_CONTROLLER_HPP
#define OSCBF_CONTROLLER_HPP

#include "constraint/base_constraint.hpp"
#include "constraint/obstacle_avoidance_constraint.hpp"
#include "constraint/task_space_constraint.hpp"
#include "constraint/joint_limit_constraint.hpp"
#include "constraint/self_collision_constraint.hpp"
#include "solver/velocity_qp_solver.hpp"
#include "utils/collision_pair_manager.hpp"
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace OSCBF {

/**
 * @brief Obstacle type enumeration
 */
enum class ObstacleType {
    SPHERE = 0,      // 球形障碍物
    PLANE = 1        // 平面障碍物
};

/**
 * @brief Main OSCBF controller class that integrates all constraints and solves QP
 * 
 * 速度控制下的 OSCBF（支持障碍物避障、任务空间限制、关节限位、自碰撞避障）：
 * - 控制输入：速度命令 v (rad/s)
 * - 名义控制：v_nom = J^+ @ (v_cart_ref + Kp @ pos_error + Kd @ vel_error)
 * - CBF 约束：Lg_h @ v >= cbf_rhs
 * - QP 优化：minimize (1/2)v^T P v + Q^T v subject to CBF constraints
 */
class OSCBFController {
public:
    /**
     * @brief Controller configuration structure
     */
    struct Config {
        int n_joints = 6;                    // Number of joints
        bool enable_task_space = true;       // Enable task space constraint
        bool enable_joint_limit = true;      // Enable joint limit constraint
        bool enable_self_collision = false;  // Enable self-collision constraint (requires Pinocchio)
        bool enable_velocity_limit = true;   // Enable velocity limit clipping (simple, no QP needed)
        double task_space_threshold = 0.1;   // Threshold for activating task space constraints
        double self_collision_threshold = 0.1; // Threshold for activating self-collision constraints
    };

    /**
     * @brief Obstacle configuration structure for dynamic obstacle creation
     * 
     * 现在包含障碍物的位置、速度、类型和所有参数，统一管理
     */
    struct ObstacleConfig {
        // 障碍物类型
        ObstacleType type = ObstacleType::SPHERE;
        
        // 位置和速度（融合进来）
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        
        // 通用参数
        double safety_distance = 0.1;
        double alpha = 10.0;
        
        // 球形障碍物参数（仅用于SPHERE类型）
        double obstacle_radius = 0.03;
        double ee_radius = 0.03;  // 整臂避障时从URDF读取，这里作为默认值
        
        // 平面障碍物参数（仅用于PLANE类型）
        // 平面方程：n^T * x + d = 0
        Eigen::Vector3d plane_normal = Eigen::Vector3d(0, 0, 1);  // 法向量（需要归一化）
        double plane_d = 0.0;  // 平面方程中的 d
    };

    /**
     * @brief Initialize OSCBF controller (with default obstacle config)
     * 
     * @param config Controller configuration
     * @param task_space_constraint Task space constraint (optional)
     * @param joint_limit_constraint Joint limit constraint (optional)
     * @param self_collision_constraint Self-collision constraint (optional)
     */
    OSCBFController(
        const Config& config,
        std::shared_ptr<TaskSpaceVelocityConstraint> task_space_constraint = nullptr,
        std::shared_ptr<JointLimitVelocityConstraint> joint_limit_constraint = nullptr,
        std::shared_ptr<SelfCollisionVelocityConstraint> self_collision_constraint = nullptr
    );

    /**
     * @brief Initialize OSCBF controller (with custom obstacle config)
     * 
     * @param config Controller configuration
     * @param default_obstacle_config Default obstacle configuration (for dynamic obstacle creation)
     * @param task_space_constraint Task space constraint (optional)
     * @param joint_limit_constraint Joint limit constraint (optional)
     * @param self_collision_constraint Self-collision constraint (optional)
     */
    OSCBFController(
        const Config& config,
        const ObstacleConfig& default_obstacle_config,
        std::shared_ptr<TaskSpaceVelocityConstraint> task_space_constraint,
        std::shared_ptr<JointLimitVelocityConstraint> joint_limit_constraint,
        std::shared_ptr<SelfCollisionVelocityConstraint> self_collision_constraint
    );

    ~OSCBFController();
    OSCBFController(OSCBFController&&) noexcept;
    OSCBFController& operator=(OSCBFController&&) noexcept;

    /**
     * @brief Compute safe velocity with CBF constraints applied to nominal velocity
     *
     * This method takes a nominal velocity command from an external controller
     * and optimizes it to satisfy all CBF safety constraints.
     *
     * @param nominal_velocity Nominal velocity from external controller (n_joints,)
     * @param q,v 与 pos_ee,vel_ee,J_ee 一致：冗余臂应传 q_target,v_target（参考轨迹）；任务空间/关节限位 CBF 作用于此位形。
     * @param pos_ee, vel_ee, J_ee 末端量与 q 对应（通常 FK/Jacobian(q)）。
     * @param obstacle_configs 动态障碍物列表（可空）。
     * @param q_geometry / v_geometry 可选：若均非空，则整臂避障与自碰撞的几何/雅可比用实测关节（q_current），与参考分离；仅开任务/关节限位时可不传。
     * @return Safe velocity vector satisfying all CBF constraints (n_joints,)
     */
    Eigen::VectorXd compute_safe_velocity(
        const Eigen::VectorXd& nominal_velocity,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& vel_ee,
        const Eigen::MatrixXd& J_ee,
        const std::vector<ObstacleConfig>& obstacle_configs = {},
        const Eigen::VectorXd* q_geometry = nullptr,
        const Eigen::VectorXd* v_geometry = nullptr
    );

    /**
     * @brief Compute nominal velocity command (optional utility method)
     *
     * This is a utility method for computing nominal velocities.
     * In typical usage, nominal velocities should come from an external controller.
     *
     * @param q Joint positions
     * @param v Joint velocities
     * @param pos_target Target position (3D)
     * @param ori_target Target orientation (3x3 rotation matrix)
     * @param pos_ee Current end-effector position (3D)
     * @param rot_ee Current end-effector rotation (3x3)
     * @param J_ee End-effector Jacobian (6x6)
     * @return Nominal velocity vector (n_joints,)
     */
    Eigen::VectorXd compute_nominal_velocity(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::Vector3d& pos_target,
        const Eigen::Matrix3d& ori_target,
        const Eigen::Vector3d& pos_ee,
        const Eigen::Matrix3d& rot_ee,
        const Eigen::MatrixXd& J_ee
    );

    /**
     * @brief Set joint velocity limits (simple clipping, no QP needed)
     * 
     * @param velocity_min Minimum joint velocities (n_joints,)
     * @param velocity_max Maximum joint velocities (n_joints,)
     */
    void setVelocityLimits(
        const Eigen::VectorXd& velocity_min,
        const Eigen::VectorXd& velocity_max
    );

    /**
     * @brief Set collision pair manager for arm-wide obstacle avoidance
     * 
     * @param collision_manager CollisionPairManager instance (must be loaded)
     */
    void setCollisionPairManager(std::shared_ptr<CollisionPairManager> collision_manager);

    Config getConfig() const;
    ObstacleConfig getDefaultObstacleConfig() const;
    std::shared_ptr<TaskSpaceVelocityConstraint> getTaskSpaceConstraint() const;
    std::shared_ptr<JointLimitVelocityConstraint> getJointLimitConstraint() const;
    std::shared_ptr<SelfCollisionVelocityConstraint> getSelfCollisionConstraint() const;
    Eigen::VectorXd getVelocityMin() const;
    Eigen::VectorXd getVelocityMax() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    static Eigen::VectorXd clipVelocity(const Impl& impl, const Eigen::VectorXd& velocity);
};

} // namespace OSCBF

#endif // OSCBF_CONTROLLER_HPP

