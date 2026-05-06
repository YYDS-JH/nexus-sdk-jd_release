#ifndef OSCBF_TORQUE_CONTROLLER_HPP
#define OSCBF_TORQUE_CONTROLLER_HPP

#include "teleop_app/oscbf/torque_cbf/constraint/base_constraint.hpp"
#include "teleop_app/oscbf/torque_cbf/constraint/obstacle_avoidance_constraint.hpp"
#include "teleop_app/oscbf/torque_cbf/constraint/task_space_constraint.hpp"
#include "teleop_app/oscbf/torque_cbf/constraint/joint_limit_constraint.hpp"
#include "teleop_app/oscbf/torque_cbf/constraint/joint_velocity_constraint.hpp"
#include "teleop_app/oscbf/torque_cbf/constraint/self_collision_constraint.hpp"
#include "teleop_app/oscbf/torque_cbf/solver/torque_qp_solver.hpp"
#include "teleop_app/oscbf/utils/collision_pair_manager.hpp"
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace OSCBF {
namespace torque_cbf {

/**
 * @brief Obstacle type enumeration
 */
enum class ObstacleType {
    SPHERE = 0,      // 球形障碍物
    PLANE = 1        // 平面障碍物
};

/**
 * @brief Main OSCBF controller class for torque control
 * 
 * 力矩控制下的 OSCBF（支持障碍物避障、任务空间限制、关节限位、自碰撞避障）：
 * - 控制输入：力矩命令 τ (N·m)
 * - 名义控制：τ_nom = M @ a_desired + C @ v + G (from OSC)
 * - CBF 约束：Lg_h2 @ τ >= cbf_rhs
 * - QP 优化：minimize (1/2)τ^T P τ + Q^T τ subject to CBF constraints
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
        bool enable_joint_velocity = true;   // Enable joint velocity constraint
        bool enable_self_collision = false;  // Enable self-collision constraint (requires Pinocchio)
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
        double alpha2 = 50.0;
        
        // 球形障碍物参数（仅用于SPHERE类型）
        double obstacle_radius = 0.03;
        double ee_radius = 0.03;
        
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
     * @param joint_velocity_constraint Joint velocity constraint (optional)
     * @param self_collision_constraint Self-collision constraint (optional)
     */
    OSCBFController(
        const Config& config,
        std::shared_ptr<TaskSpaceTorqueConstraint> task_space_constraint = nullptr,
        std::shared_ptr<JointLimitTorqueConstraint> joint_limit_constraint = nullptr,
        std::shared_ptr<JointVelocityTorqueConstraint> joint_velocity_constraint = nullptr,
        std::shared_ptr<SelfCollisionTorqueConstraint> self_collision_constraint = nullptr
    );

    /**
     * @brief Initialize OSCBF controller (with custom obstacle config)
     * 
     * @param config Controller configuration
     * @param default_obstacle_config Default obstacle configuration (for dynamic obstacle creation)
     * @param task_space_constraint Task space constraint (optional)
     * @param joint_limit_constraint Joint limit constraint (optional)
     * @param joint_velocity_constraint Joint velocity constraint (optional)
     * @param self_collision_constraint Self-collision constraint (optional)
     */
    OSCBFController(
        const Config& config,
        const ObstacleConfig& default_obstacle_config,
        std::shared_ptr<TaskSpaceTorqueConstraint> task_space_constraint,
        std::shared_ptr<JointLimitTorqueConstraint> joint_limit_constraint,
        std::shared_ptr<JointVelocityTorqueConstraint> joint_velocity_constraint,
        std::shared_ptr<SelfCollisionTorqueConstraint> self_collision_constraint
    );

    ~OSCBFController();
    OSCBFController(OSCBFController&&) noexcept;
    OSCBFController& operator=(OSCBFController&&) noexcept;

    /**
     * @brief Compute safe torque that satisfies all CBF constraints
     * 
     * 加速度 a 从名义力矩计算：a = M^{-1} * (τ_nom - c - G)
     * 其中 c = C(q,v)*v 是科氏力和向心力的合力向量
     * 
     * @param nominal_torque Nominal torque from OSC (n_joints,)
     * @param q Joint positions (n_joints,) - state information
     * @param v Current joint velocities (n_joints,) - state information
     * @param pos_ee End-effector position (3D) - state information
     * @param vel_ee End-effector velocity (3D) - state information
     * @param J_ee End-effector Jacobian (3x6 or 6x6) - state information
     * @param M Joint space inertia matrix (n_joints x n_joints) - dynamics information
     * @param c Coriolis and centrifugal force vector (n_joints,) - dynamics information, c = C(q,v)*v
     * @param G Gravity vector (n_joints,) - dynamics information
     * @param pos_obstacles Obstacle positions (3D) - environment information (supports any number)
     * @param vel_obstacles Obstacle velocities (3D) - environment information (supports any number)
     * @param obstacle_configs Optional obstacle configurations (if empty, uses default config)
     * @return Safe torque vector satisfying all CBF constraints (n_joints,)
     */
    Eigen::VectorXd compute_safe_torque(
        const Eigen::VectorXd& nominal_torque,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& vel_ee,
        const Eigen::MatrixXd& J_ee_full,
        const Eigen::MatrixXd& M,
        const Eigen::VectorXd& c,
        const Eigen::VectorXd& G,
        const std::vector<ObstacleConfig>& obstacle_configs = {}
    );

    Config getConfig() const;
    std::shared_ptr<TorqueQPSolver> getQPSolver() const;

    /**
     * @brief Set torque limits
     * 
     * @param torque_min Minimum torque limits (n_joints,)
     * @param torque_max Maximum torque limits (n_joints,)
     */
    void setTorqueLimits(
        const Eigen::VectorXd& torque_min,
        const Eigen::VectorXd& torque_max
    );

    /**
     * @brief Set collision pair manager for arm-wide obstacle avoidance
     * 
     * @param collision_manager CollisionPairManager instance (must be loaded)
     */
    void setCollisionPairManager(std::shared_ptr<CollisionPairManager> collision_manager);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // OSCBF_TORQUE_CONTROLLER_HPP
