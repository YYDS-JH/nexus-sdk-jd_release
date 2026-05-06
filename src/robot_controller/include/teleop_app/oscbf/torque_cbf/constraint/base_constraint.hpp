#ifndef BASE_TORQUE_CONSTRAINT_HPP
#define BASE_TORQUE_CONSTRAINT_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>

namespace OSCBF {
namespace torque_cbf {

/**
 * @brief Base class for all torque-based Control Barrier Function constraints.
 * 
 * 力矩控制下的控制仿射系统：
 * 对于力矩执行器，系统动力学为：
 *     M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
 * 
 * 其中：
 * - q: 关节位置
 * - q̇: 关节速度
 * - q̈: 关节加速度
 * - M(q): 惯性矩阵
 * - C(q,q̇): 科里奥利矩阵
 * - G(q): 重力向量
 * - τ: 力矩控制输入
 * 
 * 对于二阶CBF（relative degree 2）：
 * - h: 一阶CBF值（如距离 - 安全距离）
 * - h2 = Lf_h + αh: 二阶CBF值
 * - 约束：ḣ2 + α2*h2 >= 0
 * - 即：Lg_h2 @ τ >= -Lf_h2 - α2*h2
 */
class BaseTorqueCBFConstraint {
public:
    /**
     * @brief Structure to hold CBF computation results
     */
    struct CBFResult {
        double h;                          // First-order CBF value (e.g., distance - safety_distance)
        double h2;                         // Second-order CBF value: h2 = Lf_h + alpha * h
        Eigen::VectorXd Lg_h2;            // Lie derivative of h2 w.r.t. torque control input
        double cbf_rhs;                    // Right-hand side of CBF constraint (-alpha2 * h2 - Lf_h2)
    };

    /**
     * @brief Initialize base torque CBF constraint
     * 
     * @param name Constraint name
     * @param alpha First-order CBF coefficient (for h2 = Lf_h + alpha * h)
     * @param alpha2 Second-order CBF coefficient (for ḣ2 + alpha2 * h2 >= 0)
     * @param min_distance_epsilon Minimum distance threshold to avoid division by zero
     */
    BaseTorqueCBFConstraint(
        const std::string& name,
        double alpha = 50.0,
        double alpha2 = 50.0,
        double min_distance_epsilon = 1e-6
    );

    virtual ~BaseTorqueCBFConstraint() = default;

    /**
     * @brief Compute CBF values and constraints for torque control (pure virtual)
     * 
     * 力矩控制下，所有约束都是相对度为2（relative degree 2），因为：
     * - 系统动力学：M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
     * - 对于约束 h(q)，需要两次求导才能得到控制输入 τ
     * 
     * 二阶CBF计算步骤：
     * 1. h = distance - safety_distance (一阶CBF)
     * 2. Lf_h = ∂h/∂q * q̇ (沿漂移项的Lie导数)
     * 3. h2 = Lf_h + alpha * h (二阶CBF)
     * 4. Lf_h2 = ∂h2/∂q * q̇ + ∂h2/∂q̇ * q̈ (沿漂移项的Lie导数)
     * 5. Lg_h2 = ∂h2/∂q * M^{-1} (沿控制输入的Lie导数)
     * 6. cbf_rhs = -alpha2 * h2 - Lf_h2
     * 
     * 约束：Lg_h2 @ τ >= cbf_rhs
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,) - used for computing Lf_h2
     * @param M Joint space inertia matrix (n_joints x n_joints) - used for computing Lg_h2
     * @return CBFResult containing (h, h2, Lg_h2, cbf_rhs)
     */
    virtual CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    ) = 0;

    /**
     * @brief Compute second-order CBF: h2 = Lf_h + alpha * h
     * 
     * @param Lf_h Lie derivative of h along the drift term f
     * @param h First-order CBF value
     * @return Second-order CBF value: h2 = Lf_h + alpha * h
     */
    double compute_h2(double Lf_h, double h) const;

    /**
     * @brief Compute CBF constraint RHS for second-order CBF: -alpha2 * h2 - Lf_h2
     * 
     * 对于相对度为2的CBF，约束为：ḣ2 + α2*h2 >= 0
     * 展开：Lf_h2 + Lg_h2 @ τ + α2*h2 >= 0
     * 因此：Lg_h2 @ τ >= -Lf_h2 - α2*h2
     * 
     * @param h2 Second-order CBF value
     * @param Lf_h2 Lie derivative of h2 along the drift term f
     * @return CBF constraint right-hand side: -alpha2 * h2 - Lf_h2
     */
    double compute_cbf_rhs(double h2, double Lf_h2 = 0.0) const;

    /**
     * @brief Normalize direction vector with robust handling of near-zero cases
     * 
     * @param direction Direction vector (3D)
     * @param default_direction Default direction if norm is too small (default: [1, 0, 0])
     * @return Pair of (normalized_direction, norm_value)
     */
    std::pair<Eigen::Vector3d, double> normalize_direction(
        const Eigen::Vector3d& direction,
        const Eigen::Vector3d& default_direction = Eigen::Vector3d(1.0, 0.0, 0.0)
    ) const;

    /**
     * @brief Ensure vector has correct shape (1D array of expected size)
     * 
     * @param vector Input vector (may be 1D or 2D)
     * @param expected_size Expected size of the vector
     * @return Flattened 1D array of correct size
     * @throws std::runtime_error If vector cannot be reshaped to expected size
     */
    Eigen::VectorXd ensure_vector_shape(
        const Eigen::VectorXd& vector,
        int expected_size
    ) const;

    // Getters
    std::string getName() const { return name_; }
    double getAlpha() const { return alpha_; }
    double getAlpha2() const { return alpha2_; }
    double getMinDistanceEpsilon() const { return min_distance_epsilon_; }

protected:
    std::string name_;
    double alpha_;      // First-order CBF coefficient
    double alpha2_;    // Second-order CBF coefficient
    double min_distance_epsilon_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // BASE_TORQUE_CONSTRAINT_HPP

