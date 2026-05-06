#ifndef BASE_CONSTRAINT_HPP
#define BASE_CONSTRAINT_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>

namespace OSCBF {

/**
 * @brief Base class for all velocity-based Control Barrier Function constraints.
 * 
 * 速度控制下的控制仿射系统：
 * 对于速度执行器，系统动力学为：
 *     q̇ = v
 *     v̇ = f(q,v) + g(q)u
 * 
 * 其中：
 * - q: 关节位置
 * - v: 关节速度
 * - u: 速度控制输入（速度命令）
 * - f(q,v): 漂移项（通常为0或由执行器动力学决定）
 * - g(q): 控制输入矩阵（通常为单位矩阵）
 * 
 * 对于理想速度执行器（直接速度控制）：
 *     q̇ = v
 *     v = u  (直接设置速度)
 * 
 * 因此，CBF 约束从力矩约束 Lg_h2 @ τ >= cbf_rhs
 * 变为速度约束 Lg_h2 @ v >= cbf_rhs
 */
class BaseVelocityCBFConstraint {
public:
    /**
     * @brief Structure to hold CBF computation results
     */
    struct CBFResult {
        double h;                          // CBF value (e.g., distance - safety_distance)
        Eigen::VectorXd Lg_h;             // Lie derivative of h w.r.t. velocity control input
        double cbf_rhs;                   // Right-hand side of CBF constraint (-Lf_h - αh)
    };

    /**
     * @brief Initialize base velocity CBF constraint
     * 
     * @param name Constraint name
     * @param alpha First-order CBF coefficient (for relative degree 1 CBF: ḣ + αh >= 0)
     * @param alpha2 Second-order CBF coefficient (deprecated for velocity control, kept for compatibility)
     * @param min_distance_epsilon Minimum distance threshold to avoid division by zero
     */
    BaseVelocityCBFConstraint(
        const std::string& name,
        double alpha = 50.0,
        double alpha2 = 0.0,
        double min_distance_epsilon = 1e-6,
        double interior_cbf_deactivate_margin = 0.0
    );

    virtual ~BaseVelocityCBFConstraint() = default;

    /**
     * @brief Compute CBF values and constraints for velocity control (pure virtual)
     * 
     * 速度控制下，所有约束都是相对度为1（relative degree 1），因为：
     * - 系统动力学：q̇ = v（控制输入直接出现在一阶导数中）
     * - 对于约束 h(q)，有：ḣ = ∂h/∂q * q̇ = ∂h/∂q * v
     * 
     * 因此，一阶CBF约束为：ḣ + αh >= 0
     * 即：Lg_h @ v >= -Lf_h - αh
     * 
     * @return CBFResult containing (h, Lg_h, cbf_rhs)
     *         - h: CBF value (e.g., distance - safety_distance)
     *         - Lg_h: Lie derivative of h w.r.t. velocity control input (1D array, shape: (n_joints,))
     *         - cbf_rhs: Right-hand side of CBF constraint (-Lf_h - αh)
     * 
     * Note:
     *     For velocity control, the constraint is: Lg_h @ v >= cbf_rhs
     *     This is a first-order CBF (relative degree 1), not second-order.
     */
    virtual CBFResult compute_cbf() = 0;

    /**
     * @brief Compute CBF constraint RHS for first-order CBF: -Lf_h - αh
     * 
     * 对于相对度为1的CBF，约束为：ḣ + αh >= 0
     * 展开：Lf_h + Lg_h @ v + αh >= 0
     * 因此：Lg_h @ v >= -Lf_h - αh
     * 
     * @param Lf_h Lie derivative of h along the drift term f
     * @param h CBF value
     * @return CBF constraint right-hand side: -Lf_h - αh
     */
    double compute_cbf_rhs(double Lf_h, double h) const;

    /**
     * @brief 供 QP 使用的 CBF 右端项：在「远离边界」内侧不收紧，避免 ḣ+αh 形式在 h 较大、α 较小时仍把 Lgᵀv≥-αh 变成对名义速度的强夹紧。
     *
     * 当 h≤0（已越界或贴边）时始终用完整 -Lf-αh 以恢复安全；当 h>interior_cbf_deactivate_margin_>0 时返回极小 RHS，使约束在物理速度限幅下自动满足。
     */
    double cbf_rhs_for_qp(double Lf_h, double h) const;

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
    double alpha_;
    double alpha2_;  // 保留但不使用
    double min_distance_epsilon_;
    /** 距 barrier 内侧超过该值则不在 QP 中收紧 CBF（0=关闭，始终用 -αh） */
    double interior_cbf_deactivate_margin_;
};

} // namespace OSCBF

#endif // BASE_CONSTRAINT_HPP

