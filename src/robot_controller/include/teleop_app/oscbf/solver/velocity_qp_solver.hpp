#ifndef VELOCITY_QP_SOLVER_HPP
#define VELOCITY_QP_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <utility>

namespace OSCBF {

/**
 * @brief QP solver for velocity-based OSCBF optimization
 * 
 * Solves quadratic programming problem:
 *   minimize: (1/2)v^T P v + Q^T v
 *   subject to: Lg_h @ v >= cbf_rhs (for each constraint)
 *               v_min <= v <= v_max
 */
class VelocityQPSolver {
public:
    /**
     * @brief Constraint pair: (Lg_h, cbf_rhs)
     */
    using ConstraintPair = std::pair<Eigen::VectorXd, double>;

    /**
     * @brief Initialize velocity QP solver
     * 
     * @param n_joints Number of joints
     * @param velocity_limits Velocity limits (2x6 array: [min, max]) in rad/s
     * @param use_slack Whether to use slack variables for soft constraints
     * @param rho Penalty weight for slack variables
     * @param max_iter Maximum number of iterations
     * @param time_limit Maximum solve time in seconds
     * @param eps_abs Absolute tolerance
     * @param eps_rel Relative tolerance
     */
    VelocityQPSolver(
        int n_joints = 6,
        const Eigen::MatrixXd& velocity_limits = Eigen::MatrixXd(),
        bool use_slack = true,
        double rho = 500.0,
        int max_iter = 20000,
        double time_limit = 0.0,
        double eps_abs = 1e-5,
        double eps_rel = 1e-5
    );

    ~VelocityQPSolver();
    VelocityQPSolver(VelocityQPSolver&&) noexcept;
    VelocityQPSolver& operator=(VelocityQPSolver&&) noexcept;

    /**
     * @brief Solve velocity QP problem
     * 
     * @param P Quadratic cost matrix (n_joints x n_joints)
     * @param Q Linear cost vector (n_joints,)
     * @param cbf_constraints List of (Lg_h, cbf_rhs) tuples
     * @param use_slack_for Indices of constraints to use slack variables for
     * @return Optimal velocity vector or empty vector if infeasible
     */
    Eigen::VectorXd solve(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& Q,
        const std::vector<ConstraintPair>& cbf_constraints,
        const std::vector<int>& use_slack_for = {}
    );

    int getNJoints() const;
    Eigen::VectorXd getVelocityMin() const;
    Eigen::VectorXd getVelocityMax() const;

    /** 与 OSCBFController::setVelocityLimits 同步；OSQP 盒约束使用此处，而非仅控制器侧 clip */
    void setVelocityBounds(const Eigen::VectorXd& velocity_min, const Eigen::VectorXd& velocity_max);

private:
#ifdef USE_OSQP_EIGEN
    Eigen::VectorXd solve_with_osqp_eigen(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& Q,
        const std::vector<ConstraintPair>& cbf_constraints,
        const std::vector<int>& use_slack_for);
#endif

    Eigen::VectorXd solve_with_osqp(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& Q,
        const std::vector<ConstraintPair>& cbf_constraints,
        const std::vector<int>& use_slack_for
    );

    Eigen::VectorXd solve_with_gradient_projection(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& Q,
        const std::vector<ConstraintPair>& cbf_constraints,
        const std::vector<int>& use_slack_for
    );

    Eigen::VectorXd solve_qp_impl(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& Q,
        const std::vector<ConstraintPair>& cbf_constraints,
        const std::vector<int>& use_slack_for
    );

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace OSCBF

#endif // VELOCITY_QP_SOLVER_HPP
