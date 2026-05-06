#ifndef TORQUE_QP_SOLVER_HPP
#define TORQUE_QP_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <utility>

namespace OSCBF {
namespace torque_cbf {

/**
 * @brief QP solver for torque-based OSCBF optimization
 * 
 * Solves quadratic programming problem:
 *   minimize: (1/2)τ^T P τ + Q^T τ
 *   subject to: Lg_h2 @ τ >= cbf_rhs (for each constraint)
 *               τ_min <= τ <= τ_max
 * 
 * 使用 OsqpEigen 进行高效求解（推荐）
 * 或回退到原始 OSQP C API
 */
class TorqueQPSolver {
public:
    /**
     * @brief Constraint pair: (Lg_h2, cbf_rhs)
     */
    using ConstraintPair = std::pair<Eigen::VectorXd, double>;

    /**
     * @brief Initialize torque QP solver
     * 
     * @param n_joints Number of joints
     * @param torque_limits Torque limits (2xn_joints array: [min, max]) in N·m
     * @param use_slack Whether to use slack variables for soft constraints
     * @param rho Penalty weight for slack variables
     * @param max_iter Maximum number of iterations
     * @param time_limit Maximum solve time in seconds
     * @param eps_abs Absolute tolerance
     * @param eps_rel Relative tolerance
     */
    TorqueQPSolver(
        int n_joints = 6,
        const Eigen::MatrixXd& torque_limits = Eigen::MatrixXd(),
        bool use_slack = true,
        double rho = 500.0,
        int max_iter = 10000,
        double time_limit = 0.002,
        double eps_abs = 1e-4,
        double eps_rel = 1e-4
    );

    ~TorqueQPSolver();
    TorqueQPSolver(TorqueQPSolver&&) noexcept;
    TorqueQPSolver& operator=(TorqueQPSolver&&) noexcept;

    /**
     * @brief Solve torque QP problem
     * 
     * @param P Quadratic cost matrix (n_joints x n_joints)
     * @param Q Linear cost vector (n_joints,)
     * @param cbf_constraints List of (Lg_h2, cbf_rhs) tuples
     * @param use_slack_for Indices of constraints to use slack variables for
     * @return Optimal torque vector or empty vector if infeasible
     */
    Eigen::VectorXd solve(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& Q,
        const std::vector<ConstraintPair>& cbf_constraints,
        const std::vector<int>& use_slack_for = {}
    );

    int getNJoints() const;
    Eigen::VectorXd getTorqueMin() const;
    Eigen::VectorXd getTorqueMax() const;

private:
#ifdef USE_OSQP_EIGEN
    Eigen::VectorXd solve_with_osqp_eigen(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& Q,
        const std::vector<ConstraintPair>& cbf_constraints,
        const std::vector<int>& use_slack_for
    );
#endif

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // TORQUE_QP_SOLVER_HPP
