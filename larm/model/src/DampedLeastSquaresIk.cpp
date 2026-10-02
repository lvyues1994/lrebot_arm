#include "DampedLeastSquaresIk.h"

#include <algorithm>
#include <random>

namespace larm::model {
namespace {

constexpr double kDamping = 1e-2;
constexpr double kMaxStep = 0.2;

struct DampedLeastSquaresIk final : IkSolver {
    DampedLeastSquaresIk(std::unique_ptr<Kinematics> kinematics_, IkLimits const &limits_)
        : kinematics{std::move(kinematics_)}, limits{limits_} {}

    IkSolution solve(IkRequest const &request, JointVector const &seed) override {
        auto best = attempt(request, clamp(seed));
        auto random = std::mt19937_64{0x6c61726dULL};
        for (int restart = 0; restart < request.restarts and not best.converged; ++restart) {
            auto const candidate = attempt(request, randomSeed(request, seed, random));
            if (candidate.converged or score(candidate) < score(best)) {
                best = candidate;
            }
        }
        return best;
    }

  private:
    static double score(IkSolution const &solution) {
        return solution.positionError + solution.orientationError;
    }

    JointVector clamp(JointVector const &q) const { return q.cwiseMax(limits.lower).cwiseMin(limits.upper); }

    JointVector randomSeed(IkRequest const &request, JointVector const &seed, std::mt19937_64 &random) const {
        auto uniform = std::uniform_real_distribution<double>{0.0, 1.0};
        auto q = JointVector{seed};
        for (std::size_t i = 0; i < dofOf(seed); ++i) {
            if (request.joints.test(i)) {
                q[idx(i)] =
                    limits.lower[idx(i)] + uniform(random) * (limits.upper[idx(i)] - limits.lower[idx(i)]);
            }
        }
        return q;
    }

    IkSolution attempt(IkRequest const &request, JointVector q) {
        auto const dof = dofOf(q);
        auto jacobian = Jacobian{6, idx(dof)};
        auto solution = IkSolution{};
        for (int iteration = 0; iteration <= request.maxIterations; ++iteration) {
            kinematics->update(q);
            auto const error = poseError(kinematics->framePose(request.frame), request.target);
            solution.position = q;
            solution.positionError = error.head<3>().norm();
            solution.orientationError = error.tail<3>().norm();
            solution.converged = solution.positionError <= request.positionTolerance and
                                 solution.orientationError <= request.orientationTolerance;
            if (solution.converged or iteration == request.maxIterations) {
                break;
            }
            kinematics->frameJacobian(request.frame, jacobian);
            for (std::size_t i = 0; i < dof; ++i) {
                if (not request.joints.test(i)) {
                    jacobian.col(idx(i)).setZero();
                }
            }
            Eigen::Matrix<double, 6, 6> const damped =
                jacobian * jacobian.transpose() +
                kDamping * kDamping * Eigen::Matrix<double, 6, 6>::Identity();
            auto step = JointVector{JointVector::Zero(idx(dof))};
            step.noalias() = jacobian.transpose() * damped.ldlt().solve(error);
            auto const largest = step.cwiseAbs().maxCoeff();
            if (largest > kMaxStep) {
                step *= kMaxStep / largest;
            }
            q = clamp(q + step);
        }
        return solution;
    }

    std::unique_ptr<Kinematics> kinematics;
    IkLimits limits;
};

} // namespace

std::unique_ptr<IkSolver> makeDampedLeastSquaresIk(std::unique_ptr<Kinematics> kinematics,
                                                   IkLimits const &limits) {
    return std::make_unique<DampedLeastSquaresIk>(std::move(kinematics), limits);
}

} // namespace larm::model
