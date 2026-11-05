#pragma once

#include "kstone/control/constraint.hpp"
#include "kstone/control/task.hpp"
#include "kstone/qp/backend.hpp"
#include "kstone/qp/daqp.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace kstone {

/// How task priority levels are turned into QP stages.
enum class SolveMode {
    weighted,  ///< One QP; levels are ignored, all tasks are summed.
    /// One prioritized QP, each level a strict priority below the constraints: use it when lower levels
    /// must not disturb higher ones. Costs more per cycle than the other modes.
    lexicographic,
    /// One weighted QP per level, level 0 first, each moving only its own tasks; later levels compensate the earlier ones.
    /// The drift earlier stages cause in a later task is cancelled in full (first order), only its error at q is scaled
    /// by gain, and LM damping follows that error, not the drift; the task must see those joints (mask columns off).
    /// Earlier levels do not yield to later ones.
    /// A hard row that joints of a later level also move limits an earlier stage only by what those
    /// joints cannot yield within their box, so e.g. an arm avoids an obstacle without slowing the waist.
    /// This applies to constraints whose defers_to_later_stages() is true (collision, CoM); other rows bind each stage.
    layered_independent,
    /// One weighted QP per level k with every task of level <= k, stages chained; earlier levels keep their
    /// step and only take up what their own stage left. Not a strict priority: conflicts are traded off.
    layered_coupled,
};

KSTONE_EXPORT const char* to_string(SolveMode mode) noexcept;

struct TaskOptions {
    /// Lower is more important (>= 0). Unset: joins only the last stage and frees no joints beside leveled tasks unless masked.
    std::optional<int> level;
    bool enabled{true};
    /// A locked task contributes no rows and holds the joints of its (non-empty) mask still.
    bool locked{false};
};

using BackendFactory = std::function<std::unique_ptr<qp::Backend>()>;

struct SolverOptions {
    SolveMode mode{SolveMode::weighted};
    double damping{1e-3};    ///< Added to diag(H) of every weighted stage, > 0 keeps H definite.
    double limit_gain{0.5};  ///< Fraction of the distance to a position limit usable per step, (0, 1].
    /// Size nv, or empty. Non-finite entries keep the model velocity limit.
    Eigen::VectorXd velocity_limit;
    /// Size nq, or empty. Only scalar joints (nq == 1) use them; finite entries tighten the model.
    Eigen::VectorXd lower_position_limit, upper_position_limit;
    Mask locked;  ///< Size nv, or empty. Coordinates held at zero velocity.
    double feasibility_tol{1e-6};
    bool check_feasibility{true};
    /// Retry a failed stage with its recovery demands (inequality rows with lo > 0 or hi < 0) scaled by 0.5,
    /// then 0.25; if still infeasible, with each violated bound relaxed to zero: a violation may not grow but
    /// may still recover. Equality rows are not relaxed. If the backend still fails while x = 0 is admissible, the stage holds.
    bool hold_retry{true};
    qp::DaqpSettings daqp;
    BackendFactory backend;  ///< Empty uses DaqpBackend(daqp).
};

struct StageReport {
    int level{0};  ///< Highest task level in the stage (-1 for an empty stage).
    qp::SolveStatus status{qp::SolveStatus::failed};
    int exitflag{0};
    int iterations{0};
    bool held{false};     ///< Solved only after the hold retry (scaled recovery or hold).
    double solve_s{0.0};  ///< Wall time of build + QP for this stage, retries included.
};

struct TaskResidual {
    std::string name;
    int level{0};          ///< -1 for an unleveled task.
    double residual{0.0};  ///< |W (J x + gain e)| at the start configuration.
};

/// Summary of a successful solve; hard failures are Status errors instead.
enum class SolveQuality {
    nominal,   ///< Every stage optimal without the hold retry.
    degraded,  ///< Some stage was held or only inexact, or the step was clipped.
};

KSTONE_EXPORT const char* to_string(SolveQuality quality) noexcept;

struct SolveResult {
    Eigen::VectorXd v;       ///< Joint velocity, size nv.
    Eigen::VectorXd q_next;  ///< Configuration after dt, size nq.
    std::vector<StageReport> stages;
    std::vector<TaskResidual> residuals;
    bool held{false};
    double step_scale{1.0};  ///< Fraction of the solved step kept by the endpoint clip.
    SolveQuality quality{SolveQuality::nominal};
};

/// Degraded when `result` or any stage is held, any stage status is not optimal, or the step was clipped.
KSTONE_EXPORT SolveQuality solve_quality(const SolveResult& result) noexcept;

/// Velocity-level differential IK. Not thread-safe; use one solver per thread.
class KSTONE_EXPORT Solver {
  public:
    static Result<Solver> create(ModelPtr model, SolverOptions options = {});

    ~Solver();
    Solver(Solver&&) noexcept;
    Solver& operator=(Solver&&) noexcept;
    Solver(const Solver&) = delete;
    Solver& operator=(const Solver&) = delete;

    const ModelPtr& model() const noexcept;

    /// Binds the task to the model. Names are unique.
    Status add_task(std::string name, std::shared_ptr<Task> task, TaskOptions options = {});
    Status remove_task(const std::string& name);
    Status set_task_options(const std::string& name, TaskOptions options);
    /// Null if not found.
    std::shared_ptr<Task> task(const std::string& name) const;

    Status add_constraint(std::string name, std::shared_ptr<Constraint> constraint);
    Status remove_constraint(const std::string& name);
    /// A disabled constraint contributes no rows and no bound tightening.
    Status set_constraint_enabled(const std::string& name, bool enabled);
    std::shared_ptr<Constraint> constraint(const std::string& name) const;

    const SolverOptions& options() const noexcept;
    Status set_options(SolverOptions options);
    /// Incremented by each successful set_options().
    std::uint64_t options_revision() const noexcept;

    /// One control step from q over dt > 0. On error v is zero and q_next == q.
    /// invalid_argument: bad q / dt; infeasible: hard limits admit no step; internal: QP failure.
    Status solve(const Eigen::Ref<const Eigen::VectorXd>& q, double dt, SolveResult& out);
    /// As above; closed-loop constraints are also evaluated at `feedback` (size nq, e.g. the measured state)
    /// advanced by the step so far. nullptr = all at q.
    Status solve(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::VectorXd* feedback, double dt,
                 SolveResult& out);

  private:
    struct Impl;
    explicit Solver(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
