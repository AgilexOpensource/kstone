#pragma once

#include "kstone/control/task.hpp"

#include <Eigen/Core>

namespace kstone {

/// Hard linear rows on the joint velocity v: lo <= A v <= hi (lo == hi for equalities).
/// Binding follows the same rules as Task.
class KSTONE_EXPORT Constraint {
  public:
    virtual ~Constraint();
    Constraint(const Constraint&) = delete;
    Constraint& operator=(const Constraint&) = delete;

    Status bind(const ModelPtr& model);
    bool bound() const noexcept { return model_ != nullptr; }
    const ModelPtr& model() const noexcept { return model_; }

    Index rows() const noexcept { return rows_; }

    /// A is rows x nv and zero on entry; lo / hi start at -inf / +inf. Requires bind().
    Status evaluate(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                    Eigen::Ref<Eigen::VectorXd> hi);

    /// Optional box contribution: intersect [lb, ub] (velocity, size nv, contains 0) with
    /// per-coordinate limits. Called once per step at the start configuration.
    virtual void tighten_bounds(const Tick& tick, Eigen::Ref<Eigen::VectorXd> lb,
                                Eigen::Ref<Eigen::VectorXd> ub) const;

    /// True for per-step budgets (velocity caps): in layered solves the motion already taken by
    /// earlier stages is subtracted from the bounds. False for state constraints, which are simply
    /// re-evaluated at each stage's configuration.
    virtual bool shares_step_budget() const noexcept { return false; }

    /// layered_independent: true lets an earlier stage use what joints moved only by later stages can still
    /// yield on a row; those stages then restore it. Only for state rows re-evaluated per stage (collision, CoM).
    virtual bool defers_to_later_stages() const noexcept { return false; }

    /// Step refinement (cutting planes). begin_step() is called before the step's first evaluation
    /// with the commanded workspace, and again with `retry` before each re-solve. After a solve the
    /// solver calls add_cuts() with the commanded workspace at the step's end; a constraint that
    /// returns > 0 rows relinearized there makes the solver re-solve the step (a few times at most).
    virtual void begin_step(const Workspace& commanded, bool retry);
    virtual Index add_cuts(const Tick& candidate);
    /// True if add_cuts() may add rows this step (asked after the first solve; false skips the end check).
    virtual bool refines() const noexcept { return false; }

    /// Endpoint clip for steps the cut passes did not settle (held, budget spent, failed re-solve).
    /// checks_end() gets the step end and its displacement dq from the start (size nv); if any
    /// constraint returns true, the solver shortens the step until admits() accepts the end tick.
    virtual bool checks_end(const Eigen::VectorXd& q_end, const Eigen::VectorXd& dq) const;
    virtual bool admits(const Tick& end);
    /// After admits() rejected an end: estimated fraction of that step in [0, 1] still admitted (default halves it).
    virtual double admitted_fraction() const noexcept { return 0.5; }

    /// Also evaluated at the solver's feedback configuration (when given); both row sets must hold.
    void set_closed_loop(bool closed_loop) noexcept { closed_loop_ = closed_loop; }
    bool closed_loop() const noexcept { return closed_loop_; }

  protected:
    explicit Constraint(Index rows = 0) : rows_(rows) {}

    virtual Status on_bind(const Model& model) = 0;
    virtual Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                           Eigen::Ref<Eigen::VectorXd> hi) = 0;
    void resize_rows(Index rows) noexcept { rows_ = rows; }

  private:
    ModelPtr model_;
    Index rows_;
    bool closed_loop_{false};
};

}  // namespace kstone
