#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/core/mask.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

#include <vector>

namespace kstone {

/// Linearization point handed to tasks and constraints. The workspace is already updated at the
/// working configuration of the current stage.
struct Tick {
    Workspace& workspace;
    double dt;
};

/// A soft objective linear in the step x = v * dt (size nv):
///   minimize | W (J x + gain * e) |^2
/// where e is the task error and J = de/dq, i.e. each step removes the fraction `gain` of the error.
/// Subclasses fill J and e; the base class applies the column mask, checks finiteness and stores
/// the result for the solver.
///
/// A task binds to one model on first use. Binding again to the same model is a no-op; binding to
/// a different model fails.
class KSTONE_EXPORT Task {
  public:
    virtual ~Task();
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    Status bind(const ModelPtr& model);
    bool bound() const noexcept { return model_ != nullptr; }
    const ModelPtr& model() const noexcept { return model_; }

    Index rows() const noexcept { return jacobian_.rows(); }

    /// Recompute J and e at the tick. Requires bind().
    Status evaluate(const Tick& tick);
    const Eigen::MatrixXd& jacobian() const noexcept { return jacobian_; }
    const Eigen::VectorXd& error() const noexcept { return error_; }
    /// Per-row weights expanded to rows(); valid after bind().
    const Eigen::VectorXd& weights() const noexcept { return weights_; }

    /// Scalar weight for every row, or one weight per row. Weights must be finite and >= 0.
    Status set_weight(double weight);
    Status set_weight(const Eigen::Ref<const Eigen::VectorXd>& weights);
    /// Fraction of the error removed per step, in [0, 1].
    Status set_gain(double gain);
    double gain() const noexcept { return gain_; }
    /// Levenberg-Marquardt damping, >= 0: adds lm * |W (gain W e)| to the diagonal of the Hessian.
    /// Linear in the error, so it vanishes as a reachable target converges.
    Status set_lm_damping(double lm);
    double lm_damping() const noexcept { return lm_; }

    /// Velocity coordinates this task may move (size nv once bound). Empty means all.
    /// The solver frees only the union of task masks; columns outside the mask are zeroed unless
    /// set_mask_columns(false), which keeps the full Jacobian so other free coordinates can help.
    Status set_mask(Mask mask);
    const Mask& mask() const noexcept { return mask_; }
    void set_mask_columns(bool zero) noexcept { mask_columns_ = zero; }
    bool mask_columns() const noexcept { return mask_columns_; }

  protected:
    explicit Task(Index rows = 0, double gain = 1.0);

    /// Validate against the model and resolve names. Called once, with model() already set
    /// (so dependent tasks can be bound to it); a failure unbinds.
    virtual Status on_bind(const Model& model) = 0;
    /// Fill J (rows x nv, zero on entry) and e (rows, zero on entry).
    virtual Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J,
                           Eigen::Ref<Eigen::VectorXd> e) = 0;
    /// Change the number of rows (e.g. external tasks). Re-expands a scalar weight.
    Status resize_rows(Index rows);

  private:
    Status expand_weights();

    ModelPtr model_;
    Eigen::MatrixXd jacobian_;
    Eigen::VectorXd error_;
    Eigen::VectorXd weights_;
    Eigen::VectorXd row_weights_;  ///< Per-row weights as set; empty means scalar_weight_.
    double scalar_weight_{1.0};
    double gain_;
    double lm_{0.0};
    Mask mask_;
    bool mask_columns_{true};
};

/// Null-space projector of the weighted primary task restricted to `columns` (velocity indices):
/// P = I - V diag(s^2 / (s^2 + lambda^2)) V^T from the SVD of (W J)[:, columns].
/// With threshold > 0 the damping becomes adaptive: lambda = damping * sqrt(1 - (s/threshold)^2)
/// for s < threshold, 0 otherwise. The primary must have been evaluated at the current tick.
KSTONE_EXPORT Status null_space_projector(const Task& primary, const std::vector<Index>& columns,
                                          double damping, double threshold, Eigen::MatrixXd& out);

/// Buffers reused by null_space_projector; no allocation once sizes repeat.
struct NullSpaceScratch {
    Eigen::MatrixXd weighted;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    std::vector<char> seen;
};

KSTONE_EXPORT Status null_space_projector(const Task& primary, const std::vector<Index>& columns,
                                          double damping, double threshold, Eigen::MatrixXd& out,
                                          NullSpaceScratch& scratch);

}  // namespace kstone
