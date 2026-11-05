#pragma once

#include "kstone/collision/query.hpp"
#include "kstone/control/constraint.hpp"

#include <limits>
#include <memory>
#include <vector>

namespace kstone {

struct CollisionConstraintOptions {
    double d_min{0.03};              ///< Distance kept between pairs (m), >= 0.
    double gain{50.0};               ///< Allowed approach speed per metre above d_min (1/s), >= 0.
    double activation_margin{0.05};  ///< Rows appear within d_min + margin (m), >= 0.
    double hysteresis{0.005};        ///< An active pair stays active up to this much further (m), >= 0.
    /// Braking deceleration (m/s^2); > 0 lets a pair approach at up to sqrt(2 a (d - d_min)),
    /// capped at half the remaining gap per step. 0 disables it.
    double approach_deceleration{5.0};
    Index max_rows{16};  ///< Fixed row count; nearest pairs win, > 0.
    /// Extra rows for cutting planes, >= 0 (0 disables them). A step whose end leaves a pair below its
    /// floor (d_min, or the recovery share of a pair that started inside d_min) is re-solved with that
    /// pair relinearized at the rejected end: the start-linearized row lets curved (sliding, rotating)
    /// steps dip below d_min.
    Index max_cuts{8};
    /// Endpoint clip floor below d_min (m), >= 0: a step end is shortened if some pair would end below
    /// min(d_min, its start distance) - clip_slack. A slack lets small steps skip the check and can deepen a penetration.
    double clip_slack{0.0};
    /// A pair already penetrating when it enters the activation band (first step, obstacle enabled, constraint
    /// re-enabled) gets no row, cut or clip floor until it is back beyond d_min: it can be moved out, or deeper.
    /// false: it is pushed out like any pair inside d_min, which a deep start may not allow (held or frozen steps).
    bool exempt_penetrating{true};
};

/// Keeps every selected pair at least d_min apart: per active pair one row
/// d_dot = J v >= -rate, rate = gain (d - d_min). Unused rows are unbounded.
/// A pair is active within d_min + activation_margin (+ hysteresis while it stays active).
/// Endpoint clip: if the cut passes did not verify the step end (held step, cut budget spent, failed
/// re-solve) and a rigid-motion bound of the step could bring some pair below its clip floor
/// (min(d_min, its start distance) - clip_slack), the solver shortens the step until none does.
/// An obstacle's approaching velocity (CollisionWorld::set_velocity) raises its rows by the closing speed
/// at the witness point; cuts and the clip take obstacles as static within a step.
class KSTONE_EXPORT CollisionConstraint final : public Constraint {
  public:
    static Result<std::shared_ptr<CollisionConstraint>> create(
        CollisionModelPtr model, std::shared_ptr<const CollisionWorld> world = nullptr,
        const CollisionConstraintOptions& options = {});

    struct Report {
        Index candidates{0};  ///< Pairs inside the activation band.
        Index rows{0};        ///< Rows used (<= max_rows).
        Index dropped{0};     ///< Candidates beyond max_rows (farthest dropped first).
        Index cuts{0};        ///< Cutting planes in use this step.
        Index exempt{0};      ///< Pairs penetrating since they entered the band, left without a row.
        Proximity nearest;    ///< Nearest pair over all pairs.
    };
    /// From the latest evaluation (layered solves evaluate once per stage).
    const Report& report() const noexcept { return report_; }
    const CollisionConstraintOptions& options() const noexcept { return options_; }
    /// Between solves only; max_rows and max_cuts cannot change (the row count is fixed at creation).
    Status set_options(const CollisionConstraintOptions& options);
    const CollisionQuery& query() const noexcept { return query_; }
    /// Forgets which pairs were in the band, so pairs penetrating at the next step start are exempt.
    void reset_tracking() noexcept { tracked_ = false; }

  private:
    CollisionConstraint(CollisionQuery query, const CollisionConstraintOptions& options);
    double obstacle_rate(const Proximity& p) const;

    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                   Eigen::Ref<Eigen::VectorXd> hi) override;
    void begin_step(const Workspace& commanded, bool retry) override;
    Index add_cuts(const Tick& candidate) override;
    bool refines() const noexcept override { return near_ && options_.max_cuts > 0; }
    bool defers_to_later_stages() const noexcept override { return true; }
    bool checks_end(const Eigen::VectorXd& q_end, const Eigen::VectorXd& dq) const override;
    bool admits(const Tick& end) override;
    double admitted_fraction() const noexcept override { return admitted_fraction_; }
    double motion_bound(const Eigen::VectorXd& dq) const;
    void track_exemptions();
    bool exempt(Index pair) const noexcept { return exempt_[static_cast<std::size_t>(pair)] != 0; }

    CollisionQuery query_;
    CollisionConstraintOptions options_;
    std::vector<char> active_;
    std::vector<char> active_start_;  // active_ at the step start, restored for a re-solve
    std::vector<Index> selected_;
    std::vector<char> seen_, next_seen_;  // in the band at the last step start
    std::vector<char> exempt_;            // penetrating since entering the band
    bool tracked_{false};                 // seen_ holds the last step start
    std::uint64_t layout_{~std::uint64_t{0}};
    Report report_;

    // Cutting planes of the current step, linearized at a rejected step end q: d(q) + g (x - q) >= floor.
    const Workspace* commanded_{nullptr};
    bool record_start_{false};
    bool near_{false};                    // some pair was in the band at the step start
    std::vector<double> start_distance_;  // per pair at the step start, +inf outside the band
    Eigen::MatrixXd cut_at_;              // nq x max_cuts
    Eigen::MatrixXd cut_gradient_;        // max_cuts x nv
    Eigen::VectorXd cut_distance_, cut_floor_, cut_delta_;
    Index cut_count_{0};

    // Endpoint clip. verified_q_ is the last end a full cut scan found clean.
    bool verified_{false};
    Eigen::VectorXd verified_q_;
    double start_nearest_{std::numeric_limits<double>::infinity()};
    double admitted_fraction_{1.0};
    double start_bound_{0.0};
    // Per robot geometry: moving ancestor joints root first (CSR), radius and start pose data.
    std::vector<std::size_t> chain_begin_;
    std::vector<JointId> chain_;
    std::vector<double> radius_;
    std::vector<char> unbounded_;    // some ancestor joint type has no motion bound
    Eigen::Matrix3Xd start_origin_;  // joint origins at the step start
    Eigen::Matrix3Xd start_center_;  // geometry bounding-sphere centers at the step start
};

}  // namespace kstone
