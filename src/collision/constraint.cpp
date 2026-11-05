#include "kstone/collision/constraint.hpp"

#include "kstone/model/pinocchio.hpp"

#include "internal.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace kstone {

namespace {

Status check_distances(const CollisionConstraintOptions& o) {
    const auto non_negative = [](double v) { return std::isfinite(v) && v >= 0.0; };
    if (!non_negative(o.d_min) || !non_negative(o.gain) || !non_negative(o.activation_margin) ||
        !non_negative(o.hysteresis) || !non_negative(o.clip_slack) || !non_negative(o.approach_deceleration))
        return make_error(ErrorCode::invalid_argument,
                          "collision constraint: options must be finite and >= 0");
    return {};
}

}  // namespace

Result<std::shared_ptr<CollisionConstraint>> CollisionConstraint::create(
    CollisionModelPtr model, std::shared_ptr<const CollisionWorld> world,
    const CollisionConstraintOptions& options) {
    if (!model) return make_error(ErrorCode::invalid_argument, "collision constraint: null collision model");
    if (Status st = check_distances(options); !st) return st.error();
    if (options.max_rows <= 0)
        return make_error(ErrorCode::invalid_argument, "collision constraint: max_rows must be > 0");
    if (options.max_cuts < 0)
        return make_error(ErrorCode::invalid_argument, "collision constraint: max_cuts must be >= 0");
    if (model->pairs().empty() && (!world || world->size() == 0) && model->geometries().empty())
        return make_error(ErrorCode::invalid_argument, "collision constraint: nothing to check");
    return std::shared_ptr<CollisionConstraint>(
        new CollisionConstraint(CollisionQuery(std::move(model), std::move(world)), options));
}

CollisionConstraint::CollisionConstraint(CollisionQuery query, const CollisionConstraintOptions& options)
    : Constraint(options.max_rows + options.max_cuts), query_(std::move(query)), options_(options) {
    selected_.reserve(static_cast<std::size_t>(options_.max_rows));
}

Status CollisionConstraint::set_options(const CollisionConstraintOptions& options) {
    if (Status st = check_distances(options); !st) return st;
    if (options.max_rows != options_.max_rows || options.max_cuts != options_.max_cuts)
        return make_error(ErrorCode::invalid_argument,
                          "collision constraint: max_rows and max_cuts cannot change");
    options_ = options;
    return {};
}

Status CollisionConstraint::on_bind(const Model& model) {
    if (query_.model()->model().get() != &model)
        return make_error(ErrorCode::invalid_argument, "collision constraint: bound to a different model");
    const Index n = options_.max_cuts;
    cut_at_.setZero(model.nq(), n);
    cut_gradient_.setZero(n, model.nv());
    cut_distance_.setZero(n);
    cut_floor_.setZero(n);
    cut_delta_.setZero(model.nv());

    const CollisionModel& collision = *query_.model();
    const auto& geometry_joints = collision.joints();
    const auto& joints = model.joints();
    const std::size_t ng = geometry_joints.size();
    chain_begin_.assign(ng + 1, 0);
    chain_.clear();
    radius_.resize(ng);
    unbounded_.assign(ng, 0);
    for (std::size_t g = 0; g < ng; ++g) {
        chain_begin_[g] = chain_.size();
        radius_[g] = collision.impl().geometries[g]->aabb_radius;
        for (JointId j = geometry_joints[g]; j != 0; j = joints[j].parent) {
            chain_.push_back(j);
            const JointType t = joints[j].type;
            if (t == JointType::mimic || t == JointType::composite) unbounded_[g] = 1;
        }
    }
    chain_begin_[ng] = chain_.size();
    start_origin_.setZero(3, static_cast<Index>(joints.size()));
    start_center_.setZero(3, static_cast<Index>(ng));
    verified_q_.setZero(model.nq());
    return {};
}

double CollisionConstraint::motion_bound(const Eigen::VectorXd& dq) const {
    // Moving joints one at a time, root first, each turns its subtree rigidly about its own start
    // origin, so a point moves at most |w| * lever + |v| per joint (|translation of exp(v, w)| <= |v|).
    const auto& joints = model()->joints();
    double bound = 0.0;
    for (std::size_t g = 0; g + 1 < chain_begin_.size(); ++g) {
        if (chain_begin_[g] == chain_begin_[g + 1]) continue;
        if (unbounded_[g]) return std::numeric_limits<double>::infinity();
        const Eigen::Vector3d c = start_center_.col(static_cast<Index>(g));
        double sum = 0.0;
        for (std::size_t k = chain_begin_[g]; k < chain_begin_[g + 1]; ++k) {
            const JointInfo& j = joints[chain_[k]];
            double w = 0.0, v = 0.0;
            switch (j.type) {
                case JointType::revolute:
                case JointType::continuous:
                    w = std::abs(dq[j.idx_v]);
                    break;
                case JointType::prismatic:
                    v = std::abs(dq[j.idx_v]);
                    break;
                case JointType::translation:
                    v = dq.segment<3>(j.idx_v).norm();
                    break;
                case JointType::spherical:
                    w = dq.segment<3>(j.idx_v).norm();
                    break;
                case JointType::planar:
                    v = dq.segment<2>(j.idx_v).norm();
                    w = std::abs(dq[j.idx_v + 2]);
                    break;
                case JointType::free_flyer:
                    v = dq.segment<3>(j.idx_v).norm();
                    w = dq.segment<3>(j.idx_v + 3).norm();
                    break;
                default:
                    return std::numeric_limits<double>::infinity();
            }
            if (w > 0.0) w *= (c - start_origin_.col(static_cast<Index>(j.id))).norm() + radius_[g];
            sum += w + v;
        }
        bound = std::max(bound, sum);
    }
    return bound;
}

bool CollisionConstraint::checks_end(const Eigen::VectorXd& q_end, const Eigen::VectorXd& dq) const {
    if (commanded_ == nullptr || record_start_ || !std::isfinite(start_nearest_)) return false;
    if (verified_ && verified_q_ == q_end) return false;
    // Each pair has at most two moving sides; a pair's floor is min(d_min, its start distance) - clip_slack.
    const double reach = 2.0 * motion_bound(dq);
    const double slack = options_.clip_slack;
    return (start_nearest_ < options_.d_min && reach > slack) ||
           start_nearest_ - reach < options_.d_min - slack;
}

bool CollisionConstraint::admits(const Tick& end) {
    if (&end.workspace != commanded_ || record_start_) return true;
    const double slack = options_.clip_slack;
    if (Status s = query_.update(end.workspace, std::max(options_.d_min - slack, 0.0), false); !s)
        return false;
    const bool same = query_.layout() == layout_;
    admitted_fraction_ = 1.0;
    for (const Proximity& p : query_.proximities()) {
        if (same && exempt(p.pair)) continue;
        double start = same ? start_distance_[static_cast<std::size_t>(p.pair)] : start_nearest_;
        if (!std::isfinite(start)) start = start_bound_;  // unrecorded pairs started at least this far apart
        const double floor = std::min(options_.d_min, start) - slack;
        if (p.distance >= floor - 1e-9) continue;
        // Distance taken linear in the step scale between the start and this end.
        const double t = start > p.distance ? (start - floor) / (start - p.distance) : 0.0;
        admitted_fraction_ = std::min(admitted_fraction_, std::clamp(t, 0.0, 1.0));
    }
    return admitted_fraction_ == 1.0;
}

void CollisionConstraint::begin_step(const Workspace& commanded, bool retry) {
    commanded_ = &commanded;
    verified_ = false;
    // Every solve of the step starts from the hysteresis state the previous step left.
    if (retry) {
        if (active_start_.size() == active_.size())
            std::copy(active_start_.begin(), active_start_.end(), active_.begin());
        return;
    }
    active_start_.assign(active_.begin(), active_.end());
    cut_count_ = 0;
    record_start_ = true;
    near_ = false;
}

Index CollisionConstraint::add_cuts(const Tick& candidate) {
    verified_ = false;
    if (options_.max_cuts == 0 || &candidate.workspace != commanded_ || record_start_) return 0;
    if (cut_count_ == options_.max_cuts) return 0;
    const double d_min = options_.d_min;
    if (Status s = query_.update(candidate.workspace, d_min, false); !s) return 0;
    if (query_.layout() != layout_) return 0;  // the pair list changed under the step
    Index added = 0;
    bool clean = true;
    for (const Proximity& p : query_.proximities()) {
        if (exempt(p.pair)) continue;
        const double start = start_distance_[static_cast<std::size_t>(p.pair)];
        // A pair outside d_min stays outside; one inside comes back out by its recovery share.
        const double floor =
            start >= d_min ? d_min : std::min(d_min, start + options_.gain * (d_min - start) * candidate.dt);
        if (p.distance >= floor - 1e-9) continue;
        if (cut_count_ == options_.max_cuts) {
            clean = false;
            break;
        }
        if (Status s = query_.jacobian(candidate.workspace, p, cut_gradient_.row(cut_count_)); !s)
            return added;
        cut_at_.col(cut_count_) = candidate.workspace.q();
        cut_distance_[cut_count_] = p.distance;
        cut_floor_[cut_count_] = floor;
        ++cut_count_;
        ++added;
    }
    if (clean && added == 0) {
        verified_ = true;
        verified_q_ = candidate.workspace.q();
    }
    return added;
}

void CollisionConstraint::track_exemptions() {
    std::fill(next_seen_.begin(), next_seen_.end(), 0);
    for (const Proximity& p : query_.proximities()) {
        const auto pair = static_cast<std::size_t>(p.pair);
        next_seen_[pair] = 1;
        // Penetrating on entering the band: no local escape is guaranteed, so leave it free until clear.
        const bool entered = !tracked_ || !seen_[pair];
        exempt_[pair] = options_.exempt_penetrating &&
                        ((exempt_[pair] && p.distance <= options_.d_min) || (entered && p.distance < 0.0));
    }
    for (std::size_t i = 0; i < exempt_.size(); ++i)
        if (!next_seen_[i]) exempt_[i] = 0;
    seen_.swap(next_seen_);
    tracked_ = true;
}

double CollisionConstraint::obstacle_rate(const Proximity& p) const {
    const CollisionPair& pair = query_.pairs()[static_cast<std::size_t>(p.pair)];
    if (!pair.obstacle) return 0.0;
    const Eigen::Vector3d v = query_.world()->point_velocity(static_cast<std::size_t>(pair.second), p.point2);
    // Only an approaching obstacle tightens the row; a receding one never loosens it.
    return std::min(p.normal.dot(v), 0.0);
}

Status CollisionConstraint::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A,
                                    Eigen::Ref<Eigen::VectorXd> lo, Eigen::Ref<Eigen::VectorXd>) {
    const double activate = options_.d_min + options_.activation_margin;
    const double keep = activate + options_.hysteresis;
    if (Status s = query_.update(tick.workspace, keep); !s) return s;
    if (query_.layout() != layout_) {
        active_.assign(query_.pairs().size(), 0);
        exempt_.assign(query_.pairs().size(), 0);
        seen_.assign(query_.pairs().size(), 0);
        next_seen_.assign(query_.pairs().size(), 0);
        tracked_ = false;
        layout_ = query_.layout();
    }
    if (&tick.workspace == commanded_ && record_start_) track_exemptions();

    // proximities() is nearest first, so the first max_rows candidates are the nearest ones.
    report_ = Report{};
    report_.nearest = query_.nearest();
    selected_.clear();
    for (const Proximity& p : query_.proximities()) {
        const auto pair = static_cast<std::size_t>(p.pair);
        const bool candidate = p.distance <= activate || (active_[pair] && p.distance <= keep);
        if (!candidate) continue;
        if (exempt_[pair]) {
            ++report_.exempt;
            continue;
        }
        ++report_.candidates;
        if (static_cast<Index>(selected_.size()) < options_.max_rows) selected_.push_back(p.pair);
    }
    std::fill(active_.begin(), active_.end(), 0);

    Index row = 0;
    for (const Proximity& p : query_.proximities()) {
        if (row == static_cast<Index>(selected_.size())) break;
        if (p.pair != selected_[static_cast<std::size_t>(row)]) continue;
        if (Status s = query_.jacobian(tick.workspace, p, A.row(row)); !s) return s;
        const double margin = p.distance - options_.d_min;
        double rate = options_.gain * margin;
        if (options_.approach_deceleration > 0.0 && margin > 0.0)
            rate = std::max(rate, std::min(std::sqrt(2.0 * options_.approach_deceleration * margin),
                                           0.5 * margin / tick.dt));
        lo[row] = -rate - obstacle_rate(p);
        active_[static_cast<std::size_t>(p.pair)] = 1;
        ++row;
    }
    report_.rows = row;
    report_.dropped = report_.candidates - row;

    if (&tick.workspace != commanded_) return {};
    if (record_start_) {
        start_distance_.assign(query_.pairs().size(), std::numeric_limits<double>::infinity());
        for (const Proximity& p : query_.proximities())
            start_distance_[static_cast<std::size_t>(p.pair)] = p.distance;
        near_ = false;
        const Proximity& nearest = query_.nearest();
        start_nearest_ = nearest.pair >= 0 && exempt(nearest.pair) ? keep : nearest.distance;
        for (const Proximity& p : query_.proximities()) {
            if (exempt(p.pair)) continue;
            near_ = true;
            start_nearest_ = std::min(start_nearest_, p.distance);
        }
        start_bound_ = keep;
        const pinocchio::Data& data = pinocchio_data(tick.workspace);
        for (Index j = 0; j < start_origin_.cols(); ++j)
            start_origin_.col(j) = data.oMi[static_cast<std::size_t>(j)].translation();
        const CollisionModel& collision = *query_.model();
        for (Index g = 0; g < start_center_.cols(); ++g) {
            const auto gi = static_cast<std::size_t>(g);
            const pinocchio::SE3 pose = data.oMi[collision.joints()[gi]] * collision.impl().placements[gi];
            start_center_.col(g) = pose.act(Eigen::Vector3d(collision.impl().geometries[gi]->aabb_center));
        }
        record_start_ = false;
    }
    // Cuts follow the normal rows, linearized about their own point: d + g (x - x_cut) + g v dt >= floor.
    for (Index k = 0; k < cut_count_; ++k) {
        if (Status s = model()->difference(cut_at_.col(k), tick.workspace.q(), cut_delta_); !s) return s;
        const double linear = cut_distance_[k] + cut_gradient_.row(k).dot(cut_delta_);
        const Index r = options_.max_rows + k;
        A.row(r) = cut_gradient_.row(k);
        lo[r] = (cut_floor_[k] + 1e-6 - linear) / tick.dt;
    }
    report_.cuts = cut_count_;
    return {};
}

}  // namespace kstone
