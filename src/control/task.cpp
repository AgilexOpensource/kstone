#include "kstone/control/task.hpp"

#include "kstone/control/constraint.hpp"

#include <Eigen/SVD>

#include <cmath>
#include <limits>

namespace kstone {

namespace {

Status check_tick(const ModelPtr& model, const Tick& tick, const char* what) {
    if (!model)
        return make_error(ErrorCode::invalid_argument, std::string(what) + " is not bound to a model");
    if (tick.workspace.model().get() != model.get())
        return make_error(ErrorCode::invalid_argument,
                          std::string(what) + ": workspace belongs to another model");
    if (!std::isfinite(tick.dt) || tick.dt <= 0)
        return make_error(ErrorCode::invalid_argument,
                          std::string(what) + ": dt must be finite and positive");
    return {};
}

}  // namespace

Task::Task(Index rows, double gain) : jacobian_(rows, 0), error_(Eigen::VectorXd::Zero(rows)), gain_(gain) {
    weights_ = Eigen::VectorXd::Constant(rows, scalar_weight_);
}

Task::~Task() = default;

Status Task::bind(const ModelPtr& model) {
    if (!model) return make_error(ErrorCode::invalid_argument, "task: null model");
    if (model_) {
        if (model_.get() == model.get()) return {};
        return make_error(ErrorCode::invalid_argument, "task: already bound to another model");
    }
    const auto nv = static_cast<std::size_t>(model->nv());
    if (mask_.size() != 0 && mask_.size() != nv)
        return make_error(ErrorCode::invalid_argument, "task: mask size does not match the model nv");
    model_ = model;
    jacobian_.setZero(jacobian_.rows(), model->nv());
    if (Status s = on_bind(*model); !s) {
        model_.reset();
        return s;
    }
    jacobian_.setZero(jacobian_.rows(), model->nv());
    error_.setZero(jacobian_.rows());
    if (Status s = expand_weights(); !s) {
        model_.reset();
        return s;
    }
    return {};
}

Status Task::evaluate(const Tick& tick) {
    if (Status s = check_tick(model_, tick, "task"); !s) return s;
    jacobian_.setZero();
    error_.setZero();
    if (Status s = compute(tick, jacobian_, error_); !s) return s;
    if (mask_.size() != 0 && mask_columns_) {
        for (Index c = 0; c < jacobian_.cols(); ++c) {
            if (!mask_.test(static_cast<std::size_t>(c))) jacobian_.col(c).setZero();
        }
    }
    if (!jacobian_.allFinite() || !error_.allFinite()) {
        return make_error(ErrorCode::invalid_argument, "task: non-finite Jacobian or error");
    }
    return {};
}

Status Task::set_weight(double weight) {
    if (!std::isfinite(weight) || weight < 0) {
        return make_error(ErrorCode::invalid_argument, "task: weight must be finite and >= 0");
    }
    scalar_weight_ = weight;
    row_weights_.resize(0);
    return expand_weights();
}

Status Task::set_weight(const Eigen::Ref<const Eigen::VectorXd>& weights) {
    if (!weights.allFinite() || (weights.array() < 0).any()) {
        return make_error(ErrorCode::invalid_argument, "task: weights must be finite and >= 0");
    }
    if (bound() && static_cast<Index>(weights.size()) != rows()) {
        return make_error(ErrorCode::invalid_argument, "task: one weight per row required");
    }
    row_weights_ = weights;
    return bound() ? expand_weights() : Status{};
}

Status Task::set_gain(double gain) {
    if (!std::isfinite(gain) || gain < 0 || gain > 1) {
        return make_error(ErrorCode::invalid_argument, "task: gain must be in [0, 1]");
    }
    gain_ = gain;
    return {};
}

Status Task::set_lm_damping(double lm) {
    if (!std::isfinite(lm) || lm < 0) {
        return make_error(ErrorCode::invalid_argument, "task: LM damping must be finite and >= 0");
    }
    lm_ = lm;
    return {};
}

Status Task::set_mask(Mask mask) {
    if (model_ && mask.size() != 0 && mask.size() != static_cast<std::size_t>(model_->nv())) {
        return make_error(ErrorCode::invalid_argument, "task: mask size does not match the model nv");
    }
    mask_ = std::move(mask);
    return {};
}

Status Task::resize_rows(Index rows) {
    if (rows < 0) return make_error(ErrorCode::invalid_argument, "task: negative row count");
    jacobian_.resize(rows, model_ ? model_->nv() : 0);
    error_.resize(rows);
    return expand_weights();
}

Status Task::expand_weights() {
    const Index rows = jacobian_.rows();
    if (row_weights_.size() == 0) {
        weights_.setConstant(rows, scalar_weight_);
        return {};
    }
    if (static_cast<Index>(row_weights_.size()) != rows) {
        weights_.setConstant(rows, scalar_weight_);
        return make_error(ErrorCode::invalid_argument, "task: per-row weights do not match the row count");
    }
    weights_ = row_weights_;
    return {};
}

Status null_space_projector(const Task& primary, const std::vector<Index>& columns, double damping,
                            double threshold, Eigen::MatrixXd& out) {
    NullSpaceScratch scratch;
    return null_space_projector(primary, columns, damping, threshold, out, scratch);
}

Status null_space_projector(const Task& primary, const std::vector<Index>& columns, double damping,
                            double threshold, Eigen::MatrixXd& out, NullSpaceScratch& scratch) {
    if (!primary.bound())
        return make_error(ErrorCode::invalid_argument, "null space: primary task is not bound");
    if (!std::isfinite(damping) || damping < 0 || !std::isfinite(threshold) || threshold < 0)
        return make_error(ErrorCode::invalid_argument,
                          "null space: damping and threshold must be finite and >= 0");
    const Eigen::MatrixXd& J = primary.jacobian();
    const Eigen::VectorXd& w = primary.weights();
    const Index nv = J.cols();
    const auto k = static_cast<Index>(columns.size());
    std::vector<char>& seen = scratch.seen;
    seen.assign(static_cast<std::size_t>(nv), 0);
    for (const Index c : columns) {
        if (c < 0 || c >= nv) return make_error(ErrorCode::out_of_range, "null space: column out of range");
        if (seen[static_cast<std::size_t>(c)])
            return make_error(ErrorCode::invalid_argument, "null space: duplicate column");
        seen[static_cast<std::size_t>(c)] = 1;
    }
    out.setIdentity(k, k);
    Index kept = 0;
    for (Index r = 0; r < J.rows(); ++r)
        if (w[r] > 0) ++kept;
    if (kept == 0 || k == 0) return {};
    // Weighted primary Jacobian restricted to the selected columns.
    Eigen::MatrixXd& weighted = scratch.weighted;
    weighted.resize(kept, k);
    Index row = 0;
    for (Index r = 0; r < J.rows(); ++r) {
        if (!(w[r] > 0)) continue;
        for (Index c = 0; c < k; ++c) weighted(row, c) = w[r] * J(r, columns[static_cast<std::size_t>(c)]);
        ++row;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd>& svd = scratch.svd;
    svd.compute(weighted, Eigen::ComputeThinV);
    svd.setThreshold(1e-9);
    const Eigen::VectorXd& sv = svd.singularValues();
    const Eigen::MatrixXd& V = svd.matrixV();
    const Index rank = svd.rank();
    for (Index i = 0; i < rank; ++i) {
        const double s = sv[i];
        double lambda = damping;
        if (threshold > 0) {
            const double ratio = s / threshold;
            lambda = ratio >= 1 ? 0.0 : damping * std::sqrt(1.0 - ratio * ratio);
        }
        const double keep = s / std::hypot(s, lambda);
        out.noalias() -= (keep * keep) * V.col(i) * V.col(i).transpose();
    }
    for (Index i = 0; i < k; ++i)
        for (Index j = i + 1; j < k; ++j) out(i, j) = out(j, i) = 0.5 * (out(i, j) + out(j, i));
    return {};
}

Constraint::~Constraint() = default;

Status Constraint::bind(const ModelPtr& model) {
    if (!model) return make_error(ErrorCode::invalid_argument, "constraint: null model");
    if (model_) {
        if (model_.get() == model.get()) return {};
        return make_error(ErrorCode::invalid_argument, "constraint: already bound to another model");
    }
    model_ = model;
    if (Status s = on_bind(*model); !s) {
        model_.reset();
        return s;
    }
    return {};
}

Status Constraint::evaluate(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                            Eigen::Ref<Eigen::VectorXd> hi) {
    if (Status s = check_tick(model_, tick, "constraint"); !s) return s;
    if (A.rows() != rows_ || A.cols() != model_->nv() || lo.size() != rows_ || hi.size() != rows_)
        return make_error(ErrorCode::invalid_argument, "constraint: output size mismatch");
    const double inf = std::numeric_limits<double>::infinity();
    A.setZero();
    lo.setConstant(-inf);
    hi.setConstant(inf);
    if (Status s = compute(tick, A, lo, hi); !s) return s;
    if (!A.allFinite() || lo.array().isNaN().any() || hi.array().isNaN().any() ||
        (lo.array() > hi.array()).any())
        return make_error(ErrorCode::invalid_argument, "constraint: non-finite rows or lo > hi");
    return {};
}

void Constraint::tighten_bounds(const Tick&, Eigen::Ref<Eigen::VectorXd>, Eigen::Ref<Eigen::VectorXd>) const {
}

void Constraint::begin_step(const Workspace&, bool) {}

Index Constraint::add_cuts(const Tick&) { return 0; }

bool Constraint::checks_end(const Eigen::VectorXd&, const Eigen::VectorXd&) const { return false; }

bool Constraint::admits(const Tick&) { return true; }

}  // namespace kstone
