#include "kstone/control/tasks.hpp"

#include "kstone/model/pinocchio.hpp"

#include <Eigen/Cholesky>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace kstone {

namespace {

constexpr double kPi = 3.141592653589793;

pinocchio::SE3 to_se3(const Eigen::Isometry3d& pose) {
    return pinocchio::SE3(pose.rotation(), pose.translation());
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

bool is_rotation(const Eigen::Matrix3d& R) {
    return R.allFinite() && (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
           std::abs(R.determinant() - 1.0) < 1e-6;
}

/// Resolve names to scalar joints (nv == 1). Empty selects all of them.
Status resolve_scalar(const Model& model, const std::vector<std::string>& names, const char* what,
                      std::vector<JointId>& joints, std::vector<Index>& indices) {
    joints.clear();
    indices.clear();
    auto accept = [](const JointInfo& j) { return j.nv == 1; };
    if (names.empty()) {
        for (const JointInfo& j : model.joints()) {
            if (j.id == 0 || !accept(j)) continue;
            joints.push_back(j.id);
            indices.push_back(j.idx_v);
        }
        if (joints.empty())
            return make_error(ErrorCode::invalid_argument,
                              std::string(what) + ": model has no scalar joints");
        return {};
    }
    for (const std::string& name : names) {
        Result<JointId> id = model.joint_id(name);
        if (!id) return id.error();
        const JointInfo& j = model.joints()[*id];
        if (!accept(j))
            return make_error(ErrorCode::invalid_argument,
                              std::string(what) + ": joint '" + name + "' is not scalar");
        if (std::find(joints.begin(), joints.end(), *id) != joints.end())
            return make_error(ErrorCode::invalid_argument,
                              std::string(what) + ": duplicate joint '" + name + "'");
        joints.push_back(*id);
        indices.push_back(j.idx_v);
    }
    return {};
}

Status check_primary(const std::shared_ptr<Task>& primary, const Task* self, const ModelPtr& model,
                     const char* what) {
    if (!primary) return make_error(ErrorCode::invalid_argument, std::string(what) + ": null primary task");
    if (primary.get() == self)
        return make_error(ErrorCode::invalid_argument,
                          std::string(what) + ": task cannot be its own primary");
    return primary->bind(model);
}

Status check_settings(const NullSpaceSettings& s) {
    if (!std::isfinite(s.damping) || s.damping < 0 || !std::isfinite(s.threshold) || s.threshold < 0)
        return make_error(ErrorCode::invalid_argument,
                          "null space: damping and threshold must be finite and >= 0");
    return {};
}

}  // namespace

// ---- FrameTask -----------------------------------------------------------------------------

FrameTask::FrameTask(std::string frame, const Eigen::Isometry3d& target)
    : Task(6, 1.0), frame_name_(std::move(frame)), target_(target) {}

Status FrameTask::set_target(const Eigen::Isometry3d& target) {
    if (!is_rotation(target.rotation()) || !target.translation().allFinite())
        return make_error(ErrorCode::invalid_argument, "frame task: target must be a finite rigid transform");
    target_ = target;
    return {};
}

Status FrameTask::on_bind(const Model& model) {
    if (!is_rotation(target_.rotation()) || !target_.translation().allFinite())
        return make_error(ErrorCode::invalid_argument, "frame task: target must be a finite rigid transform");
    Result<FrameId> id = model.frame_id(frame_name_);
    if (!id) return id.error();
    frame_ = *id;
    reference_dirty_ = true;
    tip_jac_.setZero(6, model.nv());
    ref_jac_.setZero(6, model.nv());
    return {};
}

Status FrameTask::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) {
    Workspace& ws = tick.workspace;
    if (reference_dirty_) {
        has_reference_ = !reference_name_.empty();
        if (has_reference_) {
            Result<FrameId> id = model()->frame_id(reference_name_);
            if (!id) return id.error();
            reference_ = *id;
        }
        reference_dirty_ = false;
    }
    Eigen::Isometry3d tip;
    if (Status s = ws.frame_pose(frame_, tip); !s) return s;
    if (Status s = ws.frame_jacobian(frame_, ReferenceFrame::local, tip_jac_); !s) return s;
    pinocchio::SE3 current = to_se3(tip);
    if (has_reference_) {
        Eigen::Isometry3d base;
        if (Status s = ws.frame_pose(reference_, base); !s) return s;
        if (Status s = ws.frame_jacobian(reference_, ReferenceFrame::local, ref_jac_); !s) return s;
        current = to_se3(base).actInv(current);
        // Relative twist in the tip frame: V_tip - Ad(current^-1) V_ref.
        tip_jac_.noalias() -= current.inverse().toActionMatrix() * ref_jac_;
    }
    const pinocchio::SE3 desired = to_se3(target_);
    e = pinocchio::log6(current.actInv(desired)).toVector();
    Eigen::Matrix<double, 6, 6> jlog =
        Eigen::Matrix<double, 6, 6>::Zero();  // silences GCC 15 maybe-uninitialized
    pinocchio::Jlog6(desired.actInv(current), jlog);
    J.noalias() = -jlog * tip_jac_;
    if (world_) {
        const Eigen::Matrix3d R = current.rotation();
        e.head<3>() = (R * e.head<3>()).eval();
        e.tail<3>() = (R * e.tail<3>()).eval();
        J.topRows<3>() = (R * J.topRows<3>()).eval();
        J.bottomRows<3>() = (R * J.bottomRows<3>()).eval();
    }
    return {};
}

// ---- JointTask -----------------------------------------------------------------------------

JointTask::JointTask(std::vector<std::string> joints, Eigen::VectorXd target)
    : Task(static_cast<Index>(joints.size()), 1.0), names_(std::move(joints)), target_(std::move(target)) {}

Status JointTask::set_target(const Eigen::Ref<const Eigen::VectorXd>& target) {
    if (target.size() != static_cast<Index>(names_.size()) || !target.allFinite())
        return make_error(ErrorCode::invalid_argument, "joint task: one finite target per joint required");
    target_ = target;
    return {};
}

Status JointTask::on_bind(const Model& model) {
    if (names_.empty()) return make_error(ErrorCode::invalid_argument, "joint task: no joints");
    if (Status s = set_target(target_); !s) return s;
    std::vector<Index> indices;
    return resolve_scalar(model, names_, "joint task", joints_, indices);
}

Status JointTask::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) {
    const Model& m = *model();
    const Eigen::VectorXd& q = tick.workspace.q();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const auto r = static_cast<Index>(i);
        const JointInfo& j = m.joints()[joints_[i]];
        Result<double> pos = m.joint_position(q, j.id);
        if (!pos) return pos.error();
        double err = *pos - target_[r];
        if (j.type == JointType::continuous) {
            err = std::remainder(err, 2 * kPi);
            if (err <= -kPi) err += 2 * kPi;
        }
        e[r] = err;
        J(r, j.idx_v) = 1.0;
    }
    return {};
}

// ---- PostureTask ---------------------------------------------------------------------------

PostureTask::PostureTask(Eigen::VectorXd target, std::vector<std::string> joints)
    : Task(0, 1.0), names_(std::move(joints)), target_(std::move(target)) {}

Status PostureTask::set_target(const Eigen::Ref<const Eigen::VectorXd>& target) {
    if (bound()) {
        if (Status s = model()->validate(target); !s) return s;
    } else if (!target.allFinite()) {
        return make_error(ErrorCode::invalid_argument, "posture task: target must be finite");
    }
    target_ = target;
    return {};
}

Status PostureTask::on_bind(const Model& model) {
    if (Status s = model.validate(target_); !s) return s;
    std::vector<JointId> joints;
    if (Status s = resolve_scalar(model, names_, "posture task", joints, indices_); !s) return s;
    diff_.setZero(model.nv());
    return resize_rows(static_cast<Index>(indices_.size()));
}

Status PostureTask::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) {
    if (Status s = model()->difference(target_, tick.workspace.q(), diff_); !s) return s;
    for (std::size_t i = 0; i < indices_.size(); ++i) {
        const auto r = static_cast<Index>(i);
        e[r] = diff_[indices_[i]];
        J(r, indices_[i]) = 1.0;
    }
    return {};
}

// ---- GazeTask ------------------------------------------------------------------------------

GazeTask::GazeTask(std::string frame, const Eigen::Vector3d& focus)
    : Task(3, 1.0), frame_name_(std::move(frame)), focus_(focus) {}

Status GazeTask::set_focus(const Eigen::Vector3d& focus) {
    if (!focus.allFinite()) return make_error(ErrorCode::invalid_argument, "gaze task: focus must be finite");
    focus_ = focus;
    return {};
}

Status GazeTask::set_axis(const Eigen::Vector3d& axis) {
    const double n = axis.norm();
    if (!axis.allFinite() || n < 1e-9)
        return make_error(ErrorCode::invalid_argument, "gaze task: axis must be finite and non-zero");
    axis_ = axis / n;
    return {};
}

Status GazeTask::set_offset(const Eigen::Vector3d& offset) {
    if (!offset.allFinite())
        return make_error(ErrorCode::invalid_argument, "gaze task: offset must be finite");
    offset_ = offset;
    return {};
}

Status GazeTask::set_min_distance(double distance) {
    if (!std::isfinite(distance) || distance < 0)
        return make_error(ErrorCode::invalid_argument, "gaze task: minimum distance must be finite and >= 0");
    min_distance_ = distance;
    return {};
}

Status GazeTask::on_bind(const Model& model) {
    if (!focus_.allFinite())
        return make_error(ErrorCode::invalid_argument, "gaze task: focus must be finite");
    Result<FrameId> id = model.frame_id(frame_name_);
    if (!id) return id.error();
    frame_ = *id;
    jac_.setZero(6, model.nv());
    linear_.setZero(3, model.nv());
    return {};
}

Status GazeTask::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) {
    Workspace& ws = tick.workspace;
    Eigen::Isometry3d pose;
    if (Status s = ws.frame_pose(frame_, pose); !s) return s;
    const Eigen::Matrix3d R = pose.rotation();
    const Eigen::Vector3d lever = R * offset_;
    const Eigen::Vector3d ray = focus_ - pose.translation() - lever;
    const double dist = ray.norm();
    if (dist <= std::max(min_distance_, 1e-12)) return {};  // inert
    if (Status s = ws.frame_jacobian(frame_, ReferenceFrame::local_world_aligned, jac_); !s) return s;
    const Eigen::Vector3d desired = ray / dist;
    const Eigen::Vector3d current = R * axis_;
    const auto Jw = jac_.bottomRows<3>();
    linear_ = jac_.topRows<3>();
    linear_.noalias() -= skew(lever) * Jw;
    const Eigen::Matrix3d proj = (Eigen::Matrix3d::Identity() - desired * desired.transpose()) / dist;
    J.noalias() = -skew(current) * Jw;
    J.noalias() += proj * linear_;
    e = current - desired;
    return {};
}

// ---- ManipulabilityTask --------------------------------------------------------------------

ManipulabilityTask::ManipulabilityTask(std::string frame, std::vector<std::string> joints,
                                       Eigen::Matrix<double, 6, 1> axes)
    : Task(1, 1.0), frame_name_(std::move(frame)), names_(std::move(joints)), axes_(axes) {}

ManipulabilityTask::~ManipulabilityTask() = default;

Status ManipulabilityTask::set_rate(double rate) {
    if (!std::isfinite(rate) || rate <= 0)
        return make_error(ErrorCode::invalid_argument, "manipulability: rate must be > 0");
    rate_ = rate;
    return {};
}

Status ManipulabilityTask::set_regularization(double r) {
    if (!std::isfinite(r) || r <= 0)
        return make_error(ErrorCode::invalid_argument, "manipulability: regularization must be > 0");
    regularization_ = r;
    return {};
}

Status ManipulabilityTask::on_bind(const Model& model) {
    for (Index i = 0; i < 6; ++i) {
        if (axes_[i] != 0.0 && axes_[i] != 1.0)
            return make_error(ErrorCode::invalid_argument, "manipulability: axes entries must be 0 or 1");
    }
    if (axes_.sum() < 1) return make_error(ErrorCode::invalid_argument, "manipulability: no axis selected");
    Result<FrameId> id = model.frame_id(frame_name_);
    if (!id) return id.error();
    frame_ = *id;
    std::vector<JointId> joints;
    if (Status s = resolve_scalar(model, names_, "manipulability", joints, indices_); !s) return s;
    q_indices_.clear();
    for (const JointId j : joints) {
        const JointInfo& info = model.joints()[j];
        q_indices_.push_back(info.nq == 1 ? info.idx_q : -1);
    }
    // Analytic if each selected chain joint has a fixed axis and no mimic joint couples columns.
    const pinocchio::Model& pin = pinocchio_model(model);
    in_chain_.assign(indices_.size(), 0);
    analytic_ = true;
    for (const auto jid : pin.supports[pin.frames[frame_].parentJoint]) {
        if (jid == 0) continue;
        const auto& joint = pin.joints[jid];
        const std::string kind = joint.shortname();
        if (kind.rfind("JointModelMimic", 0) == 0) analytic_ = false;
        const auto it = std::find(indices_.begin(), indices_.end(), static_cast<Index>(joint.idx_v()));
        if (it == indices_.end() || joint.nv() != 1) continue;
        in_chain_[static_cast<std::size_t>(it - indices_.begin())] = 1;
        const bool fixed_axis = kind.rfind("JointModelR", 0) == 0 || kind.rfind("JointModelP", 0) == 0 ||
                                kind.rfind("JointModelH", 0) == 0;
        if (!fixed_axis) analytic_ = false;
    }
    probe_ = std::make_unique<Workspace>(this->model());
    jac_.setZero(6, model.nv());
    const auto rows = static_cast<Index>(axes_.sum());
    weights_.setZero(rows, static_cast<Index>(indices_.size()));
    selected_.setZero(rows, static_cast<Index>(indices_.size()));
    gram_.setIdentity(rows, rows);
    llt_.compute(gram_);  // sizes the factorization buffers
    q0_.setZero(model.nq());
    step_.setZero(model.nv());
    shifted_.setZero(model.nq());
    gradient_.setZero(model.nv());
    return {};
}

Status ManipulabilityTask::value_at(const Eigen::Ref<const Eigen::VectorXd>& q, double& value) {
    // Only the frame's support chain is evaluated; a full Workspace::update costs several times more.
    const pinocchio::Model& pin = pinocchio_model(*model());
    jac_.setZero();
    if (pin.frames[frame_].parentJoint != 0)
        pinocchio::computeFrameJacobian(pin, pinocchio_data(*probe_), q, frame_, pinocchio::LOCAL, jac_);
    Index row = 0;
    for (Index a = 0; a < 6; ++a) {
        if (axes_[a] == 0.0) continue;
        for (std::size_t c = 0; c < indices_.size(); ++c)
            selected_(row, static_cast<Index>(c)) = jac_(a, indices_[c]);
        ++row;
    }
    const double r2 = regularization_ * regularization_;
    gram_.noalias() = selected_ * selected_.transpose();
    gram_.diagonal().array() += r2;
    llt_.compute(gram_);
    if (llt_.info() != Eigen::Success)
        return make_error(ErrorCode::internal, "manipulability: factorization failed");
    value = llt_.matrixLLT().diagonal().array().log().sum();  // 0.5 log det
    return {};
}

Status ManipulabilityTask::measure(const Eigen::Ref<const Eigen::VectorXd>& q, double& value,
                                   Eigen::VectorXd* gradient) {
    if (!bound()) return make_error(ErrorCode::invalid_argument, "manipulability: task is not bound");
    if (Status s = model()->validate(q); !s) return s;
    q0_ = q;
    const Eigen::VectorXd& q0 = q0_;
    if (Status s = value_at(q0, value); !s) return s;
    if (!gradient) return {};
    gradient->setZero(model()->nv());
    if (analytic_) {
        // df/dq_i = <G^-1 S, dS/dq_i>; for j before i on the chain dJ_j/dq_i = ad(J_j) J_i (LOCAL).
        weights_ = llt_.solve(selected_);
        const auto k = static_cast<Index>(indices_.size());
        for (Index c = 0; c < k; ++c) {
            if (!in_chain_[static_cast<std::size_t>(c)]) continue;
            const Index i = indices_[static_cast<std::size_t>(c)];
            const Eigen::Vector3d vi = jac_.col(i).head<3>(), wi = jac_.col(i).tail<3>();
            double g = 0;
            for (Index d = 0; d < k; ++d) {
                const Index j = indices_[static_cast<std::size_t>(d)];
                if (!in_chain_[static_cast<std::size_t>(d)] || j >= i) continue;
                const Eigen::Vector3d vj = jac_.col(j).head<3>(), wj = jac_.col(j).tail<3>();
                Eigen::Matrix<double, 6, 1> dj;
                dj << wj.cross(vi) + vj.cross(wi), wj.cross(wi);
                Index row = 0;
                for (Index a = 0; a < 6; ++a) {
                    if (axes_[a] == 0.0) continue;
                    g += weights_(row++, d) * dj[a];
                }
            }
            (*gradient)[i] = g;
        }
        return {};
    }
    constexpr double h = 1e-5;
    shifted_ = q0;
    for (std::size_t c = 0; c < indices_.size(); ++c) {
        const Index i = indices_[c];
        double plus = 0, minus = 0;
        if (const Index qi = q_indices_[c]; qi >= 0) {
            // Same result as integrate for nq == 1 joints, without a whole-model pass.
            shifted_[qi] = q0[qi] + h;
            if (Status s = value_at(shifted_, plus); !s) return s;
            shifted_[qi] = q0[qi] - h;
            if (Status s = value_at(shifted_, minus); !s) return s;
            shifted_[qi] = q0[qi];
            (*gradient)[i] = (plus - minus) / (2 * h);
            continue;
        }
        step_.setZero();
        step_[i] = h;
        if (Status s = model()->integrate(q0, step_, shifted_); !s) return s;
        if (Status s = value_at(shifted_, plus); !s) return s;
        step_[i] = -h;
        if (Status s = model()->integrate(q0, step_, shifted_); !s) return s;
        if (Status s = value_at(shifted_, minus); !s) return s;
        shifted_ = q0;
        (*gradient)[i] = (plus - minus) / (2 * h);
    }
    return {};
}

Status ManipulabilityTask::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J,
                                   Eigen::Ref<Eigen::VectorXd> e) {
    double value = 0;
    if (Status s = measure(tick.workspace.q(), value, &gradient_); !s) return s;
    J.row(0) = gradient_.transpose();
    e[0] = -rate_ * tick.dt;
    return {};
}

// ---- NullSpacePostureTask ------------------------------------------------------------------

NullSpacePostureTask::NullSpacePostureTask(std::shared_ptr<Task> primary, Eigen::VectorXd target,
                                           std::vector<std::string> joints)
    : Task(0, 1.0), primary_(std::move(primary)), names_(std::move(joints)), target_(std::move(target)) {}

Status NullSpacePostureTask::set_target(const Eigen::Ref<const Eigen::VectorXd>& target) {
    if (bound()) {
        if (Status s = model()->validate(target); !s) return s;
    } else if (!target.allFinite()) {
        return make_error(ErrorCode::invalid_argument, "null-space posture: target must be finite");
    }
    target_ = target;
    return {};
}

Status NullSpacePostureTask::set_settings(NullSpaceSettings settings) {
    if (Status s = check_settings(settings); !s) return s;
    settings_ = settings;
    return {};
}

Status NullSpacePostureTask::set_velocity_scale(double scale) {
    if (!std::isfinite(scale) || scale <= 0 || scale > 1)
        return make_error(ErrorCode::invalid_argument,
                          "null-space posture: velocity scale must be in (0, 1]");
    velocity_scale_ = scale;
    return {};
}

Status NullSpacePostureTask::set_velocity_limit(const Eigen::Ref<const Eigen::VectorXd>& limit) {
    if (bound() && limit.size() != static_cast<Index>(indices_.size()))
        return make_error(ErrorCode::invalid_argument,
                          "null-space posture: one velocity limit per joint required");
    if ((limit.array().isFinite() && limit.array() <= 0).any())
        return make_error(ErrorCode::invalid_argument,
                          "null-space posture: finite velocity limits must be > 0");
    limit_ = limit;
    return {};
}

Status NullSpacePostureTask::on_bind(const Model& model) {
    if (Status s = check_primary(primary_, this, this->model(), "null-space posture"); !s) return s;
    if (Status s = model.validate(target_); !s) return s;
    std::vector<JointId> joints;
    if (Status s = resolve_scalar(model, names_, "null-space posture", joints, indices_); !s) return s;
    const auto k = static_cast<Index>(indices_.size());
    if (limit_.size() == 0) limit_.setConstant(k, std::numeric_limits<double>::infinity());
    if (limit_.size() != k)
        return make_error(ErrorCode::invalid_argument,
                          "null-space posture: one velocity limit per joint required");
    diff_.setZero(model.nv());
    selected_.setZero(k);
    projected_.setZero(k);
    return resize_rows(k);
}

Status NullSpacePostureTask::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J,
                                     Eigen::Ref<Eigen::VectorXd> e) {
    if (Status s = primary_->evaluate(tick); !s) return s;
    if (Status s = null_space_projector(*primary_, indices_, settings_.damping, settings_.threshold,
                                        projector_, scratch_);
        !s)
        return s;
    if (Status s = model()->difference(target_, tick.workspace.q(), diff_); !s) return s;
    const auto k = static_cast<Index>(indices_.size());
    for (Index c = 0; c < k; ++c) selected_[c] = diff_[indices_[static_cast<std::size_t>(c)]];
    projected_.noalias() = projector_ * selected_;
    const Eigen::VectorXd& model_limit = model()->velocity_limit();
    double ratio = 0;
    for (Index c = 0; c < k; ++c) {
        double limit = limit_[c];
        if (!std::isfinite(limit)) limit = model_limit[indices_[static_cast<std::size_t>(c)]];
        if (!std::isfinite(limit) || limit <= 1e-10 || limit >= 1e20) continue;
        const double budget = velocity_scale_ * limit * tick.dt;
        ratio = std::max(ratio, gain() * std::abs(projected_[c]) / budget);
    }
    if (ratio > 1) projected_ /= ratio;
    e = projected_;
    for (Index c = 0; c < k; ++c) J.col(indices_[static_cast<std::size_t>(c)]) = projector_.col(c);
    return {};
}

// ---- NullSpaceManipulabilityTask -----------------------------------------------------------

NullSpaceManipulabilityTask::NullSpaceManipulabilityTask(std::shared_ptr<Task> primary,
                                                         std::shared_ptr<ManipulabilityTask> preference)
    : Task(0, 1.0), primary_(std::move(primary)), preference_(std::move(preference)) {}

Status NullSpaceManipulabilityTask::set_settings(NullSpaceSettings settings) {
    if (Status s = check_settings(settings); !s) return s;
    settings_ = settings;
    return {};
}

Status NullSpaceManipulabilityTask::on_bind(const Model&) {
    if (Status s = check_primary(primary_, this, model(), "null-space manipulability"); !s) return s;
    if (!preference_)
        return make_error(ErrorCode::invalid_argument, "null-space manipulability: null preference task");
    if (Status s = preference_->bind(model()); !s) return s;
    ascent_.setZero(static_cast<Index>(preference_->indices().size()));
    selected_.setZero(ascent_.size());
    gradient_.setZero(model()->nv());
    return resize_rows(ascent_.size());
}

Status NullSpaceManipulabilityTask::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J,
                                            Eigen::Ref<Eigen::VectorXd> e) {
    if (Status s = primary_->evaluate(tick); !s) return s;
    const std::vector<Index>& idx = preference_->indices();
    if (Status s = null_space_projector(*primary_, idx, settings_.damping, settings_.threshold, projector_,
                                        scratch_);
        !s)
        return s;
    double value = 0;
    if (Status s = preference_->measure(tick.workspace.q(), value, &gradient_); !s) return s;
    const auto k = static_cast<Index>(idx.size());
    for (Index c = 0; c < k; ++c) selected_[c] = gradient_[idx[static_cast<std::size_t>(c)]];
    ascent_.noalias() = projector_ * selected_;
    const double rate = preference_->rate() * tick.dt;
    const double scale = std::max(ascent_.norm(), 0.5);
    e = -(rate / (scale * scale)) * ascent_;
    for (Index c = 0; c < k; ++c) J.col(idx[static_cast<std::size_t>(c)]) = projector_.col(c);
    return {};
}

// ---- ExternalTask --------------------------------------------------------------------------

ExternalTask::ExternalTask() : Task(0, 1.0) {}

Status ExternalTask::set(const Eigen::Ref<const Eigen::MatrixXd>& J,
                         const Eigen::Ref<const Eigen::VectorXd>& e) {
    if (J.rows() != e.size())
        return make_error(ErrorCode::invalid_argument, "external task: J rows must match e size");
    if (!J.allFinite() || !e.allFinite())
        return make_error(ErrorCode::invalid_argument, "external task: non-finite input");
    if (bound() && J.cols() != model()->nv())
        return make_error(ErrorCode::invalid_argument, "external task: J must have nv columns");
    J_ = J;
    e_ = e;
    return resize_rows(J_.rows());
}

void ExternalTask::clear() {
    J_.resize(0, J_.cols());
    e_.resize(0);
    (void)resize_rows(0);
}

Status ExternalTask::on_bind(const Model& model) {
    if (J_.rows() > 0 && J_.cols() != model.nv())
        return make_error(ErrorCode::invalid_argument, "external task: J must have nv columns");
    return resize_rows(J_.rows());
}

Status ExternalTask::compute(const Tick&, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) {
    if (J_.rows() != J.rows()) return make_error(ErrorCode::internal, "external task: row count changed");
    if (J_.rows() == 0) return {};
    J = J_;
    e = e_;
    return {};
}

}  // namespace kstone
