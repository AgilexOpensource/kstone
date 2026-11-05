#include "kstone/control/constraints.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <limits>

namespace kstone {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

}  // namespace

// ---- JointVelocityConstraint ---------------------------------------------------------------

JointVelocityConstraint::JointVelocityConstraint(std::vector<std::string> joints, Eigen::VectorXd limits)
    : Constraint(0), names_(std::move(joints)), limits_(std::move(limits)) {}

Status JointVelocityConstraint::on_bind(const Model& model) {
    indices_.clear();
    if (names_.empty()) {
        for (const JointInfo& j : model.joints())
            if (j.id != 0 && j.nv == 1) indices_.push_back(j.idx_v);
    } else {
        for (const std::string& name : names_) {
            Result<JointId> id = model.joint_id(name);
            if (!id) return id.error();
            const JointInfo& j = model.joints()[*id];
            if (j.nv != 1)
                return make_error(ErrorCode::invalid_argument,
                                  "joint velocity: joint '" + name + "' is not scalar");
            indices_.push_back(j.idx_v);
        }
    }
    const auto k = static_cast<Index>(indices_.size());
    if (limits_.size() == 1 && k != 1) limits_ = Eigen::VectorXd::Constant(k, limits_[0]);
    if (limits_.size() != k)
        return make_error(ErrorCode::invalid_argument, "joint velocity: one limit or one per joint required");
    if (limits_.array().isNaN().any() || (limits_.array() < 0).any())
        return make_error(ErrorCode::invalid_argument, "joint velocity: limits must be >= 0");
    return {};
}

Status JointVelocityConstraint::set_limits(Eigen::VectorXd limits) {
    if (limits.array().isNaN().any() || (limits.array() < 0).any())
        return make_error(ErrorCode::invalid_argument, "joint velocity: limits must be >= 0");
    if (!indices_.empty()) {
        const auto k = static_cast<Index>(indices_.size());
        if (limits.size() == 1 && k != 1) limits = Eigen::VectorXd::Constant(k, limits[0]);
        if (limits.size() != k)
            return make_error(ErrorCode::invalid_argument,
                              "joint velocity: one limit or one per joint required");
    }
    limits_ = std::move(limits);
    return {};
}

Status JointVelocityConstraint::compute(const Tick&, Eigen::Ref<Eigen::MatrixXd>, Eigen::Ref<Eigen::VectorXd>,
                                        Eigen::Ref<Eigen::VectorXd>) {
    return {};
}

void JointVelocityConstraint::tighten_bounds(const Tick&, Eigen::Ref<Eigen::VectorXd> lb,
                                             Eigen::Ref<Eigen::VectorXd> ub) const {
    for (std::size_t i = 0; i < indices_.size(); ++i) {
        const Index c = indices_[i];
        const double limit = limits_[static_cast<Index>(i)];
        lb[c] = std::max(lb[c], -limit);
        ub[c] = std::min(ub[c], limit);
    }
}

// ---- SupportPolygon ------------------------------------------------------------------------

Result<SupportPolygon> SupportPolygon::create(std::string frame, std::vector<Eigen::Vector2d> vertices,
                                              std::string subtree) {
    const std::size_t n = vertices.size();
    if (n < 3)
        return make_error(ErrorCode::invalid_argument, "support polygon: at least 3 vertices required");
    for (const auto& v : vertices)
        if (!v.allFinite())
            return make_error(ErrorCode::invalid_argument, "support polygon: non-finite vertex");
    double area = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& a = vertices[i];
        const auto& b = vertices[(i + 1) % n];
        area += a.x() * b.y() - b.x() * a.y();
    }
    if (area < 0) std::reverse(vertices.begin(), vertices.end());
    SupportPolygon p;
    p.frame_ = std::move(frame);
    p.subtree_ = std::move(subtree);
    const auto edges = static_cast<Index>(n);
    p.normals_.resize(edges, 2);
    p.offsets_.resize(edges);
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Vector2d& a = vertices[i];
        const Eigen::Vector2d e = vertices[(i + 1) % n] - a;
        const Eigen::Vector2d f = vertices[(i + 2) % n] - vertices[(i + 1) % n];
        if (e.norm() < 1e-9)
            return make_error(ErrorCode::invalid_argument, "support polygon: degenerate edge");
        if (e.x() * f.y() - e.y() * f.x() <= 1e-12)
            return make_error(ErrorCode::invalid_argument,
                              "support polygon: vertices must be strictly convex");
        const Eigen::Vector2d normal = Eigen::Vector2d(e.y(), -e.x()).normalized();
        const auto r = static_cast<Index>(i);
        p.normals_.row(r) = normal.transpose();
        p.offsets_[r] = normal.dot(a);
    }
    // Largest inscribed circle: max r s.t. n_i . c + r <= offset_i, by enumerating active triples.
    double best = 0;
    for (Index i = 0; i < edges; ++i)
        for (Index j = i + 1; j < edges; ++j)
            for (Index k = j + 1; k < edges; ++k) {
                Eigen::Matrix3d M;
                M << p.normals_(i, 0), p.normals_(i, 1), 1, p.normals_(j, 0), p.normals_(j, 1), 1,
                    p.normals_(k, 0), p.normals_(k, 1), 1;
                if (std::abs(M.determinant()) < 1e-12) continue;
                const Eigen::Vector3d sol =
                    M.inverse() * Eigen::Vector3d(p.offsets_[i], p.offsets_[j], p.offsets_[k]);
                const Eigen::Vector2d c = sol.head<2>();
                if (((p.normals_ * c + Eigen::VectorXd::Constant(edges, sol[2]) - p.offsets_).array() > 1e-9)
                        .any())
                    continue;
                best = std::max(best, sol[2]);
            }
    p.inset_limit_ = best;
    return p;
}

Status SupportPolygon::bind(const Model& model, const std::vector<Payload>& payloads) {
    Result<FrameId> frame = model.frame_id(frame_);
    if (!frame) return frame.error();
    JointId root = 0;
    if (!subtree_.empty()) {
        Result<JointId> id = model.joint_id(subtree_);
        if (!id) return id.error();
        root = *id;
    }
    std::vector<FrameId> frames;
    std::vector<char> used;
    for (const Payload& load : payloads) {
        Result<FrameId> id = model.frame_id(load.frame);
        if (!id) return id.error();
        frames.push_back(*id);
        used.push_back(subtree_.empty() || model.frame_in_subtree(*id, root));
    }
    model_ = &model;
    frame_id_ = *frame;
    root_ = root;
    payload_frames_ = std::move(frames);
    payload_used_ = std::move(used);
    return {};
}

Status SupportPolygon::evaluate(Workspace& workspace, const std::vector<Payload>& payloads,
                                Evaluation& out) const {
    const ModelPtr& model = workspace.model();
    if (model_ != model.get() || payloads.size() != payload_frames_.size())
        return make_error(ErrorCode::invalid_argument,
                          "support polygon: not bound to this model and payloads");
    const Index nv = model->nv();
    const JointId root = root_;
    Eigen::Vector3d com;
    Eigen::MatrixXd& Jc = out.com_jacobian;
    Jc.resize(3, nv);
    double mass = 0;
    if (subtree_.empty()) {
        if (Status s = workspace.center_of_mass(com); !s) return s;
        if (Status s = workspace.center_of_mass_jacobian(Jc); !s) return s;
        mass = model->total_mass();
    } else {
        if (Status s = workspace.subtree_center_of_mass(root, com); !s) return s;
        if (Status s = workspace.subtree_center_of_mass_jacobian(root, Jc); !s) return s;
        mass = model->subtree_mass(root);
    }
    Eigen::MatrixXd& J = out.frame_jacobian;
    J.resize(6, nv);
    if (!payloads.empty()) {
        Eigen::Vector3d sum = mass * com;
        Eigen::MatrixXd& Jsum = out.weighted;
        Jsum.resize(3, nv);
        Jsum = mass * Jc;
        double total = mass;
        for (std::size_t i = 0; i < payloads.size(); ++i) {
            const Payload& load = payloads[i];
            if (!std::isfinite(load.mass) || load.mass < 0 || !load.offset.allFinite())
                return make_error(ErrorCode::invalid_argument, "support polygon: invalid payload");
            if (load.mass == 0 || !payload_used_[i]) continue;
            const FrameId id = payload_frames_[i];
            Eigen::Isometry3d pose;
            if (Status s = workspace.frame_pose(id, pose); !s) return s;
            if (Status s = workspace.frame_jacobian(id, ReferenceFrame::local, J); !s) return s;
            const Eigen::Matrix3d R = pose.rotation();
            sum += load.mass * (pose.translation() + R * load.offset);
            Jsum.noalias() += load.mass * R * (J.topRows<3>() - skew(load.offset) * J.bottomRows<3>());
            total += load.mass;
        }
        if (!(total > 0)) return make_error(ErrorCode::invalid_argument, "support polygon: zero total mass");
        com = sum / total;
        Jc = Jsum / total;
    }
    Eigen::Isometry3d pose;
    if (Status s = workspace.frame_pose(frame_id_, pose); !s) return s;
    if (Status s = workspace.frame_jacobian(frame_id_, ReferenceFrame::local, J); !s) return s;
    const Eigen::Matrix3d Rt = pose.rotation().transpose();
    const Eigen::Vector3d relative = Rt * (com - pose.translation());
    Eigen::MatrixXd& full = out.full;
    full.resize(3, nv);
    full.noalias() = Rt * Jc;
    full -= J.topRows<3>();
    full.noalias() += skew(relative) * J.bottomRows<3>();
    out.world_com = com;
    out.frame_pose = pose;
    out.com = relative.head<2>();
    out.jacobian = full.topRows<2>();
    out.slack = offsets_;
    out.slack.noalias() -= normals_ * out.com;
    return {};
}

void SupportPolygon::inset_vertices(double inset, Eigen::Ref<Eigen::Matrix2Xd> out) const noexcept {
    const Index n = edges();
    // Vertex i joins edges i - 1 and i.
    for (Index i = 0; i < n; ++i) {
        const Index j = (i + n - 1) % n;
        Eigen::Matrix2d M;
        M.row(0) = normals_.row(j);
        M.row(1) = normals_.row(i);
        out.col(i) = M.inverse() * Eigen::Vector2d(offsets_[j] - inset, offsets_[i] - inset);
    }
}

// ---- CenterOfMassConstraint ----------------------------------------------------------------

CenterOfMassConstraint::CenterOfMassConstraint(SupportPolygon polygon)
    : Constraint(polygon.edges()), polygon_(std::move(polygon)) {}

Status CenterOfMassConstraint::set_margin(double margin) {
    if (!std::isfinite(margin) || margin > polygon_.inset_limit() + 1e-12)
        return make_error(ErrorCode::invalid_argument, "center of mass: margin exceeds the inscribed radius");
    margin_ = margin;
    return {};
}

Status CenterOfMassConstraint::set_gain(double gain) {
    if (!std::isfinite(gain) || gain <= 0 || gain > 1)
        return make_error(ErrorCode::invalid_argument, "center of mass: gain must be in (0, 1]");
    gain_ = gain;
    return {};
}

Status CenterOfMassConstraint::set_recovery(double speed) {
    if (!std::isfinite(speed) || speed <= 0)
        return make_error(ErrorCode::invalid_argument, "center of mass: recovery speed must be > 0");
    recovery_ = speed;
    return {};
}

Status CenterOfMassConstraint::set_payloads(std::vector<Payload> payloads) {
    for (const Payload& p : payloads) {
        if (!std::isfinite(p.mass) || p.mass < 0 || !p.offset.allFinite())
            return make_error(ErrorCode::invalid_argument,
                              "center of mass: payload mass must be finite and >= 0");
    }
    if (bound())
        if (Status s = polygon_.bind(*model(), payloads); !s) return s;
    payloads_ = std::move(payloads);
    return {};
}

Status CenterOfMassConstraint::set_columns(Mask columns) {
    if (bound() && columns.size() != 0 && columns.size() != static_cast<std::size_t>(model()->nv()))
        return make_error(ErrorCode::invalid_argument, "center of mass: column mask size != nv");
    columns_ = std::move(columns);
    resize_rows(columns_.size() != 0 ? 2 * polygon_.edges() : polygon_.edges());
    return {};
}

Status CenterOfMassConstraint::on_bind(const Model& model) {
    if (columns_.size() != 0 && columns_.size() != static_cast<std::size_t>(model.nv()))
        return make_error(ErrorCode::invalid_argument, "center of mass: column mask size != nv");
    return polygon_.bind(model, payloads_);
}

Status CenterOfMassConstraint::compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A,
                                       Eigen::Ref<Eigen::VectorXd>, Eigen::Ref<Eigen::VectorXd> hi) {
    if (Status s = polygon_.evaluate(tick.workspace, payloads_, eval_); !s) return s;
    const Index n = polygon_.edges();
    A.topRows(n).noalias() = polygon_.normals() * eval_.jacobian;
    for (Index i = 0; i < n; ++i) hi[i] = std::max(gain_ * (eval_.slack[i] - margin_) / tick.dt, -recovery_);
    if (columns_.size() == 0) return {};
    // Recovery joints get the first block; the others the second, which may also spend what the recovery
    // joints can pull back next step, but never push out once they cannot.
    const Model& m = *model();
    const Eigen::VectorXd& q = tick.workspace.q();
    capacity_.setZero(n);
    for (Index j = 0; j < A.cols(); ++j) {
        if (!columns_.test(static_cast<std::size_t>(j))) {
            A.block(n, j, n, 1) = A.block(0, j, n, 1);
            A.block(0, j, n, 1).setZero();
            continue;
        }
        const JointInfo& joint =
            m.joints()[static_cast<std::size_t>(m.velocity_owner()[static_cast<std::size_t>(j)])];
        const double vmax = m.velocity_limit()[j];
        const bool scalar = joint.nq == 1 && joint.nv == 1 && m.has_position_limit(joint.id);
        const double down =
            scalar ? (q[joint.idx_q] - m.lower_position_limit()[joint.idx_q]) / tick.dt : kInf;
        const double up = scalar ? (m.upper_position_limit()[joint.idx_q] - q[joint.idx_q]) / tick.dt : kInf;
        for (Index i = 0; i < n; ++i) {
            const double a = A(i, j);
            const double room = std::min(vmax, std::max(a > 0 ? down : up, 0.0));
            if (std::isfinite(room))
                capacity_[i] += std::abs(a) * room;
            else if (a != 0)
                capacity_[i] = kInf;
        }
    }
    for (Index i = 0; i < n; ++i)
        hi[n + i] =
            std::max(gain_ * (eval_.slack[i] - margin_) / tick.dt + std::min(capacity_[i], recovery_), 0.0);
    return {};
}

// ---- ExternalConstraint --------------------------------------------------------------------

ExternalConstraint::ExternalConstraint() : Constraint(0) {}

Status ExternalConstraint::set(const Eigen::Ref<const Eigen::MatrixXd>& A,
                               const Eigen::Ref<const Eigen::VectorXd>& lo,
                               const Eigen::Ref<const Eigen::VectorXd>& hi) {
    if (lo.size() != A.rows() || hi.size() != A.rows())
        return make_error(ErrorCode::invalid_argument,
                          "external constraint: lo / hi must have one entry per row");
    if (!A.allFinite() || lo.array().isNaN().any() || hi.array().isNaN().any() ||
        (lo.array() > hi.array()).any())
        return make_error(ErrorCode::invalid_argument, "external constraint: non-finite rows or lo > hi");
    if (bound() && A.cols() != model()->nv())
        return make_error(ErrorCode::invalid_argument, "external constraint: A must have nv columns");
    A_ = A;
    lo_ = lo;
    hi_ = hi;
    resize_rows(A_.rows());
    return {};
}

void ExternalConstraint::clear() {
    A_.resize(0, A_.cols());
    lo_.resize(0);
    hi_.resize(0);
    resize_rows(0);
}

Status ExternalConstraint::on_bind(const Model& model) {
    if (A_.rows() > 0 && A_.cols() != model.nv())
        return make_error(ErrorCode::invalid_argument, "external constraint: A must have nv columns");
    return {};
}

Status ExternalConstraint::compute(const Tick&, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                                   Eigen::Ref<Eigen::VectorXd> hi) {
    if (A_.rows() == 0) return {};
    A = A_;
    lo = lo_;
    hi = hi_;
    return {};
}

}  // namespace kstone
