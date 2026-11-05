#include <pinocchio/algorithm/joint-configuration.hpp>

#include "impl.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_set>
#include <utility>

namespace kstone {

namespace {

JointType classify(const pinocchio::JointModel& joint) {
    const std::string name = joint.shortname();
    if (name == "JointModelRX" || name == "JointModelRY" || name == "JointModelRZ" ||
        name == "JointModelRevoluteUnaligned")
        return JointType::revolute;
    if (name == "JointModelRUBX" || name == "JointModelRUBY" || name == "JointModelRUBZ" ||
        name == "JointModelRevoluteUnboundedUnaligned")
        return JointType::continuous;
    if (name == "JointModelPX" || name == "JointModelPY" || name == "JointModelPZ" ||
        name == "JointModelPrismaticUnaligned")
        return JointType::prismatic;
    if (name == "JointModelPlanar") return JointType::planar;
    if (name == "JointModelFreeFlyer") return JointType::free_flyer;
    if (name == "JointModelSpherical") return JointType::spherical;
    if (name == "JointModelTranslation") return JointType::translation;
    if (name == "JointModelMimic") return JointType::mimic;
    return JointType::composite;
}

FrameKind classify(pinocchio::FrameType type) {
    switch (type) {
        case pinocchio::OP_FRAME:
            return FrameKind::operational;
        case pinocchio::JOINT:
            return FrameKind::joint;
        case pinocchio::FIXED_JOINT:
            return FrameKind::fixed_joint;
        case pinocchio::SENSOR:
            return FrameKind::sensor;
        case pinocchio::BODY:
        default:
            return FrameKind::body;
    }
}

Error size_error(const char* what, Index got, Index expected) {
    return make_error(ErrorCode::invalid_argument, std::string(what) + " has size " + std::to_string(got) +
                                                       ", expected " + std::to_string(expected));
}

// Pinocchio uses +-max() as "no limit"; only a finite, non-degenerate range counts.
bool is_limited(double lower, double upper) {
    constexpr double big = 1e20;
    return std::isfinite(lower) && std::isfinite(upper) && lower > -big && upper < big &&
           upper > lower + 1e-6;
}

}  // namespace

const char* to_string(JointType type) noexcept {
    switch (type) {
        case JointType::revolute:
            return "revolute";
        case JointType::continuous:
            return "continuous";
        case JointType::prismatic:
            return "prismatic";
        case JointType::planar:
            return "planar";
        case JointType::free_flyer:
            return "free_flyer";
        case JointType::spherical:
            return "spherical";
        case JointType::translation:
            return "translation";
        case JointType::mimic:
            return "mimic";
        case JointType::composite:
            return "composite";
        case JointType::universe:
            return "universe";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------------------------
// construction

std::optional<pinocchio::JointModel> root_joint_model(RootJoint root) {
    switch (root) {
        case RootJoint::free_flyer:
            return pinocchio::JointModel(pinocchio::JointModelFreeFlyer());
        case RootJoint::fixed:
            break;
    }
    return std::nullopt;
}

Result<ModelPtr> make_model(pinocchio::Model model, std::vector<MimicRelation> mimics) {
    auto impl = std::make_unique<Model::Impl>();
    impl->pin = std::move(model);
    const pinocchio::Model& pin = impl->pin;
    const auto njoints = static_cast<std::size_t>(pin.njoints);
    if (njoints == 0) return make_error(ErrorCode::invalid_argument, "model has no universe joint");

    impl->joints.reserve(njoints);
    impl->joints.push_back(JointInfo{pin.names[0], JointType::universe, 0, 0, 0, 0, 0, 0});
    for (std::size_t j = 1; j < njoints; ++j) {
        const auto& jm = pin.joints[j];
        impl->joints.push_back(JointInfo{pin.names[j], classify(jm), j, pin.parents[j], jm.idx_q(), jm.nq(),
                                         jm.idx_v(), jm.nv()});
        impl->joint_by_name.emplace(pin.names[j], j);
    }
    impl->frames.reserve(pin.frames.size());
    for (std::size_t f = 0; f < pin.frames.size(); ++f) {
        const auto& frame = pin.frames[f];
        impl->frames.push_back(FrameInfo{frame.name, classify(frame.type), f, frame.parentJoint});
        impl->frame_by_name.emplace(frame.name, f);
    }

    // Root: a free-flyer joint directly under the universe, if it is the only child.
    if (njoints > 1 && pin.parents[1] == 0) {
        const JointType t = impl->joints[1].type;
        bool sole_child = true;
        for (std::size_t j = 2; j < njoints; ++j) sole_child = sole_child && pin.parents[j] != 0;
        if (sole_child && t == JointType::free_flyer) impl->root = RootJoint::free_flyer;
    }

    impl->velocity_owner.assign(static_cast<std::size_t>(pin.nv), 0);
    for (const auto& joint : impl->joints)
        for (Index k = 0; k < joint.nv; ++k)
            impl->velocity_owner[static_cast<std::size_t>(joint.idx_v + k)] = joint.id;

    impl->subtree_mass.resize(njoints);
    for (std::size_t j = 0; j < njoints; ++j) impl->subtree_mass[j] = pin.inertias[j].mass();
    for (std::size_t j = njoints - 1; j > 0; --j) impl->subtree_mass[pin.parents[j]] += impl->subtree_mass[j];

    impl->has_limit.assign(njoints, false);
    for (const auto& joint : impl->joints) {
        if (joint.nq != 1) continue;
        const double lo = pin.lowerPositionLimit[joint.idx_q], hi = pin.upperPositionLimit[joint.idx_q];
        if (std::isfinite(lo) && std::isfinite(hi) && lo > hi)
            return make_error(ErrorCode::invalid_argument, "joint " + joint.name + ": lower position limit " +
                                                               std::to_string(lo) + " > upper " +
                                                               std::to_string(hi));
        impl->has_limit[joint.id] =
            is_limited(pin.lowerPositionLimit[joint.idx_q], pin.upperPositionLimit[joint.idx_q]);
    }
#if PINOCCHIO_VERSION_AT_LEAST(4, 0, 0)
    impl->velocity_limit = pin.upperVelocityLimit;
    impl->effort_limit = pin.upperEffortLimit;
#else
    impl->velocity_limit = pin.velocityLimit;
    impl->effort_limit = pin.effortLimit;
#endif

    std::unordered_set<std::string> followers;
    for (const auto& m : mimics) {
        if (!impl->joint_by_name.count(m.follower) || !impl->joint_by_name.count(m.leader))
            return make_error(ErrorCode::invalid_argument,
                              "mimic relation " + m.follower + " -> " + m.leader + " names an unknown joint");
        if (m.follower == m.leader)
            return make_error(ErrorCode::invalid_argument, "joint " + m.follower + " mimics itself");
        if (!followers.insert(m.follower).second)
            return make_error(ErrorCode::invalid_argument,
                              "joint " + m.follower + " has several mimic relations");
    }
    impl->mimics = std::move(mimics);
    return ModelFactory::create(std::move(impl));
}

const pinocchio::Model& pinocchio_model(const Model& model) noexcept { return model.impl().pin; }

Model::Model(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Model::~Model() = default;

Result<ModelPtr> Model::with_limits(const std::vector<JointLimitOverride>& overrides) const {
    auto impl = std::make_unique<Impl>(*impl_);
    pinocchio::Model& pin = impl->pin;
    auto fail = [](const std::string& joint, const std::string& msg) {
        return make_error(ErrorCode::invalid_argument, "joint " + joint + ": " + msg);
    };
    std::unordered_set<std::string> seen;
    for (const auto& o : overrides) {
        auto it = impl->joint_by_name.find(o.joint);
        if (it == impl->joint_by_name.end()) return fail(o.joint, "unknown joint");
        if (!seen.insert(o.joint).second) return fail(o.joint, "overridden twice");
        const JointInfo& info = impl->joints[it->second];
        if (info.nv == 0) return fail(o.joint, "has no own velocity (coupled mimic or fixed)");

        if (o.lower || o.upper) {
            if (info.type != JointType::revolute && info.type != JointType::prismatic)
                return fail(o.joint, std::string("position limits need a revolute/prismatic joint, got ") +
                                         to_string(info.type));
            double& lo = pin.lowerPositionLimit[info.idx_q];
            double& hi = pin.upperPositionLimit[info.idx_q];
            for (const auto& [value, name] : {std::pair{o.lower, "lower"}, std::pair{o.upper, "upper"}})
                if (value && !std::isfinite(*value))
                    return fail(o.joint, std::string(name) + " limit is not finite");
            if (o.lower) lo = *o.lower;
            if (o.upper) hi = *o.upper;
            if (!is_limited(lo, hi)) return fail(o.joint, "position range is empty or unbounded");
            impl->has_limit[info.id] = true;
        }
        if (o.velocity) {
            const double v = *o.velocity;
            if (!std::isfinite(v) || v <= 0) return fail(o.joint, "velocity limit must be finite and > 0");
            impl->velocity_limit.segment(info.idx_v, info.nv).setConstant(v);
        }
    }
#if PINOCCHIO_VERSION_AT_LEAST(4, 0, 0)
    pin.upperVelocityLimit = impl->velocity_limit;
#else
    pin.velocityLimit = impl->velocity_limit;
#endif
    return ModelFactory::create(std::move(impl));
}

// ---------------------------------------------------------------------------------------------
// basics / topology

const std::string& Model::name() const noexcept { return impl_->pin.name; }
Index Model::nq() const noexcept { return impl_->pin.nq; }
Index Model::nv() const noexcept { return impl_->pin.nv; }
RootJoint Model::root() const noexcept { return impl_->root; }
const ModelSource& Model::source() const noexcept { return impl_->source; }
const std::vector<JointInfo>& Model::joints() const noexcept { return impl_->joints; }
const std::vector<FrameInfo>& Model::frames() const noexcept { return impl_->frames; }
const std::vector<MimicRelation>& Model::mimics() const noexcept { return impl_->mimics; }
const std::vector<JointId>& Model::velocity_owner() const noexcept { return impl_->velocity_owner; }

Result<JointId> Model::joint_id(std::string_view name) const {
    if (name == impl_->joints[0].name) return JointId{0};
    auto it = impl_->joint_by_name.find(std::string(name));
    if (it == impl_->joint_by_name.end())
        return make_error(ErrorCode::not_found, "unknown joint '" + std::string(name) + "'");
    return it->second;
}

Result<FrameId> Model::frame_id(std::string_view name) const {
    auto it = impl_->frame_by_name.find(std::string(name));
    if (it == impl_->frame_by_name.end())
        return make_error(ErrorCode::not_found, "unknown frame '" + std::string(name) + "'");
    return it->second;
}

bool Model::is_ancestor(JointId ancestor, JointId joint) const noexcept {
    const auto n = impl_->joints.size();
    if (ancestor >= n || joint >= n) return false;
    for (;;) {
        if (joint == ancestor) return true;
        if (joint == 0) return false;
        joint = impl_->joints[joint].parent;
    }
}

Result<std::vector<JointId>> Model::joint_chain(JointId from, JointId to) const {
    const auto n = impl_->joints.size();
    if (from >= n || to >= n) return make_error(ErrorCode::out_of_range, "joint id out of range");
    if (!is_ancestor(from, to))
        return make_error(
            ErrorCode::invalid_argument,
            "joint " + impl_->joints[from].name + " is not an ancestor of " + impl_->joints[to].name);
    std::vector<JointId> chain;
    for (JointId j = to; j != from; j = impl_->joints[j].parent) chain.push_back(j);
    std::reverse(chain.begin(), chain.end());
    return chain;
}

bool Model::frame_in_subtree(FrameId frame, JointId root) const noexcept {
    return frame < impl_->frames.size() && is_ancestor(root, impl_->frames[frame].parent_joint);
}

Mask Model::velocity_mask(const std::vector<JointId>& joints) const {
    Mask mask = Mask::none(static_cast<std::size_t>(nv()));
    for (JointId j : joints) {
        if (j >= impl_->joints.size()) continue;
        const auto& info = impl_->joints[j];
        mask.set_range(static_cast<std::size_t>(info.idx_v), static_cast<std::size_t>(info.nv));
    }
    return mask;
}

Mask Model::subtree_velocity_mask(JointId root) const {
    Mask mask = Mask::none(static_cast<std::size_t>(nv()));
    for (const auto& info : impl_->joints)
        if (is_ancestor(root, info.id))
            mask.set_range(static_cast<std::size_t>(info.idx_v), static_cast<std::size_t>(info.nv));
    return mask;
}

// ---------------------------------------------------------------------------------------------
// limits / inertia

const Eigen::VectorXd& Model::lower_position_limit() const noexcept { return impl_->pin.lowerPositionLimit; }
const Eigen::VectorXd& Model::upper_position_limit() const noexcept { return impl_->pin.upperPositionLimit; }
const Eigen::VectorXd& Model::velocity_limit() const noexcept { return impl_->velocity_limit; }
const Eigen::VectorXd& Model::effort_limit() const noexcept { return impl_->effort_limit; }

bool Model::has_position_limit(JointId joint) const noexcept {
    return joint < impl_->has_limit.size() && impl_->has_limit[joint];
}

double Model::total_mass() const noexcept { return impl_->subtree_mass[0]; }

double Model::subtree_mass(JointId root) const noexcept {
    return root < impl_->subtree_mass.size() ? impl_->subtree_mass[root] : 0.0;
}

// ---------------------------------------------------------------------------------------------
// configuration space

Eigen::VectorXd Model::neutral() const { return pinocchio::neutral(impl_->pin); }

namespace {

// Unit-norm segment of a joint's q: (offset from idx_q, size), or size 0 if none.
std::pair<Index, Index> unit_segment(JointType type) {
    switch (type) {
        case JointType::free_flyer:
            return {3, 4};
        case JointType::spherical:
            return {0, 4};
        case JointType::planar:
            return {2, 2};
        case JointType::continuous:
            return {0, 2};
        default:
            return {0, 0};
    }
}

std::string unit_error(const JointInfo& joint, Index at, Index size, double norm, const char* hint) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.9g", norm);
    return "q: joint " + joint.name + " has a " + (size == 4 ? "quaternion" : "(cos, sin) pair") +
           " of norm " + buf + " at q[" + std::to_string(at) + "]" + hint;
}

}  // namespace

Status Model::validate(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    if (!q.allFinite()) return make_error(ErrorCode::invalid_argument, "q contains non-finite values");
    if (pinocchio::isNormalized(impl_->pin, q, 1e-8)) return {};
    for (const auto& joint : impl_->joints) {
        const auto [offset, size] = unit_segment(joint.type);
        if (size == 0) continue;
        const double norm = q.segment(joint.idx_q + offset, size).norm();
        if (std::abs(norm - 1.0) > 1e-8)
            return make_error(ErrorCode::invalid_argument,
                              unit_error(joint, joint.idx_q + offset, size, norm, "; see Model::normalize"));
    }
    return make_error(ErrorCode::invalid_argument, "q has non-normalized joint coordinates");
}

Status Model::normalize(Eigen::Ref<Eigen::VectorXd> q, double tolerance) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    if (!q.allFinite()) return make_error(ErrorCode::invalid_argument, "q contains non-finite values");
    if (!(tolerance >= 0.0))
        return make_error(ErrorCode::invalid_argument, "normalize: tolerance must be >= 0");
    for (const auto& joint : impl_->joints) {
        const auto [offset, size] = unit_segment(joint.type);
        if (size == 0) continue;
        const double norm = q.segment(joint.idx_q + offset, size).norm();
        if (!(std::abs(norm - 1.0) <= tolerance))
            return make_error(ErrorCode::invalid_argument, unit_error(joint, joint.idx_q + offset, size, norm,
                                                                      ", beyond the normalize tolerance"));
    }
    for (const auto& joint : impl_->joints) {
        const auto [offset, size] = unit_segment(joint.type);
        if (size > 0) q.segment(joint.idx_q + offset, size).normalize();
    }
    return {};
}

Status Model::check_position_limits(const Eigen::Ref<const Eigen::VectorXd>& q, double tolerance) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    const auto& pin = impl_->pin;
    for (const auto& joint : impl_->joints) {
        if (!impl_->has_limit[joint.id]) continue;
        const double value = q[joint.idx_q];
        if (value < pin.lowerPositionLimit[joint.idx_q] - tolerance ||
            value > pin.upperPositionLimit[joint.idx_q] + tolerance)
            return make_error(ErrorCode::out_of_range,
                              "joint " + joint.name + " position " + std::to_string(value) + " outside [" +
                                  std::to_string(pin.lowerPositionLimit[joint.idx_q]) + ", " +
                                  std::to_string(pin.upperPositionLimit[joint.idx_q]) + "]");
    }
    return {};
}

Status Model::integrate(const Eigen::Ref<const Eigen::VectorXd>& q,
                        const Eigen::Ref<const Eigen::VectorXd>& v, Eigen::Ref<Eigen::VectorXd> out) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    if (v.size() != nv()) return size_error("v", v.size(), nv());
    if (out.size() != nq()) return size_error("out", out.size(), nq());
    // Squared norms inside Pinocchio's exp maps overflow (and assert in debug) far below DBL_MAX.
    if (!v.allFinite() || (v.size() > 0 && v.cwiseAbs().maxCoeff() > 1e100))
        return make_error(ErrorCode::invalid_argument, "integrate: v must be finite with |v_i| <= 1e100");
    pinocchio::integrate(impl_->pin, q, v, out);
    return {};
}

Status Model::difference(const Eigen::Ref<const Eigen::VectorXd>& q0,
                         const Eigen::Ref<const Eigen::VectorXd>& q1, Eigen::Ref<Eigen::VectorXd> out) const {
    if (q0.size() != nq()) return size_error("q0", q0.size(), nq());
    if (q1.size() != nq()) return size_error("q1", q1.size(), nq());
    if (out.size() != nv()) return size_error("out", out.size(), nv());
    pinocchio::difference(impl_->pin, q0, q1, out);
    return {};
}

Result<double> Model::joint_position(const Eigen::Ref<const Eigen::VectorXd>& q, JointId joint) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    if (joint >= impl_->joints.size()) return make_error(ErrorCode::out_of_range, "joint id out of range");
    const auto& info = impl_->joints[joint];
    if (info.nv != 1)
        return make_error(ErrorCode::unsupported, "joint " + info.name + " is not a scalar joint");
    if (info.nq == 1) return q[info.idx_q];
    if (info.nq == 2) return std::atan2(q[info.idx_q + 1], q[info.idx_q]);
    return make_error(ErrorCode::unsupported, "joint " + info.name + " has no scalar position");
}

Status Model::set_joint_position(Eigen::Ref<Eigen::VectorXd> q, JointId joint, double value) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    if (joint >= impl_->joints.size()) return make_error(ErrorCode::out_of_range, "joint id out of range");
    const auto& info = impl_->joints[joint];
    if (info.nv != 1)
        return make_error(ErrorCode::unsupported, "joint " + info.name + " is not a scalar joint");
    if (info.nq == 1) {
        q[info.idx_q] = value;
    } else if (info.nq == 2) {
        q[info.idx_q] = std::cos(value);
        q[info.idx_q + 1] = std::sin(value);
    } else {
        return make_error(ErrorCode::unsupported, "joint " + info.name + " has no scalar position");
    }
    return {};
}

Result<Eigen::Isometry3d> Model::root_pose(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (impl_->root == RootJoint::fixed) return pose;
    const Index i = impl_->joints[1].idx_q;
    pose.translation() = q.segment<3>(i);
    // Stored as (x, y, z, w).
    Eigen::Quaterniond rot(q[i + 6], q[i + 3], q[i + 4], q[i + 5]);
    if (rot.norm() < 1e-12) return make_error(ErrorCode::invalid_argument, "root quaternion is zero");
    pose.linear() = rot.normalized().toRotationMatrix();
    return pose;
}

Status Model::set_root_pose(Eigen::Ref<Eigen::VectorXd> q, const Eigen::Isometry3d& pose) const {
    if (q.size() != nq()) return size_error("q", q.size(), nq());
    if (impl_->root == RootJoint::fixed) return make_error(ErrorCode::unsupported, "model has a fixed base");
    const Index i = impl_->joints[1].idx_q;
    const Eigen::Quaterniond rot(pose.linear());
    q.segment<3>(i) = pose.translation();
    q.segment<4>(i + 3) = rot.normalized().coeffs();  // (x, y, z, w)
    return {};
}

// ---------------------------------------------------------------------------------------------
// NameMap

Result<NameMap> NameMap::create(const ModelPtr& model, const std::vector<std::string>& names) {
    if (!model) return make_error(ErrorCode::invalid_argument, "null model");
    NameMap map;
    map.model_ = model;
    map.names_ = names;
    std::unordered_set<std::string> seen;
    for (const auto& name : names) {
        if (!seen.insert(name).second)
            return make_error(ErrorCode::invalid_argument, "duplicate joint '" + name + "'");
        auto id = model->joint_id(name);
        if (!id) return std::move(id).error();
        const auto& info = model->joints()[*id];
        if (info.nv != 1 || (info.nq != 1 && info.nq != 2))
            return make_error(ErrorCode::unsupported, "joint '" + name + "' is not a scalar joint");
        map.joints_.push_back(*id);
        map.idx_v_.push_back(info.idx_v);
    }
    return map;
}

Status NameMap::scatter_positions(const Eigen::Ref<const Eigen::VectorXd>& external,
                                  Eigen::Ref<Eigen::VectorXd> q) const {
    if (static_cast<std::size_t>(external.size()) != size())
        return size_error("external", external.size(), static_cast<Index>(size()));
    if (q.size() != model_->nq()) return size_error("q", q.size(), model_->nq());
    for (std::size_t k = 0; k < joints_.size(); ++k) {
        const auto& info = model_->joints()[joints_[k]];
        const double value = external[static_cast<Index>(k)];
        if (info.nq == 1) {
            q[info.idx_q] = value;
        } else {
            q[info.idx_q] = std::cos(value);
            q[info.idx_q + 1] = std::sin(value);
        }
    }
    return {};
}

Status NameMap::gather_positions(const Eigen::Ref<const Eigen::VectorXd>& q,
                                 Eigen::Ref<Eigen::VectorXd> external) const {
    if (static_cast<std::size_t>(external.size()) != size())
        return size_error("external", external.size(), static_cast<Index>(size()));
    if (q.size() != model_->nq()) return size_error("q", q.size(), model_->nq());
    for (std::size_t k = 0; k < joints_.size(); ++k) {
        const auto& info = model_->joints()[joints_[k]];
        external[static_cast<Index>(k)] =
            info.nq == 1 ? q[info.idx_q] : std::atan2(q[info.idx_q + 1], q[info.idx_q]);
    }
    return {};
}

Status NameMap::scatter_velocities(const Eigen::Ref<const Eigen::VectorXd>& external,
                                   Eigen::Ref<Eigen::VectorXd> v) const {
    if (static_cast<std::size_t>(external.size()) != size())
        return size_error("external", external.size(), static_cast<Index>(size()));
    if (v.size() != model_->nv()) return size_error("v", v.size(), model_->nv());
    for (std::size_t k = 0; k < idx_v_.size(); ++k) v[idx_v_[k]] = external[static_cast<Index>(k)];
    return {};
}

Status NameMap::gather_velocities(const Eigen::Ref<const Eigen::VectorXd>& v,
                                  Eigen::Ref<Eigen::VectorXd> external) const {
    if (static_cast<std::size_t>(external.size()) != size())
        return size_error("external", external.size(), static_cast<Index>(size()));
    if (v.size() != model_->nv()) return size_error("v", v.size(), model_->nv());
    for (std::size_t k = 0; k < idx_v_.size(); ++k) external[static_cast<Index>(k)] = v[idx_v_[k]];
    return {};
}

}  // namespace kstone
