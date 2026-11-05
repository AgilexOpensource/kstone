#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "impl.hpp"
#include <stdexcept>

namespace kstone {

namespace {

Error shape_error(const char* what, Index rows, Index cols, Index er, Index ec) {
    return make_error(ErrorCode::invalid_argument, std::string(what) + " is " + std::to_string(rows) + "x" +
                                                       std::to_string(cols) + ", expected " +
                                                       std::to_string(er) + "x" + std::to_string(ec));
}

void to_isometry(const pinocchio::SE3& placement, Eigen::Isometry3d& out) {
    out.setIdentity();
    out.linear() = placement.rotation();
    out.translation() = placement.translation();
}

pinocchio::ReferenceFrame to_pinocchio(ReferenceFrame ref) {
    switch (ref) {
        case ReferenceFrame::local:
            return pinocchio::LOCAL;
        case ReferenceFrame::world:
            return pinocchio::WORLD;
        case ReferenceFrame::local_world_aligned:
            break;
    }
    return pinocchio::LOCAL_WORLD_ALIGNED;
}

}  // namespace

pinocchio::Data& pinocchio_data(Workspace& workspace) noexcept { return workspace.impl().data; }
const pinocchio::Data& pinocchio_data(const Workspace& workspace) noexcept { return workspace.impl().data; }

Workspace::Workspace(ModelPtr model) : model_(std::move(model)) {
    if (!model_) throw std::invalid_argument("kstone::Workspace requires a model");
    impl_ = std::make_unique<Impl>(model_->impl().pin);
}

Workspace::~Workspace() = default;
Workspace::Workspace(Workspace&&) noexcept = default;
Workspace& Workspace::operator=(Workspace&&) noexcept = default;

const Eigen::VectorXd& Workspace::q() const noexcept { return impl_->q; }

Status Workspace::require_updated() const {
    if (!updated_) return make_error(ErrorCode::invalid_argument, "workspace has not been updated");
    return {};
}

Status Workspace::update(const Eigen::Ref<const Eigen::VectorXd>& q) {
    const auto& pin = model_->impl().pin;
    if (q.size() != pin.nq)
        return make_error(ErrorCode::invalid_argument,
                          "q has size " + std::to_string(q.size()) + ", expected " + std::to_string(pin.nq));
    pinocchio::computeJointJacobians(pin, impl_->data, q);
    pinocchio::updateFramePlacements(pin, impl_->data);
    impl_->q = q;
    updated_ = true;
    com_ready_ = false;
    return {};
}

Status Workspace::frame_pose(FrameId frame, Eigen::Isometry3d& out) const {
    if (auto s = require_updated(); !s) return s;
    if (frame >= model_->frames().size()) return make_error(ErrorCode::out_of_range, "frame id out of range");
    to_isometry(impl_->data.oMf[frame], out);
    return {};
}

Status Workspace::joint_pose(JointId joint, Eigen::Isometry3d& out) const {
    if (auto s = require_updated(); !s) return s;
    if (joint >= model_->joints().size()) return make_error(ErrorCode::out_of_range, "joint id out of range");
    to_isometry(impl_->data.oMi[joint], out);
    return {};
}

Status Workspace::frame_jacobian(FrameId frame, ReferenceFrame ref, Eigen::Ref<Eigen::MatrixXd> out) {
    if (auto s = require_updated(); !s) return s;
    if (frame >= model_->frames().size()) return make_error(ErrorCode::out_of_range, "frame id out of range");
    const Index nv = model_->nv();
    if (out.rows() != 6 || out.cols() != nv) return shape_error("jacobian", out.rows(), out.cols(), 6, nv);
    out.setZero();
    // Frames welded to the universe have a zero Jacobian; Pinocchio misindexes them when nv == 0.
    if (model_->impl().pin.frames[frame].parentJoint == 0) return {};
    pinocchio::getFrameJacobian(model_->impl().pin, impl_->data, frame, to_pinocchio(ref), out);
    return {};
}

Status Workspace::ensure_com() {
    if (auto s = require_updated(); !s) return s;
    if (com_ready_) return {};
    const auto& pin = model_->impl().pin;
    auto& data = impl_->data;
    const double total = model_->total_mass();
    if (!(total > 0.0)) return make_error(ErrorCode::invalid_argument, "model has zero total mass");
    pinocchio::jacobianCenterOfMass(pin, data, true);
    // Pinocchio's whole-model CoM ignores inertia welded to the universe; fold it back in.
    const auto& base = pin.inertias[0];
    const double moving = total - base.mass();
    impl_->com = (moving * data.com[0] + base.mass() * base.lever()) / total;
    impl_->com_scale = moving / total;
    com_ready_ = true;
    return {};
}

Status Workspace::center_of_mass(Eigen::Vector3d& out) {
    if (auto s = ensure_com(); !s) return s;
    out = impl_->com;
    return {};
}

Status Workspace::center_of_mass_jacobian(Eigen::Ref<Eigen::MatrixXd> out) {
    const Index nv = model_->nv();
    if (out.rows() != 3 || out.cols() != nv) return shape_error("jacobian", out.rows(), out.cols(), 3, nv);
    if (auto s = ensure_com(); !s) return s;
    out = impl_->com_scale * impl_->data.Jcom;
    return {};
}

Status Workspace::subtree_center_of_mass(JointId root, Eigen::Vector3d& out) {
    if (root == 0) return center_of_mass(out);
    if (root >= model_->joints().size()) return make_error(ErrorCode::out_of_range, "joint id out of range");
    if (!(model_->subtree_mass(root) > 0.0))
        return make_error(ErrorCode::invalid_argument,
                          "subtree of " + model_->joints()[root].name + " has zero mass");
    if (auto s = ensure_com(); !s) return s;
    out = impl_->data.com[root];
    return {};
}

Status Workspace::subtree_center_of_mass_jacobian(JointId root, Eigen::Ref<Eigen::MatrixXd> out) {
    if (root == 0) return center_of_mass_jacobian(out);
    if (root >= model_->joints().size()) return make_error(ErrorCode::out_of_range, "joint id out of range");
    const Index nv = model_->nv();
    if (out.rows() != 3 || out.cols() != nv) return shape_error("jacobian", out.rows(), out.cols(), 3, nv);
    if (!(model_->subtree_mass(root) > 0.0))
        return make_error(ErrorCode::invalid_argument,
                          "subtree of " + model_->joints()[root].name + " has zero mass");
    if (auto s = ensure_com(); !s) return s;
    out.setZero();
    pinocchio::getJacobianSubtreeCenterOfMass(model_->impl().pin, impl_->data, root, out);
    return {};
}

}  // namespace kstone
