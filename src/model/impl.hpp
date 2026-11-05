#pragma once

// Internal: shared between model sources. Not installed.

#include "kstone/model/model.hpp"
#include "kstone/model/pinocchio.hpp"
#include "kstone/model/workspace.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace kstone {

struct Model::Impl {
    pinocchio::Model pin;
    RootJoint root{RootJoint::fixed};
    std::vector<JointInfo> joints;
    std::vector<FrameInfo> frames;
    std::vector<MimicRelation> mimics;
    std::vector<JointId> velocity_owner;
    std::vector<double> subtree_mass;
    std::vector<bool> has_limit;
    std::unordered_map<std::string, JointId> joint_by_name;
    std::unordered_map<std::string, FrameId> frame_by_name;  ///< first frame with a given name wins
    Eigen::VectorXd velocity_limit;
    Eigen::VectorXd effort_limit;
    ModelSource source;
};

struct Workspace::Impl {
    explicit Impl(const pinocchio::Model& model) : data(model), q(Eigen::VectorXd::Zero(model.nq)) {}
    pinocchio::Data data;
    Eigen::VectorXd q;
    Eigen::Vector3d com{Eigen::Vector3d::Zero()};  ///< whole-model CoM including base inertia
    double com_scale{1.0};                         ///< moving mass / total mass
};

struct ModelFactory {
    static ModelPtr create(std::unique_ptr<Model::Impl> impl) { return ModelPtr(new Model(std::move(impl))); }
    // Only before the model is shared.
    static void set_source(const Model& model, ModelSource source) {
        model.impl_->source = std::move(source);
    }
};

}  // namespace kstone
