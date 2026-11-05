#include "kstone/module/builtins.hpp"

#include "builtin_schemas.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace kstone {

namespace {

Status velocity_mask_of(const Model& model, const std::vector<std::string>& names, Mask& out) {
    std::vector<JointId> ids;
    for (const std::string& name : names) {
        Result<JointId> id = model.joint_id(name);
        if (!id) return std::move(id).error();
        ids.push_back(*id);
    }
    out = model.velocity_mask(ids);
    return {};
}

bool rigid(const Eigen::Isometry3d& pose) {
    const Eigen::Matrix3d r = pose.linear();
    return r.allFinite() && pose.translation().allFinite() &&
           (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
           std::abs(r.determinant() - 1.0) < 1e-6;
}

Status resolve_scalar(const Model& model, const std::vector<std::string>& names, const std::string& who,
                      std::vector<JointInfo>& out) {
    for (const std::string& name : names) {
        Result<JointId> jid = model.joint_id(name);
        if (!jid) return std::move(jid).error();
        const JointInfo& info = model.joints()[*jid];
        if (info.type == JointType::mimic) {
            std::string leader;
            for (const MimicRelation& m : model.mimics())
                if (m.follower == name) leader = m.leader;
            return make_error(ErrorCode::invalid_argument,
                              who + ": " + name + " is a coupled mimic follower; use its leader " + leader);
        }
        if (info.nv != 1)
            return make_error(ErrorCode::invalid_argument, who + ": " + name + " is not scalar");
        out.push_back(info);
    }
    return {};
}

// Joints tied by a mimic relation cannot take separate targets.
Status reject_mimic_pairs(const Model& model, const std::vector<std::string>& names, const std::string& who) {
    auto root = [&](const std::string& n) {
        for (const MimicRelation& m : model.mimics())
            if (m.follower == n) return m.leader;
        return n;
    };
    for (std::size_t i = 0; i < names.size(); ++i)
        for (std::size_t k = i + 1; k < names.size(); ++k)
            if (names[i] != names[k] && root(names[i]) == root(names[k]))
                return make_error(ErrorCode::invalid_argument, who + ": " + names[i] + " and " + names[k] +
                                                                   " are coupled by a mimic; list only one");
    return {};
}

// A configured position outside the limits is usually a unit slip (degrees for radians).
Status check_within_limits(const Model& model, const std::vector<JointInfo>& joints,
                           const Eigen::Ref<const Eigen::VectorXd>& values, const std::string& who,
                           const char* what) {
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const JointInfo& j = joints[i];
        if (j.nq != 1) continue;
        const double lo = model.lower_position_limit()[j.idx_q], hi = model.upper_position_limit()[j.idx_q];
        const double x = values[static_cast<Index>(i)];
        if (x < lo - 1e-9 || x > hi + 1e-9) {
            std::ostringstream out;
            out << who << ": " << what << ' ' << x << " of " << j.name << " is outside [" << lo << ", " << hi
                << ']';
            return make_error(ErrorCode::invalid_argument, out.str());
        }
    }
    return {};
}

// Revolute-unbounded joints store (cos, sin).
double scalar_position(const JointInfo& j, const Eigen::Ref<const Eigen::VectorXd>& q) {
    return j.nq == 2 ? std::atan2(q[j.idx_q + 1], q[j.idx_q]) : q[j.idx_q];
}

Status configure_home(Homing& home, const HomeConfig& config, const std::string& who) {
    if (Status s = home.set_config(config); !s)
        return make_error(s.error().code, who + ": " + s.error().message);
    return {};
}

bool finite_positive(double x) { return std::isfinite(x) && x > 0.0; }

Status with_who(Status s, const std::string& who) {
    if (s) return s;
    return make_error(s.error().code, who + ": " + s.error().message);
}

Status not_configured(const std::string& who) {
    return make_error(ErrorCode::invalid_argument, who + ": not configured");
}
Status no_bounded_joints(const std::string& who) {
    return make_error(ErrorCode::invalid_argument, who + ": no bounded joints for joint_limits");
}

bool touches_motion(const ConfigNode& params) {
    return params.find("trajectory") || params.find("home_motion");
}

// Powers follow the units: velocity s, acceleration s^2, jerk s^3.
MotionLimits scaled(MotionLimits m, double s) {
    m.velocity *= s;
    m.acceleration *= s * s;
    m.deceleration *= s * s;
    m.reversal_deceleration *= s * s;
    m.jerk *= s * s * s;
    return m;
}

}  // namespace

// ---- frame_task --------------------------------------------------------------------------------------

FrameTaskModule::FrameTaskModule(std::string id, FrameTaskModuleConfig config)
    : Module(std::move(id)), config_(std::move(config)), defaults_(config_) {}

Status FrameTaskModule::set_target(const Eigen::Isometry3d& target) {
    if (!rigid(target))
        return make_error(ErrorCode::invalid_argument, "frame_task '" + id() + "': target is not rigid");
    if (Status s = check_unlocked(); !s) return s;
    if (home_.active())
        return make_error(ErrorCode::invalid_argument, "frame_task '" + id() + "': homing in progress");
    target_ = target;
    has_target_ = true;
    request_control();
    return {};
}

Status FrameTaskModule::read_port(std::size_t index, PortValue& out) const {
    if (index != 0) return Module::read_port(index, out);
    out.pose = target_;
    return {};
}

Status FrameTaskModule::write_port(std::size_t index, const PortValue& value) {
    if (index != 0) return Module::write_port(index, value);
    return set_target(value.pose);
}

Status FrameTaskModule::request_home() {
    if (!task_) return make_error(ErrorCode::invalid_argument, "frame_task '" + id() + "': not configured");
    if (!enabled()) return make_error(ErrorCode::invalid_argument, "frame_task '" + id() + "': disabled");
    if (Status s = check_unlocked(); !s) return s;
    home_.request();
    request_control();
    return {};
}

namespace {
Status apply_cost(FrameTask& task, const FrameTaskModuleConfig& c) {
    Eigen::Matrix<double, 6, 1> w;
    w << c.weight * c.position_weight, c.weight * c.orientation_weight;
    if (Status s = task.set_weight(w); !s) return s;
    if (Status s = task.set_gain(c.gain); !s) return s;
    return task.set_lm_damping(c.lm_damping);
}
}  // namespace

Status FrameTaskModule::tune(const ConfigNode& params) {
    const std::string who = "frame_task '" + id() + "'";
    if (!task_) return not_configured(who);
    if (home_.active() && touches_motion(params))
        return make_error(ErrorCode::invalid_argument,
                          who + ": trajectory and home_motion cannot change while homing");
    FrameTaskModuleConfig next = config_;
    if (Status s = builtin_schemas::frame_task().patch(params, next); !s) return s;
    if (Status s = next.trajectory.validate(); !s) return with_who(s, who);
    if (Status s = Homing::validate(next.home_motion); !s) return with_who(s, who);
    if (!finite_positive(next.position_tolerance) || !finite_positive(next.orientation_tolerance))
        return make_error(ErrorCode::invalid_argument, who + ": home tolerances must be finite and > 0");
    if (next.locked && next.joints.empty())
        return make_error(ErrorCode::invalid_argument, who + ": locking requires explicit joints");
    PoseTrajectoryConfig home = next.trajectory;
    home.translation = scaled(next.trajectory.translation, next.home_motion.velocity_scale);
    home.rotation = scaled(next.trajectory.rotation, next.home_motion.velocity_scale);
    if (Status s = home.validate(); !s) return with_who(s, who);

    if (Status s = apply_cost(*task_, next); !s) return s;
    if (params.find("trajectory"))
        if (Status s = trajectory_->tune(next.trajectory); !s) return s;
    if (Status s = home_.set_config(next.home_motion); !s) return s;
    home_trajectory_ = home;
    config_ = std::move(next);
    return {};
}

ConfigNode FrameTaskModule::tunables() const { return builtin_schemas::frame_task().values(config_); }
ConfigNode FrameTaskModule::tune_schema() const {
    return builtin_schemas::frame_task().tune_schema(defaults_);
}

Status FrameTaskModule::finish(HomeStatus status) {
    home_.finish(status);
    if (Status s = trajectory_->tune(config_.trajectory); !s) return s;
    if (status != HomeStatus::succeeded && trajectory_->initialized()) target_ = trajectory_->ramp_pose();
    has_target_ = true;
    return {};
}

Status FrameTaskModule::prepare(Cycle& cycle) {
    if (home_.update(cycle))
        if (Status s = finish(home_.status()); !s) return s;
    // Held without a target (e.g. since start): the port follows the observed pose; a running idle task holds its own.
    if (!running() && !has_target_ && !home_.active()) {
        Eigen::Isometry3d observed;
        if (observe(cycle.workspace, observed).ok()) target_ = observed;
    }
    home_.align(cycle, *model_, driven_);
    return {};
}

Claim FrameTaskModule::claim() const {
    if (home_.active()) return home_.claim(driven_);
    return {nullptr, 0, config_.on_held};
}

Status FrameTaskModule::on_disabled(const Cycle&) {
    return home_.active() ? finish(HomeStatus::aborted) : Status{};
}

Status FrameTaskModule::on_preempted(const Cycle&, const std::string&) {
    if (home_.active())
        if (Status s = finish(HomeStatus::aborted); !s) return s;
    // Idle: the next reset() holds the pose observed then.
    has_target_ = false;
    return {};
}

Status FrameTaskModule::configure(ModuleSetup& setup) {
    const Model& model = *setup.model();
    model_ = &model;
    Result<FrameId> frame = model.frame_id(config_.frame);
    if (!frame) return std::move(frame).error();
    frame_ = *frame;
    has_reference_ = !config_.reference.empty();
    if (has_reference_) {
        Result<FrameId> ref = model.frame_id(config_.reference);
        if (!ref) return std::move(ref).error();
        reference_ = *ref;
    }
    Mask driven;
    if (config_.joints.empty()) {
        Result<std::vector<JointId>> chain = model.joint_chain(0, model.frames()[frame_].parent_joint);
        if (!chain) return std::move(chain).error();
        driven = model.velocity_mask(*chain);
    } else if (Status s = velocity_mask_of(model, config_.joints, driven); !s) {
        return s;
    }
    const std::string who = "frame_task '" + id() + "'";
    if (!config_.joints.empty()) {
        const JointId fj = model.frames()[frame_].parent_joint;
        const JointId rj = has_reference_ ? model.frames()[reference_].parent_joint : JointId{0};
        bool moves = false;
        for (const std::string& name : config_.joints) {
            const JointId j = model.joint_id(name).value();
            moves = moves || model.is_ancestor(j, fj) != model.is_ancestor(j, rj);
        }
        if (!moves)
            return make_error(ErrorCode::invalid_argument, who + ": none of the joints moves frame '" +
                                                               config_.frame + "' relative to its reference");
    }
    if (Status s = configure_home(home_, config_.home_motion, who); !s) return s;
    if (!finite_positive(config_.position_tolerance) || !finite_positive(config_.orientation_tolerance))
        return make_error(ErrorCode::invalid_argument, who + ": home tolerances must be finite and > 0");
    if (config_.home && !rigid(*config_.home))
        return make_error(ErrorCode::invalid_argument, who + ": home is not rigid");
    if (config_.locked && config_.joints.empty())
        return make_error(ErrorCode::invalid_argument, who + ": locking requires explicit joints");
    Result<PoseTrajectory> trajectory = PoseTrajectory::create(config_.trajectory);
    if (!trajectory) return std::move(trajectory).error();
    trajectory_.emplace(std::move(*trajectory));
    home_trajectory_ = config_.trajectory;
    home_trajectory_.translation = scaled(config_.trajectory.translation, config_.home_motion.velocity_scale);
    home_trajectory_.rotation = scaled(config_.trajectory.rotation, config_.home_motion.velocity_scale);
    if (Status s = home_trajectory_.validate(); !s) return s;
    // Reads before the first step report the initial pose, not identity.
    Workspace at_initial(setup.model());
    if (Status s = at_initial.update(setup.initial()); !s) return s;
    if (Status s = observe(at_initial, target_); !s) return s;
    if (config_.home) {
        home_pose_ = *config_.home;
    } else {
        home_pose_ = target_;
        if (!has_reference_) {
            Result<Eigen::Isometry3d> root = model.root_pose(setup.initial());
            if (!root) return std::move(root).error();
            home_pose_ = root->inverse() * home_pose_;
            home_in_root_ = true;
        }
    }

    task_ = std::make_shared<FrameTask>(config_.frame, Eigen::Isometry3d::Identity());
    if (Status s = apply_cost(*task_, config_); !s) return s;
    // `joints` scopes what the task frees; the mode decides whether other free joints (e.g. the waist) help.
    if (!config_.joints.empty())
        if (Status s = task_->set_mask(driven); !s) return s;
    task_->set_mask_columns(false);
    task_->set_reference(config_.reference);
    task_->set_world_axes(config_.world_axes);
    Result<TaskHandle> handle = setup.add_task("frame", task_, config_.task);
    if (!handle) return std::move(handle).error();
    handle_ = *handle;
    lock_applied_ = false;
    PortInfo port;
    port.name = "target";
    port.type = PortType::pose;
    port.frame = config_.frame;
    port.reference = config_.reference;
    declare_port(std::move(port));
    declare_output({"planned", OutputType::point});
    declare_output({"lag", OutputType::number, 0, OutputRole::value, "m"});
    declare_output({"speed", OutputType::number, 0, OutputRole::value, "m/s"});
    declare_output({"angular_speed", OutputType::number, 0, OutputRole::value, "rad/s"});
    declare_output({"error", OutputType::number, 0, OutputRole::value, "m"});
    declare_output({"angular_error", OutputType::number, 0, OutputRole::value, "rad"});
    driven_ = driven;
    return setup.set_driven(std::move(driven));
}

Status FrameTaskModule::update_outputs(Workspace& at) {
    Eigen::Isometry3d ramp;
    // A held task's ramp is frozen at preemption; reset() restarts it from the observed pose.
    if (running() && trajectory_ && trajectory_->initialized()) {
        ramp = trajectory_->ramp_pose();
        speed_ = trajectory_->linear_velocity().norm();
        angular_speed_ = trajectory_->angular_velocity().norm();
    } else {
        if (Status s = observe(at, ramp); !s) return s;
        speed_ = angular_speed_ = 0.0;
    }
    lag_ = has_target_ ? (target_.translation() - ramp.translation()).norm() : 0.0;
    error_ = angular_error_ = 0.0;
    if (has_target_) {
        Eigen::Isometry3d actual;
        if (Status s = observe(at, actual); !s) return s;
        error_ = (target_.translation() - actual.translation()).norm();
        angular_error_ = Eigen::AngleAxisd(target_.linear().transpose() * actual.linear()).angle();
    }
    planned_ = ramp.translation();
    if (has_reference_) {
        Eigen::Isometry3d ref;
        if (Status s = at.frame_pose(reference_, ref); !s) return s;
        planned_ = ref * planned_;
    }
    return {};
}

Status FrameTaskModule::read_output(std::size_t index, OutputValue& out) const {
    switch (index) {
        case 0:
            out.points = planned_;
            return {};
        case 1:
            out.number = lag_;
            return {};
        case 2:
            out.number = speed_;
            return {};
        case 3:
            out.number = angular_speed_;
            return {};
        case 4:
            out.number = error_;
            return {};
        case 5:
            out.number = angular_error_;
            return {};
        default:
            return Module::read_output(index, out);
    }
}

Status FrameTaskModule::observe(const Workspace& workspace, Eigen::Isometry3d& pose) const {
    if (Status s = workspace.frame_pose(frame_, pose); !s) return s;
    if (!has_reference_) return {};
    Eigen::Isometry3d ref;
    if (Status s = workspace.frame_pose(reference_, ref); !s) return s;
    pose = ref.inverse() * pose;
    return {};
}

Status FrameTaskModule::reset(const Cycle& cycle) {
    Eigen::Isometry3d observed;
    if (Status s = observe(cycle.workspace, observed); !s) return s;
    if (cycle.rebase || !has_target_) target_ = observed;
    if (Status s = trajectory_->reset(observed); !s) return s;
    return task_->set_target(observed);
}

Status FrameTaskModule::contribute(const Cycle& cycle) {
    Eigen::Isometry3d observed;
    if (Status s = observe(cycle.workspace, observed); !s) return s;
    Eigen::Isometry3d home = home_pose_;
    if (home_in_root_ && home_.active()) {
        Result<Eigen::Isometry3d> root = model_->root_pose(cycle.q);
        if (!root) return std::move(root).error();
        home = *root * home_pose_;
    }
    if (home_.start()) {
        // Restart the ramp from the observed pose: a stale ramp left by an unreachable target bends the home path.
        if (Status s = trajectory_->reset(observed); !s) return s;
        if (Status s = trajectory_->tune(home_trajectory_); !s) return s;
        target_ = home;
        has_target_ = true;
    } else if (home_.status() == HomeStatus::executing) {
        if (home_in_root_) target_ = home;
        const Eigen::Isometry3d error = home.inverse() * observed;
        const double distance = error.translation().norm(), angle = Eigen::AngleAxisd(error.linear()).angle();
        const bool within = distance < config_.position_tolerance && angle < config_.orientation_tolerance;
        home_.progress(cycle.now, distance + angle, within);
        if (trajectory_->stopped() && within)
            if (Status s = finish(HomeStatus::succeeded); !s) return s;
    }
    const bool lock = config_.locked && !home_.active();
    set_task_locked(handle_, lock);
    if (lock_applied_ && !lock && !home_.active()) {
        target_ = observed;
        if (Status s = trajectory_->reset(observed); !s) return s;
    }
    lock_applied_ = lock;
    if (lock) {
        if (Status s = trajectory_->reset(observed); !s) return s;
        return task_->set_target(observed);
    }
    if (Status s = trajectory_->step(observed, target_, cycle.dt); !s) return s;
    return task_->set_target(trajectory_->ramp_pose());
}

// ---- joint_target ------------------------------------------------------------------------------------

JointTargetModule::JointTargetModule(std::string id, JointTargetModuleConfig config)
    : Module(std::move(id)), config_(std::move(config)), defaults_(config_) {}

Status JointTargetModule::configure(ModuleSetup& setup) {
    const Model& model = *setup.model();
    model_ = &model;
    const std::string who = "joint_target '" + id() + "'";
    const auto n = static_cast<Index>(config_.joints.size());
    if (n == 0) return make_error(ErrorCode::invalid_argument, who + ": no joints");
    if (!finite_positive(config_.tolerance))
        return make_error(ErrorCode::invalid_argument, who + ": tolerance must be finite and > 0");
    if (Status s = configure_home(home_, config_.home_motion, who); !s) return s;
    if (config_.home.size() != 0 && (config_.home.size() != n || !config_.home.allFinite()))
        return make_error(ErrorCode::invalid_argument, who + ": home needs one finite value per joint");
    if (Status s = resolve_scalar(model, config_.joints, who, joints_); !s) return s;
    if (Status s = reject_mimic_pairs(model, config_.joints, who); !s) return s;
    if (config_.home.size() != 0)
        if (Status s = check_within_limits(model, joints_, config_.home, who, "home"); !s) return s;
    for (const JointInfo& info : joints_) wrap_.push_back(info.type == JointType::continuous);
    if (Status s = velocity_mask_of(model, config_.joints, mask_); !s) return s;

    if (config_.velocity_limit.size() == 0) {
        limit_.resize(n);
        for (Index i = 0; i < n; ++i)
            limit_[i] = model.velocity_limit()[joints_[static_cast<std::size_t>(i)].idx_v];
    } else if (config_.velocity_limit.size() == n) {
        limit_ = config_.velocity_limit;
    } else {
        return make_error(ErrorCode::invalid_argument,
                          "joint_target '" + id() + "': velocity_limit size != joints");
    }
    for (Index i = 0; i < n; ++i)
        if (!std::isfinite(limit_[i]) || limit_[i] <= 0.0)
            return make_error(ErrorCode::invalid_argument, "joint_target '" + id() + "': velocity limit of " +
                                                               config_.joints[static_cast<std::size_t>(i)] +
                                                               " must be finite and > 0");
    // Same cap size in both configs, so switching copies without allocating.
    if (config_.trajectory.velocity_cap.size() == 0)
        config_.trajectory.velocity_cap.setConstant(n, std::numeric_limits<double>::infinity());
    Result<JointTrajectory> trajectory = JointTrajectory::create(n, config_.trajectory, wrap_);
    if (!trajectory) return std::move(trajectory).error();
    trajectory_.emplace(std::move(*trajectory));
    home_trajectory_ = config_.trajectory;
    home_trajectory_.trigger = JointTrigger::continuous;
    home_trajectory_.velocity_scale = 1.0;
    home_trajectory_.velocity_cap = config_.home_motion.velocity_scale * limit_;
    if (Status s = home_trajectory_.validate(n); !s) return s;
    home_pose_.resize(n);
    for (std::size_t i = 0; i < joints_.size(); ++i)
        home_pose_[static_cast<Index>(i)] = config_.home.size() != 0
                                                ? config_.home[static_cast<Index>(i)]
                                                : scalar_position(joints_[i], setup.initial());

    target_.resize(n);
    for (std::size_t i = 0; i < joints_.size(); ++i)
        target_[static_cast<Index>(i)] = scalar_position(joints_[i], setup.initial());
    measured_.setZero(n);
    measured_v_.setZero(n);
    task_ = std::make_shared<JointTask>(config_.joints, target_);
    if (Status s = task_->set_weight(config_.weight); !s) return s;
    if (Status s = task_->set_mask(mask_); !s) return s;
    if (Status s = task_->set_gain(config_.gain); !s) return s;
    TaskOptions options = config_.task;
    // Joint control outranks pose tasks that share these joints.
    if (!options.level) options.level = 0;
    Result<TaskHandle> handle = setup.add_task("joints", task_, options);
    if (!handle) return std::move(handle).error();
    handle_ = *handle;

    PortInfo port;
    port.name = "target";
    port.type = PortType::vector;
    port.labels = config_.joints;
    constexpr double inf = std::numeric_limits<double>::infinity();
    port.lower.setConstant(n, -inf);
    port.upper.setConstant(n, inf);
    for (Index i = 0; i < n; ++i) {
        const JointInfo& j = joints_[static_cast<std::size_t>(i)];
        if (j.nq != 1 || !model.has_position_limit(j.id)) continue;
        port.lower[i] = model.lower_position_limit()[j.idx_q];
        port.upper[i] = model.upper_position_limit()[j.idx_q];
    }
    declare_port(std::move(port));
    return setup.set_driven(mask_);
}

Status JointTargetModule::read_port(std::size_t index, PortValue& out) const {
    if (index != 0) return Module::read_port(index, out);
    out.vector = target_;
    return {};
}

Status JointTargetModule::write_port(std::size_t index, const PortValue& value) {
    if (index != 0) return Module::write_port(index, value);
    return set_target(value.vector);
}

Status JointTargetModule::set_target(const Eigen::Ref<const Eigen::VectorXd>& target) {
    if (target.size() != static_cast<Index>(joints_.size()) || !target.allFinite())
        return make_error(ErrorCode::invalid_argument, "joint_target '" + id() + "': bad target");
    if (Status s = check_unlocked(); !s) return s;
    if (home_.active())
        return make_error(ErrorCode::invalid_argument, "joint_target '" + id() + "': homing in progress");
    target_ = target;
    if (!ports().empty()) target_ = target_.cwiseMax(ports()[0].lower).cwiseMin(ports()[0].upper);
    has_target_ = true;
    claiming_ = true;
    settled_ = false;
    yielded_ = false;
    fresh_ = true;
    return {};
}

Status JointTargetModule::request_home() {
    if (!task_) return make_error(ErrorCode::invalid_argument, "joint_target '" + id() + "': not configured");
    if (!enabled()) return make_error(ErrorCode::invalid_argument, "joint_target '" + id() + "': disabled");
    if (Status s = check_unlocked(); !s) return s;
    home_.request();
    yielded_ = false;
    request_control();
    return {};
}

Status JointTargetModule::tune(const ConfigNode& params) {
    const std::string who = "joint_target '" + id() + "'";
    if (!task_) return not_configured(who);
    if (home_.active() && touches_motion(params))
        return make_error(ErrorCode::invalid_argument,
                          who + ": trajectory and home_motion cannot change while homing");
    JointTargetModuleConfig next = config_;
    // tunables() echoes the resolved read-only home; an unchanged echo is accepted so params() round-trips.
    ConfigNode patch = params;
    if (const ConfigNode* h = params.find("home"); h && params.kind() == ConfigNode::Kind::map &&
                                                   h->kind() == ConfigNode::Kind::list &&
                                                   static_cast<Index>(h->size()) == home_pose_.size()) {
        bool same = true;
        for (Index i = 0; i < home_pose_.size() && same; ++i) {
            const ConfigNode& v = h->items()[static_cast<std::size_t>(i)];
            same = v.kind() == ConfigNode::Kind::number && v.as_number() == home_pose_[i];
        }
        if (same) {
            patch = ConfigNode::map({});
            for (const auto& [k, v] : params.entries())
                if (k != "home") patch.set(k, v);
        }
    }
    if (Status s = builtin_schemas::joint_target().patch(patch, next); !s) return s;
    const auto n = static_cast<Index>(joints_.size());
    if (next.trajectory.velocity_cap.size() == 0)
        next.trajectory.velocity_cap.setConstant(n, std::numeric_limits<double>::infinity());
    if (Status s = next.trajectory.validate(n); !s) return with_who(s, who);
    if (Status s = Homing::validate(next.home_motion); !s) return with_who(s, who);
    if (!finite_positive(next.tolerance))
        return make_error(ErrorCode::invalid_argument, who + ": tolerance must be finite and > 0");
    JointTrajectoryConfig home = next.trajectory;
    home.trigger = JointTrigger::continuous;
    home.velocity_scale = 1.0;
    home.velocity_cap = next.home_motion.velocity_scale * limit_;
    if (Status s = home.validate(n); !s) return with_who(s, who);

    if (Status s = task_->set_weight(next.weight); !s) return s;
    if (Status s = task_->set_gain(next.gain); !s) return s;
    if (Status s = trajectory_->configure(next.trajectory); !s) return s;
    if (Status s = home_.set_config(next.home_motion); !s) return s;
    home_trajectory_ = std::move(home);
    config_ = std::move(next);
    return {};
}

ConfigNode JointTargetModule::tunables() const {
    ConfigNode out = builtin_schemas::joint_target().values(config_);
    // Read-only: resolved home targets, so clients can offer "back to initial".
    ConfigNode list = ConfigNode::list();
    for (Index i = 0; i < home_pose_.size(); ++i) list.append(home_pose_[i]);
    out.set("home", std::move(list));
    return out;
}
ConfigNode JointTargetModule::tune_schema() const {
    return builtin_schemas::joint_target().tune_schema(defaults_);
}

Status JointTargetModule::finish(HomeStatus status) {
    home_.finish(status);
    if (status != HomeStatus::succeeded) target_ = trajectory_->reference();
    if (Status s = trajectory_->configure(config_.trajectory); !s) return s;
    has_target_ = true;
    // A finished home keeps the joints until other input takes them; an aborted one lets go.
    claiming_ = status != HomeStatus::aborted;
    settled_ = claiming_;
    return {};
}

void JointTargetModule::measure(const Cycle& cycle) {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const JointInfo& j = joints_[i];
        const auto k = static_cast<Index>(i);
        measured_[k] = scalar_position(j, cycle.q);
        measured_v_[k] = cycle.v[j.idx_v];
    }
}

bool JointTargetModule::arrived(const Eigen::VectorXd& goal) const {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const auto k = static_cast<Index>(i);
        double e = goal[k] - measured_[k];
        if (wrap_[i]) e = std::remainder(e, 2.0 * M_PI);
        if (std::abs(e) >= config_.tolerance) return false;
    }
    return true;
}

Status JointTargetModule::prepare(Cycle& cycle) {
    fresh_now_ = fresh_;
    fresh_ = false;
    now_ = cycle.now;
    if (Status s = step_home(cycle); !s) return s;
    if (fresh_now_ || home_.active()) {
        input_at_ = now_;
        release_pending_ = false;
    } else if (release_pending_ && now_ - input_at_ >= config_.release_after_s) {
        release_pending_ = false;
        if (holds_claim()) {
            claiming_ = false;
            yielded_ = true;
        }
    }
    // Held with nothing pending: the port follows the joints, matching what reset() resumes from.
    if (!running() && !home_.active() && (!has_target_ || (config_.claim_idle && !claiming_))) {
        measure(cycle);
        target_ = measured_;
    }
    home_.align(cycle, *model_, mask_);
    return {};
}

Status JointTargetModule::step_home(const Cycle& cycle) {
    if (home_.update(cycle)) return finish(home_.status());
    if (home_.status() == HomeStatus::executing) {
        measure(cycle);
        double error = 0.0;
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            const auto k = static_cast<Index>(i);
            double e = home_pose_[k] - measured_[k];
            if (wrap_[i]) e = std::remainder(e, 2.0 * M_PI);
            error = std::max(error, std::abs(e));
        }
        const bool within = arrived(home_pose_);
        home_.progress(cycle.now, error, within);
        // Home is continuous, so the profile stays active; it has settled once its velocity is zero.
        if (!trajectory_->velocity().isZero(1e-9)) return {};
        return within ? finish(HomeStatus::succeeded) : Status{};
    }
    // A continuous profile never goes inactive; it is at rest once its velocity is zero.
    if (!claiming_ || settled_ || (trajectory_->active() && !trajectory_->velocity().isZero(1e-9))) return {};
    measure(cycle);
    settled_ = arrived(target_);
    // claim_idle keeps the joints anyway; idle again, it resumes from where they are.
    if (settled_ && config_.claim_idle) claiming_ = false;
    return {};
}

Claim JointTargetModule::claim() const {
    if (home_.active()) return home_.claim(mask_);
    return {holds_claim() ? &mask_ : nullptr, config_.priority, config_.on_held};
}

bool JointTargetModule::holds_claim() const noexcept {
    return claiming_ || (config_.claim_idle && !yielded_);
}

bool JointTargetModule::yield_control() {
    // A joint target given in the same cycle wins.
    if (home_.active() || fresh_now_ || !holds_claim()) return false;
    if (now_ - input_at_ < config_.release_after_s) {
        release_pending_ = true;
        return false;
    }
    claiming_ = false;
    yielded_ = true;
    return true;
}

Status JointTargetModule::on_disabled(const Cycle&) {
    return home_.active() ? finish(HomeStatus::aborted) : Status{};
}

Status JointTargetModule::on_preempted(const Cycle&, const std::string&) {
    if (home_.active())
        if (Status s = finish(HomeStatus::aborted); !s) return s;
    // Idle: the next reset() holds the positions measured then.
    claiming_ = false;
    has_target_ = false;
    return {};
}

Status JointTargetModule::reset(const Cycle& cycle) {
    measure(cycle);
    // claim_idle resumes idle from where the joints are.
    if (cycle.rebase || !has_target_ || (config_.claim_idle && !claiming_)) target_ = measured_;
    trajectory_->reset();
    return task_->set_target(measured_);
}

Status JointTargetModule::contribute(const Cycle& cycle) {
    measure(cycle);
    if (home_.start()) {
        if (Status s = trajectory_->configure(home_trajectory_); !s) return s;
        target_ = home_pose_;
        has_target_ = true;
    }
    const bool active = (config_.hold_idle && !yielded_) || holds_claim() || home_.active();
    set_task_active(handle_, active);
    if (!active) {
        target_ = measured_;
        trajectory_->reset();
        return task_->set_target(measured_);
    }
    if (Status s = trajectory_->step(measured_, target_, measured_v_, limit_, cycle.dt); !s) return s;
    return task_->set_target(trajectory_->reference());
}

// ---- gaze --------------------------------------------------------------------------------------------

namespace {
Eigen::VectorXd velocity_limit(double max_velocity) {
    return Eigen::VectorXd::Constant(
        1, max_velocity > 0.0 ? max_velocity : std::numeric_limits<double>::infinity());
}
}  // namespace

GazeModule::GazeModule(std::string id, GazeModuleConfig config)
    : Module(std::move(id)), config_(std::move(config)), defaults_(config_) {}

Status GazeModule::tune(const ConfigNode& params) {
    const std::string who = "gaze '" + id() + "'";
    if (!task_) return not_configured(who);
    GazeModuleConfig next = config_;
    if (Status s = builtin_schemas::gaze().patch(params, next); !s) return s;
    if (!std::isfinite(next.target_idle_s))
        return make_error(ErrorCode::invalid_argument, who + ": target_idle_s must be finite");
    if (!std::isfinite(next.max_velocity))
        return make_error(ErrorCode::invalid_argument, who + ": max_velocity must be finite and >= 0");
    if (Status s = velocity_->set_limits(velocity_limit(next.max_velocity)); !s) return with_who(s, who);
    for (Task* t : {static_cast<Task*>(task_.get()), static_cast<Task*>(home_task_.get())}) {
        if (Status s = t->set_weight(next.weight); !s) return s;
        if (Status s = t->set_gain(next.gain); !s) return s;
    }
    config_ = std::move(next);
    return {};
}

ConfigNode GazeModule::tunables() const { return builtin_schemas::gaze().values(config_); }
ConfigNode GazeModule::tune_schema() const { return builtin_schemas::gaze().tune_schema(defaults_); }

Status GazeModule::configure(ModuleSetup& setup) {
    const Model& model = *setup.model();
    const std::string who = "gaze '" + id() + "'";
    if (config_.joints.empty()) return make_error(ErrorCode::invalid_argument, who + ": no joints");
    frame_ids_.clear();
    for (const std::string& f : config_.frames) {
        Result<FrameId> fid = model.frame_id(f);
        if (!fid) return with_who(fid.error(), who);
        frame_ids_.push_back(*fid);
    }
    free_root_ = model.root() == RootJoint::free_flyer;
    if (!std::isfinite(config_.max_velocity) || config_.max_velocity < 0.0)
        return make_error(ErrorCode::invalid_argument, who + ": max_velocity must be finite and >= 0");
    if (!std::isfinite(config_.target_idle_s))
        return make_error(ErrorCode::invalid_argument, who + ": target_idle_s must be finite");
    if (Status s = resolve_scalar(model, config_.joints, who, joints_); !s) return s;
    if (Status s = velocity_mask_of(model, config_.joints, mask_); !s) return s;

    const auto n = static_cast<Index>(joints_.size());
    task_ = std::make_shared<GazeTask>(config_.frame, Eigen::Vector3d::Zero());
    if (Status s = task_->set_axis(config_.optical_axis); !s) return s;
    if (Status s = task_->set_offset(config_.offset); !s) return s;
    if (Status s = task_->set_weight(config_.weight); !s) return s;
    if (Status s = task_->set_gain(config_.gain); !s) return s;
    if (Status s = task_->set_mask(mask_); !s) return s;
    home_.resize(n);
    for (std::size_t i = 0; i < joints_.size(); ++i)
        home_[static_cast<Index>(i)] = scalar_position(joints_[i], setup.initial());
    home_task_ = std::make_shared<JointTask>(config_.joints, home_);
    if (Status s = home_task_->set_weight(config_.weight); !s) return s;
    if (Status s = home_task_->set_gain(config_.gain); !s) return s;
    if (Status s = home_task_->set_mask(mask_); !s) return s;
    Result<TaskHandle> aim = setup.add_task("gaze", task_, config_.task);
    if (!aim) return std::move(aim).error();
    aim_handle_ = *aim;
    Result<TaskHandle> home = setup.add_task("home", home_task_, config_.task);
    if (!home) return std::move(home).error();
    home_handle_ = *home;
    velocity_ =
        std::make_shared<JointVelocityConstraint>(config_.joints, velocity_limit(config_.max_velocity));
    if (Status s = setup.add_constraint("velocity", velocity_); !s) return s;

    const std::size_t k = frame_ids_.size();
    anchor_.assign(k, Eigen::Isometry3d::Identity());
    moved_at_.assign(k, 0.0);
    seen_.assign(k, 0);
    active_.assign(k, 0);
    declare_output({"focus", OutputType::point});
    declare_output({"active_frames", OutputType::number});
    return setup.set_driven(mask_);
}

Status GazeModule::prepare(Cycle& cycle) {
    constexpr double moved = 1e-4;
    // contribute() refreshes the focus only while running; a held gaze shows none.
    if (!running()) {
        focus_.setConstant(std::numeric_limits<double>::quiet_NaN());
        active_count_ = 0;
    }
    // Last cycle's solved poses: prepare() runs before the workspace is updated, also while held.
    const Workspace& ws = cycle.workspace;
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    if (ws.updated() && free_root_)
        if (Status s = ws.joint_pose(1, root); !s) return s;
    for (std::size_t i = 0; i < frame_ids_.size(); ++i) {
        // The first cycle counts as a move, so the head looks at the frames for target_idle_s after start.
        if (!seen_[i]) {
            moved_at_[i] = cycle.now;
            seen_[i] = 2;
        }
        if (ws.updated()) {
            Eigen::Isometry3d pose;
            if (Status s = ws.frame_pose(frame_ids_[i], pose); !s) return s;
            const Eigen::Isometry3d rel = root.inverse() * pose;
            if (seen_[i] != 1) {
                anchor_[i] = rel;
                seen_[i] = 1;
            } else if ((rel.translation() - anchor_[i].translation()).norm() > moved ||
                       Eigen::AngleAxisd(anchor_[i].linear().transpose() * rel.linear()).angle() > moved) {
                anchor_[i] = rel;
                moved_at_[i] = cycle.now;
                // A moving frame asks for the head; a joint target keeps it per its release_after_s.
                request_control();
            }
        }
        active_[i] = config_.target_idle_s <= 0.0 || cycle.now - moved_at_[i] <= config_.target_idle_s;
    }
    return {};
}

Claim GazeModule::claim() const {
    if (frame_ids_.empty()) return {};
    return {&mask_, priority::yielding};
}

Status GazeModule::contribute(const Cycle& cycle) {
    Eigen::Vector3d focus = Eigen::Vector3d::Zero();
    active_count_ = 0;
    for (std::size_t i = 0; i < frame_ids_.size(); ++i) {
        if (!active_[i]) continue;
        Eigen::Isometry3d pose;
        if (Status s = cycle.workspace.frame_pose(frame_ids_[i], pose); !s) return s;
        focus += pose.translation();
        ++active_count_;
    }
    const bool aiming = active_count_ > 0;
    focus_.setConstant(std::numeric_limits<double>::quiet_NaN());
    if (aiming) {
        focus_ = focus / static_cast<double>(active_count_);
        if (Status s = task_->set_focus(focus_); !s) return s;
    }
    set_task_active(aim_handle_, aiming);
    set_task_active(home_handle_, !aiming);
    return {};
}

Status GazeModule::read_output(std::size_t index, OutputValue& out) const {
    switch (index) {
        case 0:
            out.points = focus_;
            return {};
        case 1:
            out.number = static_cast<double>(active_count_);
            return {};
        default:
            return Module::read_output(index, out);
    }
}

// ---- center_of_mass ----------------------------------------------------------------------------------

CenterOfMassModule::CenterOfMassModule(std::string id, CenterOfMassModuleConfig config)
    : Module(std::move(id)), config_(std::move(config)), defaults_(config_) {}

Status CenterOfMassModule::tune(const ConfigNode& params) {
    const std::string who = "center_of_mass '" + id() + "'";
    if (!constraint_) return not_configured(who);
    CenterOfMassModuleConfig next = config_;
    if (Status s = builtin_schemas::center_of_mass().patch(params, next); !s) return s;
    if (next.margin > constraint_->polygon().inset_limit() + 1e-12)
        return make_error(ErrorCode::invalid_argument, who + ": margin exceeds the inscribed radius");
    // set_payloads is the only setter that can still fail (unknown frame), so it goes first.
    if (params.find("payloads"))
        if (Status s = constraint_->set_payloads(next.payloads); !s) return with_who(s, who);
    if (Status s = constraint_->set_margin(next.margin); !s) return with_who(s, who);
    if (Status s = constraint_->set_gain(next.gain); !s) return with_who(s, who);
    if (Status s = constraint_->set_recovery(next.recovery_speed); !s) return with_who(s, who);
    constraint_->set_closed_loop(next.closed_loop);
    config_ = std::move(next);
    return {};
}

ConfigNode CenterOfMassModule::tunables() const { return builtin_schemas::center_of_mass().values(config_); }
ConfigNode CenterOfMassModule::tune_schema() const {
    return builtin_schemas::center_of_mass().tune_schema(defaults_);
}

std::vector<std::string> CenterOfMassModule::dependencies() const {
    std::vector<std::string> out;
    for (const auto& t : config_.recovery_tasks)
        if (t) out.push_back(t->id());
    return out;
}

Status CenterOfMassModule::configure(ModuleSetup& setup) {
    Result<SupportPolygon> polygon = SupportPolygon::create(config_.frame, config_.polygon, config_.subtree);
    if (!polygon) return std::move(polygon).error();
    constraint_ = std::make_shared<CenterOfMassConstraint>(std::move(*polygon));
    if (Status s = constraint_->set_margin(config_.margin); !s) return s;
    if (Status s = constraint_->set_gain(config_.gain); !s) return s;
    if (Status s = constraint_->set_recovery(config_.recovery_speed); !s) return s;
    if (Status s = constraint_->set_payloads(config_.payloads); !s) return s;
    constraint_->set_closed_loop(config_.closed_loop);
    if (!config_.recovery_tasks.empty()) {
        Mask columns(static_cast<std::size_t>(setup.model()->nv()));
        for (const auto& t : config_.recovery_tasks) {
            if (!t)
                return make_error(ErrorCode::invalid_argument,
                                  "center_of_mass '" + id() + "': recovery task is null");
            columns |= t->driven();
        }
        if (Status s = constraint_->set_columns(std::move(columns)); !s) return s;
    }
    const Eigen::Index n = constraint_->polygon().edges();
    local_.resize(2, n);
    for (int i = 0; i < 4; ++i) points_[i].setZero(3, i < 2 ? 1 : n);
    declare_output({"com", OutputType::point});
    declare_output({"ground", OutputType::point});
    declare_output({"polygon", OutputType::polygon, n, OutputRole::boundary});
    declare_output({"margin_polygon", OutputType::polygon, n, OutputRole::warning});
    declare_output({"min_slack", OutputType::number, 0, OutputRole::margin, "m"});
    declare_output({"inside", OutputType::flag});
    return setup.add_constraint("support", constraint_);
}

Status CenterOfMassModule::update_outputs(Workspace& at) {
    const SupportPolygon& polygon = constraint_->polygon();
    if (Status s = polygon.evaluate(at, constraint_->payloads(), display_); !s) return s;
    const Eigen::Isometry3d& pose = display_.frame_pose;
    points_[0] = display_.world_com;
    points_[1] = pose * Eigen::Vector3d(display_.com.x(), display_.com.y(), 0.0);
    const double margin = constraint_->margin();
    for (int k = 0; k < 2; ++k) {
        polygon.inset_vertices(k == 0 ? 0.0 : margin, local_);
        auto& out = points_[2 + k];
        for (Eigen::Index i = 0; i < local_.cols(); ++i)
            out.col(i) = pose * Eigen::Vector3d(local_(0, i), local_(1, i), 0.0);
    }
    min_slack_ = display_.slack.minCoeff();
    inside_ = min_slack_ >= margin - 1e-12;
    return {};
}

Status CenterOfMassModule::read_output(std::size_t index, OutputValue& out) const {
    if (index < 4) {
        out.points = points_[index];
    } else if (index == 4) {
        out.number = min_slack_;
    } else if (index == 5) {
        out.flag = inside_;
    } else {
        return Module::read_output(index, out);
    }
    return {};
}

// ---- null_space_preference ---------------------------------------------------------------------------

NullSpacePreferenceModule::NullSpacePreferenceModule(std::string id, NullSpacePreferenceModuleConfig config)
    : Module(std::move(id)),
      config_(std::move(config)),
      defaults_(config_),
      preference_(config_.preference) {}

Status NullSpacePreferenceModule::posture_target(const NullSpacePreferenceModuleConfig& c,
                                                 Eigen::VectorXd& out) const {
    const Model& model = *model_;
    const std::string who = "null_space_preference '" + id() + "'";
    if (c.reference == PostureReference::current) {
        if (c.posture.size() != 0 || !c.posture_values.empty())
            return make_error(ErrorCode::invalid_argument,
                              who + ": posture and posture_values are not used with reference: current");
        out = have_q_ ? last_q_ : initial_;
        return {};
    }
    out = c.reference == PostureReference::neutral ? neutral_ : initial_;
    if (c.reference == PostureReference::neutral) clamp_neutral(c, out);
    if (c.reference != PostureReference::custom && c.posture.size() != 0)
        return make_error(ErrorCode::invalid_argument, who + ": posture is used only with reference: custom");
    if (c.reference == PostureReference::custom) {
        if (c.posture.size() != static_cast<Index>(infos_.size())) {
            std::string names;
            for (const JointInfo& j : infos_) names += (names.empty() ? "" : ", ") + j.name;
            return make_error(ErrorCode::invalid_argument,
                              who + ": custom posture needs " + std::to_string(infos_.size()) + " values (" +
                                  names + "), got " + std::to_string(c.posture.size()));
        }
        if (Status s = check_within_limits(model, infos_, c.posture, who, "posture"); !s) return s;
        for (std::size_t i = 0; i < infos_.size(); ++i)
            if (Status s = model.set_joint_position(out, infos_[i].id, c.posture[static_cast<Index>(i)]); !s)
                return s;
    }
    for (const auto& [name, value] : c.posture_values) {
        const JointInfo* info = nullptr;
        for (const JointInfo& j : infos_)
            if (j.name == name) info = &j;
        if (!info)
            return make_error(ErrorCode::invalid_argument,
                              who + ": posture_values." + name + " is not one of the module's joints");
        const Eigen::Matrix<double, 1, 1> v(value);
        if (Status s = check_within_limits(model, {*info}, v, who, "posture_values"); !s) return s;
        if (Status s = model.set_joint_position(out, info->id, value); !s) return s;
    }
    return {};
}

// Neutral of the bounded joints not set by posture_values, clamped into the effective limits; allocation-free.
void NullSpacePreferenceModule::clamp_neutral(const NullSpacePreferenceModuleConfig& c,
                                              Eigen::VectorXd& out) const {
    if (!lower_ || !upper_) return;
    for (const JointInfo& j : infos_) {
        if (j.nq != 1) continue;
        bool given = false;
        for (const auto& [name, value] : c.posture_values) given = given || name == j.name;
        if (!given) out[j.idx_q] = std::clamp(neutral_[j.idx_q], (*lower_)[j.idx_q], (*upper_)[j.idx_q]);
    }
}

// Per module joint, the pre-switch q kept within the effective limits.
Eigen::VectorXd NullSpacePreferenceModule::seed_from_q() const {
    const Eigen::VectorXd& q = have_q_ ? last_q_ : initial_;
    Eigen::VectorXd seed(static_cast<Index>(infos_.size()));
    for (std::size_t i = 0; i < infos_.size(); ++i) {
        const JointInfo& j = infos_[i];
        double v = scalar_position(j, q);
        if (j.nq == 1 && lower_ && upper_) v = std::clamp(v, (*lower_)[j.idx_q], (*upper_)[j.idx_q]);
        seed[static_cast<Index>(i)] = v;
    }
    return seed;
}

Status NullSpacePreferenceModule::tune(const ConfigNode& params) {
    const std::string who = "null_space_preference '" + id() + "'";
    if (!posture_) return not_configured(who);
    NullSpacePreferenceModuleConfig next = config_;
    next.preference = preference_;
    if (Status s = builtin_schemas::null_space_preference().patch(params, next); !s) return s;
    // A reference alone: custom takes the configured posture, else the pre-switch q, then the configured values.
    if (params.find("reference")) {
        const bool custom = next.reference == PostureReference::custom;
        if (!params.find("posture"))
            next.posture = !custom                         ? Eigen::VectorXd()
                           : defaults_.posture.size() != 0 ? defaults_.posture
                                                           : seed_from_q();
        if (!params.find("posture_values"))
            next.posture_values = custom && !params.find("posture") ? defaults_.posture_values
                                                                    : decltype(next.posture_values){};
    }
    if (next.reseed_current) {
        if (params.find("reference") || params.find("posture") || params.find("posture_values"))
            return make_error(ErrorCode::invalid_argument,
                              who +
                                  ": reseed_current sets the posture; give it without reference, posture or "
                                  "posture_values");
        next.reseed_current = false;
        next.preference = NullSpacePreference::posture;
        next.reference = PostureReference::custom;
        next.posture = seed_from_q();
        next.posture_values.clear();
    }
    if (next.preference == NullSpacePreference::joint_limits && !joint_limits_) return no_bounded_joints(who);
    Eigen::VectorXd target;
    if (Status s = posture_target(next, target); !s) return s;
    // Custom keeps one concrete value per joint, so tunables() shows what the posture is.
    if (next.reference == PostureReference::custom) {
        for (std::size_t i = 0; i < infos_.size(); ++i)
            next.posture[static_cast<Index>(i)] = scalar_position(infos_[i], target);
        next.posture_values.clear();
    }
    if (Status s = posture_->set_settings(next.projector); !s) return with_who(s, who);
    if (Status s = manipulability_->set_settings(next.projector); !s) return with_who(s, who);
    if (Status s = posture_->set_velocity_scale(next.posture_velocity_scale); !s) return with_who(s, who);
    if (Status s = posture_->set_weight(next.posture_weight); !s) return with_who(s, who);
    if (Status s = manipulability_->set_weight(next.manipulability_weight); !s) return with_who(s, who);
    if (joint_limits_) {
        if (Status s = joint_limits_->set_settings(next.projector); !s) return with_who(s, who);
        if (Status s = joint_limits_->set_band(next.joint_limits_band); !s) return with_who(s, who);
        if (Status s = joint_limits_->set_rate(next.joint_limits_rate); !s) return with_who(s, who);
        if (Status s = joint_limits_->set_weight(next.joint_limits_weight); !s) return with_who(s, who);
    }
    if (Status s = posture_->set_target(target); !s) return with_who(s, who);
    preference_ = next.preference;
    config_ = std::move(next);
    return {};
}

ConfigNode NullSpacePreferenceModule::tunables() const {
    NullSpacePreferenceModuleConfig current = config_;
    current.preference = preference_;
    return builtin_schemas::null_space_preference().values(current);
}

ConfigNode NullSpacePreferenceModule::tune_schema() const {
    return builtin_schemas::null_space_preference().tune_schema(defaults_);
}

std::vector<std::string> NullSpacePreferenceModule::dependencies() const {
    if (!config_.primary) return {};
    return {config_.primary->id()};
}

Status NullSpacePreferenceModule::configure(ModuleSetup& setup) {
    const Model& model = *setup.model();
    const std::string who = "null_space_preference '" + id() + "'";
    if (!config_.primary || !config_.primary->task())
        return make_error(ErrorCode::invalid_argument, who + ": primary is null or not configured");
    std::vector<std::string> joints = config_.joints;
    if (joints.empty())
        for (const JointInfo& j : model.joints())
            if (j.nv == 1 && config_.primary->driven().test(static_cast<std::size_t>(j.idx_v)))
                joints.push_back(j.name);
    std::vector<JointInfo> infos;
    if (Status s = resolve_scalar(model, joints, who, infos); !s) return s;
    if (infos.empty()) return make_error(ErrorCode::invalid_argument, who + ": no scalar joints");
    // The stage is masked to the primary's chain, so other joints would be silently ignored.
    for (const JointInfo& j : infos)
        if (!config_.primary->driven().test(static_cast<std::size_t>(j.idx_v)))
            return make_error(
                ErrorCode::invalid_argument,
                who + ": joint '" + j.name + "' is not driven by primary '" + config_.primary->id() + "'");

    if (config_.reseed_current)
        return make_error(ErrorCode::invalid_argument, who + ": reseed_current is a tune-only action");
    model_ = setup.model();
    infos_ = std::move(infos);
    initial_ = setup.initial();
    last_q_ = initial_;
    have_q_ = false;
    neutral_ = model.neutral();
    scratch_ = neutral_;
    lower_ = &setup.lower_limit();
    upper_ = &setup.upper_limit();
    limits_seen_ = ~std::uint64_t{0};
    if (config_.reference == PostureReference::custom && config_.posture.size() == 0)
        config_.posture = seed_from_q();
    Eigen::VectorXd posture;
    if (Status s = posture_target(config_, posture); !s) return s;

    const std::shared_ptr<FrameTask>& primary = config_.primary->task();
    posture_ = std::make_shared<NullSpacePostureTask>(primary, posture, joints);
    if (Status s = posture_->set_settings(config_.projector); !s) return s;
    if (Status s = posture_->set_velocity_scale(config_.posture_velocity_scale); !s) return s;
    if (Status s = posture_->set_weight(config_.posture_weight); !s) return s;

    auto manipulability =
        std::make_shared<ManipulabilityTask>(primary->frame(), joints, config_.manipulability_axes);
    if (Status s = manipulability->set_rate(config_.manipulability_rate); !s) return s;
    manipulability_ = std::make_shared<NullSpaceManipulabilityTask>(primary, std::move(manipulability));
    if (Status s = manipulability_->set_settings(config_.projector); !s) return s;
    if (Status s = manipulability_->set_weight(config_.manipulability_weight); !s) return s;

    // Unbounded (continuous) joints have no limits to keep away from; with none bounded, joint_limits is absent.
    std::vector<std::string> bounded;
    for (const JointInfo& j : infos_)
        if (j.nq == 1) bounded.push_back(j.name);
    joint_limits_.reset();
    if (!bounded.empty()) {
        joint_limits_ = std::make_shared<NullSpaceJointLimitTask>(primary, bounded);
        if (Status s = joint_limits_->set_settings(config_.projector); !s) return s;
        if (Status s = joint_limits_->set_band(config_.joint_limits_band); !s) return s;
        if (Status s = joint_limits_->set_rate(config_.joint_limits_rate); !s) return s;
        if (Status s = joint_limits_->set_weight(config_.joint_limits_weight); !s) return s;
    } else if (preference_ == NullSpacePreference::joint_limits) {
        return no_bounded_joints(who);
    }

    const std::pair<const char*, std::shared_ptr<Task>> tasks[] = {
        {"posture", posture_}, {"manipulability", manipulability_}, {"joint_limits", joint_limits_}};
    TaskOptions options = config_.task;
    if (!options.level) options.level = config_.primary->task_options().level;
    for (std::size_t k = 0; k < 3; ++k) {
        handles_[k] = TaskHandle{};
        if (!tasks[k].second) continue;
        // Same joints as the primary's chain, so the stage frees only that chain.
        if (Status s = tasks[k].second->set_mask(config_.primary->driven()); !s) return s;
        Result<TaskHandle> h = setup.add_task(tasks[k].first, tasks[k].second, options);
        if (!h) return std::move(h).error();
        handles_[k] = *h;
    }
    // Driving the primary's chain, a claim on it holds this module along with the primary.
    return setup.set_driven(config_.primary->driven());
}

Status NullSpacePreferenceModule::contribute(const Cycle& cycle) {
    const bool on = config_.primary->enabled() && config_.primary->held_by() == nullptr;
    for (std::size_t k = 0; k < 3; ++k)
        set_task_active(handles_[k], on && k == static_cast<std::size_t>(preference_));
    last_q_ = cycle.q;
    have_q_ = true;
    if (cycle.lower_limit && cycle.upper_limit && cycle.limits_revision != limits_seen_) {
        limits_seen_ = cycle.limits_revision;
        lower_ = cycle.lower_limit;
        upper_ = cycle.upper_limit;
        if (config_.reference == PostureReference::neutral) {
            scratch_ = posture_->target();
            clamp_neutral(config_, scratch_);
            if (scratch_ != posture_->target())
                if (Status s = posture_->set_target(scratch_); !s) return s;
        }
    }
    if (preference_ == NullSpacePreference::posture && config_.reference == PostureReference::current)
        return posture_->set_target(cycle.q);
    return {};
}

// ---- collision base --------------------------------------------------------------------------------

Status CollisionModuleBase::configure_collision(ModuleSetup& setup, CollisionModelPtr model,
                                                std::shared_ptr<const CollisionWorld> world,
                                                const CollisionConstraintOptions& options, bool closed_loop,
                                                bool robot_visuals) {
    Result<std::shared_ptr<CollisionConstraint>> c = CollisionConstraint::create(model, world, options);
    if (!c) return std::move(c).error();
    constraint_ = std::move(*c);
    constraint_->set_closed_loop(closed_loop);
    geometries_ = robot_visuals ? model->geometries().size() : 0;
    if (robot_visuals)
        for (const CollisionGeometry& g : model->geometries())
            declare_visual({g.name, g.frame, g.placement, g.shape});
    const std::size_t obstacles = world ? world->size() : 0;
    for (std::size_t i = 0; i < obstacles; ++i)
        declare_visual({world->name(i), {}, world->pose(i), world->shape(i)});
    const auto n = static_cast<Eigen::Index>(visuals().size());
    declare_output({"min_distance", OutputType::number, 0, OutputRole::margin, "m"});
    declare_output({"closest", OutputType::segment, 0, OutputRole::alert});
    declare_output({"closest_pair", OutputType::visuals, 2, OutputRole::alert});
    declare_output({"band", OutputType::visuals, n, OutputRole::warning});
    declare_output({"pair", OutputType::text});
    declare_output({"pairs", OutputType::number});
    declare_output({"active_pairs", OutputType::number});
    pair_.reserve(128);
    display_.emplace(std::move(model), std::move(world));
    closest_pair_.reserve(2);
    band_.reserve(static_cast<std::size_t>(n));
    in_band_.assign(static_cast<std::size_t>(n), 0);
    return setup.add_constraint("collision", constraint_);
}

Status CollisionModuleBase::on_disabled(const Cycle&) {
    if (constraint_) constraint_->reset_tracking();
    return {};
}

int CollisionModuleBase::visual_of(const CollisionPair& pair, bool second) const noexcept {
    const auto geometries = static_cast<Index>(geometries_);
    if (geometries_ == 0 && (!second || !pair.obstacle)) return -1;
    const Index i = !second ? pair.first : pair.obstacle ? geometries + pair.second : pair.second;
    return i < static_cast<Index>(visuals().size()) ? static_cast<int>(i) : -1;
}

Status CollisionModuleBase::update_outputs(Workspace& at) {
    const CollisionConstraintOptions& o = constraint_->options();
    const double band = o.d_min + o.activation_margin;
    if (Status s = display_->update(at, band); !s) return s;
    const auto& pairs = display_->pairs();
    nearest_ = display_->nearest();
    pair_.clear();
    if (nearest_.pair >= 0) {
        const CollisionPair& p = pairs[static_cast<std::size_t>(nearest_.pair)];
        const auto& geometries = display_->model()->geometries();
        pair_.append(geometries[static_cast<std::size_t>(p.first)].name).append(" / ");
        pair_.append(p.obstacle ? display_->world()->name(static_cast<std::size_t>(p.second))
                                : geometries[static_cast<std::size_t>(p.second)].name);
    }
    closest_pair_.clear();
    if (nearest_.pair >= 0 && nearest_.distance <= band)
        for (const bool second : {false, true})
            if (const int v = visual_of(pairs[static_cast<std::size_t>(nearest_.pair)], second); v >= 0)
                closest_pair_.push_back(v);
    std::fill(in_band_.begin(), in_band_.end(), 0);
    band_.clear();
    for (const Proximity& p : display_->proximities()) {
        if (p.distance > band) break;
        for (const bool second : {false, true}) {
            const int v = visual_of(pairs[static_cast<std::size_t>(p.pair)], second);
            if (v >= 0 && !in_band_[static_cast<std::size_t>(v)]) {
                in_band_[static_cast<std::size_t>(v)] = 1;
                band_.push_back(v);
            }
        }
    }
    return {};
}

Status CollisionModuleBase::read_output(std::size_t index, OutputValue& out) const {
    switch (index) {
        case 0:
            out.number = nearest_.distance;
            break;
        case 1:
            out.points.resize(3, 2);
            out.points.col(0) = nearest_.point1;
            out.points.col(1) = nearest_.point2;
            break;
        case 2:
            out.visuals = closest_pair_;
            break;
        case 3:
            out.visuals = band_;
            break;
        case 4:
            out.text = pair_;
            break;
        case 5:
            out.number = static_cast<double>(display_->pairs().size());
            break;
        case 6:
            out.number = static_cast<double>(constraint_->report().candidates);
            break;
        default:
            return Module::read_output(index, out);
    }
    return {};
}

// ---- self_collision ----------------------------------------------------------------------------------

SelfCollisionModule::SelfCollisionModule(std::string id, SelfCollisionModuleConfig config)
    : CollisionModuleBase(std::move(id)), config_(std::move(config)), defaults_(config_) {}

Status SelfCollisionModule::tune(const ConfigNode& params) {
    const std::string who = "self_collision '" + id() + "'";
    if (!constraint()) return not_configured(who);
    SelfCollisionModuleConfig next = config_;
    if (Status s = builtin_schemas::self_collision().patch(params, next); !s) return s;
    if (Status s = constraint()->set_options(next.constraint); !s) return with_who(s, who);
    constraint()->set_closed_loop(next.closed_loop);
    config_ = std::move(next);
    return {};
}

ConfigNode SelfCollisionModule::tunables() const { return builtin_schemas::self_collision().values(config_); }
ConfigNode SelfCollisionModule::tune_schema() const {
    return builtin_schemas::self_collision().tune_schema(defaults_);
}

Status SelfCollisionModule::configure(ModuleSetup& setup) {
    if (!config_.model)
        return make_error(ErrorCode::invalid_argument, "self_collision '" + id() + "': model is null");
    return configure_collision(setup, config_.model, nullptr, config_.constraint, config_.closed_loop, true);
}

// ---- world_collision ---------------------------------------------------------------------------------

WorldCollisionModule::WorldCollisionModule(std::string id, WorldCollisionModuleConfig config)
    : CollisionModuleBase(std::move(id)), config_(std::move(config)), defaults_(config_) {}

Status WorldCollisionModule::tune(const ConfigNode& params) {
    const std::string who = "world_collision '" + id() + "'";
    if (!constraint()) return not_configured(who);
    WorldCollisionModuleConfig next = config_;
    if (Status s = builtin_schemas::world_collision().patch(params, next); !s) return s;
    if (Status s = constraint()->set_options(next.constraint); !s) return with_who(s, who);
    constraint()->set_closed_loop(next.closed_loop);
    config_ = std::move(next);
    return {};
}

ConfigNode WorldCollisionModule::tunables() const {
    return builtin_schemas::world_collision().values(config_);
}
ConfigNode WorldCollisionModule::tune_schema() const {
    return builtin_schemas::world_collision().tune_schema(defaults_);
}

Status WorldCollisionModule::configure(ModuleSetup& setup) {
    const std::string who = "world_collision '" + id() + "'";
    if (!config_.model) return make_error(ErrorCode::invalid_argument, who + ": model is null");
    if (!config_.model->pairs().empty())
        return make_error(ErrorCode::invalid_argument, who + ": model has self pairs (use self_collision)");
    if (config_.obstacles.empty()) return make_error(ErrorCode::invalid_argument, who + ": no obstacles");
    auto world = std::make_shared<CollisionWorld>();
    std::vector<ObstacleId> ids;
    for (const WorldObstacle& o : config_.obstacles) {
        if (o.name.empty()) return make_error(ErrorCode::invalid_argument, who + ": obstacle name is empty");
        if (o.name.find('/') != std::string::npos)
            return make_error(ErrorCode::invalid_argument,
                              who + ": obstacle name '" + o.name + "' contains '/'");
        for (std::size_t i = 0; i < world->size(); ++i)
            if (world->name(i) == o.name)
                return make_error(ErrorCode::invalid_argument, who + ": duplicate obstacle '" + o.name + "'");
        Result<ObstacleId> id = world->add(o.name, o.shape, o.pose);
        if (!id) return with_who(std::move(id).error(), who);
        if (Status s = world->set_enabled(*id, o.enabled); !s) return with_who(s, who);
        ids.push_back(*id);
    }
    for (const WorldObstacle& o : config_.obstacles) {
        PortInfo pose;
        pose.name = o.name;
        pose.type = PortType::pose;
        declare_port(std::move(pose));
        PortInfo enabled;
        enabled.name = o.name + "/enabled";
        enabled.type = PortType::flag;
        declare_port(std::move(enabled));
    }
    world_ = world;
    view_ = world;
    ids_ = std::move(ids);
    motion_.assign(ids_.size(), Motion{});
    return configure_collision(setup, config_.model, view_, config_.constraint, config_.closed_loop, false);
}

namespace {
Status no_obstacle(const std::string& id) {
    return make_error(ErrorCode::out_of_range, "world_collision '" + id + "': no such obstacle");
}
}  // namespace

Status WorldCollisionModule::set_obstacle_pose(std::size_t obstacle, const Eigen::Isometry3d& pose) {
    if (!world_ || obstacle >= ids_.size()) return no_obstacle(id());
    if (Status s = world_->set_pose(ids_[obstacle], pose); !s) return s;
    motion_[obstacle].written = true;
    return {};
}

Status WorldCollisionModule::prepare(Cycle& cycle) {
    const double timeout = config_.velocity_timeout;
    for (std::size_t o = 0; o < motion_.size(); ++o) {
        Motion& m = motion_[o];
        const Eigen::Isometry3d& pose = world_->pose(o);
        const double elapsed = cycle.now - m.sampled;
        if (m.written) {
            Eigen::Vector3d linear = Eigen::Vector3d::Zero(), angular = Eigen::Vector3d::Zero();
            // A first write, or one after a gap, is a jump rather than motion.
            if (timeout > 0.0 && m.seen && elapsed > 0.0 && elapsed <= timeout) {
                linear = (pose.translation() - m.last.translation()) / elapsed;
                const Eigen::AngleAxisd turn(pose.linear() * m.last.linear().transpose());
                angular = turn.axis() * (turn.angle() / elapsed);
            }
            if (Status s = world_->set_velocity(ids_[o], linear, angular); !s) return s;
            m.last = pose;
            m.sampled = cycle.now;
            m.written = false;
            m.seen = true;
        } else if (m.seen && elapsed > timeout) {
            if (Status s = world_->set_velocity(ids_[o], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
                !s)
                return s;
            m.seen = false;
        }
    }
    return {};
}

Status WorldCollisionModule::set_obstacle_enabled(std::size_t obstacle, bool enabled) {
    if (!world_ || obstacle >= ids_.size()) return no_obstacle(id());
    return world_->set_enabled(ids_[obstacle], enabled);
}

Status WorldCollisionModule::read_port(std::size_t index, PortValue& out) const {
    const std::size_t o = index / 2;
    if (!world_ || o >= ids_.size()) return Module::read_port(index, out);
    if (index % 2 == 0)
        out.pose = world_->pose(o);
    else
        out.flag = world_->enabled(o);
    return {};
}

Status WorldCollisionModule::write_port(std::size_t index, const PortValue& value) {
    if (index % 2 == 0) return set_obstacle_pose(index / 2, value.pose);
    return set_obstacle_enabled(index / 2, value.flag);
}

bool WorldCollisionModule::visual_placement(std::size_t i, Eigen::Isometry3d& placement) const {
    if (i < geometry_visuals() || !world_) return Module::visual_placement(i, placement);
    const std::size_t o = i - geometry_visuals();
    if (o >= world_->size()) return false;
    placement = world_->pose(o);
    return world_->enabled(o);
}

// ---- chassis -----------------------------------------------------------------------------------------

ChassisModule::ChassisModule(std::string id, ChassisModuleConfig config)
    : Module(std::move(id)), config_(std::move(config)), defaults_(config_) {}

Status ChassisModule::tune(const ConfigNode& params) {
    const std::string who = "chassis '" + id() + "'";
    if (!controller_) return not_configured(who);
    ChassisModuleConfig next = config_;
    if (Status s = builtin_schemas::chassis().patch(params, next); !s) return s;
    if (Status s = controller_->tune_pose_limits(next.pose_limits); !s) {
        std::string msg = s.error().message;
        if (msg.rfind("chassis: ", 0) == 0) msg.erase(0, 9);
        return make_error(s.error().code, who + ": " + msg);
    }
    config_ = std::move(next);
    return {};
}

ConfigNode ChassisModule::tunables() const { return builtin_schemas::chassis().values(config_); }
ConfigNode ChassisModule::tune_schema() const { return builtin_schemas::chassis().tune_schema(defaults_); }

Status ChassisModule::configure(ModuleSetup& setup) {
    const Model& model = *setup.model();
    if (model.root() != RootJoint::free_flyer)
        return make_error(ErrorCode::invalid_argument,
                          "chassis '" + id() + "': model root must be free_flyer");
    model_ = &model;
    const JointInfo& root = model.joints().at(1);
    Mask locked(static_cast<std::size_t>(model.nv()));
    locked.set_range(static_cast<std::size_t>(root.idx_v), static_cast<std::size_t>(root.nv));
    Result<ChassisController> ctrl = ChassisController::create(config_.chassis);
    if (!ctrl) return std::move(ctrl).error();
    if (Status s = ctrl->set_scheduler_period(config_.period); !s) return s;
    config_.pose_limits = defaults_.pose_limits = ctrl->pose_limits();
    controller_.emplace(std::move(*ctrl));
    if (Status s = setup.lock(locked); !s) return s;

    const ChassisConfig& cc = controller_->config();
    const double lateral = cc.kinematics == ChassisKinematics::omni ? cc.max_linear : 0.0;
    PortInfo port;
    port.name = "velocity";
    port.type = PortType::vector;
    port.labels = {"vx", "vy", "wz"};
    port.lower = Eigen::Vector3d(-cc.max_linear, -lateral, -cc.max_angular);
    port.upper = Eigen::Vector3d(cc.max_linear, lateral, cc.max_angular);
    port.timeout = cc.command_timeout_s;
    declare_port(port);
    port.name = "pose_goal";
    port.labels = {"x", "y", "yaw"};
    port.lower = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    port.upper = -port.lower;
    declare_port(std::move(port));
    const std::pair<const char*, const char*> numbers[] = {{"vx", "m/s"}, {"vy", "m/s"}, {"wz", "rad/s"},
                                                           {"x", "m"},    {"y", "m"},    {"yaw", "rad"}};
    for (const auto& [name, unit] : numbers)
        declare_output({name, OutputType::number, 0, OutputRole::value, unit});
    declare_output({"active", OutputType::flag});
    declare_output({"fault", OutputType::flag});
    for (const auto& [name, unit] : {std::pair<const char*, const char*>{"nominal_vx", "m/s"},
                                     {"nominal_vy", "m/s"},
                                     {"nominal_wz", "rad/s"}})
        declare_output({name, OutputType::number, 0, OutputRole::value, unit});
    declare_output({"planned", OutputType::point});
    declare_output({"goal", OutputType::point});
    for (const char* name : {"state", "reason", "command", "command_mode"})
        declare_output({name, OutputType::text});
    declare_output({"command_priority", OutputType::number});
    declare_output({"command_fresh", OutputType::flag});
    declare_output({"source", OutputType::text});
    declare_output({"phase", OutputType::text});
    return setup.set_driven(std::move(locked));
}

Status ChassisModule::read_port(std::size_t index, PortValue& out) const {
    if (index > 1) return Module::read_port(index, out);
    const bool fresh = port_mode_ == index &&
                       (port_pending_ || (port_stamp_ && now_ - *port_stamp_ <= ports()[index].timeout));
    out.vector.resize(3);
    if (index == 0) {
        const Twist2D t = fresh ? port_command_ : Twist2D{};
        out.vector << t.vx, t.vy, t.wz;
    } else {
        const Pose2D p = fresh ? port_goal_ : output_.pose;
        out.vector << p.x, p.y, p.yaw;
    }
    return {};
}

Status ChassisModule::write_port(std::size_t index, const PortValue& value) {
    if (index > 1) return Module::write_port(index, value);
    if (value.vector.size() != 3 || !value.vector.allFinite())
        return make_error(ErrorCode::invalid_argument,
                          "chassis '" + id() + "': " + ports()[index].name + " needs 3 finite values");
    if (index == 0)
        port_command_ = {value.vector[0], value.vector[1], value.vector[2]};
    else
        port_goal_ = {value.vector[0], value.vector[1], value.vector[2]};
    port_mode_ = index;
    port_pending_ = true;
    return {};
}

Status ChassisModule::read_output(std::size_t index, OutputValue& out) const {
    const Twist2D& v = output_.velocity;
    const Pose2D& p = output_.pose;
    switch (index) {
        case 0:
            out.number = v.vx;
            break;
        case 1:
            out.number = v.vy;
            break;
        case 2:
            out.number = v.wz;
            break;
        case 3:
            out.number = p.x;
            break;
        case 4:
            out.number = p.y;
            break;
        case 5:
            out.number = p.yaw;
            break;
        case 6:
            out.flag = output_.state == ChassisState::active;
            break;
        case 7:
            out.flag = output_.state == ChassisState::fault;
            break;
        case 8:
            out.number = controller_->nominal_target().vx;
            break;
        case 9:
            out.number = controller_->nominal_target().vy;
            break;
        case 10:
            out.number = controller_->nominal_target().wz;
            break;
        case 11: {
            const Pose2D& r = controller_->ramp_pose();
            out.points = Eigen::Vector3d(r.x, r.y, 0.0);
            break;
        }
        case 12:
            out.points = command_.pose_mode && fresh()
                             ? Eigen::Vector3d(command_.pose.x, command_.pose.y, 0.0)
                             : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            break;
        case 13:
            out.text = to_string(output_.state);
            break;
        case 14:
            out.text = to_string(output_.reason);
            break;
        case 15:
            out.text = command_.source;
            break;
        case 16:
            out.text = command_.pose_mode ? "pose" : "velocity";
            break;
        case 17:
            out.number = command_.priority;
            break;
        case 18:
            out.flag = fresh();
            break;
        case 19:
            out.text = to_string(output_.source);
            break;
        case 20:
            out.text = to_string(controller_->diff_phase());
            break;
        default:
            return Module::read_output(index, out);
    }
    return {};
}

Status ChassisModule::prepare(Cycle& cycle) {
    ChassisController& c = *controller_;
    if (c.state() == ChassisState::configured) {
        if (Status s = c.start(cycle.now); !s) return s;
    } else if (stepped_ && cycle.index != last_index_ + 1) {
        if (c.state() == ChassisState::active)
            if (Status s = c.pause(cycle.now); !s) return s;
        if (c.state() == ChassisState::paused || c.state() == ChassisState::fault)
            if (Status s = c.resynchronize(cycle.now); !s) return s;
    }
    if (cycle.dt > 0.0)
        if (Status s = c.set_scheduler_period(cycle.dt); !s) return s;
    now_ = cycle.now;
    if (port_pending_) {
        port_pending_ = false;
        Result<bool> accepted = port_mode_ == 0 ? c.submit_velocity(port_command_, cycle.now, source_)
                                                : c.submit_pose(port_goal_, cycle.now, source_);
        if (!accepted) return std::move(accepted).error();
        if (*accepted)
            port_stamp_ = cycle.now;
        else
            port_stamp_.reset();
    }
    output_ = c.step(cycle.now);
    command_ = c.command_snapshot();
    last_index_ = cycle.index;
    stepped_ = true;
    return model_->set_root_pose(cycle.q, chassis_root_pose(c.pose(), config_.center_in_root));
}

}  // namespace kstone
