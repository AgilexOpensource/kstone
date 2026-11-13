#include "kstone/runtime/engine.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <fstream>
#include <pthread.h>
#include <sched.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;
#endif

namespace kstone {

namespace {

Status validate_thread(const ThreadOptions& t) {
    if (t.priority == 0 && t.cpus.empty()) return {};
#if defined(__linux__)
    if (t.priority != 0) {
        const int lo = sched_get_priority_min(SCHED_FIFO), hi = sched_get_priority_max(SCHED_FIFO);
        if (t.priority < lo || t.priority > hi)
            return make_error(ErrorCode::invalid_argument, "engine: thread.priority must be 0 or in [" +
                                                               std::to_string(lo) + ", " +
                                                               std::to_string(hi) + "]");
    }
    std::vector<int> cpus = t.cpus;
    std::sort(cpus.begin(), cpus.end());
    for (std::size_t i = 0; i < cpus.size(); ++i) {
        if (cpus[i] < 0 || cpus[i] >= CPU_SETSIZE)
            return make_error(ErrorCode::invalid_argument, "engine: thread.cpus entry " +
                                                               std::to_string(cpus[i]) + " is outside [0, " +
                                                               std::to_string(CPU_SETSIZE) + ")");
        if (i > 0 && cpus[i] == cpus[i - 1])
            return make_error(ErrorCode::invalid_argument,
                              "engine: thread.cpus lists " + std::to_string(cpus[i]) + " twice");
    }
    return {};
#else
    return make_error(ErrorCode::unsupported,
                      "engine: thread.priority and thread.cpus are only supported on Linux");
#endif
}

// Applies ThreadOptions to the calling thread (warning on failure) and restores what it changed on destruction.
class ThreadScope {
  public:
    ThreadScope() = default;
    ThreadScope(const ThreadScope&) = delete;
    ThreadScope& operator=(const ThreadScope&) = delete;

#if defined(__linux__)
    void apply(const ThreadOptions& t, const Logger& log) {
        const pthread_t self = pthread_self();
        if (t.lock_memory) {
            if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0)
                note(log, LogLevel::info, "memory locked");
            else
                note(log, LogLevel::warn,
                     std::string("cannot lock memory: ") + std::strerror(errno) +
                         " (needs CAP_IPC_LOCK or a larger `ulimit -l`); running unlocked");
        }
        if (!t.cpus.empty()) {
            std::string list;
            for (int c : t.cpus) list += (list.empty() ? "" : ", ") + std::to_string(c);
            cpu_set_t set;
            CPU_ZERO(&set);
            for (int c : t.cpus) CPU_SET(static_cast<std::size_t>(c), &set);
            int rc = pthread_getaffinity_np(self, sizeof(old_cpus_), &old_cpus_);
            if (rc == 0) rc = pthread_setaffinity_np(self, sizeof(set), &set);
            if (rc == 0) {
                affinity_set_ = true;
                note(log, LogLevel::info, "pinned to cpus [" + list + "]");
            } else {
                note(log, LogLevel::warn,
                     "cannot pin to thread.cpus [" + list + "]: " + std::strerror(rc) +
                         (rc == EINVAL ? " (none online or allowed for this process)" : "") +
                         "; running unpinned");
            }
        }
        if (t.priority != 0) {
            sched_param p{};
            p.sched_priority = t.priority;
            int rc = pthread_getschedparam(self, &old_policy_, &old_param_);
            if (rc == 0) rc = pthread_setschedparam(self, SCHED_FIFO, &p);
            if (rc == 0) {
                sched_set_ = true;
                note(log, LogLevel::info, "SCHED_FIFO priority " + std::to_string(t.priority));
            } else {
                note(log, LogLevel::warn,
                     "cannot set SCHED_FIFO priority " + std::to_string(t.priority) + ": " +
                         std::strerror(rc) +
                         (rc == EPERM ? " (needs CAP_SYS_NICE or RLIMIT_RTPRIO, see `ulimit -r`)" : "") +
                         "; running with the default scheduler");
            }
        }
    }

    ~ThreadScope() {
        const pthread_t self = pthread_self();
        if (sched_set_) pthread_setschedparam(self, old_policy_, &old_param_);
        if (affinity_set_) pthread_setaffinity_np(self, sizeof(old_cpus_), &old_cpus_);
    }

  private:
    static void note(const Logger& log, LogLevel level, const std::string& message) {
        if (log.enabled(level)) log.log(level, "engine", "run: " + message);
    }

    bool affinity_set_{false};
    bool sched_set_{false};
    cpu_set_t old_cpus_{};
    int old_policy_{SCHED_OTHER};
    sched_param old_param_{};
#else
    void apply(const ThreadOptions&, const Logger&) {}
#endif
};

/// fnmatch(3) subset without flags: '*', '?', '[...]' with ranges and '!' or '^' negation.
bool glob_match(const std::string& pattern, const std::string& text) {
    std::size_t p = 0, t = 0, star = std::string::npos, resume = 0;
    const auto single = [&](std::size_t& pi, char c) {
        if (pattern[pi] == '?') return ++pi, true;
        if (pattern[pi] == '[') {
            std::size_t i = pi + 1;
            const bool negate = i < pattern.size() && (pattern[i] == '!' || pattern[i] == '^');
            if (negate) ++i;
            bool hit = false;
            const std::size_t first = i;
            for (; i < pattern.size() && (pattern[i] != ']' || i == first); ++i) {
                if (i + 2 < pattern.size() && pattern[i + 1] == '-' && pattern[i + 2] != ']') {
                    hit = hit || (pattern[i] <= c && c <= pattern[i + 2]);
                    i += 2;
                } else {
                    hit = hit || pattern[i] == c;
                }
            }
            if (i < pattern.size()) {
                pi = i + 1;
                return hit != negate;
            }
        }
        return pattern[pi] == c ? (++pi, true) : false;  // literal, or '[' without a closing ']'
    };
    while (t < text.size()) {
        if (p < pattern.size() && pattern[p] == '*') {
            star = ++p;
            resume = t;
        } else if (p < pattern.size() && single(p, text[t])) {
            ++t;
        } else if (star != std::string::npos) {
            p = star;
            t = ++resume;
        } else {
            return false;
        }
    }
    while (p < pattern.size() && pattern[p] == '*') ++p;
    return p == pattern.size();
}

#if defined(__linux__)
bool has_effective_capability(int cap) {
    std::ifstream in("/proc/self/status");
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("CapEff:", 0) != 0) continue;
        const unsigned long long mask = std::stoull(line.substr(7), nullptr, 16);
        return ((mask >> cap) & 1ULL) != 0;
    }
    return false;
}

struct LimitNeed {
    bool rtprio{false};
    bool memlock{false};
};

LimitNeed missing_limits(const ThreadOptions& t) {
    constexpr int cap_ipc_lock = 14, cap_sys_nice = 23;
    LimitNeed need;
    rlimit r{};
    if (t.priority > 0 && !has_effective_capability(cap_sys_nice) && getrlimit(RLIMIT_RTPRIO, &r) == 0 &&
        r.rlim_cur != RLIM_INFINITY && r.rlim_cur < static_cast<rlim_t>(t.priority))
        need.rtprio = true;
    if (t.lock_memory && !has_effective_capability(cap_ipc_lock) && getrlimit(RLIMIT_MEMLOCK, &r) == 0 &&
        r.rlim_cur != RLIM_INFINITY)
        need.memlock = true;
    return need;
}
#endif

}  // namespace

Status elevate_thread_limits(const ThreadOptions& thread, const Logger& log) {
#if defined(__linux__)
    if (auto s = validate_thread(thread); !s) return s;
    const LimitNeed need = missing_limits(thread);
    if (!need.rtprio && !need.memlock) return {};
    const int tty = ::open("/dev/tty", O_RDWR | O_CLOEXEC);
    if (tty < 0)
        return make_error(ErrorCode::unsupported,
                          "elevate: no controlling terminal for the sudo password prompt; raise the limits "
                          "another way (setcap, limits.d) or run from a terminal");
    ::close(tty);

    std::vector<std::string> args = {"sudo",
                                     "-p",
                                     "[kstone] sudo password for %u to raise real-time limits: ",
                                     "--",
                                     "prlimit",
                                     "--pid",
                                     std::to_string(::getpid())};
    std::string what;
    if (need.rtprio) {
        args.push_back("--rtprio=" + std::to_string(thread.priority));
        what = "rtprio " + std::to_string(thread.priority);
    }
    if (need.memlock) {
        args.push_back("--memlock=unlimited");
        what += std::string(what.empty() ? "" : ", ") + "memlock unlimited";
    }
    std::vector<char*> argv;
    for (std::string& a : args) argv.push_back(a.data());
    argv.push_back(nullptr);
    if (log.enabled(LogLevel::info))
        log.log(LogLevel::info, "engine", "elevate: running sudo prlimit for " + what);

    pid_t pid = 0;
    const int rc = ::posix_spawnp(&pid, "sudo", nullptr, nullptr, argv.data(), environ);
    if (rc != 0)
        return make_error(ErrorCode::io_error,
                          std::string("elevate: cannot start sudo: ") + std::strerror(rc));
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR)
            return make_error(ErrorCode::io_error, std::string("elevate: waitpid: ") + std::strerror(errno));
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
        return make_error(ErrorCode::io_error,
                          "elevate: sudo prlimit failed (exit " +
                              std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1) + ")");
    const LimitNeed after = missing_limits(thread);
    if (after.rtprio || after.memlock)
        return make_error(ErrorCode::io_error, "elevate: limits still too low after sudo prlimit");
    if (log.enabled(LogLevel::info)) log.log(LogLevel::info, "engine", "elevate: raised " + what);
    return {};
#else
    (void)thread;
    (void)log;
    return make_error(ErrorCode::unsupported, "elevate: only supported on Linux");
#endif
}

Result<FeedbackLayout> resolve_feedback_groups(const Model& model, const std::vector<FeedbackGroup>& groups) {
    const auto& joints = model.joints();
    constexpr std::size_t none = static_cast<std::size_t>(-1);
    std::vector<JointId> leader(joints.size());
    for (std::size_t i = 0; i < joints.size(); ++i) leader[i] = static_cast<JointId>(i);
    std::vector<bool> follower(joints.size(), false);
    for (const MimicRelation& rel : model.mimics()) {
        Result<JointId> f = model.joint_id(rel.follower);
        Result<JointId> l = model.joint_id(rel.leader);
        if (!f || !l) continue;
        follower[*f] = true;
        leader[*f] = *l;
    }
    const auto eligible = [&](const JointInfo& j) {
        return j.type != JointType::universe && !follower[j.id];
    };
    FeedbackLayout out;
    out.joint_group.assign(joints.size(), none);
    for (std::size_t g = 0; g < groups.size(); ++g) {
        const FeedbackGroup& fg = groups[g];
        const std::string label = "engine: feedback group '" + fg.name + "'";
        if (fg.name.empty())
            return make_error(ErrorCode::invalid_argument, "engine: feedback group name is empty");
        if (fg.name == "default")
            return make_error(ErrorCode::invalid_argument,
                              "engine: feedback group name 'default' is reserved");
        if (std::find(out.names.begin(), out.names.end(), fg.name) != out.names.end())
            return make_error(ErrorCode::invalid_argument, label + " is declared twice");
        if (fg.joints.empty()) return make_error(ErrorCode::invalid_argument, label + " has no joints");
        out.names.push_back(fg.name);
        Status st;
        const auto assign = [&](const JointInfo& j) {
            std::size_t& slot = out.joint_group[j.id];
            if (slot != none && slot != g && st)
                st = make_error(ErrorCode::invalid_argument,
                                "engine: joint '" + j.name + "' is in feedback groups '" + groups[slot].name +
                                    "' and '" + fg.name + "'");
            slot = g;
        };
        for (const std::string& name : fg.joints) {
            if (Result<JointId> id = model.joint_id(name)) {
                const JointInfo& j = joints[*id];
                if (!eligible(j))
                    return make_error(ErrorCode::invalid_argument,
                                      label + ": joint '" + name + "' is the universe or a mimic follower");
                assign(j);
            } else {
                bool matched = false;
                for (const JointInfo& j : joints)
                    if (eligible(j) && glob_match(name, j.name)) {
                        assign(j);
                        matched = true;
                    }
                if (!matched)
                    return make_error(ErrorCode::invalid_argument,
                                      label + ": '" + name + "' matches no joint");
            }
            if (!st) return std::move(st).error();
        }
    }
    bool ungrouped = groups.empty();
    for (const JointInfo& j : joints)
        if (eligible(j) && out.joint_group[j.id] == none) {
            out.joint_group[j.id] = out.names.size();
            ungrouped = true;
        }
    if (ungrouped) out.names.emplace_back("default");
    for (const JointInfo& j : joints) {
        if (j.type == JointType::universe)
            out.joint_group[j.id] = out.names.size();
        else if (follower[j.id])
            out.joint_group[j.id] = out.joint_group[leader[j.id]];
    }
    return out;
}

RobotInterface::~RobotInterface() = default;
EngineObserver::~EngineObserver() = default;

LoopbackRobot::LoopbackRobot(Eigen::VectorXd q, Eigen::Index nv, std::shared_ptr<const Clock> clock)
    : clock_(clock ? std::move(clock) : make_steady_clock()) {
    state_.q = std::move(q);
    state_.v = Eigen::VectorXd::Zero(nv);
}

Status LoopbackRobot::read(RobotState& state) {
    state.q = state_.q;
    state.v = state_.v;
    state.stamp_ns = clock_->now_ns();
    std::fill(state.group_stamps_ns.begin(), state.group_stamps_ns.end(), state.stamp_ns);
    return {};
}

Status LoopbackRobot::write(const RobotCommand& command) {
    if (command.q.size() != state_.q.size() || command.v.size() != state_.v.size())
        return make_error(ErrorCode::invalid_argument, "loopback robot: command size mismatch");
    state_.q = command.q;
    state_.v = command.v;
    return {};
}

struct Engine::Impl {
    Impl(ModuleSet m, std::shared_ptr<RobotInterface> r, EngineOptions o, const Context& c)
        : modules(std::move(m)), robot(std::move(r)), options(o), context(c) {
        if (!context.clock) context.clock = make_steady_clock();
    }

    ModuleSet modules;
    std::shared_ptr<RobotInterface> robot;
    EngineOptions options;
    Context context;
    double dt{0.0};
    std::int64_t dt_ns{0};
    std::int64_t timeout_ns{0};
    std::int64_t prev_stamp_ns{-1};

    struct Group {
        std::string name;
        std::vector<std::pair<Index, Index>> q_segments;  ///< (start, size) in q.
        std::vector<Index> dofs;                          ///< Velocity indices.
        Mask mask;                                        ///< Size nv.
        bool fresh{true};
        bool attached{false};
        bool stale_last{false};
        bool hold_pending{false};  ///< Targets set while stale must be dropped on the next fresh cycle.
        bool refresh{false};       ///< estimate(): tracking error recomputed this cycle.
        std::int64_t fresh_since_ns{-1};  ///< Clock time the group turned fresh while detached; -1 = stale.
        std::int64_t err_stamp_ns{-1};
    };
    std::vector<Group> groups;
    std::vector<std::size_t> joint_group;  ///< Per JointId.
    std::vector<std::size_t> dof_group;    ///< Per dof.
    bool has_default{false};  ///< The last group is "default" (falls back to RobotState::stamp_ns).
    EngineStats stats;
    RobotState state;
    RobotCommand command;
    SolveResult result;
    std::vector<std::shared_ptr<EngineObserver>> observers;
    std::atomic<bool> paused{false};
    std::atomic<std::uint64_t> passes{0};
    std::atomic<bool> resume_pending{false};
    std::atomic<double> pending_rate{0.0};  ///< > 0: rate_hz to switch to at the next cycle.
    double held_time{0.0};                  ///< Engine time spent paused; module time is `now` minus this.
    bool stepped{false};   ///< A cycle has succeeded; `now` must not go below stats.time after that.
    bool fresh{true};      ///< All groups.
    bool attached{false};  ///< All groups.
    bool written{false};

    struct Mimic {
        Index follower_q, leader_q;
        double multiplier, offset;
    };
    std::vector<Mimic> mimics;
    std::vector<bool> feedback;  ///< Per dof: tracking error carried into the measured estimate.
    Eigen::VectorXd ideal_q, ideal_v;
    Eigen::VectorXd err;  ///< measured - ideal, held between feedback samples.
    Eigen::VectorXd measured_q;
    bool has_measured{false};

    struct PositionLimit {
        JointId joint;
        Index iq;
        Index iv;
        double lower;
        double upper;
    };
    Eigen::VectorXd max_v;  ///< Scaled model velocity limit per dof; inf where the model has none.
    Eigen::VectorXd raw_q;
    Eigen::VectorXd dv;
    std::vector<PositionLimit> position_limits;

    void init_guard() {
        const auto& model = *modules.model();
        const auto& limit = model.velocity_limit();
        max_v.resize(model.nv());
        for (Index i = 0; i < model.nv(); ++i) {
            const double l = limit[i];
            // Same validity rule as the solver box: 0 (URDF without a limit) and pinocchio's "max" mean none.
            max_v[i] = std::isfinite(l) && l > 1e-10 && l < 1e20 ? l * options.safety.velocity_scale
                                                                 : std::numeric_limits<double>::infinity();
        }
        raw_q.resize(model.nq());
        dv.resize(model.nv());
        for (const auto& j : model.joints())
            if (j.nq == 1 && j.nv == 1 && model.has_position_limit(j.id))
                position_limits.push_back({j.id, j.idx_q, j.idx_v, model.lower_position_limit()[j.idx_q],
                                           model.upper_position_limit()[j.idx_q]});
    }

    Status init_feedback() {
        const auto& model = *modules.model();
        std::vector<bool> follower(model.joints().size(), false);
        for (const MimicRelation& rel : model.mimics()) {
            Result<JointId> f = model.joint_id(rel.follower);
            Result<JointId> l = model.joint_id(rel.leader);
            if (!f || !l) continue;
            follower[*f] = true;
            const JointInfo& fj = model.joints()[*f];
            const JointInfo& lj = model.joints()[*l];
            if (fj.nq == 1 && fj.nv == 1 && lj.nq == 1 && lj.nv == 1)
                mimics.push_back({fj.idx_q, lj.idx_q, rel.multiplier, rel.offset});
        }
        feedback.assign(static_cast<std::size_t>(model.nv()), false);
        auto scalar = [&](const JointInfo& j) { return j.id != 0 && j.nv == 1 && !follower[j.id]; };
        if (options.feedback_joints.empty()) {
            for (const JointInfo& j : model.joints())
                if (scalar(j)) feedback[static_cast<std::size_t>(j.idx_v)] = true;
        }
        for (const std::string& name : options.feedback_joints) {
            if (Result<JointId> id = model.joint_id(name)) {
                const JointInfo& j = model.joints()[*id];
                if (!scalar(j))
                    return make_error(ErrorCode::invalid_argument, "engine: feedback joint '" + name +
                                                                       "' is not a scalar, non-mimic joint");
                feedback[static_cast<std::size_t>(j.idx_v)] = true;
                continue;
            }
            bool matched = false;
            for (const JointInfo& j : model.joints())
                if (scalar(j) && glob_match(name, j.name)) {
                    feedback[static_cast<std::size_t>(j.idx_v)] = true;
                    matched = true;
                }
            if (!matched)
                return make_error(ErrorCode::invalid_argument,
                                  "engine: feedback joint '" + name + "' matches no scalar, non-mimic joint");
        }
        err = Eigen::VectorXd::Zero(model.nv());
        return {};
    }

    Status init_groups() {
        const auto& model = *modules.model();
        Result<FeedbackLayout> layout = resolve_feedback_groups(model, options.feedback_groups);
        if (!layout) return std::move(layout).error();
        const std::size_t n = layout->names.size();
        groups.resize(n);
        for (std::size_t g = 0; g < n; ++g) {
            groups[g].name = layout->names[g];
            groups[g].mask = Mask(static_cast<std::size_t>(model.nv()));
        }
        has_default = n > options.feedback_groups.size();
        joint_group = std::move(layout->joint_group);
        dof_group.assign(static_cast<std::size_t>(model.nv()), 0);
        for (const JointInfo& j : model.joints()) {
            const std::size_t g = joint_group[j.id];
            if (g >= n) continue;
            Group& grp = groups[g];
            if (j.nq > 0) {
                if (!grp.q_segments.empty() &&
                    grp.q_segments.back().first + grp.q_segments.back().second == j.idx_q)
                    grp.q_segments.back().second += j.nq;
                else
                    grp.q_segments.emplace_back(j.idx_q, j.nq);
            }
            for (Index i = j.idx_v; i < j.idx_v + j.nv; ++i) {
                grp.dofs.push_back(i);
                grp.mask.set(static_cast<std::size_t>(i));
                dof_group[static_cast<std::size_t>(i)] = g;
            }
        }
        state.group_stamps_ns.assign(n, -1);
        command.group_attached.assign(n, 0);
        return {};
    }

    std::int64_t group_stamp(std::size_t g) const noexcept {
        const std::int64_t s = state.group_stamps_ns[g];
        return s < 0 && has_default && g + 1 == groups.size() ? state.stamp_ns : s;
    }

    void derive_mimics(Eigen::VectorXd& q) const {
        for (const Mimic& m : mimics) q[m.follower_q] = m.multiplier * q[m.leader_q] + m.offset;
    }

    /// Commanded state <- measured, projected onto the model (mimics derived, scalar limits clamped).
    void sync_all() {
        ideal_q = state.q;
        derive_mimics(ideal_q);
        for (const auto& p : position_limits) ideal_q[p.iq] = std::clamp(ideal_q[p.iq], p.lower, p.upper);
        ideal_v.setZero();
        err.setZero();
        for (Group& g : groups) g.err_stamp_ns = -1;
        modules.resynchronize(false);
    }

    /// Same for group g's joints only; resynchronizes the modules driving them (rebase: dropping held targets).
    void sync_group(std::size_t g, bool counted, bool rebase) {
        Group& grp = groups[g];
        for (const auto& [start, size] : grp.q_segments)
            ideal_q.segment(start, size) = state.q.segment(start, size);
        derive_mimics(ideal_q);
        for (const auto& p : position_limits)
            if (joint_group[p.joint] == g) ideal_q[p.iq] = std::clamp(ideal_q[p.iq], p.lower, p.upper);
        for (const Index i : grp.dofs) ideal_v[i] = err[i] = 0.0;
        grp.err_stamp_ns = -1;
        if (groups.size() == 1)
            modules.resynchronize(rebase);
        else
            modules.resynchronize(rebase, grp.mask);
        if (counted) stats.resyncs++;
    }

    /// Held tracking error of the attached groups -> measured estimate.
    Status estimate() {
        const auto& model = *modules.model();
        bool refresh = false;
        for (std::size_t g = 0; g < groups.size(); ++g) {
            Group& grp = groups[g];
            const std::int64_t stamp = group_stamp(g);
            grp.refresh = grp.attached && (stamp < 0 || stamp != grp.err_stamp_ns);
            refresh = refresh || grp.refresh;
            if (grp.refresh) grp.err_stamp_ns = stamp;
            if (!grp.attached) grp.err_stamp_ns = -1;
        }
        if (refresh)
            if (auto s = model.difference(ideal_q, state.q, dv); !s) return s;
        for (Index i = 0; i < model.nv(); ++i) {
            const Group& grp = groups[dof_group[static_cast<std::size_t>(i)]];
            if (!grp.attached)
                err[i] = 0.0;
            else if (grp.refresh)
                err[i] = feedback[static_cast<std::size_t>(i)] ? dv[i] : 0.0;
        }
        if (auto s = model.integrate(ideal_q, err, measured_q); !s) return s;
        derive_mimics(measured_q);
        return {};
    }

    std::string dof_name(Index i) const {
        const auto& model = *modules.model();
        const auto& joint = model.joints()[model.velocity_owner()[static_cast<std::size_t>(i)]];
        if (joint.nv == 1) return joint.name;
        return joint.name + "[" + std::to_string(i - joint.idx_v) + "]";
    }

    Error reject(ErrorCode code, std::string message) {
        stats.guard_rejects++;
        return make_error(code, "engine guard: " + std::move(message));
    }

    /// r -> command; alloc-free unless it fails.
    Status guard(const SolveResult& r) {
        const auto& model = *modules.model();
        const CommandGuardOptions& g = options.safety;
        if (r.q_next.size() != model.nq() || r.v.size() != model.nv())
            return reject(ErrorCode::invalid_argument,
                          "command has nq=" + std::to_string(r.q_next.size()) +
                              ", nv=" + std::to_string(r.v.size()) + "; the model has nq=" +
                              std::to_string(model.nq()) + ", nv=" + std::to_string(model.nv()));
        if (!r.q_next.allFinite() || !r.v.allFinite())
            return reject(ErrorCode::invalid_argument, "command is not finite");
        command.q = r.q_next;
        command.v = r.v;
        const bool clamp = g.mode == GuardMode::clamp;
        const double tol = g.tolerance;
        bool clamped = false;
        for (Index i = 0; i < model.nv(); ++i) {
            double v = command.v[i];
            if (std::abs(v) > max_v[i] + tol) {
                if (!clamp)
                    return reject(ErrorCode::out_of_range, "joint " + dof_name(i) + " velocity " +
                                                               std::to_string(v) + " exceeds limit " +
                                                               std::to_string(max_v[i]));
                v = std::clamp(v, -max_v[i], max_v[i]);
            }
            dv[i] = (v - command.v[i]) * dt;
            if (v != command.v[i]) clamped = true;
            command.v[i] = v;
        }
        // Apply only the velocity change, keeping what modules wrote into q in prepare() (e.g. a chassis root).
        if (clamped) {
            raw_q = command.q;
            if (auto s = model.integrate(raw_q, dv, command.q); !s) return s;
        }
        for (const auto& p : position_limits) {
            const double q = command.q[p.iq];
            double lower = std::min(p.lower, ideal_q[p.iq]);
            double upper = std::max(p.upper, ideal_q[p.iq]);
            if (has_measured) {
                lower = std::min(lower, measured_q[p.iq]);
                upper = std::max(upper, measured_q[p.iq]);
            }
            if (q >= lower - tol && q <= upper + tol) continue;
            if (!clamp)
                return reject(ErrorCode::out_of_range, "joint " + model.joints()[p.joint].name +
                                                           " position " + std::to_string(q) + " outside [" +
                                                           std::to_string(p.lower) + ", " +
                                                           std::to_string(p.upper) + "]");
            const double qc = std::clamp(q, lower, upper);
            command.v[p.iv] += (qc - q) / dt;
            command.q[p.iq] = qc;
            clamped = true;
        }
        if (clamped) stats.guard_clamps++;
        return {};
    }

    bool take_pass() noexcept {
        std::uint64_t n = passes.load(std::memory_order_acquire);
        while (n > 0 && !passes.compare_exchange_weak(n, n - 1, std::memory_order_acq_rel)) {
        }
        return n > 0;
    }

    void set_rate(double hz) noexcept {
        options.rate_hz = hz;
        dt = 1.0 / hz;
        dt_ns = static_cast<std::int64_t>(std::llround(1e9 / hz));
    }

    bool apply_rate() noexcept {
        const double hz = pending_rate.exchange(0.0, std::memory_order_acq_rel);
        if (hz <= 0.0 || hz == options.rate_hz) return false;
        set_rate(hz);
        return true;
    }

    Status read() {
        state.stamp_ns = -1;
        std::fill(state.group_stamps_ns.begin(), state.group_stamps_ns.end(), -1);
        if (auto s = robot->read(state); !s) return s;
        if (state.group_stamps_ns.size() != groups.size())
            return make_error(ErrorCode::invalid_argument, "robot state: the driver resized group_stamps_ns");
        if (state.stamp_ns >= 0 && state.stamp_ns == prev_stamp_ns) stats.repeated_reads++;
        prev_stamp_ns = state.stamp_ns;
        const auto& model = *modules.model();
        if (state.q.size() != model.nq() || state.v.size() != model.nv())
            return make_error(ErrorCode::invalid_argument,
                              "robot state has nq=" + std::to_string(state.q.size()) +
                                  ", nv=" + std::to_string(state.v.size()) + "; the model has nq=" +
                                  std::to_string(model.nq()) + ", nv=" + std::to_string(model.nv()));
        if (!state.q.allFinite() || !state.v.allFinite())
            return make_error(ErrorCode::invalid_argument, "robot state is not finite");
        if (auto s = model.normalize(state.q); !s)
            return make_error(s.error().code, "robot state: " + s.error().message);
        return {};
    }
};

Engine::Engine(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
Engine::~Engine() = default;
Engine::Engine(Engine&&) noexcept = default;
Engine& Engine::operator=(Engine&&) noexcept = default;

Result<Engine> Engine::create(ModuleSet modules, std::shared_ptr<RobotInterface> robot, EngineOptions options,
                              const Context& context) {
    if (!robot) return make_error(ErrorCode::invalid_argument, "engine: robot is null");
    if (!std::isfinite(options.rate_hz) || options.rate_hz <= 0.0)
        return make_error(ErrorCode::invalid_argument, "engine: rate_hz must be > 0");
    if (!std::isfinite(options.feedback_timeout_s) || options.feedback_timeout_s < 0.0)
        return make_error(ErrorCode::invalid_argument, "engine: feedback_timeout_s must be >= 0");
    if (options.max_failed_cycles < 1)
        return make_error(ErrorCode::invalid_argument, "engine: max_failed_cycles must be >= 1");
    const CommandGuardOptions& g = options.safety;
    if (!std::isfinite(g.velocity_scale) || g.velocity_scale <= 0.0)
        return make_error(ErrorCode::invalid_argument, "engine: safety.velocity_scale must be > 0");
    if (!std::isfinite(g.tolerance) || g.tolerance < 0.0)
        return make_error(ErrorCode::invalid_argument, "engine: safety.tolerance must be >= 0");
    if (auto s = validate_thread(options.thread); !s) return std::move(s).error();
    if (!modules.configured()) {
        if (auto s = modules.set_feedback_timeout(options.feedback_timeout_s); !s)
            return std::move(s).error();
        if (auto s = modules.configure(); !s) return std::move(s).error();
    }
    auto impl = std::make_unique<Impl>(std::move(modules), std::move(robot), options, context);
    if (std::llround(1e9 / options.rate_hz) <= 0)
        return make_error(ErrorCode::invalid_argument, "engine: rate_hz is too high");
    impl->set_rate(options.rate_hz);
    impl->timeout_ns = static_cast<std::int64_t>(std::llround(options.feedback_timeout_s * 1e9));
    if (options.feedback_timeout_s > 0.0 && impl->timeout_ns <= 0) impl->timeout_ns = 1;
    if (auto s = impl->init_groups(); !s) return std::move(s).error();
    if (auto s = impl->read(); !s) return std::move(s).error();
    impl->init_guard();
    if (auto s = impl->init_feedback(); !s) return std::move(s).error();
    impl->ideal_v = Eigen::VectorXd::Zero(impl->state.v.size());
    impl->measured_q = impl->state.q;
    impl->sync_all();
    const std::int64_t created_ns = impl->context.clock->now_ns();
    impl->attached = true;
    for (std::size_t i = 0; i < impl->groups.size(); ++i) {
        Impl::Group& grp = impl->groups[i];
        grp.attached = impl->timeout_ns <= 0;
        const std::int64_t stamp = impl->group_stamp(i);
        if (!grp.attached && stamp >= 0 && created_ns - stamp <= impl->timeout_ns)
            grp.fresh_since_ns = created_ns;
        grp.hold_pending = !grp.attached && grp.fresh_since_ns < 0;
        impl->attached = impl->attached && grp.attached;
    }
    impl->result.q_next = impl->ideal_q;
    impl->result.v = impl->ideal_v;
    impl->command.q = impl->ideal_q;
    impl->command.v = impl->ideal_v;
    return Engine(std::move(impl));
}

Status Engine::step(double now) {
    Status s = step_impl(now);
    if (!s) {
        impl_->stats.failed_cycles++;
        if (impl_->context.log.enabled(LogLevel::error))
            impl_->context.log.log(LogLevel::error, "engine", s.error().message);
    }
    return s;
}

Status Engine::step_impl(double now) {
    Impl& d = *impl_;
    if (!std::isfinite(now)) return make_error(ErrorCode::invalid_argument, "step: now must be finite");
    if (d.stepped && now < d.stats.time) {
        std::ostringstream m;
        m << "step: now went backwards (" << now << " < " << d.stats.time << ")";
        return make_error(ErrorCode::invalid_argument, m.str());
    }
    d.apply_rate();
    const Clock& clock = *d.context.clock;
    for (const auto& o : d.observers) o->before_step(*this);
    const std::int64_t start = clock.now_ns();
    if (auto s = d.read(); !s) return s;
    d.fresh = true;
    for (Impl::Group& g : d.groups) {
        const std::int64_t stamp = d.group_stamp(static_cast<std::size_t>(&g - d.groups.data()));
        const bool fresh = d.timeout_ns <= 0 || (stamp >= 0 && start - stamp <= d.timeout_ns);
        if (!fresh && !g.stale_last && d.context.log.enabled(LogLevel::warn))
            d.context.log.log(LogLevel::warn, "engine",
                              (d.groups.size() > 1 ? "feedback group '" + g.name + "': " : std::string()) +
                                  (stamp < 0 ? "robot state has no stamp; feedback stale"
                                             : "robot state is stale (" +
                                                   std::to_string((start - stamp) / 1000000) + " ms old)"));
        g.stale_last = !fresh;
        g.fresh = fresh;
        d.fresh = d.fresh && fresh;
        if (!fresh) {
            g.attached = false;
            d.attached = false;
            g.fresh_since_ns = -1;
            g.hold_pending = true;
        } else if (!g.attached && g.fresh_since_ns < 0) {
            g.fresh_since_ns = start;
        }
    }
    d.written = false;
    d.has_measured = false;
    const bool held = d.paused.load(std::memory_order_acquire) && !d.take_pass();
    if (held) {
        d.held_time += d.dt;
        d.stats.held_cycles++;
    } else {
        const bool resume = d.resume_pending.exchange(false, std::memory_order_acq_rel);
        for (std::size_t i = 0; i < d.groups.size(); ++i) {
            Impl::Group& g = d.groups[i];
            if (g.fresh && !g.attached) {
                // Track the robot through the window; counted once per detach. Targets set before the robot
                // state was known are dropped.
                d.sync_group(i, g.hold_pending, g.hold_pending);
                g.hold_pending = false;
                // Feedback must stay fresh for a whole timeout window before the commanded state is trusted.
                if (start - g.fresh_since_ns >= d.timeout_ns) {
                    g.attached = true;
                    g.fresh_since_ns = -1;
                    d.stats.attaches++;
                }
            } else if (g.attached && resume) {
                d.sync_group(i, true, false);
            }
        }
        d.attached = true;
        for (const Impl::Group& g : d.groups) {
            d.attached = d.attached && g.attached;
            d.has_measured = d.has_measured || g.attached;
        }
        if (d.has_measured)
            if (auto s = d.estimate(); !s) return s;
        // Modules have advanced from here on; a failure re-seeds them from the last written command.
        const auto fail = [&d](Status s) {
            d.modules.resynchronize(false);
            return s;
        };
        if (auto s = d.modules.step(d.ideal_q, d.ideal_v, d.has_measured ? &d.measured_q : nullptr,
                                    now - d.held_time, d.dt, d.result);
            !s)
            return fail(std::move(s));
        const Mask& aligned = d.modules.aligned();
        for (Index i = 0; i < d.err.size(); ++i)
            if (aligned.test(static_cast<std::size_t>(i))) d.err[i] = 0.0;
        if (d.options.safety.mode == GuardMode::off) {
            d.command.q = d.result.q_next;
            d.command.v = d.result.v;
        } else if (auto s = d.guard(d.result); !s) {
            return fail(std::move(s));
        }
        d.command.fresh = d.attached;
        for (std::size_t i = 0; i < d.groups.size(); ++i) d.command.group_attached[i] = d.groups[i].attached;
        if (auto s = d.robot->write(d.command); !s) return fail(std::move(s));
        d.written = true;
        if (!d.fresh) d.stats.stale_cycles++;
        d.ideal_q = d.command.q;
        d.ideal_v = d.command.v;
    }
    d.stats.cycles++;
    d.stats.time = now;
    d.stepped = true;
    const std::int64_t cycle_ns = clock.now_ns() - start;
    d.stats.cycle_hist.add(cycle_ns);
    d.stats.last_cycle_s = static_cast<double>(cycle_ns) * 1e-9;
    if (d.stats.last_cycle_s > d.stats.max_cycle_s) d.stats.max_cycle_s = d.stats.last_cycle_s;
    for (const auto& o : d.observers) o->after_step(*this);
    return {};
}

void Engine::add_observer(std::shared_ptr<EngineObserver> observer) {
    if (observer) impl_->observers.push_back(std::move(observer));
}

Status Engine::run(double duration_s, const std::atomic<bool>* stop) {
    Impl& d = *impl_;
    if (std::isnan(duration_s)) return make_error(ErrorCode::invalid_argument, "engine: run duration is NaN");
    if (!(duration_s > 0.0) && !stop)
        return make_error(ErrorCode::invalid_argument,
                          "engine: run needs a positive duration or a stop flag");
    ThreadScope scope;
    if (d.options.realtime) scope.apply(d.options.thread, d.context.log);
    const bool bounded = duration_s > 0.0;
    const Clock& clock = *d.context.clock;
    const double t0 = d.stats.time;
    // `now` = base + k * dt, re-based whenever the rate changes so it stays exact within a segment.
    double base = t0;
    std::uint64_t k = 0;
    std::int64_t deadline = clock.now_ns();
    int failed = 0;
    for (;;) {
        if (stop && stop->load(std::memory_order_relaxed)) break;
        if (const double old_dt = d.dt; d.apply_rate()) {
            base += static_cast<double>(k) * old_dt;
            k = 0;
        }
        const double dt = d.dt;
        const double now = base + static_cast<double>(k + 1) * dt;
        if (bounded && now - t0 > duration_s + 0.5 * dt) break;
        ++k;
        Status s = step(now);
        if (d.dt != dt) {  // Applied inside step(): the next cycle starts a new segment.
            base = now;
            k = 0;
        }
        if (!s) {
            if (++failed >= d.options.max_failed_cycles) return s;
        } else {
            failed = 0;
        }
        deadline += d.dt_ns;
        std::int64_t remaining = deadline - clock.now_ns();
        if (remaining >
            d.dt_ns) {  // Clock stalled or went back: wait one period instead of the accumulated gap.
            remaining = d.dt_ns;
            deadline = clock.now_ns() + remaining;
        }
        if (remaining > 0) {
            // Sliced so a stop request is seen within ~50 ms even at very low rates.
            constexpr std::int64_t slice_ns = 50'000'000;
            const auto wake = std::chrono::steady_clock::now() + std::chrono::nanoseconds(remaining);
            for (auto t = std::chrono::steady_clock::now(); t < wake; t = std::chrono::steady_clock::now()) {
                if (stop && stop->load(std::memory_order_relaxed)) return {};
                std::this_thread::sleep_for(
                    std::min<std::chrono::nanoseconds>(wake - t, std::chrono::nanoseconds(slice_ns)));
            }
        } else {
            const double lateness = static_cast<double>(-remaining) * 1e-9;
            if (lateness > d.stats.max_lateness_s) d.stats.max_lateness_s = lateness;
            d.stats.overruns++;
            deadline = clock.now_ns();  // Re-anchor instead of bursting to catch up.
        }
    }
    return {};
}

void Engine::set_paused(bool paused) noexcept {
    impl_->passes.store(0, std::memory_order_release);
    const bool was = impl_->paused.exchange(paused, std::memory_order_acq_rel);
    if (paused)
        impl_->resume_pending.store(false, std::memory_order_release);
    else if (was)
        impl_->resume_pending.store(true, std::memory_order_release);
}
bool Engine::paused() const noexcept { return impl_->paused.load(std::memory_order_acquire); }
void Engine::step_paused(std::uint64_t cycles) noexcept {
    impl_->passes.fetch_add(cycles, std::memory_order_acq_rel);
}

Status Engine::set_rate_hz(double rate_hz) {
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0 || std::llround(1e9 / rate_hz) <= 0)
        return make_error(ErrorCode::invalid_argument, "engine: rate_hz must be > 0 and <= 1e9");
    impl_->pending_rate.store(rate_hz, std::memory_order_release);
    return {};
}

double Engine::dt() const noexcept { return impl_->dt; }
const EngineOptions& Engine::options() const noexcept { return impl_->options; }
const Context& Engine::context() const noexcept { return impl_->context; }
const EngineStats& Engine::stats() const noexcept { return impl_->stats; }
void Engine::reset_stats() noexcept {
    const double time = impl_->stats.time;
    impl_->stats = EngineStats{};
    impl_->stats.time = time;
}
const RobotState& Engine::state() const noexcept { return impl_->state; }
const SolveResult& Engine::result() const noexcept { return impl_->result; }
const RobotCommand& Engine::command() const noexcept { return impl_->command; }
bool Engine::written() const noexcept { return impl_->written; }
bool Engine::fresh() const noexcept { return impl_->fresh; }
bool Engine::attached() const noexcept { return impl_->attached; }
std::size_t Engine::group_count() const noexcept { return impl_->groups.size(); }
const std::string& Engine::group_name(std::size_t i) const noexcept {
    static const std::string empty;
    return i < impl_->groups.size() ? impl_->groups[i].name : empty;
}
bool Engine::group_fresh(std::size_t i) const noexcept {
    return i < impl_->groups.size() && impl_->groups[i].fresh;
}
bool Engine::group_attached(std::size_t i) const noexcept {
    return i < impl_->groups.size() && impl_->groups[i].attached;
}
const Eigen::VectorXd& Engine::ideal() const noexcept { return impl_->ideal_q; }
const Eigen::VectorXd* Engine::measured() const noexcept {
    return impl_->has_measured ? &impl_->measured_q : nullptr;
}
ModuleSet& Engine::modules() noexcept { return impl_->modules; }
const ModuleSet& Engine::modules() const noexcept { return impl_->modules; }
RobotInterface& Engine::robot() noexcept { return *impl_->robot; }

}  // namespace kstone
