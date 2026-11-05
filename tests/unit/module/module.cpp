#include "kstone/module/module.hpp"

#include "kstone/control/constraints.hpp"
#include "kstone/control/tasks.hpp"

#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load_arm() {
    auto m = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

Mask mask_of(const ModelPtr& m, const std::vector<std::string>& names) {
    std::vector<JointId> ids;
    for (const auto& n : names) ids.push_back(m->joint_id(n).value());
    return m->velocity_mask(ids);
}

Index v_index(const ModelPtr& m, const std::string& name) {
    return m->joints()[m->joint_id(name).value()].idx_v;
}

// Drives `joints` toward `target` with one JointTask; claims them while `claiming`.
struct Probe final : Module {
    Probe(std::string id, std::vector<std::string> j, double t = 1.0)
        : Module(std::move(id)), joints(std::move(j)), target(t) {}
    const char* type() const noexcept override { return "probe"; }
    std::vector<std::string> dependencies() const override { return deps; }

    Status configure(ModuleSetup& setup) override {
        mask = Mask(static_cast<std::size_t>(setup.model()->nv()));
        model = setup.model();
        if (!joints.empty()) {
            mask = mask_of(setup.model(), joints);
            auto task = std::make_shared<JointTask>(
                joints, Eigen::VectorXd::Constant(static_cast<Index>(joints.size()), target));
            Result<TaskHandle> h = setup.add_task("joints", task);
            if (!h) return std::move(h).error();
            handle = *h;
            if (Status s = setup.set_driven(mask); !s) return s;
        }
        if (constraint)
            if (Status s = setup.add_constraint("limit", constraint); !s) return s;
        if (lock_joints) return setup.lock(mask);
        if (fail) return make_error(ErrorCode::invalid_argument, "boom");
        return {};
    }
    Status prepare(Cycle&) override {
        if (!claim_joints.empty()) claim_mask = mask_of(model, claim_joints);
        ++prepares;
        return {};
    }
    Claim claim() const override {
        if (!claiming) return {nullptr, 0, on_held};
        return {claim_joints.empty() ? &mask : &claim_mask, priority, on_held};
    }
    Status on_preempted(const Cycle&, const std::string& by) override {
        ++preemptions;
        preempted_by = by;
        contributes_at_preempt = contributes;
        return {};
    }
    Status reset(const Cycle&) override {
        ++resets;
        return {};
    }
    Status contribute(const Cycle&) override {
        ++contributes;
        set_task_active(foreign.valid() ? foreign : handle, task_on);
        return {};
    }

    ModelPtr model;
    std::vector<std::string> joints;
    double target;
    std::vector<std::string> deps;
    std::vector<std::string> claim_joints;  ///< Empty = claim every driven joint.
    std::shared_ptr<Constraint> constraint;
    Mask mask, claim_mask;
    TaskHandle handle, foreign;
    bool claiming{false};
    bool lock_joints{false};
    bool fail{false};
    bool task_on{true};
    int priority{0};
    OnHeld on_held{OnHeld::wait};
    int prepares{0}, resets{0}, contributes{0};
    int preemptions{0}, contributes_at_preempt{-1};
    std::string preempted_by;
};

struct Rig {
    ModelPtr model = load_arm();
    ModuleSet set = make();
    Eigen::VectorXd q = model->neutral();
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv());
    SolveResult out;
    double now = 0.0;

    ModuleSet make() {
        auto s = ModuleSet::create(model);
        REQUIRE(s);
        return std::move(*s);
    }
    std::shared_ptr<Probe> add(std::shared_ptr<Probe> p) {
        REQUIRE(set.add(p));
        return p;
    }
    void step() {
        now += 0.01;
        Status s = set.step(q, v, now, 0.01, out);
        REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
    }
    bool active(const std::string& task) const {
        for (const auto& r : out.residuals)
            if (r.name == task) return true;
        return false;
    }
};

std::string held(const Module& m) { return m.held_by() ? *m.held_by() : std::string("-"); }
}  // namespace

TEST_CASE("module set: higher priority claim holds the lower one") {
    Rig r;
    auto low = r.add(std::make_shared<Probe>("low", std::vector<std::string>{"j1"}));
    auto high = r.add(std::make_shared<Probe>("high", std::vector<std::string>{"j1", "j2"}));
    low->claiming = high->claiming = true;
    low->priority = 1;
    high->priority = 2;
    REQUIRE(r.set.configure());
    r.step();
    CHECK(high->running());
    CHECK_FALSE(low->running());
    CHECK(held(*low) == "high");
    CHECK(r.active("high/joints"));
    CHECK_FALSE(r.active("low/joints"));
    CHECK(low->prepares == 1);  // held modules still prepare
    CHECK(low->contributes == 0);

    high->claiming = false;
    r.step();
    CHECK(low->running());
    // high drives j1, which low now holds.
    CHECK_FALSE(high->running());
    CHECK(held(*high) == "low");
}

TEST_CASE("module set: equal priority is first come, first served") {
    Rig r;
    auto a = r.add(std::make_shared<Probe>("a", std::vector<std::string>{"j1"}));
    auto b = r.add(std::make_shared<Probe>("b", std::vector<std::string>{"j1"}));
    REQUIRE(r.set.configure());
    b->claiming = true;
    r.step();
    a->claiming = true;  // later claim loses despite earlier step order
    r.step();
    CHECK(b->running());
    CHECK(held(*a) == "b");
    b->claiming = false;
    r.step();
    CHECK(a->running());
    CHECK(held(*b) == "a");

    // Both start in the same cycle: step order decides.
    a->claiming = false;
    r.step();
    a->claiming = b->claiming = true;
    r.step();
    CHECK(a->running());
    CHECK(held(*b) == "a");
}

TEST_CASE("module set: driven overlap holds a module and resumes it with reset") {
    Rig r;
    auto claimer = r.add(std::make_shared<Probe>("claimer", std::vector<std::string>{"j1"}));
    auto arm = r.add(std::make_shared<Probe>("arm", std::vector<std::string>{"j1", "j2", "j3"}));
    auto other = r.add(std::make_shared<Probe>("other", std::vector<std::string>{"j4"}));
    REQUIRE(r.set.configure());
    r.step();
    CHECK(arm->resets == 1);
    CHECK(arm->contributes == 1);
    CHECK(r.active("arm/joints"));

    claimer->claiming = true;
    r.step();
    CHECK(held(*arm) == "claimer");
    CHECK_FALSE(arm->running());
    CHECK(other->running());
    CHECK(arm->contributes == 1);
    CHECK_FALSE(r.active("arm/joints"));
    CHECK(r.active("other/joints"));

    claimer->claiming = false;
    r.step();
    CHECK(arm->running());
    CHECK(arm->resets == 2);
    CHECK(other->resets == 1);
    CHECK(r.active("arm/joints"));

    // A low-priority claim on joints a higher-priority claimant drives (but does not claim) is held.
    arm->claim_joints = {"j3"};
    arm->claiming = true;
    arm->priority = 5;
    claimer->claiming = true;
    r.step();
    CHECK(arm->running());
    CHECK(held(*claimer) == "arm");
    CHECK(other->running());
}

TEST_CASE("module set: on_held abort calls on_preempted once per preemption, wait does not") {
    Rig r;
    auto high = r.add(std::make_shared<Probe>("high", std::vector<std::string>{"j1"}));
    auto waiter = r.add(std::make_shared<Probe>("waiter", std::vector<std::string>{"j1", "j2"}));
    auto aborter = r.add(std::make_shared<Probe>("aborter", std::vector<std::string>{"j1", "j3"}));
    aborter->on_held = OnHeld::abort;
    REQUIRE(r.set.configure());
    CHECK(std::string(to_string(OnHeld::abort)) == "abort");
    r.step();
    REQUIRE(waiter->running());
    REQUIRE(aborter->running());

    high->claiming = true;
    high->priority = 10;
    r.step();
    CHECK(held(*waiter) == "high");
    CHECK(held(*aborter) == "high");
    CHECK(waiter->preemptions == 0);
    CHECK(waiter->preempted() == 1);
    CHECK(aborter->preemptions == 1);
    CHECK(aborter->preempted() == 1);
    CHECK(aborter->preempted_by == "high");
    CHECK(aborter->contributes_at_preempt == 1);  // before any contribute() of this cycle
    for (int i = 0; i < 3; ++i) r.step();
    CHECK(aborter->preemptions == 1);  // only on the transition
    CHECK(aborter->preempted() == 1);

    high->claiming = false;
    r.step();
    CHECK(aborter->running());
    CHECK(aborter->resets == 2);  // resumes through reset() as with wait
    CHECK(waiter->resets == 2);

    high->claiming = true;
    r.step();
    CHECK(aborter->preemptions == 2);
    CHECK(aborter->preempted() == 2);

    // Disabling is not a preemption.
    high->claiming = false;
    r.step();
    REQUIRE(aborter->running());
    aborter->set_enabled(false);
    r.step();
    CHECK_FALSE(aborter->running());
    CHECK(aborter->preemptions == 2);
    CHECK(aborter->preempted() == 2);
}

TEST_CASE("module set: disabling a module removes its tasks and constraints") {
    Rig r;
    auto mover = r.add(std::make_shared<Probe>("mover", std::vector<std::string>{"j1"}, 1.5));
    auto limit = r.add(std::make_shared<Probe>("limit", std::vector<std::string>{}));
    limit->constraint = std::make_shared<JointVelocityConstraint>(std::vector<std::string>{"j1"},
                                                                  Eigen::VectorXd::Constant(1, 0.1));
    REQUIRE(r.set.configure());
    const Index j1 = v_index(r.model, "j1");
    r.step();
    CHECK(std::abs(r.out.v[j1]) <= 0.1 + 1e-6);
    CHECK(std::abs(r.out.v[j1]) > 0.05);

    limit->set_enabled(false);
    r.step();
    CHECK_FALSE(limit->running());
    CHECK(std::abs(r.out.v[j1]) > 0.2);

    limit->set_enabled(true);
    mover->set_enabled(false);
    r.step();
    CHECK_FALSE(r.active("mover/joints"));
    CHECK(r.out.v.isZero());
    CHECK(mover->prepares == 2);

    mover->set_enabled(true);
    r.step();
    CHECK(mover->resets == 2);
    CHECK(std::abs(r.out.v[j1]) <= 0.1 + 1e-6);
}

TEST_CASE("module set: set_task_active switches a task within the same step") {
    Rig r;
    auto p = r.add(std::make_shared<Probe>("p", std::vector<std::string>{"j1"}));
    REQUIRE(r.set.configure());
    r.step();
    CHECK(r.active("p/joints"));
    p->task_on = false;
    r.step();
    CHECK_FALSE(r.active("p/joints"));
    p->task_on = true;
    r.step();
    CHECK(r.active("p/joints"));
}

TEST_CASE("module set: a task handle only switches its own module's task") {
    Rig r;
    auto a = r.add(std::make_shared<Probe>("a", std::vector<std::string>{"j1"}));
    auto b = r.add(std::make_shared<Probe>("b", std::vector<std::string>{"j2"}));
    REQUIRE(r.set.configure());
    CHECK(a->handle.valid());
    CHECK_FALSE(TaskHandle{}.valid());
    b->foreign = a->handle;
    b->task_on = false;
    r.step();
    CHECK(r.active("a/joints"));
    CHECK(r.active("b/joints"));
}

TEST_CASE("module set: lock merges into the solver options") {
    Rig l;
    l.add(std::make_shared<Probe>("mover", std::vector<std::string>{"j1", "j2"}, 1.0));
    auto locker = std::make_shared<Probe>("locker", std::vector<std::string>{"j1"});
    locker->lock_joints = true;
    locker->set_enabled(false);
    l.add(locker);
    REQUIRE(l.set.configure());
    CHECK(l.set.solver().options().locked.test(static_cast<std::size_t>(v_index(l.model, "j1"))));
    l.step();
    CHECK(l.out.v[v_index(l.model, "j1")] == 0.0);
    CHECK(std::abs(l.out.v[v_index(l.model, "j2")]) > 0.0);
}

TEST_CASE("module set: configure orders by dependency and rejects bad graphs") {
    {
        Rig r;
        auto b = r.add(std::make_shared<Probe>("b", std::vector<std::string>{}));
        auto a = r.add(std::make_shared<Probe>("a", std::vector<std::string>{}));
        auto c = r.add(std::make_shared<Probe>("c", std::vector<std::string>{}));
        b->deps = {"a"};
        REQUIRE(r.set.configure());
        REQUIRE(r.set.modules().size() == 3);
        CHECK(r.set.modules()[0]->id() == "a");
        CHECK(r.set.modules()[1]->id() == "b");
        CHECK(r.set.modules()[2]->id() == "c");
        CHECK(r.set.configured());
        CHECK_FALSE(r.set.configure());
        CHECK_FALSE(r.set.add(std::make_shared<Probe>("d", std::vector<std::string>{})));
        (void)c;
    }
    {
        Rig r;
        auto a = r.add(std::make_shared<Probe>("a", std::vector<std::string>{}));
        a->deps = {"ghost"};
        auto s = r.set.configure();
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::not_found);
        CHECK_FALSE(r.set.configured());
        CHECK_FALSE(r.set.step(r.q, r.v, 0.0, 0.01, r.out));
    }
    {
        Rig r;
        auto a = r.add(std::make_shared<Probe>("a", std::vector<std::string>{}));
        auto b = r.add(std::make_shared<Probe>("b", std::vector<std::string>{}));
        a->deps = {"b"};
        b->deps = {"a"};
        auto s = r.set.configure();
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::invalid_argument);
        CHECK(s.error().message.find("a, b") != std::string::npos);
    }
    {
        Rig r;
        r.add(std::make_shared<Probe>("a", std::vector<std::string>{}));
        CHECK_FALSE(r.set.add(std::make_shared<Probe>("a", std::vector<std::string>{})));
        CHECK_FALSE(r.set.add(std::make_shared<Probe>("", std::vector<std::string>{})));
        CHECK_FALSE(r.set.add(nullptr));
        auto bad = r.add(std::make_shared<Probe>("bad", std::vector<std::string>{}));
        bad->fail = true;
        auto s = r.set.configure();
        REQUIRE_FALSE(s);
        CHECK(s.error().message.find("module 'bad'") != std::string::npos);
    }
}

TEST_CASE("module set: step validates its inputs") {
    Rig r;
    r.add(std::make_shared<Probe>("a", std::vector<std::string>{"j1"}));
    REQUIRE(r.set.configure());
    CHECK_FALSE(r.set.step(Eigen::VectorXd::Zero(1), r.v, 0.0, 0.01, r.out));
    CHECK_FALSE(r.set.step(r.q, r.v, 0.0, 0.0, r.out));
    CHECK_FALSE(r.set.step(r.q, r.v, std::nan(""), 0.01, r.out));
    Eigen::VectorXd bad = r.q;
    bad[0] = std::nan("");
    CHECK_FALSE(r.set.step(bad, r.v, 0.0, 0.01, r.out));
    CHECK(r.set.step(r.q, r.v, 0.0, 0.01, r.out));
}

TEST_CASE("module set: step normalizes a near-unit root quaternion and rejects a zero one") {
    ModelOptions free;
    free.root = RootJoint::free_flyer;
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), free, Context::defaults());
    REQUIRE(model);
    auto set = ModuleSet::create(*model);
    REQUIRE(set);
    REQUIRE(set->add(std::make_shared<Probe>("a", std::vector<std::string>{"j1"})));
    REQUIRE(set->configure());
    Eigen::VectorXd q = (*model)->neutral();
    const Eigen::VectorXd v = Eigen::VectorXd::Zero((*model)->nv());
    SolveResult out;
    q.segment<4>(3) *= 1.0001;
    REQUIRE(set->step(q, v, 0.01, 0.01, out));
    CHECK(out.q_next.segment<4>(3).norm() == doctest::Approx(1.0).epsilon(1e-12));
    q.segment<4>(3).setZero();
    const Status s = set->step(q, v, 0.02, 0.01, out);
    REQUIRE_FALSE(s);
    CHECK(s.error().message.find("quaternion of norm 0") != std::string::npos);
}
