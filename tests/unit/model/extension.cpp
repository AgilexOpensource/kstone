#include "kstone/core/context.hpp"
#include "kstone/model/loader.hpp"
#include "kstone/model/pinocchio.hpp"

#include <doctest/doctest.h>

#include <memory>
#include <string>

using namespace kstone;

namespace {
// A toy format: the document text is the number of revolute-z links in a serial chain.
class ChainLoader final : public ModelLoader {
  public:
    explicit ChainLoader(std::string tag) : tag_(std::move(tag)) {}
    std::string format() const override { return tag_; }
    std::vector<std::string> extensions() const override { return {"chain"}; }
    Result<LoadedModel> load(const LoadRequest& request) const override {
        std::string text(request.text);
        if (text.empty()) {
            auto read = read_text_file(request.path);
            if (!read) return read.error();
            text = *read;
        }
        const int n = std::stoi(text);
        if (n <= 0) return Error{ErrorCode::parse_error, "chain length must be positive"};
        LoadedModel out;
        out.model.name = tag_;
        pinocchio::JointIndex parent = 0;
        for (int i = 0; i < n; ++i) {
            const std::string name = "c" + std::to_string(i);
            const pinocchio::SE3 placement(Eigen::Matrix3d::Identity(),
                                           Eigen::Vector3d(i == 0 ? 0.0 : 1.0, 0, 0));
            parent = out.model.addJoint(parent, pinocchio::JointModelRZ(), placement, name);
            out.model.appendBodyToJoint(parent, pinocchio::Inertia::FromSphere(1.0, 0.1));
            out.model.addJointFrame(parent);
        }
        return out;
    }

  private:
    std::string tag_;
};
}  // namespace

TEST_CASE("custom loaders extend the registry") {
    Context ctx = Context::defaults();
    ctx.loaders.add(std::make_shared<ChainLoader>("chain"));
    CHECK(ctx.loaders.formats() == std::vector<std::string>{"urdf", "mjcf", "chain"});
    CHECK(ctx.loaders.find_extension(".CHAIN"));
    CHECK(ctx.loaders.find_format("Chain"));

    auto m = Model::load(ModelSource::text("3", "chain"), {}, ctx);
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    CHECK((*m)->name() == "chain");
    CHECK((*m)->nv() == 3);
    CHECK((*m)->total_mass() == doctest::Approx(3.0));
    CHECK(pinocchio_model(**m).njoints == 4);

    CHECK(Model::load(ModelSource::text("0", "chain"), {}, ctx).error().code == ErrorCode::parse_error);

    ModelOptions floating;
    floating.root = RootJoint::free_flyer;
    CHECK(Model::load(ModelSource::text("2", "chain"), floating, ctx).error().code == ErrorCode::unsupported);

    Workspace ws(*m);
    Eigen::VectorXd q = (*m)->neutral();
    q[1] = 1.5707963267948966;  // pi / 2
    REQUIRE(ws.update(q));
    Eigen::Isometry3d tip;
    REQUIRE(ws.joint_pose(3, tip));
    CHECK(tip.translation().isApprox(Eigen::Vector3d(1.0, 1.0, 0.0)));
    CHECK(pinocchio_data(ws).oMi[3].translation().isApprox(tip.translation()));
}

TEST_CASE("later registrations override built-in formats") {
    Context ctx = Context::defaults();
    ctx.loaders.add(std::make_shared<ChainLoader>("urdf"));
    auto m = Model::load(ModelSource::text("2", "urdf"), {}, ctx);
    REQUIRE(m);
    CHECK((*m)->nv() == 2);
    CHECK(ctx.loaders.formats().size() == 2);
}

TEST_CASE("make_model infers the root and validates mimics") {
    pinocchio::Model pm;
    const auto root = root_joint_model(RootJoint::free_flyer);
    REQUIRE(root);
    const auto base = pm.addJoint(0, *root, pinocchio::SE3::Identity(), "floating");
    const auto a = pm.addJoint(base, pinocchio::JointModelPX(), pinocchio::SE3::Identity(), "a");
    pm.addJoint(a, pinocchio::JointModelPX(), pinocchio::SE3::Identity(), "b");
    CHECK_FALSE(root_joint_model(RootJoint::fixed));

    auto bad = make_model(pm, {MimicRelation{"b", "ghost", 1.0, 0.0}});
    CHECK(bad.error().code == ErrorCode::invalid_argument);

    auto m = make_model(pm, {MimicRelation{"b", "a", 2.0, 0.0}});
    REQUIRE(m);
    CHECK((*m)->root() == RootJoint::free_flyer);
    CHECK((*m)->nv() == 8);
    CHECK((*m)->mimics().size() == 1);
}
