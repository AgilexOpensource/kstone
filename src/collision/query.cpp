#include "kstone/collision/query.hpp"

#include "kstone/model/pinocchio.hpp"

#include <pinocchio/algorithm/jacobian.hpp>

#include "internal.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace kstone {
namespace {

coal::Transform3s to_coal(const pinocchio::SE3& pose) {
    return coal::Transform3s(pose.rotation(), pose.translation());
}

// Upper bound on how far any point of a geometry inside its bounding sphere moved between two poses.
double motion_bound(const pinocchio::SE3& from, const pinocchio::SE3& to,
                    const coal::CollisionGeometry& geometry) {
    const Eigen::Vector3d c(geometry.aabb_center);
    // ||R_to - R_from||_2 = 2 sin(theta / 2) = sqrt(3 - trace(R_from^T R_to)).
    const double spin = std::sqrt(std::max(3.0 - (from.rotation().transpose() * to.rotation()).trace(), 0.0));
    return (to.act(c) - from.act(c)).norm() + spin * geometry.aabb_radius;
}

}  // namespace

struct CollisionQuery::Impl {
    CollisionModelPtr model;
    std::shared_ptr<const CollisionWorld> world;
    std::uint64_t world_revision{~std::uint64_t{0}};
    std::uint64_t layout{0};

    std::vector<CollisionPair> pairs;
    std::vector<coal::ComputeDistance> functors;  ///< One per pair (keeps the GJK solver state).
    std::vector<coal::Vec3s> guesses;             ///< Warm start per pair.
    std::vector<pinocchio::SE3> robot_poses;
    std::vector<Proximity> proximities;
    Proximity nearest;
    std::vector<std::pair<double, std::size_t>> order;             ///< Distance lower bound per pair.
    std::vector<Eigen::Vector3d> robot_centers, obstacle_centers;  ///< World bounding-sphere centers.
    // Per pair: both poses at its last exact evaluation and the distance found there (-inf before any).
    std::vector<std::pair<pinocchio::SE3, pinocchio::SE3>> last_poses;
    std::vector<double> last_distance;

    coal::DistanceRequest request{true, true};
    coal::DistanceResult result;
    Eigen::Matrix<double, 6, Eigen::Dynamic> J;

    // Last evaluated inputs and results; a repeated query at the same poses reuses them.
    bool cached{false};
    double cached_threshold{0.0};
    bool cached_exact{false};
    std::vector<pinocchio::SE3> cached_robot;
    std::vector<std::pair<Eigen::Isometry3d, bool>> cached_obstacles;
    std::vector<Proximity> cached_proximities;
    Proximity cached_nearest;

    void rebuild() {
        const auto& geometries = model->impl().geometries;
        pairs.clear();
        functors.clear();
        for (const auto& [a, b] : model->pairs()) pairs.push_back(CollisionPair{a, b, false});
        const std::size_t obstacles = world ? world->impl().obstacles.size() : 0;
        // Geometry welded to the world cannot move away from an obstacle: no pair.
        for (std::size_t g = 0; g < geometries.size(); ++g)
            for (std::size_t o = 0; o < obstacles && model->joints()[g] != 0; ++o)
                pairs.push_back(CollisionPair{static_cast<Index>(g), static_cast<Index>(o), true});
        functors.reserve(pairs.size());
        for (const CollisionPair& p : pairs) {
            const coal::CollisionGeometry* g1 = geometries[static_cast<std::size_t>(p.first)].get();
            const coal::CollisionGeometry* g2 =
                p.obstacle ? world->impl().obstacles[static_cast<std::size_t>(p.second)].geometry.get()
                           : geometries[static_cast<std::size_t>(p.second)].get();
            functors.emplace_back(g1, g2);
        }
        guesses.assign(pairs.size(), coal::Vec3s(1.0, 0.0, 0.0));
        last_poses.assign(pairs.size(), {pinocchio::SE3::Identity(), pinocchio::SE3::Identity()});
        last_distance.assign(pairs.size(), -std::numeric_limits<double>::infinity());
        proximities.clear();
        proximities.reserve(pairs.size());
        order.clear();
        order.reserve(pairs.size());
        obstacle_centers.resize(obstacles);
        world_revision = world ? world->revision() : 0;
        cached = false;
        ++layout;
    }
};

CollisionQuery::CollisionQuery(CollisionModelPtr model, std::shared_ptr<const CollisionWorld> world)
    : impl_(std::make_unique<Impl>()) {
    impl_->model = std::move(model);
    impl_->world = std::move(world);
    impl_->request.gjk_initial_guess = coal::GJKInitialGuess::CachedGuess;
    // Default GJK tolerance gives ~1e-4 witness/normal error; EPA stays default (tight EPA misreports depth).
    impl_->request.gjk_tolerance = 1e-12;
    if (impl_->model) {
        impl_->robot_poses.resize(impl_->model->geometries().size());
        impl_->robot_centers.resize(impl_->model->geometries().size());
        impl_->J.setZero(6, impl_->model->model()->nv());
        impl_->rebuild();
    }
}

CollisionQuery::~CollisionQuery() = default;
CollisionQuery::CollisionQuery(CollisionQuery&&) noexcept = default;
CollisionQuery& CollisionQuery::operator=(CollisionQuery&&) noexcept = default;

Status CollisionQuery::update(const Workspace& workspace, double threshold, bool exact_nearest) {
    Impl& s = *impl_;
    if (!s.model) return make_error(ErrorCode::invalid_argument, "collision query: null collision model");
    if (workspace.model() != s.model->model())
        return make_error(ErrorCode::invalid_argument, "collision query: workspace belongs to another model");
    if (!workspace.updated())
        return make_error(ErrorCode::invalid_argument, "collision query: workspace not updated");
    if (!(threshold >= 0.0))
        return make_error(ErrorCode::invalid_argument, "collision query: threshold must be >= 0");
    if (s.world && s.world->revision() != s.world_revision) s.rebuild();

    const pinocchio::Data& data = pinocchio_data(workspace);
    const auto& joints = s.model->joints();
    const auto& placements = s.model->impl().placements;
    for (std::size_t g = 0; g < s.robot_poses.size(); ++g)
        s.robot_poses[g] = data.oMi[joints[g]] * placements[g];

    static const std::vector<CollisionWorld::Obstacle> no_obstacles;
    const auto& obstacles = s.world ? s.world->impl().obstacles : no_obstacles;
    // Absorbs round-off jitter of a resting robot; compared against the cached pose, so the error cannot accumulate.
    constexpr double pose_tol = 1e-12;
    const auto same_pose = [](const pinocchio::SE3& a, const pinocchio::SE3& b) {
        return (a.rotation() - b.rotation()).cwiseAbs().maxCoeff() <= pose_tol &&
               (a.translation() - b.translation()).cwiseAbs().maxCoeff() <= pose_tol;
    };
    const auto same_inputs = [&] {
        if (s.cached_robot.size() != s.robot_poses.size() || s.cached_obstacles.size() != obstacles.size())
            return false;
        for (std::size_t g = 0; g < s.robot_poses.size(); ++g)
            if (!same_pose(s.cached_robot[g], s.robot_poses[g])) return false;
        for (std::size_t o = 0; o < obstacles.size(); ++o)
            if (s.cached_obstacles[o].second != obstacles[o].enabled ||
                s.cached_obstacles[o].first.matrix() != obstacles[o].pose.matrix())
                return false;
        return true;
    };
    if (s.cached && threshold <= s.cached_threshold && (s.cached_exact || !exact_nearest) && same_inputs()) {
        s.proximities.clear();
        for (const Proximity& p : s.cached_proximities)
            if (p.distance <= threshold) s.proximities.push_back(p);
        s.nearest = exact_nearest           ? s.cached_nearest
                    : s.proximities.empty() ? Proximity{}
                                            : s.proximities.front();
        return {};
    }
    s.cached = false;

    // Lower bounds on the exact distance: the bounding-sphere gap, and the last exact distance minus how far
    // either side moved since (distance is 1-Lipschitz in point motion). Visiting pairs by ascending bound,
    // the scan stops once the bound exceeds the threshold and, for an exact nearest, the nearest so far.
    const auto& geometries = s.model->impl().geometries;
    const auto se3_of = [&](const CollisionPair& p, bool second) -> pinocchio::SE3 {
        if (!second) return s.robot_poses[static_cast<std::size_t>(p.first)];
        if (!p.obstacle) return s.robot_poses[static_cast<std::size_t>(p.second)];
        const Eigen::Isometry3d& pose = obstacles[static_cast<std::size_t>(p.second)].pose;
        return pinocchio::SE3(pose.linear(), pose.translation());
    };
    for (std::size_t g = 0; g < geometries.size(); ++g)
        s.robot_centers[g] = s.robot_poses[g].act(Eigen::Vector3d(geometries[g]->aabb_center));
    for (std::size_t o = 0; o < obstacles.size(); ++o)
        s.obstacle_centers[o] = obstacles[o].pose * Eigen::Vector3d(obstacles[o].geometry->aabb_center);
    s.order.clear();
    for (std::size_t i = 0; i < s.pairs.size(); ++i) {
        const CollisionPair& p = s.pairs[i];
        const auto a = static_cast<std::size_t>(p.first), b = static_cast<std::size_t>(p.second);
        if (p.obstacle && !obstacles[b].enabled) continue;
        const double r2 = p.obstacle ? obstacles[b].geometry->aabb_radius : geometries[b]->aabb_radius;
        const Eigen::Vector3d& c2 = p.obstacle ? s.obstacle_centers[b] : s.robot_centers[b];
        double bound = (s.robot_centers[a] - c2).norm() - geometries[a]->aabb_radius - r2;
        if (std::isfinite(s.last_distance[i])) {
            const coal::CollisionGeometry& g2 = p.obstacle ? *obstacles[b].geometry : *geometries[b];
            const double moved = motion_bound(s.last_poses[i].first, s.robot_poses[a], *geometries[a]) +
                                 motion_bound(s.last_poses[i].second, se3_of(p, true), g2);
            bound = std::max(bound, s.last_distance[i] - moved);
        }
        s.order.emplace_back(bound, i);
    }
    std::sort(s.order.begin(), s.order.end());

    s.proximities.clear();
    s.nearest = Proximity{};
    for (const auto& [bound, i] : s.order) {
        if (bound > threshold && (!exact_nearest || bound > s.nearest.distance)) break;
        const CollisionPair& p = s.pairs[i];
        const pinocchio::SE3 pose1 = se3_of(p, false), pose2 = se3_of(p, true);
        const coal::Transform3s tf1 = to_coal(pose1), tf2 = to_coal(pose2);
        bool finite = false;
        for (int attempt = 0; attempt < 2 && !finite; ++attempt) {
            s.request.cached_gjk_guess = attempt == 0 ? s.guesses[i] : coal::Vec3s(1.0, 0.0, 0.0);
            s.result.clear();
            try {
                s.functors[i](tf1, tf2, s.request, s.result);
            } catch (const std::exception& e) {
                if (attempt == 1)
                    return make_error(
                        ErrorCode::internal,
                        "collision query '" + pair_name(static_cast<Index>(i)) + "': " + e.what());
                continue;
            }
            finite = std::isfinite(s.result.min_distance) && s.result.normal.allFinite() &&
                     s.result.nearest_points[0].allFinite() && s.result.nearest_points[1].allFinite();
        }
        if (!finite)
            return make_error(ErrorCode::internal, "collision query '" + pair_name(static_cast<Index>(i)) +
                                                       "': non-finite distance");
        s.guesses[i] = s.result.cached_gjk_guess;
        s.last_poses[i] = {pose1, pose2};
        s.last_distance[i] = s.result.min_distance;

        Proximity prox;
        prox.pair = static_cast<Index>(i);
        prox.distance = s.result.min_distance;
        prox.point1 = s.result.nearest_points[0];
        prox.point2 = s.result.nearest_points[1];
        prox.normal = s.result.normal;
        const double norm = prox.normal.norm();
        if (norm > 0.0) prox.normal /= norm;
        if (prox.distance < s.nearest.distance ||
            (prox.distance == s.nearest.distance && prox.pair < s.nearest.pair))
            s.nearest = prox;
        if (prox.distance <= threshold) s.proximities.push_back(prox);
    }
    std::sort(s.proximities.begin(), s.proximities.end(), [](const Proximity& a, const Proximity& b) {
        return a.distance < b.distance || (a.distance == b.distance && a.pair < b.pair);
    });
    if (!exact_nearest) s.nearest = s.proximities.empty() ? Proximity{} : s.proximities.front();
    s.cached_robot = s.robot_poses;
    s.cached_obstacles.resize(obstacles.size());
    for (std::size_t o = 0; o < obstacles.size(); ++o)
        s.cached_obstacles[o] = {obstacles[o].pose, obstacles[o].enabled};
    s.cached_proximities = s.proximities;
    s.cached_nearest = s.nearest;
    s.cached_threshold = threshold;
    s.cached_exact = exact_nearest;
    s.cached = true;
    return {};
}

const std::vector<Proximity>& CollisionQuery::proximities() const noexcept { return impl_->proximities; }
const Proximity& CollisionQuery::nearest() const noexcept { return impl_->nearest; }
const std::vector<CollisionPair>& CollisionQuery::pairs() const noexcept { return impl_->pairs; }
std::uint64_t CollisionQuery::layout() const noexcept { return impl_->layout; }
const CollisionModelPtr& CollisionQuery::model() const noexcept { return impl_->model; }
const std::shared_ptr<const CollisionWorld>& CollisionQuery::world() const noexcept { return impl_->world; }

std::string CollisionQuery::pair_name(Index pair) const {
    const Impl& s = *impl_;
    if (pair < 0 || static_cast<std::size_t>(pair) >= s.pairs.size()) return {};
    const CollisionPair& p = s.pairs[static_cast<std::size_t>(pair)];
    const auto& geometries = s.model->geometries();
    std::string second = p.obstacle ? s.world->impl().obstacles[static_cast<std::size_t>(p.second)].name
                                    : geometries[static_cast<std::size_t>(p.second)].name;
    return geometries[static_cast<std::size_t>(p.first)].name + "/" + second;
}

Status CollisionQuery::jacobian(const Workspace& workspace, const Proximity& proximity,
                                Eigen::Ref<Eigen::RowVectorXd, 0, Eigen::InnerStride<>> out) {
    Impl& s = *impl_;
    if (!s.model || workspace.model() != s.model->model())
        return make_error(ErrorCode::invalid_argument,
                          "collision jacobian: workspace belongs to another model");
    if (out.size() != s.model->model()->nv())
        return make_error(ErrorCode::invalid_argument, "collision jacobian: output must have nv entries");
    if (proximity.pair < 0 || static_cast<std::size_t>(proximity.pair) >= s.pairs.size())
        return make_error(ErrorCode::out_of_range, "collision jacobian: unknown pair");
    const pinocchio::Model& pin = pinocchio_model(*s.model->model());
    const pinocchio::Data& data = pinocchio_data(workspace);
    const CollisionPair& p = s.pairs[static_cast<std::size_t>(proximity.pair)];
    const Eigen::Vector3d& n = proximity.normal;

    out.setZero();
    // d_dot = n . (v(point2) - v(point1)); v(p) = v_joint + w x (p - joint origin).
    const auto add = [&](JointId joint, const Eigen::Vector3d& point, double sign) {
        if (joint == 0) return;
        s.J.setZero();
        pinocchio::getJointJacobian(pin, data, joint, pinocchio::LOCAL_WORLD_ALIGNED, s.J);
        const Eigen::Vector3d r = point - data.oMi[joint].translation();
        out.noalias() += sign * (n.transpose() * s.J.topRows<3>());
        out.noalias() += sign * (r.cross(n).transpose() * s.J.bottomRows<3>());
    };
    const auto& joints = s.model->joints();
    add(joints[static_cast<std::size_t>(p.first)], proximity.point1, -1.0);
    if (!p.obstacle) add(joints[static_cast<std::size_t>(p.second)], proximity.point2, 1.0);
    return {};
}

}  // namespace kstone
