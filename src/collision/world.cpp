#include "internal.hpp"
#include <algorithm>

namespace kstone {

CollisionWorld::CollisionWorld() : impl_(std::make_unique<Impl>()) {}
CollisionWorld::~CollisionWorld() = default;

Result<ObstacleId> CollisionWorld::add(std::string name, Shape shape, const Eigen::Isometry3d& pose) {
    if (name.empty()) return make_error(ErrorCode::invalid_argument, "obstacle name must not be empty");
    if (find(name)) return make_error(ErrorCode::invalid_argument, "duplicate obstacle '" + name + "'");
    if (!collision_detail::finite_pose(pose))
        return make_error(ErrorCode::invalid_argument, "obstacle '" + name + "': pose must be finite");
    auto geometry = collision_detail::make_geometry(shape);
    if (!geometry)
        return make_error(geometry.error().code, "obstacle '" + name + "': " + geometry.error().message);
    Obstacle o;
    o.id = next_id_++;
    o.name = std::move(name);
    o.shape = std::move(shape);
    o.pose = pose;
    o.geometry = std::move(*geometry);
    impl_->obstacles.push_back(std::move(o));
    ++revision_;
    return impl_->obstacles.back().id;
}

Status CollisionWorld::remove(ObstacleId id) {
    auto& v = impl_->obstacles;
    auto it = std::find_if(v.begin(), v.end(), [id](const Obstacle& o) { return o.id == id; });
    if (it == v.end()) return make_error(ErrorCode::not_found, "unknown obstacle id " + std::to_string(id));
    v.erase(it);
    ++revision_;
    return {};
}

Status CollisionWorld::set_pose(ObstacleId id, const Eigen::Isometry3d& pose) {
    if (!collision_detail::finite_pose(pose))
        return make_error(ErrorCode::invalid_argument, "obstacle pose must be finite");
    for (Obstacle& o : impl_->obstacles)
        if (o.id == id) {
            o.pose = pose;
            return {};
        }
    return make_error(ErrorCode::not_found, "unknown obstacle id " + std::to_string(id));
}

Status CollisionWorld::set_velocity(ObstacleId id, const Eigen::Vector3d& linear,
                                    const Eigen::Vector3d& angular) {
    if (!linear.allFinite() || !angular.allFinite())
        return make_error(ErrorCode::invalid_argument, "obstacle velocity must be finite");
    for (Obstacle& o : impl_->obstacles)
        if (o.id == id) {
            o.linear = linear;
            o.angular = angular;
            return {};
        }
    return make_error(ErrorCode::not_found, "unknown obstacle id " + std::to_string(id));
}

Status CollisionWorld::set_enabled(ObstacleId id, bool enabled) {
    for (Obstacle& o : impl_->obstacles)
        if (o.id == id) {
            o.enabled = enabled;
            return {};
        }
    return make_error(ErrorCode::not_found, "unknown obstacle id " + std::to_string(id));
}

void CollisionWorld::clear() {
    impl_->obstacles.clear();
    ++revision_;
}

std::size_t CollisionWorld::size() const noexcept { return impl_->obstacles.size(); }

Result<ObstacleId> CollisionWorld::find(std::string_view name) const {
    for (const Obstacle& o : impl_->obstacles)
        if (o.name == name) return o.id;
    return make_error(ErrorCode::not_found, "unknown obstacle '" + std::string(name) + "'");
}

const std::string& CollisionWorld::name(std::size_t i) const { return impl_->obstacles.at(i).name; }
const Shape& CollisionWorld::shape(std::size_t i) const { return impl_->obstacles.at(i).shape; }
const Eigen::Isometry3d& CollisionWorld::pose(std::size_t i) const { return impl_->obstacles.at(i).pose; }
bool CollisionWorld::enabled(std::size_t i) const { return impl_->obstacles.at(i).enabled; }
const Eigen::Vector3d& CollisionWorld::linear_velocity(std::size_t i) const {
    return impl_->obstacles.at(i).linear;
}
const Eigen::Vector3d& CollisionWorld::angular_velocity(std::size_t i) const {
    return impl_->obstacles.at(i).angular;
}
Eigen::Vector3d CollisionWorld::point_velocity(std::size_t i, const Eigen::Vector3d& point) const {
    const Obstacle& o = impl_->obstacles.at(i);
    return o.linear + o.angular.cross(point - o.pose.translation());
}

}  // namespace kstone
