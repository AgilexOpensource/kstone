#include "kstone/model/pinocchio.hpp"

#include <coal/BVH/BVH_model.h>
#include <coal/mesh_loader/loader.h>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "internal.hpp"
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <exception>
#include <map>
#include <set>
#include <sstream>
#include <urdf_parser/urdf_parser.h>

namespace kstone {
namespace {

namespace fs = std::filesystem;

NamePair ordered(const std::string& a, const std::string& b) {
    return a < b ? NamePair(a, b) : NamePair(b, a);
}

bool contains(const std::vector<std::string>& values, const std::string& value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

/// Index of the first group the joint belongs to, or -1.
int group_of(const std::string& joint, const std::vector<BodyGroup>& groups) {
    for (std::size_t g = 0; g < groups.size(); ++g) {
        const BodyGroup& group = groups[g];
        if (contains(group.exclude_joints, joint)) continue;
        if (contains(group.joint_names, joint) ||
            std::any_of(group.joint_prefixes.begin(), group.joint_prefixes.end(),
                        [&](const std::string& prefix) { return joint.rfind(prefix, 0) == 0; }))
            return static_cast<int>(g);
    }
    return -1;
}

Status drop_initial_collisions(const pinocchio::Model& pin, const std::vector<CollisionGeometry>& geometries,
                               const std::vector<JointId>& joints, const CollisionModel::Impl& impl,
                               const Eigen::VectorXd& initial, const Context& context,
                               std::vector<std::pair<Index, Index>>& pairs) {
    const Eigen::VectorXd q = initial.size() == 0 ? pinocchio::neutral(pin) : initial;
    if (q.size() != pin.nq || !q.allFinite())
        return make_error(ErrorCode::invalid_argument, "self collision: initial must be finite with " +
                                                           std::to_string(pin.nq) + " values");
    pinocchio::Data data(pin);
    pinocchio::forwardKinematics(pin, data, q);
    const auto pose = [&](Index g) {
        const pinocchio::SE3 p =
            data.oMi[joints[static_cast<std::size_t>(g)]] * impl.placements[static_cast<std::size_t>(g)];
        return coal::Transform3s(p.rotation(), p.translation());
    };
    coal::DistanceRequest request{false, true};
    const auto kept = std::remove_if(pairs.begin(), pairs.end(), [&](const std::pair<Index, Index>& p) {
        coal::DistanceResult result;
        try {
            coal::distance(impl.geometries[static_cast<std::size_t>(p.first)].get(), pose(p.first),
                           impl.geometries[static_cast<std::size_t>(p.second)].get(), pose(p.second), request,
                           result);
        } catch (const std::exception&) {
            return false;
        }
        if (!(result.min_distance < 0)) return false;
        std::ostringstream msg;
        msg << "self pair '" << geometries[static_cast<std::size_t>(p.first)].name << "' / '"
            << geometries[static_cast<std::size_t>(p.second)].name
            << "' penetrates at the initial posture (distance " << result.min_distance << " m); excluded";
        context.log.log(LogLevel::warn, "collision.model", msg.str());
        return true;
    });
    pairs.erase(kept, pairs.end());
    return {};
}

struct SourceText {
    std::string text;
    fs::path dir;  ///< Directory for relative references inside the document.
};

Result<SourceText> read_source(const ModelSource& source, const Context& context) {
    SourceText out;
    if (source.kind == ModelSource::Kind::text) {
        out.text = source.content;
        out.dir = source.base_dir;
        return out;
    }
    auto path = context.resources.resolve(source.location, source.base_dir);
    if (!path) return std::move(path).error();
    auto text = read_text_file(*path);
    if (!text) return std::move(text).error();
    out.text = std::move(*text);
    out.dir = path->parent_path();
    return out;
}

Result<std::vector<NamePair>> parse_srdf(const std::string& text) {
    namespace pt = boost::property_tree;
    pt::ptree tree;
    try {
        std::istringstream stream(text);
        pt::read_xml(stream, tree);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::parse_error, std::string("invalid SRDF: ") + e.what());
    }
    const auto robot = tree.get_child_optional("robot");
    if (!robot) return make_error(ErrorCode::parse_error, "invalid SRDF: no <robot> element");
    std::set<NamePair> pairs;
    for (const auto& [tag, node] : *robot) {
        if (tag != "disable_collisions") continue;
        const std::string a = node.get<std::string>("<xmlattr>.link1", "");
        const std::string b = node.get<std::string>("<xmlattr>.link2", "");
        if (a.empty() || b.empty())
            return make_error(ErrorCode::parse_error,
                              "invalid SRDF: disable_collisions needs link1 and link2");
        if (a != b) pairs.insert(ordered(a, b));
    }
    return std::vector<NamePair>(pairs.begin(), pairs.end());
}

Result<Eigen::Matrix3Xd> load_mesh_vertices(const fs::path& path, const Eigen::Vector3d& scale) {
    try {
        coal::MeshLoader loader;
        const coal::BVHModelPtr_t mesh = loader.load(path.string(), scale);
        if (!mesh || !mesh->vertices || mesh->num_vertices == 0)
            return make_error(ErrorCode::parse_error, "mesh '" + path.string() + "' has no vertices");
        Eigen::Matrix3Xd out(3, mesh->num_vertices);
        for (unsigned int i = 0; i < mesh->num_vertices; ++i)
            out.col(static_cast<Index>(i)) = (*mesh->vertices)[i];
        return out;
    } catch (const std::exception& e) {
        return make_error(ErrorCode::parse_error, "mesh '" + path.string() + "': " + e.what());
    }
}

Eigen::Isometry3d to_isometry(const ::urdf::Pose& pose) {
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    double x, y, z, w;
    pose.rotation.getQuaternion(x, y, z, w);
    out.linear() = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
    out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return out;
}

Result<Shape> urdf_shape(const ::urdf::Geometry& geometry, const fs::path& dir, const Context& context) {
    switch (geometry.type) {
        case ::urdf::Geometry::SPHERE:
            return Shape::sphere(static_cast<const ::urdf::Sphere&>(geometry).radius);
        case ::urdf::Geometry::BOX: {
            const auto& d = static_cast<const ::urdf::Box&>(geometry).dim;
            return Shape::box(d.x, d.y, d.z);
        }
        case ::urdf::Geometry::CYLINDER: {
            const auto& c = static_cast<const ::urdf::Cylinder&>(geometry);
            return Shape::cylinder(c.radius, c.length);
        }
#ifdef KSTONE_URDF_HAS_CAPSULE
        case ::urdf::Geometry::CAPSULE: {
            const auto& c = static_cast<const ::urdf::Capsule&>(geometry);
            return Shape::capsule(c.radius, c.length);
        }
#endif
        case ::urdf::Geometry::MESH: {
            const auto& m = static_cast<const ::urdf::Mesh&>(geometry);
            auto path = context.resources.resolve(m.filename, dir);
            if (!path) return std::move(path).error();
            auto vertices = load_mesh_vertices(*path, Eigen::Vector3d(m.scale.x, m.scale.y, m.scale.z));
            if (!vertices) return std::move(vertices).error();
            return Shape::convex(std::move(*vertices));
        }
    }
    return make_error(ErrorCode::unsupported, "unsupported URDF geometry type");
}

}  // namespace

CollisionModel::CollisionModel() : impl_(std::make_unique<Impl>()) {}
CollisionModel::~CollisionModel() = default;

Result<Index> CollisionModel::geometry_index(std::string_view name) const {
    for (std::size_t i = 0; i < geometries_.size(); ++i)
        if (geometries_[i].name == name) return static_cast<Index>(i);
    return make_error(ErrorCode::not_found, "unknown collision geometry '" + std::string(name) + "'");
}

Result<std::vector<NamePair>> srdf_disabled_pairs(const ModelSource& srdf, const Context& context) {
    auto source = read_source(srdf, context);
    if (!source) return std::move(source).error();
    return parse_srdf(source->text);
}

Result<CollisionModelPtr> CollisionModel::create(ModelPtr model, std::vector<CollisionGeometry> geometries,
                                                 const CollisionModelOptions& options,
                                                 const Context& context) {
    if (!model) return make_error(ErrorCode::invalid_argument, "collision model: null model");
    const pinocchio::Model& pin = pinocchio_model(*model);
    std::shared_ptr<CollisionModel> out(new CollisionModel());
    out->model_ = model;
    std::map<std::string, JointId> body_of;  // frame and geometry names -> joint
    std::set<std::string> names;
    for (CollisionGeometry& g : geometries) {
        if (g.name.empty())
            return make_error(ErrorCode::invalid_argument, "collision geometry name must not be empty");
        if (!names.insert(g.name).second)
            return make_error(ErrorCode::invalid_argument, "duplicate collision geometry '" + g.name + "'");
        auto frame = model->frame_id(g.frame);
        if (!frame)
            return make_error(ErrorCode::not_found,
                              "collision geometry '" + g.name + "': unknown frame '" + g.frame + "'");
        if (!collision_detail::finite_pose(g.placement))
            return make_error(ErrorCode::invalid_argument,
                              "collision geometry '" + g.name + "': placement must be finite");
        auto geometry = collision_detail::make_geometry(g.shape);
        if (!geometry)
            return make_error(geometry.error().code,
                              "collision geometry '" + g.name + "': " + geometry.error().message);
        const pinocchio::Frame& f = pin.frames[*frame];
        const JointId joint = f.parentJoint;
        out->impl_->geometries.push_back(std::move(*geometry));
        out->impl_->placements.push_back(f.placement * collision_detail::to_se3(g.placement));
        out->joints_.push_back(joint);
        body_of.emplace(g.frame, joint);
        body_of.emplace(g.name, joint);
    }
    out->geometries_ = std::move(geometries);
    if (!options.self_collision) return CollisionModelPtr(std::move(out));

    std::set<NamePair> disabled;
    if (options.srdf) {
        auto pairs = srdf_disabled_pairs(*options.srdf, context);
        if (!pairs) return std::move(pairs).error();
        disabled.insert(pairs->begin(), pairs->end());
    }
    std::set<std::pair<JointId, JointId>> excluded;
    const auto body = [&](const std::string& name) -> Result<JointId> {
        if (const auto it = body_of.find(name); it != body_of.end()) return it->second;
        if (const auto frame = model->frame_id(name))
            return static_cast<JointId>(pin.frames[*frame].parentJoint);
        return make_error(ErrorCode::not_found, "unknown frame or geometry '" + name + "'");
    };
    for (const auto& [a, b] : options.exclude_pairs) {
        const auto ja = body(a), jb = body(b);
        if (!ja || !jb)
            return make_error(ErrorCode::not_found, "exclude_pairs: " + (!ja ? ja : jb).error().message);
        excluded.emplace(std::min(*ja, *jb), std::max(*ja, *jb));
    }
    // pinocchio leaves the universe's subtree as {0}, so it is expanded to every joint here.
    std::vector<JointId> all_joints(static_cast<std::size_t>(pin.njoints));
    for (std::size_t j = 0; j < all_joints.size(); ++j) all_joints[j] = static_cast<JointId>(j);
    const auto subtree = [&](JointId j) {
        return j == 0 ? all_joints : std::vector<JointId>(pin.subtrees[j].begin(), pin.subtrees[j].end());
    };
    for (const auto& [a, b] : options.exclude_subtrees) {
        const auto ja = body(a), jb = body(b);
        if (!ja || !jb)
            return make_error(ErrorCode::not_found, "exclude_subtrees: " + (!ja ? ja : jb).error().message);
        for (const auto sa : subtree(*ja))
            for (const auto sb : subtree(*jb))
                excluded.emplace(std::min<JointId>(sa, sb), std::max<JointId>(sa, sb));
    }
    for (std::size_t g = 0; g < options.groups.size(); ++g) {
        const bool used = std::any_of(pin.names.begin(), pin.names.end(), [&](const std::string& joint) {
            return group_of(joint, options.groups) == static_cast<int>(g);
        });
        if (!used)
            return make_error(ErrorCode::invalid_argument,
                              "groups: '" + options.groups[g].name + "' matches no joint");
    }
    std::vector<int> groups;
    for (const JointId j : out->joints_) groups.push_back(group_of(pin.names[j], options.groups));

    const auto& gs = out->geometries_;
    const std::size_t n = gs.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t k = i + 1; k < n; ++k) {
            const JointId a = out->joints_[i], b = out->joints_[k];
            if (a == b || (a != 0 && pin.parents[a] == b) || (b != 0 && pin.parents[b] == a)) continue;
            if (groups[i] >= 0 && groups[i] == groups[k]) continue;
            if (disabled.count(ordered(gs[i].frame, gs[k].frame))) continue;
            if (excluded.count({std::min(a, b), std::max(a, b)})) continue;
            out->pairs_.emplace_back(static_cast<Index>(i), static_cast<Index>(k));
        }
    }
    if (options.exclude_initial_collisions)
        if (Status s = drop_initial_collisions(pin, out->geometries_, out->joints_, *out->impl_,
                                               options.initial, context, out->pairs_);
            !s)
            return std::move(s).error();
    if (out->pairs_.empty())
        return make_error(ErrorCode::invalid_argument, "self collision: no pairs left after filtering");
    return CollisionModelPtr(std::move(out));
}

Result<CollisionModelPtr> CollisionModel::from_urdf(ModelPtr model, const ModelSource& urdf,
                                                    const CollisionModelOptions& options,
                                                    const Context& context) {
    if (!model) return make_error(ErrorCode::invalid_argument, "collision model: null model");
    auto source = read_source(urdf, context);
    if (!source) return std::move(source).error();
    ::urdf::ModelInterfaceSharedPtr tree;
    try {
        tree = ::urdf::parseURDF(source->text);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::parse_error, std::string("invalid URDF: ") + e.what());
    }
    if (!tree) return make_error(ErrorCode::parse_error, "invalid URDF document");

    const pinocchio::Model& pin = pinocchio_model(*model);
    // Collision names that are unique across the document keep their name; others become <link>_<i>.
    std::map<std::string, int> name_count;
    for (const auto& [link_name, link] : tree->links_)
        if (link)
            for (const auto& c : link->collision_array)
                if (c && !c->name.empty()) ++name_count[c->name];

    std::vector<CollisionGeometry> geometries;
    // Pinocchio body frames are in kinematic tree order: a deterministic, document-like order.
    for (const pinocchio::Frame& frame : pin.frames) {
        if (frame.type != pinocchio::BODY) continue;
        const ::urdf::LinkConstSharedPtr link = tree->getLink(frame.name);
        if (!link) continue;
        for (std::size_t i = 0; i < link->collision_array.size(); ++i) {
            const auto& c = link->collision_array[i];
            if (!c || !c->geometry) continue;
            auto shape = urdf_shape(*c->geometry, source->dir, context);
            if (!shape)
                return make_error(shape.error().code, "link '" + frame.name + "' collision " +
                                                          std::to_string(i) + ": " + shape.error().message);
            CollisionGeometry g;
            g.name =
                !c->name.empty() && name_count[c->name] == 1 ? c->name : frame.name + "_" + std::to_string(i);
            g.frame = frame.name;
            g.placement = to_isometry(c->origin);
            g.shape = std::move(*shape);
            geometries.push_back(std::move(g));
        }
    }
    if (geometries.empty()) return make_error(ErrorCode::not_found, "URDF has no collision geometries");

    CollisionModelOptions resolved = options;
    // A relative SRDF file is looked up next to the URDF first.
    if (resolved.srdf && resolved.srdf->kind == ModelSource::Kind::file && resolved.srdf->base_dir.empty())
        resolved.srdf->base_dir = source->dir;
    return create(std::move(model), std::move(geometries), resolved, context);
}

}  // namespace kstone
