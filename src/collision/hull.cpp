#include "hull.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace kstone::collision_detail {
namespace {

struct Face {
    std::array<int, 3> v{};
    Eigen::Vector3d n{Eigen::Vector3d::Zero()};
    double d{0.0};
    std::vector<int> outside;
    bool alive{true};
    unsigned visit{0};
};

class Builder {
  public:
    explicit Builder(const Eigen::Matrix3Xd& p) : p_(p), n_(static_cast<std::int64_t>(p.cols())) {}

    Result<Hull> run() {
        if (p_.cols() < 4) return degenerate();
        if (!p_.allFinite()) return make_error(ErrorCode::invalid_argument, "convex hull: non-finite vertex");
        const Eigen::Vector3d lo = p_.rowwise().minCoeff(), hi = p_.rowwise().maxCoeff();
        tol_ =
            1e-9 * std::max((hi - lo).norm(), std::max(lo.cwiseAbs().maxCoeff(), hi.cwiseAbs().maxCoeff()));
        if (!(tol_ > 0.0)) return degenerate();
        std::array<int, 4> s{};
        if (!simplex(s)) return degenerate();

        const Eigen::Vector3d centre = (pt(s[0]) + pt(s[1]) + pt(s[2]) + pt(s[3])) / 4.0;
        const std::size_t combos[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
        for (const auto& c : combos) {
            int a = s[c[0]], b = s[c[1]], e = s[c[2]];
            if (plane(a, b, e).first.dot(centre - pt(a)) > 0.0) std::swap(b, e);
            if (!add_face(a, b, e)) return broken();
        }
        std::vector<int> rest;
        for (int i = 0; i < static_cast<int>(n_); ++i)
            if (std::find(s.begin(), s.end(), i) == s.end()) rest.push_back(i);
        assign(rest, 0);

        for (std::size_t fi = 0; fi < faces_.size(); ++fi)
            if (faces_[fi].alive && !faces_[fi].outside.empty() && !expand(static_cast<int>(fi)))
                return broken();
        return output();
    }

  private:
    Eigen::Vector3d pt(int i) const { return p_.col(i); }

    std::pair<Eigen::Vector3d, double> plane(int a, int b, int c) const {
        Eigen::Vector3d n = (pt(b) - pt(a)).cross(pt(c) - pt(a));
        const double len = n.norm();
        if (len <= 0.0) return {Eigen::Vector3d::Zero(), 0.0};
        n /= len;
        return {n, n.dot(pt(a))};
    }

    double dist(const Face& f, int i) const { return f.n.dot(pt(i)) - f.d; }

    std::int64_t key(int a, int b) const { return static_cast<std::int64_t>(a) * n_ + b; }

    static Error degenerate() { return make_error(ErrorCode::invalid_argument, "convex hull is degenerate"); }
    static Error broken() { return make_error(ErrorCode::internal, "convex hull: inconsistent topology"); }

    bool simplex(std::array<int, 4>& s) const {
        std::array<int, 6> ext{};
        for (Eigen::Index k = 0; k < 3; ++k) {
            p_.row(k).minCoeff(&ext[static_cast<std::size_t>(2 * k)]);
            p_.row(k).maxCoeff(&ext[static_cast<std::size_t>(2 * k + 1)]);
        }
        double best = -1.0;
        for (int i : ext)
            for (int j : ext)
                if (double d = (pt(i) - pt(j)).squaredNorm(); d > best) best = d, s[0] = i, s[1] = j;
        if (std::sqrt(best) <= tol_) return false;

        const Eigen::Vector3d axis = (pt(s[1]) - pt(s[0])).normalized();
        best = -1.0;
        for (int i = 0; i < static_cast<int>(n_); ++i) {
            const Eigen::Vector3d r = pt(i) - pt(s[0]);
            if (double d = (r - axis * axis.dot(r)).norm(); d > best) best = d, s[2] = i;
        }
        if (best <= tol_) return false;

        const auto [n, d0] = plane(s[0], s[1], s[2]);
        best = -1.0;
        for (int i = 0; i < static_cast<int>(n_); ++i)
            if (double d = std::abs(n.dot(pt(i)) - d0); d > best) best = d, s[3] = i;
        return best > tol_;
    }

    bool add_face(int a, int b, int c) {
        const std::size_t id = faces_.size();
        Face f;
        f.v = {a, b, c};
        std::tie(f.n, f.d) = plane(a, b, c);
        faces_.push_back(std::move(f));
        for (std::size_t k = 0; k < 3; ++k)
            if (!edges_.emplace(key(faces_[id].v[k], faces_[id].v[(k + 1) % 3]), static_cast<int>(id)).second)
                return false;
        return true;
    }

    void assign(const std::vector<int>& points, std::size_t first) {
        for (int i : points)
            for (std::size_t f = first; f < faces_.size(); ++f)
                if (dist(faces_[f], i) > tol_) {
                    faces_[f].outside.push_back(i);
                    break;
                }
    }

    bool expand(int start) {
        const Face& sf = faces_[static_cast<std::size_t>(start)];
        const int eye = *std::max_element(sf.outside.begin(), sf.outside.end(),
                                          [&](int x, int y) { return dist(sf, x) < dist(sf, y); });
        ++stamp_;
        std::vector<int> visible{start}, stack{start};
        std::vector<std::pair<int, int>> horizon;
        faces_[static_cast<std::size_t>(start)].visit = stamp_;
        while (!stack.empty()) {
            const int f = stack.back();
            stack.pop_back();
            for (std::size_t k = 0; k < 3; ++k) {
                const int a = faces_[static_cast<std::size_t>(f)].v[k];
                const int b = faces_[static_cast<std::size_t>(f)].v[(k + 1) % 3];
                const auto twin = edges_.find(key(b, a));
                if (twin == edges_.end()) return false;
                Face& t = faces_[static_cast<std::size_t>(twin->second)];
                if (t.visit == stamp_) continue;
                if (dist(t, eye) > tol_) {
                    t.visit = stamp_;
                    visible.push_back(twin->second);
                    stack.push_back(twin->second);
                } else {
                    horizon.emplace_back(a, b);
                }
            }
        }

        std::vector<int> orphans;
        for (int f : visible) {
            Face& face = faces_[static_cast<std::size_t>(f)];
            for (int i : face.outside)
                if (i != eye) orphans.push_back(i);
            face.outside.clear();
            face.alive = false;
            for (std::size_t k = 0; k < 3; ++k) edges_.erase(key(face.v[k], face.v[(k + 1) % 3]));
        }
        const std::size_t first = faces_.size();
        for (const auto& [a, b] : horizon)
            if (!add_face(a, b, eye)) return false;
        assign(orphans, first);
        return true;
    }

    Result<Hull> output() const {
        std::vector<int> remap(static_cast<std::size_t>(n_), -1);
        Hull hull;
        std::vector<int> order;
        for (const Face& f : faces_) {
            if (!f.alive) continue;
            std::array<int, 3> t{};
            for (std::size_t k = 0; k < 3; ++k) {
                int& r = remap[static_cast<std::size_t>(f.v[k])];
                if (r < 0) {
                    r = static_cast<int>(order.size());
                    order.push_back(f.v[k]);
                }
                t[k] = r;
            }
            hull.triangles.push_back(t);
        }
        for (const auto& entry : edges_) {
            const std::int64_t e = entry.first;
            const auto a = static_cast<int>(e / n_), b = static_cast<int>(e % n_);
            if (!edges_.count(key(b, a))) return broken();
        }
        hull.vertices.resize(3, static_cast<Eigen::Index>(order.size()));
        for (std::size_t i = 0; i < order.size(); ++i)
            hull.vertices.col(static_cast<Eigen::Index>(i)) = pt(order[i]);
        return hull;
    }

    const Eigen::Matrix3Xd& p_;
    std::int64_t n_;
    double tol_{0.0};
    unsigned stamp_{0};
    std::vector<Face> faces_;
    std::unordered_map<std::int64_t, int> edges_;
};

}  // namespace

Result<Hull> convex_hull(const Eigen::Matrix3Xd& points) { return Builder(points).run(); }

}  // namespace kstone::collision_detail
