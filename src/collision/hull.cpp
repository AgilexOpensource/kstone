#include "hull.hpp"

extern "C" {
#include <libqhull_r/qhull_ra.h>
}

#include <Eigen/Geometry>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace kstone::collision_detail {
namespace {

Error degenerate() { return make_error(ErrorCode::invalid_argument, "convex hull is degenerate"); }

struct FileCloser {
    void operator()(std::FILE* f) const { std::fclose(f); }
};

std::string first_line(std::FILE* f) {
    if (!f) return {};
    std::rewind(f);
    char buf[256] = {};
    if (!std::fgets(buf, sizeof buf, f)) return {};
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    return s;
}

class Qhull {
  public:
    explicit Qhull(std::FILE* err) { qh_zero(&qh_, err); }
    Qhull(const Qhull&) = delete;
    Qhull& operator=(const Qhull&) = delete;
    ~Qhull() {
        qh_freeqhull(&qh_, !qh_ALL);
        int curlong = 0, totlong = 0;
        qh_memfreeshort(&qh_, &curlong, &totlong);
    }
    qhT* get() { return &qh_; }

  private:
    qhT qh_{};
};

}  // namespace

Result<Hull> convex_hull(const Eigen::Matrix3Xd& points) {
    if (points.cols() < 4) return degenerate();
    if (!points.allFinite()) return make_error(ErrorCode::invalid_argument, "convex hull: non-finite vertex");

    // qhull may rescale its input in place, so hand it a copy.
    std::vector<coordT> coords(points.data(), points.data() + points.size());
    // Messages go to a scratch file so they never reach stderr; the first line becomes the error text.
    std::unique_ptr<std::FILE, FileCloser> err(std::tmpfile());
    // qhull must never get a null stream; without a scratch file the error text is lost, not the hull.
    if (!err) err.reset(std::fopen("/dev/null", "w"));
    if (!err) return make_error(ErrorCode::io_error, "convex hull: cannot open a stream for qhull messages");
    Qhull q(err.get());
    qhT* qh = q.get();
    // Qt triangulates the merged facets; qhull's default merging handles coplanar and near-coplanar input.
    char options[] = "qhull Qt";
    const int code = qh_new_qhull(qh, 3, static_cast<int>(points.cols()), coords.data(), False, options,
                                  nullptr, err.get());
    if (code == qh_ERRsingular) return degenerate();
    if (code != qh_ERRnone)
        return make_error(ErrorCode::internal,
                          "convex hull: qhull error " + std::to_string(code) + ": " + first_line(err.get()));

    Hull hull;
    std::vector<int> remap(static_cast<std::size_t>(points.cols()), -1);
    std::vector<int> order;
    for (facetT* f = qh->facet_list; f && f->next; f = f->next) {
        if (f->upperdelaunay) continue;
        if (!f->simplicial || qh_setsize(qh, f->vertices) != 3)
            return make_error(ErrorCode::internal, "convex hull: qhull returned a non-triangular facet");
        std::array<int, 3> id{};
        for (std::size_t k = 0; k < 3; ++k)
            id[k] = qh_pointid(qh, static_cast<vertexT*>(f->vertices->e[k].p)->point);
        // Topological orientation, not the facet normal: Qt slivers can have a near-zero cross product.
        if (f->toporient ^ qh_ORIENTclock) std::swap(id[1], id[2]);
        std::array<int, 3> t{};
        for (std::size_t k = 0; k < 3; ++k) {
            int& r = remap[static_cast<std::size_t>(id[k])];
            if (r < 0) {
                r = static_cast<int>(order.size());
                order.push_back(id[k]);
            }
            t[k] = r;
        }
        hull.triangles.push_back(t);
    }
    if (order.size() < 4) return degenerate();
    hull.vertices.resize(3, static_cast<Eigen::Index>(order.size()));
    for (std::size_t i = 0; i < order.size(); ++i)
        hull.vertices.col(static_cast<Eigen::Index>(i)) = points.col(order[i]);
    return hull;
}

}  // namespace kstone::collision_detail
