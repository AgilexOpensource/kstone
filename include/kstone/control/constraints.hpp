#pragma once

#include "kstone/control/constraint.hpp"
#include "kstone/core/mask.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace kstone {

/// |v_i| <= limit_i for the selected scalar joints (box only, no rows). Infinite limits are free.
class KSTONE_EXPORT JointVelocityConstraint final : public Constraint {
  public:
    /// limits: one value for all joints or one per joint, >= 0.
    JointVelocityConstraint(std::vector<std::string> joints, Eigen::VectorXd limits);

    /// Same shape rules as the constructor; +inf = unlimited. After bind, a single value applies to all joints.
    Status set_limits(Eigen::VectorXd limits);

    void tighten_bounds(const Tick& tick, Eigen::Ref<Eigen::VectorXd> lb,
                        Eigen::Ref<Eigen::VectorXd> ub) const override;
    bool shares_step_budget() const noexcept override { return true; }

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                   Eigen::Ref<Eigen::VectorXd> hi) override;

    std::vector<std::string> names_;
    std::vector<Index> indices_;
    Eigen::VectorXd limits_;
};

/// Point mass carried by a frame (offset in that frame).
struct Payload {
    std::string frame;
    double mass{0.0};
    Eigen::Vector3d offset{Eigen::Vector3d::Zero()};
};

/// Convex support polygon in the xy plane of a frame, and the projected centre of mass of the
/// whole model or of a joint subtree (plus payloads attached to frames in that subtree).
class KSTONE_EXPORT SupportPolygon {
  public:
    /// Vertices in either winding; at least 3, strictly convex. Empty subtree means the whole model.
    static Result<SupportPolygon> create(std::string frame, std::vector<Eigen::Vector2d> vertices,
                                         std::string subtree = {});

    struct Evaluation {
        Eigen::Vector3d world_com;
        Eigen::Vector2d com;                                           ///< In the polygon frame.
        Eigen::MatrixXd jacobian;                                      ///< 2 x nv, d com / dq.
        Eigen::VectorXd slack;                                         ///< Per edge, >= 0 inside.
        Eigen::Isometry3d frame_pose{Eigen::Isometry3d::Identity()};   ///< Polygon frame in the world.
        Eigen::MatrixXd com_jacobian, frame_jacobian, weighted, full;  ///< Scratch reused between calls.
    };

    /// Resolves the frame, subtree and payload frames once; repeat after changing payloads or the model.
    Status bind(const Model& model, const std::vector<Payload>& payloads);
    bool bound() const noexcept { return model_ != nullptr; }
    /// Precondition: bind() with the workspace's model and these payloads (same frames, in order).
    Status evaluate(Workspace& workspace, const std::vector<Payload>& payloads, Evaluation& out) const;

    const std::string& frame() const noexcept { return frame_; }
    const std::string& subtree() const noexcept { return subtree_; }
    Index edges() const noexcept { return normals_.rows(); }
    const Eigen::MatrixXd& normals() const noexcept { return normals_; }  ///< edges x 2, outward.
    /// Vertices of the polygon moved `inset` inward (negative = outward), counter-clockwise, 2 x edges.
    /// Precondition: inset <= inset_limit(); out must be 2 x edges().
    void inset_vertices(double inset, Eigen::Ref<Eigen::Matrix2Xd> out) const noexcept;
    /// Radius of the largest inscribed circle: the largest admissible inward margin.
    double inset_limit() const noexcept { return inset_limit_; }

  private:
    SupportPolygon() = default;

    std::string frame_, subtree_;
    const Model* model_{nullptr};
    FrameId frame_id_{0};
    JointId root_{0};
    std::vector<FrameId> payload_frames_;
    std::vector<char> payload_used_;  ///< Payload frame lies in the subtree.
    Eigen::MatrixXd normals_;
    Eigen::VectorXd offsets_;
    double inset_limit_{0.0};
};

/// Keep the projected centre of mass at least `margin` inside the support polygon
/// (negative margin allows that much outside). Outside the margin the constraint only asks
/// for a recovery speed instead of an instant return.
class KSTONE_EXPORT CenterOfMassConstraint final : public Constraint {
  public:
    explicit CenterOfMassConstraint(SupportPolygon polygon);

    /// |margin| <= polygon.inset_limit() for positive margins.
    Status set_margin(double margin);
    /// Fraction of the slack usable per step, in (0, 1].
    Status set_gain(double gain);
    /// Speed (m/s) toward the inside allowed when outside the margin, > 0.
    Status set_recovery(double speed);
    /// Masses must be finite and >= 0.
    Status set_payloads(std::vector<Payload> payloads);
    /// Recovery columns; empty = all joints. Only these pull the CoM back. The other joints may use the slack plus
    /// what the recovery joints can still pull back in one step (velocity and position limits, at most the recovery
    /// speed), and never push out once outside the margin: they stop when the recovery joints saturate.
    Status set_columns(Mask columns);
    const SupportPolygon& polygon() const noexcept { return polygon_; }
    const std::vector<Payload>& payloads() const noexcept { return payloads_; }
    double margin() const noexcept { return margin_; }

    bool defers_to_later_stages() const noexcept override { return true; }

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                   Eigen::Ref<Eigen::VectorXd> hi) override;

    SupportPolygon polygon_;
    std::vector<Payload> payloads_;
    Mask columns_;
    double margin_{0.0}, gain_{0.5}, recovery_{0.1};
    SupportPolygon::Evaluation eval_;
    Eigen::VectorXd capacity_;
};

/// Rows supplied by the caller each step: lo <= A v <= hi.
class KSTONE_EXPORT ExternalConstraint final : public Constraint {
  public:
    ExternalConstraint();

    Status set(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::VectorXd>& lo,
               const Eigen::Ref<const Eigen::VectorXd>& hi);
    void clear();

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> lo,
                   Eigen::Ref<Eigen::VectorXd> hi) override;

    Eigen::MatrixXd A_;
    Eigen::VectorXd lo_, hi_;
};

}  // namespace kstone
