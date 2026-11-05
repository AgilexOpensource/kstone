#pragma once

#include "kstone/qp/backend.hpp"

#include <memory>

namespace kstone::qp {

struct DaqpSettings {
    double primal_tol = 1e-6;
    int iteration_limit = 1000;
    // Workspaces kept for distinct (n, m) layouts, least recently used evicted (>= 1).
    // Staged solves alternate between a few layouts; each keeps its own warm start.
    int cached_layouts = 8;
};

// DAQP (dual active set) backend for plain QPs. Rebuilds only for an (n, m)
// layout not in the cache; otherwise updates that workspace in place.
class KSTONE_EXPORT DaqpBackend final : public Backend {
  public:
    explicit DaqpBackend(DaqpSettings settings = {});
    ~DaqpBackend() override;
    DaqpBackend(DaqpBackend&&) noexcept;
    DaqpBackend& operator=(DaqpBackend&&) noexcept;
    DaqpBackend(const DaqpBackend&) = delete;
    DaqpBackend& operator=(const DaqpBackend&) = delete;

    const char* name() const noexcept override { return "daqp"; }
    Status solve(const Problem& problem, Solution& out) override;
    void reset() noexcept override;

    const DaqpSettings& settings() const noexcept;
    // Number of full workspace rebuilds so far (for diagnostics/tests).
    long rebuilds() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone::qp
