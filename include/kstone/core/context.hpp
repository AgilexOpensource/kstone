#pragma once

#include "kstone/core/clock.hpp"
#include "kstone/core/export.hpp"
#include "kstone/core/log.hpp"
#include "kstone/core/resources.hpp"
#include "kstone/model/loader.hpp"

#include <memory>

namespace kstone {

/// Explicit environment passed to every operation that needs I/O, logging or time.
/// There is no global state: each embedding (CLI, server, bindings, tests) builds its own.
/// Treat as immutable once shared across threads.
struct KSTONE_EXPORT Context {
    Logger log;
    std::shared_ptr<const Clock> clock;
    ResourceResolver resources;
    ModelLoaderRegistry loaders;

    /// stderr sink at warn, steady clock, default resolver, builtin loaders.
    static Context defaults();
};

}  // namespace kstone
