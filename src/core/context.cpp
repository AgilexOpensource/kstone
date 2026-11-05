#include "kstone/core/context.hpp"

namespace kstone {

Context Context::defaults() {
    Context context;
    context.clock = make_steady_clock();
    context.log = Logger(make_stderr_sink(), LogLevel::warn, context.clock);
    context.loaders = ModelLoaderRegistry::builtin();
    return context;
}

}  // namespace kstone
