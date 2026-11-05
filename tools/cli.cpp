#include "cli.hpp"

#include "kstone/core/version.hpp"
#include "kstone/module/registry.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/runtime/config.hpp"
#include "kstone/runtime/debug_server.hpp"
#include "kstone/runtime/doctor.hpp"

#include <atomic>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

namespace {

std::atomic<bool> g_stop{false};
static_assert(std::atomic<bool>::is_always_lock_free, "the signal handler needs a lock-free flag");

extern "C" void on_signal(int) { g_stop.store(true); }

int usage(std::FILE* out) {
    std::fprintf(
        out,
        "usage: kstone run <config.yaml> [options]   run until the duration elapses or "
        "Ctrl-C/SIGTERM/SIGHUP\n"
        "         --duration <s>                     stop after s seconds of engine time (0: no limit)\n"
        "         --realtime                         pace cycles with the clock (overrides the config)\n"
        "         --serve [host:port|port]           debug server (127.0.0.1:7311); non-loopback needs a "
        "token\n"
        "         --token-file <path>                server token (16+ bytes); else $KSTONE_TOKEN if set\n"
        "       kstone check <config.yaml> [options] build the engine without running it\n"
        "       common options:\n"
        "         --set <name>=<value>               define ${name} for the config (repeatable;\n"
        "                                            wins over the environment and ${name:-default};\n"
        "                                            an unreferenced name is an error)\n"
        "       kstone doctor [--json]                check the build, loaded libraries and a smoke solve\n"
        "       kstone schema [module_type]           list built-in module types, or print one's JSON "
        "Schema\n"
        "       kstone --version\n");
    return out == stdout ? 0 : 2;
}

int fail(const kstone::Error& e) {
    std::fprintf(stderr, "kstone: %s\n", e.message.c_str());
    return 1;
}

bool parse_seconds(const char* text, double& out) {
    char* end = nullptr;
    out = std::strtod(text, &end);
    return end != text && *end == '\0' && out >= 0.0;
}

// Whole file minus trailing whitespace.
bool read_token(const char* path, std::string& out) {
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return false;
    char buffer[1025];
    const std::size_t n = std::fread(buffer, 1, sizeof(buffer), f);
    const bool ok = std::ferror(f) == 0;
    std::fclose(f);
    out.assign(buffer, n);
    while (!out.empty() && std::isspace(static_cast<unsigned char>(out.back())) != 0) out.pop_back();
    return ok;
}

// "host:port", "host", ":port" or "port".
bool parse_endpoint(const std::string& text, kstone::DebugServerOptions& out) {
    if (!text.empty() && text.find_first_not_of("0123456789") == std::string::npos)
        return parse_endpoint(":" + text, out);
    const std::size_t colon = text.rfind(':');
    if (colon == std::string::npos) {
        out.host = text;
        return !text.empty();
    }
    if (colon > 0) out.host = text.substr(0, colon);
    const std::string port = text.substr(colon + 1);
    char* end = nullptr;
    const unsigned long value = std::strtoul(port.c_str(), &end, 10);
    if (port.empty() || *end != '\0' || value > 65535) return false;
    out.port = static_cast<std::uint16_t>(value);
    return true;
}

}  // namespace

int kstone_cli_main(int argc, char** argv) {
    if (argc < 2) return usage(stderr);
    const std::string command = argv[1];
    if (command == "-h" || command == "--help") return usage(stdout);
    if (command == "--version") {
        std::printf("kstone %s\n", kstone::library_version_string());
        return 0;
    }
    if (command == "doctor") {
        const bool json = argc == 3 && std::string(argv[2]) == "--json";
        if (argc > 3 || (argc == 3 && !json)) {
            std::fprintf(stderr, "kstone: unexpected argument '%s'\n", argv[argc - 1]);
            return usage(stderr);
        }
        const kstone::DoctorReport report = kstone::doctor();
        if (json)
            std::printf("%s\n", kstone::to_json(kstone::to_config_node(report)).c_str());
        else
            std::fputs(kstone::to_text(report).c_str(), stdout);
        return report.ok() ? 0 : 1;
    }
    if (command == "schema") {
        if (argc > 3) {
            std::fprintf(stderr, "kstone: unexpected argument '%s'\n", argv[3]);
            return usage(stderr);
        }
        const kstone::ModuleRegistry registry = kstone::ModuleRegistry::with_builtins();
        if (argc == 2) {
            for (const std::string& type : registry.types()) std::printf("%s\n", type.c_str());
            return 0;
        }
        auto schema = registry.schema(argv[2]);
        if (!schema) return fail(schema.error());
        std::printf("%s\n", kstone::to_json(*schema).c_str());
        return 0;
    }
    if (command != "run" && command != "check") {
        std::fprintf(stderr, "kstone: unknown command '%s'\n", command.c_str());
        return usage(stderr);
    }
    if (argc < 3) {
        std::fprintf(stderr, "kstone: %s needs a config file\n", command.c_str());
        return usage(stderr);
    }

    double duration = 0.0;
    bool realtime = false;
    bool serve = false;
    const char* token_file = nullptr;
    kstone::DebugServerOptions serve_options;
    kstone::YamlOptions yaml = kstone::YamlOptions::files();
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool needs_value =
            arg == "--set" || (command == "run" && (arg == "--duration" || arg == "--token-file"));
        if (needs_value && i + 1 >= argc) {
            std::fprintf(stderr, "kstone: %s needs a value\n", arg.c_str());
            return usage(stderr);
        }
        if (arg == "--set") {
            const std::string def = argv[++i];
            const std::size_t eq = def.find('=');
            if (eq == std::string::npos || eq == 0) {
                std::fprintf(stderr, "kstone: bad --set '%s', expected name=value\n", def.c_str());
                return usage(stderr);
            }
            yaml.vars[def.substr(0, eq)] = def.substr(eq + 1);
            continue;
        }
        if (command == "run" && arg == "--duration") {
            if (!parse_seconds(argv[++i], duration)) {
                std::fprintf(stderr, "kstone: bad --duration '%s', expected seconds >= 0\n", argv[i]);
                return usage(stderr);
            }
            continue;
        }
        if (command == "run" && arg == "--realtime") {
            realtime = true;
            continue;
        }
        if (command == "run" && arg == "--serve") {
            serve = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                if (!parse_endpoint(argv[++i], serve_options)) {
                    std::fprintf(stderr, "kstone: bad --serve address '%s'\n", argv[i]);
                    return usage(stderr);
                }
            }
            continue;
        }
        if (command == "run" && arg == "--token-file") {
            token_file = argv[++i];
            continue;
        }
        std::fprintf(stderr, "kstone: unexpected argument '%s'\n", arg.c_str());
        return usage(stderr);
    }

    if (token_file != nullptr && !serve) {
        std::fprintf(stderr, "kstone: --token-file needs --serve\n");
        return usage(stderr);
    }

    const kstone::Context context = kstone::Context::defaults();
    const std::filesystem::path path = argv[2];
    yaml.resources = &context.resources;
    auto config = kstone::load_yaml(path, yaml);
    if (!config) return fail(config.error());
    if (realtime) config->set("realtime", true);
    auto engine = kstone::build_engine(*config, path.parent_path(), {}, context);
    if (!engine) return fail(engine.error());
    const auto& model = *engine->modules().model();
    std::printf("loaded %s: nq=%ld nv=%ld, %zu modules, %.1f Hz%s\n", argv[2], static_cast<long>(model.nq()),
                static_cast<long>(model.nv()), engine->modules().modules().size(), engine->options().rate_hz,
                engine->options().realtime ? "" : " (not realtime)");
    std::fflush(stdout);
    if (command == "check") return 0;

    std::shared_ptr<kstone::DebugServer> server;
    if (serve) {
        if (token_file != nullptr) {
            if (!read_token(token_file, serve_options.token)) {
                std::fprintf(stderr, "kstone: cannot read --token-file '%s'\n", token_file);
                return 1;
            }
        } else if (const char* env = std::getenv("KSTONE_TOKEN"); env != nullptr) {
            serve_options.token = env;
        }
        auto started = kstone::DebugServer::start(*engine, serve_options);
        if (!started) return fail(started.error());
        server = std::move(*started);
        std::printf("serving on %s:%u (%s)\n", serve_options.host.c_str(),
                    static_cast<unsigned>(server->port()),
                    serve_options.token.empty() ? "no authentication" : "token required");
        std::fflush(stdout);
    }

    g_stop.store(false);
    const auto old_int = std::signal(SIGINT, on_signal);
    const auto old_term = std::signal(SIGTERM, on_signal);
    const auto old_hup = std::signal(
        SIGHUP, on_signal);  // A closed terminal or dropped SSH session still stops the robot cleanly.
    const kstone::Status s = engine->run(duration, &g_stop);
    // Restored for an embedding host such as the Python entry point.
    std::signal(SIGINT, old_int);
    std::signal(SIGTERM, old_term);
    std::signal(SIGHUP, old_hup);
    if (server) server->stop();
    const kstone::EngineStats& st = engine->stats();
    std::printf("cycles=%llu time=%.3fs overruns=%llu cycle p50/p99/p999/max=%.3f/%.3f/%.3f/%.3fms\n",
                static_cast<unsigned long long>(st.cycles), st.time,
                static_cast<unsigned long long>(st.overruns), st.cycle_hist.quantile(0.5) * 1e3,
                st.cycle_hist.quantile(0.99) * 1e3, st.cycle_hist.quantile(0.999) * 1e3,
                st.max_cycle_s * 1e3);
    if (st.failed_cycles + st.stale_cycles + st.guard_rejects + st.guard_clamps > 0)
        std::printf("failed=%llu stale=%llu guard rejects/clamps=%llu/%llu\n",
                    static_cast<unsigned long long>(st.failed_cycles),
                    static_cast<unsigned long long>(st.stale_cycles),
                    static_cast<unsigned long long>(st.guard_rejects),
                    static_cast<unsigned long long>(st.guard_clamps));
    std::fflush(stdout);
    if (!s) return fail(s.error());
    return 0;
}
