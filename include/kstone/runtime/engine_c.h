/* Engine C ABI. Pure C so hosts in any language (C, Rust, Go, C#, ...) can drive an engine through FFI.
 *
 *     kstone_engine* e = NULL;
 *     char err[256];
 *     if (kstone_engine_load("robot.yaml", NULL, &e, err, sizeof err) != KSTONE_ENGINE_OK) { ... }
 *     double target[7] = {0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0};
 *     kstone_engine_set_port(e, "arm", "target", target, 7, err, sizeof err);
 *     for (int k = 0; k < 100; ++k) kstone_engine_step(e, k * kstone_engine_dt(e), err, sizeof err);
 *     kstone_engine_destroy(e);
 *
 * Threading: call everything on one thread, except kstone_engine_request_stop and kstone_engine_set_paused,
 * which may be called from any thread, including while kstone_engine_run is cycling. Hosts that need to write
 * ports during a run should drive the loop themselves with kstone_engine_step.
 * error/error_size: on failure a NUL-terminated message is written (truncated to error_size); may be NULL / 0.
 * Port values are flat doubles: number and flag 1 (flag: non-zero = true), vector its size,
 * pose 7 = translation xyz then rotation quaternion wxyz (normalized on write). */
#ifndef KSTONE_RUNTIME_ENGINE_C_H
#define KSTONE_RUNTIME_ENGINE_C_H

#include "kstone/core/export.hpp"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bumped on incompatible changes; new trailing fields are signalled by struct_size instead. */
#define KSTONE_ENGINE_ABI_VERSION 1u

/* Return codes; the non-zero values match kstone::ErrorCode. */
enum {
    KSTONE_ENGINE_OK = 0,
    KSTONE_ENGINE_INVALID_ARGUMENT = 1,
    KSTONE_ENGINE_OUT_OF_RANGE = 2,
    KSTONE_ENGINE_NOT_FOUND = 3,
    KSTONE_ENGINE_PARSE_ERROR = 4,
    KSTONE_ENGINE_IO_ERROR = 5,
    KSTONE_ENGINE_UNSUPPORTED = 6,
    KSTONE_ENGINE_INTERNAL = 7,
    KSTONE_ENGINE_INFEASIBLE = 8
};

typedef struct kstone_engine kstone_engine;

/* Reserved for future options; pass NULL for the defaults. */
typedef struct kstone_engine_options {
    uint32_t struct_size; /* sizeof(kstone_engine_options) */
} kstone_engine_options;

/* Last cycle. Set struct_size and point the arrays at caller buffers (nq or nv doubles) or NULL to skip them;
 * kstone_engine_snapshot fills the rest and never allocates. */
typedef struct kstone_engine_status {
    uint32_t struct_size; /* sizeof(kstone_engine_status) */
    uint64_t cycles;
    uint64_t overruns;
    uint64_t repeated_reads;
    uint64_t guard_rejects;
    uint64_t guard_clamps;
    uint64_t held_cycles;
    uint64_t stale_cycles;
    uint64_t resyncs;
    uint64_t failed_cycles;
    double time;         /* now of the last cycle (s) */
    double last_cycle_s; /* wall time of the last read-solve-write */
    double max_cycle_s;
    double max_lateness_s;
    double cycle_p50_s;
    double cycle_p99_s;
    double cycle_p999_s;
    int paused;
    int fresh;    /* last read was fresh */
    int written;  /* last successful cycle wrote a command */
    int degraded; /* last solve was degraded: a stage held or inexact, or the step clipped */
    int held;
    double step_scale;
    int64_t state_stamp_ns; /* -1 = unknown */
    int command_fresh;
    int has_measured;
    double* state_q;   /* nq: last read */
    double* state_v;   /* nv */
    double* command_q; /* nq: last guarded command */
    double* command_v; /* nv */
    double* ideal;     /* nq: commanded state the next solve starts from */
    double* measured;  /* nq: measured estimate; meaningful only when has_measured */
} kstone_engine_status;

KSTONE_EXPORT uint32_t kstone_engine_abi_version(void);

/* Loads a YAML configuration file, like kstone::load_engine. options may be NULL. */
KSTONE_EXPORT int kstone_engine_load(const char* path, const kstone_engine_options* options,
                                     kstone_engine** out, char* error, size_t error_size);
/* Same from YAML text; includes and relative paths resolve against base_dir (NULL = current directory). */
KSTONE_EXPORT int kstone_engine_create(const char* config_yaml, const char* base_dir,
                                       const kstone_engine_options* options, kstone_engine** out, char* error,
                                       size_t error_size);
/* NULL is ignored. Must not race with any other call on the engine. */
KSTONE_EXPORT void kstone_engine_destroy(kstone_engine* engine);

KSTONE_EXPORT size_t kstone_engine_nq(const kstone_engine* engine);
KSTONE_EXPORT size_t kstone_engine_nv(const kstone_engine* engine);
KSTONE_EXPORT double kstone_engine_dt(const kstone_engine* engine);

/* One read-solve-write cycle at time now (s), see kstone::Engine::step. */
KSTONE_EXPORT int kstone_engine_step(kstone_engine* engine, double now, char* error, size_t error_size);
/* Cycles for duration_s (<= 0: until stopped) or until kstone_engine_request_stop; see kstone::Engine::run.
 * A stop requested while no run is active ends the next run immediately; each run clears the request on return. */
KSTONE_EXPORT int kstone_engine_run(kstone_engine* engine, double duration_s, char* error, size_t error_size);
KSTONE_EXPORT void kstone_engine_request_stop(kstone_engine* engine);
KSTONE_EXPORT void kstone_engine_set_paused(kstone_engine* engine, int paused);

KSTONE_EXPORT int kstone_engine_snapshot(const kstone_engine* engine, kstone_engine_status* out, char* error,
                                         size_t error_size);

/* Takes effect on the next cycle; n must match the port's flat size. */
KSTONE_EXPORT int kstone_engine_set_port(kstone_engine* engine, const char* module, const char* port,
                                         const double* values, size_t n, char* error, size_t error_size);
/* *n receives the flat size; values may be NULL to query it, otherwise capacity must be at least that size. */
KSTONE_EXPORT int kstone_engine_read_port(const kstone_engine* engine, const char* module, const char* port,
                                          double* values, size_t capacity, size_t* n, char* error,
                                          size_t error_size);

#ifdef __cplusplus
}
#endif

#endif
