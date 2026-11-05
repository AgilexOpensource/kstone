/* Robot driver plugin ABI. Pure C so drivers can be built with any compiler or C++ standard library.
 *
 * A plugin is a shared library exporting KSTONE_ROBOT_PLUGIN_SYMBOL:
 *
 *     #include <kstone/runtime/robot_plugin.h>
 *     KSTONE_ROBOT_PLUGIN_EXPORT const kstone_robot_plugin* kstone_robot_plugin_get(void) { return &my_plugin; }
 *
 * Selected from the engine configuration with
 *
 *     robot: {type: plugin, library: libmy_arm.so, params: {...}}
 *
 * Every call happens on the engine's control thread. read/write run once per cycle and must not block
 * longer than the cycle budget; avoid allocation there. All vectors use the model's q / v layout
 * (see kstone_robot_config.joints).
 * A failed cycle writes nothing and the engine never synthesizes a stop: enforce your own command timeout. */
#ifndef KSTONE_RUNTIME_ROBOT_PLUGIN_H
#define KSTONE_RUNTIME_ROBOT_PLUGIN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bumped on incompatible changes; new trailing fields are signalled by struct_size instead. */
#define KSTONE_ROBOT_ABI_VERSION 1u
#define KSTONE_ROBOT_PLUGIN_SYMBOL "kstone_robot_plugin_get"

#if defined(_WIN32)
#define KSTONE_ROBOT_PLUGIN_EXPORT __declspec(dllexport)
#else
#define KSTONE_ROBOT_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

/* Return codes; the non-zero values match kstone::ErrorCode. */
enum {
    KSTONE_ROBOT_OK = 0,
    KSTONE_ROBOT_INVALID_ARGUMENT = 1,
    KSTONE_ROBOT_OUT_OF_RANGE = 2,
    KSTONE_ROBOT_NOT_FOUND = 3,
    KSTONE_ROBOT_PARSE_ERROR = 4,
    KSTONE_ROBOT_IO_ERROR = 5,
    KSTONE_ROBOT_UNSUPPORTED = 6,
    KSTONE_ROBOT_INTERNAL = 7
};

/* One model joint with configuration coordinates q[idx_q, idx_q + nq) and velocities v[idx_v, idx_v + nv). */
typedef struct kstone_robot_joint {
    const char* name;
    size_t idx_q;
    size_t nq;
    size_t idx_v;
    size_t nv;
} kstone_robot_joint;

/* Valid only during create(); copy anything the driver keeps. */
typedef struct kstone_robot_config {
    uint32_t struct_size;
    size_t nq;
    size_t nv;
    double rate_hz;
    const kstone_robot_joint* joints; /* Model joints in q order; mimic followers have nq = nv = 0. */
    size_t joint_count;
    const double* q0;        /* robot.initial, or the model's neutral configuration (size nq). */
    const char* params_json; /* robot.params as a JSON object ("{}" when absent). */
    const char* base_dir;    /* Directory of the configuration file, for relative driver paths. */
} kstone_robot_config;

/* error/error_size: on failure write a NUL-terminated message (truncate to error_size). */
typedef struct kstone_robot_plugin {
    uint32_t abi_version; /* KSTONE_ROBOT_ABI_VERSION */
    uint32_t struct_size; /* sizeof(kstone_robot_plugin) */
    const char* name;

    /* Connects to the robot. The first read() follows immediately and must return the measured state. */
    int (*create)(const kstone_robot_config* config, void** handle, char* error, size_t error_size);
    /* Called once per successful create(), also after errors; must leave the robot safe (hold or brake). */
    void (*destroy)(void* handle);
    /* Measured state. An error aborts the cycle before solving. *stamp_ns (preset to -1 = unknown) is the
     * CLOCK_MONOTONIC time of the measurement in ns. Quaternions and continuous-joint (cos, sin) pairs are
     * renormalized if their norm is within 1e-3 of 1, else the read fails. */
    int (*read)(void* handle, double* q, size_t nq, double* v, size_t nv, int64_t* stamp_ns, char* error,
                size_t error_size);
    /* q: configuration to reach after one cycle (1 / rate_hz); v: velocity to get there. Called every cycle;
     * fresh = 0 while the feedback is stale (feedback_timeout_s; an unknown *stamp_ns counts as stale) and the
     * driver decides whether to send such a command. */
    int (*write)(void* handle, const double* q, size_t nq, const double* v, size_t nv, int fresh, char* error,
                 size_t error_size);
} kstone_robot_plugin;

typedef const kstone_robot_plugin* (*kstone_robot_plugin_fn)(void);

#ifdef __cplusplus
}
#endif

#endif /* KSTONE_RUNTIME_ROBOT_PLUGIN_H */
