/* Minimal robot driver plugin: an ideal robot that reaches every commanded configuration.
 * Copy this file as the starting point for a real driver: open the bus/SDK in create(), fill q/v from the
 * latest feedback in read(), send the command in write(), and bring the robot to a safe stop in destroy().
 *
 * Build: cc -shared -fPIC -I<prefix>/include example_robot.c -o libexample_robot.so
 *    or: find_package(kstone) + add_library(example_robot MODULE ...) linked to kstone::robot_plugin
 * Use:   robot: {type: plugin, library: ./libexample_robot.so, params: {}} */
#define _POSIX_C_SOURCE 199309L
#include <kstone/runtime/robot_plugin.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct example_robot {
    size_t nq;
    size_t nv;
    double* q;
    double* v;
} example_robot;

static int example_create(const kstone_robot_config* config, void** handle, char* error, size_t error_size) {
    example_robot* r = (example_robot*)calloc(1, sizeof *r);
    if (r == NULL) {
        snprintf(error, error_size, "out of memory");
        return KSTONE_ROBOT_INTERNAL;
    }
    r->nq = config->nq;
    r->nv = config->nv;
    r->q = (double*)calloc(config->nq + 1, sizeof(double));
    r->v = (double*)calloc(config->nv + 1, sizeof(double));
    if (r->q == NULL || r->v == NULL) {
        free(r->q);
        free(r->v);
        free(r);
        snprintf(error, error_size, "out of memory");
        return KSTONE_ROBOT_INTERNAL;
    }
    /* A real driver would read the measured configuration here instead. */
    memcpy(r->q, config->q0, config->nq * sizeof(double));
    *handle = r;
    return KSTONE_ROBOT_OK;
}

static void example_destroy(void* handle) {
    example_robot* r = (example_robot*)handle;
    free(r->q);
    free(r->v);
    free(r);
}

static int example_read(void* handle, double* q, size_t nq, double* v, size_t nv, int64_t* stamp_ns,
                        char* error, size_t error_size) {
    example_robot* r = (example_robot*)handle;
    struct timespec now;
    if (nq != r->nq || nv != r->nv) {
        snprintf(error, error_size, "state size mismatch");
        return KSTONE_ROBOT_INVALID_ARGUMENT;
    }
    memcpy(q, r->q, nq * sizeof(double));
    memcpy(v, r->v, nv * sizeof(double));
    /* A real driver stamps with the time the feedback was measured, not the time it was read. */
    if (clock_gettime(CLOCK_MONOTONIC, &now) == 0) *stamp_ns = (int64_t)now.tv_sec * 1000000000 + now.tv_nsec;
    return KSTONE_ROBOT_OK;
}

static int example_write(void* handle, const double* q, size_t nq, const double* v, size_t nv, int fresh,
                         char* error, size_t error_size) {
    example_robot* r = (example_robot*)handle;
    (void)
        fresh; /* Decide here whether to send commands solved without fresh feedback; this one always does. */
    if (nq != r->nq || nv != r->nv) {
        snprintf(error, error_size, "command size mismatch");
        return KSTONE_ROBOT_INVALID_ARGUMENT;
    }
    memcpy(r->q, q, nq * sizeof(double));
    memcpy(r->v, v, nv * sizeof(double));
    return KSTONE_ROBOT_OK;
}

static const kstone_robot_plugin example_plugin = {
    KSTONE_ROBOT_ABI_VERSION,
    sizeof(kstone_robot_plugin),
    "example",
    example_create,
    example_destroy,
    example_read,
    example_write,
};

KSTONE_ROBOT_PLUGIN_EXPORT const kstone_robot_plugin* kstone_robot_plugin_get(void) {
    return &example_plugin;
}
