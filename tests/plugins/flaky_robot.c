/* Test-only plugin: ideal robot whose failures are selected through params_json substrings. */
#define _POSIX_C_SOURCE 199309L
#include <kstone/runtime/robot_plugin.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct flaky {
    size_t nq;
    size_t nv;
    int fail_read;
    int fail_write;
    int reject_stale;
    int report_groups;
    double q[64];
    double v[64];
} flaky;

static int flaky_create(const kstone_robot_config* c, void** handle, char* error, size_t error_size) {
    const char* p = c->params_json;
    if (strstr(p, "\"fail_create\":true")) {
        snprintf(error, error_size, "no robot on %s", "can0");
        return KSTONE_ROBOT_IO_ERROR;
    }
    if (c->struct_size < sizeof(kstone_robot_config) || c->nq > 64 || c->nv > 64)
        return KSTONE_ROBOT_UNSUPPORTED;
    /* The fixture arm: j1 is the first joint and the rate comes from rate_hz. */
    if (c->joint_count == 0 || strcmp(c->joints[0].name, "j1") != 0 || c->rate_hz != 100.0) {
        snprintf(error, error_size, "unexpected config");
        return KSTONE_ROBOT_INVALID_ARGUMENT;
    }
    flaky* r = (flaky*)calloc(1, sizeof *r);
    if (r == NULL) return KSTONE_ROBOT_INTERNAL;
    r->nq = c->nq;
    r->nv = c->nv;
    r->fail_read = strstr(p, "\"fail_read\":true") != NULL;
    r->fail_write = strstr(p, "\"fail_write\":true") != NULL;
    r->reject_stale = strstr(p, "\"reject_stale\":true") != NULL;
    r->report_groups = strstr(p, "\"report_groups\":true") != NULL;
    if (r->report_groups) {
        /* Expects feedback_groups [{name: hand, joints: [finger_left_joint]}]; the mimic follows its leader. */
        int ok =
            c->group_count == 2 && strcmp(c->groups[0], "hand") == 0 && strcmp(c->groups[1], "default") == 0;
        for (size_t i = 0; ok && i < c->joint_count; ++i)
            ok = c->joints[i].group == (strncmp(c->joints[i].name, "finger_", 7) == 0 ? 0u : 1u);
        if (!ok) {
            free(r);
            snprintf(error, error_size, "unexpected groups");
            return KSTONE_ROBOT_INVALID_ARGUMENT;
        }
    }
    memcpy(r->q, c->q0, c->nq * sizeof(double));
    *handle = r;
    return KSTONE_ROBOT_OK;
}

static void flaky_destroy(void* handle) { free(handle); }

static int flaky_read(void* handle, double* q, size_t nq, double* v, size_t nv, int64_t* stamp_ns,
                      int64_t* group_stamps_ns, size_t group_count, char* error, size_t error_size) {
    flaky* r = (flaky*)handle;
    struct timespec now;
    (void)stamp_ns; /* Left unknown on purpose; with report_groups only group 0 is stamped. */
    if (r->report_groups && group_count > 0 && clock_gettime(CLOCK_MONOTONIC, &now) == 0)
        group_stamps_ns[0] = (int64_t)now.tv_sec * 1000000000 + now.tv_nsec;
    if (r->fail_read) {
        snprintf(error, error_size, "encoder timeout");
        return KSTONE_ROBOT_IO_ERROR;
    }
    memcpy(q, r->q, nq * sizeof(double));
    memcpy(v, r->v, nv * sizeof(double));
    return KSTONE_ROBOT_OK;
}

static int flaky_write(void* handle, const double* q, size_t nq, const double* v, size_t nv, int fresh,
                       const uint8_t* group_attached, size_t group_count, char* error, size_t error_size) {
    flaky* r = (flaky*)handle;
    if (r->report_groups && group_count == 2 && group_attached[0]) {
        snprintf(error, error_size, "fresh=%d groups=%zu hand=%d default=%d", fresh, group_count,
                 group_attached[0], group_attached[1]);
        return KSTONE_ROBOT_IO_ERROR;
    }
    if (r->reject_stale && !fresh) {
        snprintf(error, error_size, "stale command");
        return KSTONE_ROBOT_IO_ERROR;
    }
    if (r->fail_write) {
        snprintf(error, error_size, "bus off");
        return 42; /* Unknown codes map to internal. */
    }
    memcpy(r->q, q, nq * sizeof(double));
    memcpy(r->v, v, nv * sizeof(double));
    return KSTONE_ROBOT_OK;
}

#ifndef FLAKY_ABI_VERSION
#define FLAKY_ABI_VERSION KSTONE_ROBOT_ABI_VERSION
#endif

static const kstone_robot_plugin plugin = {
    FLAKY_ABI_VERSION, sizeof(kstone_robot_plugin), "flaky", flaky_create, flaky_destroy, flaky_read,
    flaky_write,
};

KSTONE_ROBOT_PLUGIN_EXPORT const kstone_robot_plugin* kstone_robot_plugin_get(void) { return &plugin; }
