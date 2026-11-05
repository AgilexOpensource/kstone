/* C99 smoke test of the engine C ABI; built as C so the header stays C-clean. */
#include "kstone/runtime/engine_c.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond)                                                           \
    do {                                                                      \
        if (!(cond)) {                                                        \
            fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                       \
        }                                                                     \
    } while (0)

static const char* kConfig =
    "model: {file: arm.urdf}\n"
    "rate_hz: 100\n"
    "realtime: false\n"
    "robot: {type: loopback}\n"
    "modules:\n"
    "  arm: {type: joint_target, joints: [j1, j2, j3]}\n"
    "  tool:\n"
    "    type: frame_task\n"
    "    frame: tool\n"
    "    trajectory:\n"
    "      translation: {velocity: 0.3, acceleration: 1, jerk: 10}\n"
    "      rotation: {velocity: 1, acceleration: 2, jerk: 20}\n";

int main(void) {
    char err[256];
    kstone_engine* e = NULL;

    CHECK(kstone_engine_abi_version() == KSTONE_ENGINE_ABI_VERSION);

    /* Errors: bad YAML, missing file, null arguments, truncated or absent buffers. */
    CHECK(kstone_engine_create("model: [", KSTONE_FIXTURES, NULL, &e, err, sizeof err) ==
          KSTONE_ENGINE_PARSE_ERROR);
    CHECK(e == NULL && strlen(err) > 0);
    CHECK(kstone_engine_load(KSTONE_FIXTURES "/missing.yaml", NULL, &e, err, sizeof err) != KSTONE_ENGINE_OK);
    CHECK(kstone_engine_create(NULL, NULL, NULL, &e, err, sizeof err) == KSTONE_ENGINE_INVALID_ARGUMENT);
    CHECK(kstone_engine_create("model: [", KSTONE_FIXTURES, NULL, &e, err, 4) == KSTONE_ENGINE_PARSE_ERROR);
    CHECK(strlen(err) == 3);
    CHECK(kstone_engine_create("model: [", KSTONE_FIXTURES, NULL, &e, NULL, 0) == KSTONE_ENGINE_PARSE_ERROR);
    kstone_engine_destroy(NULL);

    CHECK(kstone_engine_load(KSTONE_FIXTURES "/arm_loopback.yaml", NULL, &e, err, sizeof err) ==
          KSTONE_ENGINE_OK);
    CHECK(e != NULL);
    kstone_engine_destroy(e);
    e = NULL;

    if (kstone_engine_create(kConfig, KSTONE_FIXTURES, NULL, &e, err, sizeof err) != KSTONE_ENGINE_OK) {
        fprintf(stderr, "create: %s\n", err);
        return 1;
    }
    const size_t nq = kstone_engine_nq(e), nv = kstone_engine_nv(e);
    CHECK(nq > 3 && nv > 3);
    CHECK(fabs(kstone_engine_dt(e) - 0.01) < 1e-12);

    /* Vector port: size query, round trip, short buffer, wrong size, non-finite, unknown names. */
    size_t n = 0;
    CHECK(kstone_engine_read_port(e, "arm", "target", NULL, 0, &n, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(n == 3);
    const double target[3] = {0.3, -0.2, 0.1};
    CHECK(kstone_engine_set_port(e, "arm", "target", target, 3, err, sizeof err) == KSTONE_ENGINE_OK);
    double got[3] = {0, 0, 0};
    CHECK(kstone_engine_read_port(e, "arm", "target", got, 3, &n, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(got[0] == 0.3 && got[1] == -0.2 && got[2] == 0.1);
    CHECK(kstone_engine_read_port(e, "arm", "target", got, 2, &n, err, sizeof err) ==
          KSTONE_ENGINE_OUT_OF_RANGE);
    CHECK(kstone_engine_set_port(e, "arm", "target", target, 2, err, sizeof err) ==
          KSTONE_ENGINE_INVALID_ARGUMENT);
    CHECK(strstr(err, "expected 3") != NULL);
    const double bad[3] = {0.0, NAN, 0.0};
    CHECK(kstone_engine_set_port(e, "arm", "target", bad, 3, err, sizeof err) ==
          KSTONE_ENGINE_INVALID_ARGUMENT);
    CHECK(kstone_engine_set_port(e, "nope", "target", target, 3, err, sizeof err) == KSTONE_ENGINE_NOT_FOUND);
    CHECK(strstr(err, "nope") != NULL);
    CHECK(kstone_engine_set_port(e, "arm", "nope", target, 3, err, sizeof err) == KSTONE_ENGINE_NOT_FOUND);

    /* Pose port: xyz + wxyz, quaternion normalized on write. */
    CHECK(kstone_engine_read_port(e, "tool", "target", NULL, 0, &n, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(n == 7);
    double pose[7];
    CHECK(kstone_engine_read_port(e, "tool", "target", pose, 7, &n, err, sizeof err) == KSTONE_ENGINE_OK);
    pose[3] = 2.0;
    pose[4] = pose[5] = pose[6] = 0.0;
    CHECK(kstone_engine_set_port(e, "tool", "target", pose, 7, err, sizeof err) == KSTONE_ENGINE_OK);
    double back[7];
    CHECK(kstone_engine_read_port(e, "tool", "target", back, 7, &n, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(fabs(back[3] - 1.0) < 1e-12 && fabs(back[0] - pose[0]) < 1e-12);
    const double zero_q[7] = {0, 0, 0, 0, 0, 0, 0};
    CHECK(kstone_engine_set_port(e, "tool", "target", zero_q, 7, err, sizeof err) ==
          KSTONE_ENGINE_INVALID_ARGUMENT);

    /* Steps and snapshot. */
    for (int k = 0; k < 200; ++k)
        if (kstone_engine_step(e, k * kstone_engine_dt(e), err, sizeof err) != KSTONE_ENGINE_OK) {
            fprintf(stderr, "step: %s\n", err);
            return 1;
        }
    double* q = calloc(nq, sizeof *q);
    double* cq = calloc(nq, sizeof *cq);
    double* v = calloc(nv, sizeof *v);
    kstone_engine_status s;
    memset(&s, 0, sizeof s);
    s.struct_size = (uint32_t)sizeof s;
    s.state_q = q;
    s.state_v = v;
    s.command_q = cq;
    CHECK(kstone_engine_snapshot(e, &s, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(s.cycles == 200);
    CHECK(s.written && s.fresh && !s.paused);
    CHECK(fabs(s.time - 199 * 0.01) < 1e-9);
    CHECK(s.step_scale > 0.0);
    /* tool holds its pose against arm, so only the direction toward the target is checked. */
    CHECK(cq[0] > 0.0 && cq[1] < 0.0);
    s.struct_size = 4;
    CHECK(kstone_engine_snapshot(e, &s, err, sizeof err) == KSTONE_ENGINE_INVALID_ARGUMENT);
    s.struct_size = (uint32_t)sizeof s;
    CHECK(kstone_engine_step(e, 1.0, err, sizeof err) == KSTONE_ENGINE_INVALID_ARGUMENT);
    CHECK(strstr(err, "backwards") != NULL);
    CHECK(kstone_engine_step(e, NAN, err, sizeof err) == KSTONE_ENGINE_INVALID_ARGUMENT);
    CHECK(kstone_engine_run(e, NAN, err, sizeof err) == KSTONE_ENGINE_INVALID_ARGUMENT);
    CHECK(kstone_engine_set_port(e, "tool", "target", NULL, 7, err, sizeof err) ==
          KSTONE_ENGINE_INVALID_ARGUMENT);
    CHECK(strstr(err, "null values") != NULL);

    /* run: a pending stop ends the next run at once, then is cleared. */
    CHECK(kstone_engine_run(e, 0.05, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(kstone_engine_snapshot(e, &s, err, sizeof err) == KSTONE_ENGINE_OK);
    const uint64_t after_run = s.cycles;
    CHECK(after_run == 205);
    kstone_engine_request_stop(e);
    CHECK(kstone_engine_run(e, 1.0, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(kstone_engine_snapshot(e, &s, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(s.cycles == after_run);
    CHECK(kstone_engine_run(e, 0.02, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(kstone_engine_snapshot(e, &s, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(s.cycles == after_run + 2);

    /* Paused cycles do not write. */
    kstone_engine_set_paused(e, 1);
    CHECK(kstone_engine_run(e, 0.01, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(kstone_engine_snapshot(e, &s, err, sizeof err) == KSTONE_ENGINE_OK);
    CHECK(s.paused && !s.written && s.held_cycles == 1);
    kstone_engine_set_paused(e, 0);

    free(q);
    free(cq);
    free(v);
    kstone_engine_destroy(e);
    if (failures)
        fprintf(stderr, "%d check(s) failed\n", failures);
    else
        printf("engine C ABI: ok\n");
    return failures ? 1 : 0;
}
