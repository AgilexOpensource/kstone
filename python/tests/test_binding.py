"""Smoke test of the kstone extension: python test_binding.py <fixtures dir>."""

import sys
import threading
import time
from pathlib import Path

import numpy as np

import kstone

fixtures = Path(sys.argv[1])

report = kstone.doctor()
assert report["ok"], report["checks"]
assert report["version"] == kstone.__version__ and report["python"]["numpy"] == np.__version__
assert [c["name"] for c in report["checks"]] == ["libraries", "solve", "collision"]

# The `kstone` console script of the wheel runs the CLI in-process.
argv = sys.argv
for args, code in ((["--version"], 0), (["check", str(fixtures / "arm_loopback.yaml")], 0),
                   (["run", str(fixtures / "arm_loopback.yaml"), "--duration", "0.05"], 0), (["bogus"], 2)):
    sys.argv = ["kstone", *args]
    assert kstone._main() == code, args
sys.argv = argv


def expect_error(code, fn, *args):
    try:
        fn(*args)
    except kstone.Error as e:
        assert e.code == code, (e.code, str(e))
        return str(e)
    raise AssertionError(f"expected kstone.Error({code})")


msg = expect_error("io_error", kstone.load_engine, fixtures / "missing.yaml")
assert "missing.yaml" in msg, msg
e = kstone.load_engine(fixtures / "arm_include.yaml", vars={"RATE": "250"})
assert e.rate_hz == 250.0 and abs(e.dt - 0.004) < 1e-12
text = (fixtures / "arm_include.yaml").read_text()
assert kstone.load_engine_from_string(text, base_dir=fixtures).rate_hz == 100.0
e = kstone.load_engine_from_string(text, base_dir=str(fixtures), vars={"RATE": "50"})
assert e.rate_hz == 50.0 and e.model.nq == e.q.shape[0]
assert "include/modules.yaml" in expect_error("not_found", kstone.load_engine_from_string, text)
assert e.safety == {"mode": "reject", "velocity_scale": 1.0, "tolerance": 1e-6}
guarded = "safety: {mode: clamp, velocity_scale: 0.5, tolerance: 0.001}\n" + text
assert kstone.load_engine_from_string(guarded, base_dir=fixtures).safety == {
    "mode": "clamp", "velocity_scale": 0.5, "tolerance": 0.001}
for bad, field in (("{mode: hold}", "safety.mode"), ("{velocity_scale: 0}", "safety.velocity_scale")):
    msg = expect_error("invalid_argument", kstone.load_engine_from_string, f"safety: {bad}\n" + text, fixtures)
    assert field in msg and "KST-E10" in msg, msg

# Kinematics on arm.urdf: j3 is continuous (nq 2, nv 1), so steps go through integrate().
s = kstone.load_solver(fixtures / "arm_loopback.yaml")
assert set(s.modules) == {"tool", "arm", "fingers"} and s.time == 0.0
assert np.allclose(s.initial, kstone.load_engine(fixtures / "arm_loopback.yaml").q)
joints = {j["name"]: j["idx_q"] for j in s.model.joints}
target = next(p["name"] for p in s.module("arm").ports if p["type"] == "vector")
s.module("tool").enabled = False
s.module("arm").write_port(target, [0.3, -0.2, 0.1])
q = s.initial
for _ in range(400):
    r = s.solve(q, 0.01)
    q = r.q_next
assert abs(s.time - 4.0) < 1e-9 and r.quality == "nominal"
angles = [q[joints["j1"]], q[joints["j2"]], np.arctan2(q[joints["j3"] + 1], q[joints["j3"]])]  # j3 continuous: cos, sin
assert np.allclose(angles, [0.3, -0.2, 0.1], atol=1e-3), angles
for args in ((q[:-1], 0.01), (q, 0.01, np.zeros(1)), (q, 0.0), (q, float("nan"))):
    try:
        s.solve(*args)
        raise AssertionError(f"accepted {args[1:]}")
    except ValueError:
        pass
assert abs(s.time - 4.0) < 1e-9
s.resynchronize(rebase=True)
s.tune_solver({"damping": 0.5})
assert s.solver_params()["damping"] == 0.5 and "damping" in s.solver_tune_schema["properties"]
s.tune_joint_limits({"velocity": {"j1": 0.5}, "position": {"lower": {"j1": -1.0}}})
assert s.joint_limits()["velocity"]["j1"] == 0.5 and s.joint_limits()["position"]["lower"]["j1"] == -1.0
expect_error("invalid_argument", s.tune_joint_limits, {"velocity": {"j1": 0.4}, "position": {"upper": {"j1": -1.5}}})
assert s.joint_limits()["velocity"]["j1"] == 0.5
s.tune_joint_limits({"reset": True})
assert not s.joint_limits().get("velocity")
plugin = (fixtures / "arm_loopback.yaml").read_text().replace("type: loopback", "type: plugin\n  library: missing.so")
assert kstone.load_solver_from_string(plugin, base_dir=fixtures).modules
expect_error("io_error", kstone.load_solver, fixtures / "missing.yaml")

urdf = fixtures / "arm.urdf"
expect_error("not_found", kstone.Model.from_file, fixtures / "missing.urdf")
m = kstone.Model.from_file(urdf)
assert kstone.Model.from_string(urdf.read_text()).nq == m.nq
assert (m.nq, m.nv) == (7, 6), (m.nq, m.nv)
assert m.joint_names[0] == "universe" and m.joint_id("j2") == m.joint_names.index("j2")
assert m.joints[m.joint_id("j3")]["nq"] == 2 and m.joints[m.joint_id("j3")]["nv"] == 1
assert m.frame_names[m.frame_id("tool")] == "tool"
expect_error("not_found", m.joint_id, "nope")
for j in m.joints:
    assert m.q_index(j["name"]) == j["idx_q"] and m.v_index(j["name"]) == j["idx_v"]
assert m.q_index(m.joint_id("j2")) == m.q_index("j2")
expect_error("not_found", m.q_index, "nope")
try:
    m.v_index(len(m.joints))
    raise AssertionError("expected IndexError")
except IndexError:
    pass
assert kstone.Engine.__module__ == "kstone" and kstone.Error.__module__ == "kstone"
expect_error("not_found", m.frame_id, "nope")
q0 = m.neutral()
assert q0.shape == (m.nq,)
for fn, args in ((m.integrate, (np.zeros(3), np.zeros(m.nv))), (m.integrate, (q0, np.zeros(2))),
                 (m.integrate, (q0, np.full(m.nv, np.nan)))):
    try:
        fn(*args)
        raise AssertionError("expected ValueError")
    except ValueError:
        pass

ws = kstone.Workspace(m)
assert ws.model.nq == m.nq
expect_error("invalid_argument", ws.frame_pose, "tool")
for bad in (np.zeros(m.nq - 1), np.zeros(m.nq + 1), [0.0] * 3):
    try:
        ws.update(bad)
        raise AssertionError("expected ValueError")
    except ValueError:
        pass
try:
    ws.update(q0)
    ws.frame_jacobian("tool", "nowhere")
    raise AssertionError("expected ValueError")
except ValueError:
    pass
for index in (len(m.frame_names), -1):
    try:
        ws.frame_pose(index)
        raise AssertionError("expected IndexError")
    except IndexError:
        pass
expect_error("io_error", kstone.Model.from_file, fixtures)

free = kstone.Model.from_file(fixtures / "floating_base.urdf", root="free_flyer")
qf = free.neutral()
qf[3:7] *= 1.0 + 1e-6
expect_error("invalid_argument", kstone.Workspace(free).update, qf)
assert abs(np.linalg.norm(free.normalize(qf)[3:7]) - 1.0) < 1e-12
qf[3:7] = 0.0
expect_error("invalid_argument", free.normalize, qf)
expect_error("invalid_argument", free.integrate, free.neutral(), np.full(free.nv, 1e308))
expect_error("invalid_argument", free.integrate, qf, np.zeros(free.nv))

rng = np.random.default_rng(7)
q = m.integrate(q0, rng.uniform(-0.8, 0.8, m.nv))
ws.update(q)
assert np.allclose(ws.q, q)
pose = ws.frame_pose("tool")
assert pose.shape == (4, 4) and np.allclose(pose[3], [0, 0, 0, 1])
assert np.allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3))
assert np.allclose(ws.frame_pose(m.frame_id("tool")), pose)
J = ws.frame_jacobian("tool")
assert J.shape == (6, m.nv)

eps = 1e-6
fd = np.zeros((6, m.nv))
probe = kstone.Workspace(m)
for i in range(m.nv):
    poses = []
    for s in (eps, -eps):
        probe.update(m.integrate(q, s * np.eye(m.nv)[i]))
        poses.append(probe.frame_pose("tool"))
    fd[:3, i] = (poses[0][:3, 3] - poses[1][:3, 3]) / (2 * eps)
    dR = poses[0][:3, :3] @ poses[1][:3, :3].T
    fd[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (4 * eps)
assert np.allclose(J, fd, atol=1e-6), np.abs(J - fd).max()
local = ws.frame_jacobian("tool", "local")
R = pose[:3, :3]
assert np.allclose(local[:3], R.T @ J[:3]) and np.allclose(local[3:], R.T @ J[3:])

j1 = m.joints[m.joint_id("j1")]
assert (m.lower_position_limit.shape, m.velocity_limit.shape, m.effort_limit.shape) == ((m.nq,), (m.nv,), (m.nv,))
assert (m.lower_position_limit[j1["idx_q"]], m.upper_position_limit[j1["idx_q"]]) == (-2.0, 2.0)
assert (m.velocity_limit[j1["idx_v"]], m.effort_limit[j1["idx_v"]]) == (1.5, 10.0)
assert m.has_position_limit("j1") and not m.has_position_limit("j3") and not m.has_position_limit(0)
m.check_position_limits(q0)
over = q0.copy()
over[j1["idx_q"]] = 2.1
assert "j1" in expect_error("out_of_range", m.check_position_limits, over)
m.check_position_limits(over, tolerance=0.2)
assert abs(m.total_mass - 4.8) < 1e-12 and m.subtree_mass(0) == m.total_mass
assert abs(m.subtree_mass("finger_left_joint") - 0.1) < 1e-12
for fn in (m.has_position_limit, m.subtree_mass, ws.joint_pose, ws.center_of_mass):
    try:
        fn(len(m.joint_names))
        raise AssertionError("expected IndexError")
    except IndexError:
        pass
expect_error("not_found", m.subtree_mass, "nope")

finger = ws.joint_pose("finger_left_joint")
assert np.allclose(ws.joint_pose(m.joint_id("finger_left_joint")), finger)
assert np.allclose(ws.center_of_mass("finger_left_joint"), finger[:3, 3])
com = ws.center_of_mass()
assert com.shape == (3,) and np.allclose(ws.center_of_mass(0), com)
Jc = ws.center_of_mass_jacobian()
assert Jc.shape == (3, m.nv) and np.allclose(ws.center_of_mass_jacobian(0), Jc)
fd = np.zeros((3, m.nv))
for i in range(m.nv):
    ends = []
    for s in (eps, -eps):
        probe.update(m.integrate(q, s * np.eye(m.nv)[i]))
        ends.append(probe.center_of_mass())
    fd[:, i] = (ends[0] - ends[1]) / (2 * eps)
assert np.allclose(Jc, fd, atol=1e-6), np.abs(Jc - fd).max()
assert ws.center_of_mass_jacobian("j2").shape == (3, m.nv)
expect_error("invalid_argument", kstone.Workspace(m).center_of_mass)
expect_error("invalid_argument", kstone.Workspace(m).joint_pose, "j1")

e = kstone.load_engine(fixtures / "arm_loopback.yaml")
assert set(e.modules) == {"tool", "arm", "fingers"}, e.modules
expect_error("not_found", e.module, "nope")
arm = e.module("arm")
assert arm.type == "joint_target" and arm.enabled
target = next(p for p in arm.ports if p["type"] == "vector")
assert len(target["labels"]) == 3
expect_error("not_found", arm.read_port, "nope")
expect_error("invalid_argument", arm.write_port, target["name"], [1.0, 2.0])
expect_error("invalid_argument", arm.write_port, target["name"], "abc")
arm.write_port(target["name"], [0.3, -0.2, 0.1])
assert np.allclose(arm.read_port(target["name"]), [0.3, -0.2, 0.1])

tool = e.module("tool")
pose = next(p for p in tool.ports if p["type"] == "pose")
expect_error("invalid_argument", tool.write_port, pose["name"], {"translation": [0, 0, 0]})
msg = expect_error("invalid_argument", tool.write_port, pose["name"], [0.3, -0.2, 0.1])
assert "'translation'" in msg and "'rotation'" in msg, msg
assert "3 finite numbers" in expect_error("invalid_argument", arm.write_port, target["name"], "abc")
expect_error("invalid_argument", tool.write_port, pose["name"], {"translation": [0, 0, 0], "rotation": [0, 0, 0, 0]})
expect_error("invalid_argument", tool.write_port, pose["name"], {"translation": [0, 0, 0], "rotation": [1, 0, 0, 0], "frame": "x"})
expect_error("invalid_argument", arm.write_port, target["name"], [True, False, True])
value = tool.read_port(pose["name"])
assert value["translation"].shape == (3,) and value["rotation"].shape == (4,)
transform = np.eye(4)
transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
transform[:3, 3] = [0.1, 0.2, 0.3]
tool.write_port(pose["name"], transform)
value = tool.read_port(pose["name"])
assert np.allclose(value["translation"], [0.1, 0.2, 0.3])
assert np.allclose(np.abs(value["rotation"]), [np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
for bad in (np.eye(3), np.diag([2.0, 1, 1, 1]), np.diag([-1.0, 1, 1, 1])):
    expect_error("invalid_argument", tool.write_port, pose["name"], bad)

assert arm.homeable and arm.tunable and arm.tune_schema["type"] == "object"
assert arm.home_state == "idle"
out = tool.outputs(e.model.neutral())
assert out["planned"].shape == (3,) and isinstance(out["lag"], float)
assert isinstance(out["error"], float) and isinstance(out["angular_error"], float)
assert arm.outputs(e.model.neutral()) == {}
assert arm.visuals() == []
world_cfg = """model: {file: slider_collision.urdf}
rate_hz: 100
robot: {type: loopback}
modules:
  world:
    type: world_collision
    urdf: slider_collision.urdf
    obstacles:
      - {name: table, shape: {type: box, size: [0.4, 0.6, 0.02]}, pose: {translation: [0.5, 0, 0.3]}}
      - {name: post, shape: {type: cylinder, size: [0.05, 0.5, 0]}, enabled: false}
"""
world = kstone.load_engine_from_string(world_cfg, base_dir=fixtures).module("world")
table, post = world.visuals()
assert table["name"] == "table" and table["frame"] == "" and table["shape"]["type"] == "box"
assert np.allclose(table["shape"]["size"], [0.4, 0.6, 0.02]) and "vertices" not in table["shape"]
assert table["placement"].shape == (4, 4) and np.allclose(table["placement"][:3, 3], [0.5, 0, 0.3])
assert post["shape"]["type"] == "cylinder" and post["placement"] is None
world.write_port("table", {"translation": [0.6, 0.1, 0.3], "rotation": [1, 0, 0, 0]})
world.write_port("post/enabled", True)
table, post = world.visuals()
assert np.allclose(table["placement"][:3, 3], [0.6, 0.1, 0.3]) and np.allclose(post["placement"], np.eye(4))
assert isinstance(e.module("fingers").params(), dict)
gain = arm.params()["gain"]
arm.tune({"gain": np.float64(0.25), "trajectory": {"velocity_scale": 0.5}})
assert arm.params()["gain"] == 0.25 and arm.params()["trajectory"]["velocity_scale"] == 0.5
msg = expect_error("invalid_argument", arm.tune, {"gain": -1.0, "weight": 2.0})
assert "104" in msg and arm.params()["gain"] == 0.25
expect_error("invalid_argument", arm.tune, {"joints": ["j1"]})
expect_error("invalid_argument", arm.tune, {"gain": object()})
arm.tune({"gain": gain})

solver = e.solver_params()
assert e.solver_tune_schema["properties"]["damping"]["default"] == solver["damping"]
e.tune_solver({"limit_gain": 0.3, "mode": "lexicographic"})
assert e.solver_params()["limit_gain"] == 0.3 and e.solver_params()["mode"] == "lexicographic"
expect_error("invalid_argument", e.tune_solver, {"damping": 0.5, "limit_gain": 2.0})
assert e.solver_params()["damping"] == solver["damping"]
e.tune_solver(solver)
assert e.solver_params() == solver

e.step()
assert e.stats["cycles"] == 1 and e.q.shape[0] > 0 and e.v.shape[0] > 0
r = e.result
assert r.quality == "nominal" and not r.held and r.v.shape == e.v.shape and r.q_next.shape == e.q.shape
assert r.stages and r.stages[0]["status"] in ("optimal", "inexact") and r.stages[0]["iterations"] >= 0
assert "arm/joints" in {t["name"] for t in r.residuals} and 0.0 < r.step_scale <= 1.0, (r.residuals, r.step_scale)
e.run(0.5)
assert e.stats["cycles"] == 51, e.stats
assert abs(e.time - 0.51) < 1e-9
assert e.feedback_timeout_s == 0.25 and e.quality in ("nominal", "degraded")
assert e.stats["repeated_reads"] == 0 and e.stats["max_lateness_s"] == 0.0, e.stats
assert e.stats["guard_rejects"] == 0 and e.stats["guard_clamps"] == 0, e.stats
assert 0.0 < e.stats["cycle_p50_s"] <= e.stats["cycle_p99_s"] <= e.stats["cycle_p999_s"] <= e.stats["max_cycle_s"]
e.reset_stats()
assert e.stats["cycles"] == 0 and e.stats["cycle_p99_s"] == 0.0 and abs(e.time - 0.51) < 1e-9, e.stats
e.run(0.5)
assert e.stats["cycles"] == 50, e.stats

assert not e.paused and e.stats["held_cycles"] == 0
e.paused = True
e.step_paused(2)
e.run(0.1)
assert e.paused and e.stats["held_cycles"] == 8, e.stats
resyncs = e.stats["resyncs"]
e.paused = False
e.step()
assert e.stats["held_cycles"] == 8 and e.stats["resyncs"] == resyncs + 1, e.stats
assert e.stats["stale_cycles"] == 0 and e.feedback_joints == [], e.stats
rate, dt0 = e.rate_hz, e.dt
e.set_rate_hz(50.0)
assert e.rate_hz == rate
t = e.time
e.step()  # The switch lands inside this cycle; its time still uses the old dt.
assert e.rate_hz == 50.0 and abs(e.dt - 0.02) < 1e-12 and abs(e.time - t - dt0) < 1e-9
for bad in (0.0, float("nan")):
    try:
        e.set_rate_hz(bad)
        raise AssertionError("set_rate_hz accepted %r" % bad)
    except kstone.Error:
        pass
e.set_rate_hz(rate)
e.step()
assert e.ideal.shape == e.q.shape
assert e.fresh and e.written and e.command["fresh"] and e.measured is not None
assert e.command["q"].shape == e.q.shape and e.command["v"].shape == e.v.shape

snap = e.snapshot()
assert snap["schema_version"] == kstone.SNAPSHOT_SCHEMA_VERSION == 1
assert snap["stats"]["cycles"] == e.stats["cycles"] and np.array_equal(snap["state"]["q"], e.q)
assert snap["quality"] == e.quality and snap["written"] and not snap["paused"]
assert np.array_equal(snap["command"]["q"], e.command["q"]) and np.array_equal(snap["ideal"], e.ideal)
assert np.array_equal(snap["measured"], e.measured)
assert [m["id"] for m in snap["modules"]] == e.modules
assert set(snap["modules"][0]["ports"]) == {p["name"] for p in e.module(e.modules[0]).ports}

# Ctrl-C-free stop from another thread; state access is refused while running.
runner = threading.Thread(target=e.run)
runner.start()
while not e.running:
    time.sleep(0.001)
try:
    e.q
    raise AssertionError("q readable while running")
except RuntimeError as err:
    assert "running" in str(err)
first = e.snapshot()
later = e.snapshot()
assert e.running and later["stats"]["cycles"] > first["stats"]["cycles"] > 0
e.stop()
runner.join(5.0)
assert not runner.is_alive() and not e.running

print("ok")
