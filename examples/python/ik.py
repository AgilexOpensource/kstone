"""Offline IK: move the tool frame of arm.urdf 5 cm and check the result with a Workspace.

    python examples/python/ik.py tests/fixtures/arm.urdf
"""
import pathlib
import sys

import numpy as np

import kstone

urdf = pathlib.Path(sys.argv[1]).resolve()
config = f"""
model: {{file: {urdf.name}}}
modules:
  tool:
    type: frame_task
    frame: tool
    gain: 0.5
    orientation_weight: [0, 0, 0]  # position only: the arm has too few joints for a full pose
    trajectory: {{}}  # unshaped: the target applies at once
"""
solver = kstone.load_solver_from_string(config, base_dir=urdf.parent)
ws = kstone.Workspace(solver.model)

q = solver.initial
ws.update(q)
target = ws.frame_pose("tool")
target[:3, 3] += [0.0, 0.05, 0.05]
solver.module("tool").write_port("target", target)

for step in range(1, 501):
    q = solver.solve(q, 0.01).q_next
    ws.update(q)
    error = np.linalg.norm(ws.frame_pose("tool")[:3, 3] - target[:3, 3])
    if error < 1e-4:
        print(f"converged in {step} steps, position error {error:.2e} m")
        sys.exit(0)
print(f"not converged, position error {error:.2e} m")
sys.exit(1)
