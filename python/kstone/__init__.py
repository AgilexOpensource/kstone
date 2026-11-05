"""kstone: inverse kinematics and control.

Quick start::

    import kstone
    solver = kstone.load_solver("robot.yaml")
    q = solver.initial
    result = solver.solve(q, dt=0.01)
"""

from ._core import (
    SNAPSHOT_SCHEMA_VERSION,
    DebugServer,
    Engine,
    Error,
    Model,
    Module,
    Solver,
    SolveResult,
    Workspace,
    __version__,
    _main,
    doctor,
    load_engine,
    load_engine_from_string,
    load_solver,
    load_solver_from_string,
)

__all__ = [
    "SNAPSHOT_SCHEMA_VERSION",
    "DebugServer",
    "Engine",
    "Error",
    "Model",
    "Module",
    "Solver",
    "SolveResult",
    "Workspace",
    "__version__",
    "doctor",
    "load_engine",
    "load_engine_from_string",
    "load_solver",
    "load_solver_from_string",
]

for _name in __all__:
    _obj = globals()[_name]
    if isinstance(_obj, type) or callable(_obj):
        try:
            _obj.__module__ = __name__
        except (AttributeError, TypeError):
            pass
del _name, _obj
