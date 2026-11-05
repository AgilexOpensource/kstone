"""kstone: inverse kinematics and control."""

import os
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

_Vector = NDArray[np.float64]
_Matrix = NDArray[np.float64]
_PathLike = str | os.PathLike[str]

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

__version__: str
SNAPSHOT_SCHEMA_VERSION: int

class Error(RuntimeError):
    """Raised by kstone; `code` names the kind of failure, e.g. 'invalid_argument'."""

    code: str

def load_engine(path: _PathLike, vars: dict[str, str] = ...) -> Engine:
    """Load an engine from a YAML config file.

    Args:
        path: Config file. Relative paths in it resolve against its folder.
        vars: Values for ${name} placeholders. Unset names fall back to
            environment variables, then to the ${name:-default} default.

    Raises:
        kstone.Error: The file can't be read or the config is invalid.

    Example:
        engine = kstone.load_engine("robot.yaml", vars={"ip": "192.168.1.10"})
    """

def load_engine_from_string(text: str, base_dir: _PathLike = ".", vars: dict[str, str] | None = None) -> Engine:
    """Load an engine from YAML text.

    Args:
        text: The config.
        base_dir: Folder that relative paths in the config resolve against.
        vars: Values for ${name} placeholders, as in load_engine.

    Raises:
        kstone.Error: The config is invalid.
    """

def load_solver(path: _PathLike, vars: dict[str, str] = ...) -> Solver:
    """Load only the solver part of a config file: model and modules, no robot or loop.

    Args:
        path: Config file; its robot section is ignored.
        vars: Values for ${name} placeholders, as in load_engine.

    Raises:
        kstone.Error: The file can't be read or the config is invalid.
    """

def load_solver_from_string(text: str, base_dir: _PathLike = ".", vars: dict[str, str] | None = None) -> Solver:
    """Load a Solver from YAML text.

    Args:
        text: The config; its robot section is ignored.
        base_dir: Folder that relative paths in the config resolve against.
        vars: Values for ${name} placeholders, as in load_engine.
    """

def doctor(print: bool = False) -> dict[str, Any]:
    """Check the installation.

    Args:
        print: Also print a readable report.

    Returns:
        A dict; result["ok"] is True when everything works.
    """

def _main() -> int:
    """Run the kstone command line with sys.argv."""

class Model:
    """Robot model loaded from URDF or MJCF; immutable."""

    @staticmethod
    def from_file(path: _PathLike, format: str = "", root: str = "fixed", mimic: str = "independent") -> Model:
        """Load a robot model from a URDF or MJCF file.

        Args:
            path: The file.
            format: 'urdf' or 'mjcf'; '' picks it from the extension (.urdf, .xml, .mjcf).
            root: 'fixed' fixes the root link to the world; 'free_flyer' adds a floating
                base with 7 entries in q (x, y, z, qx, qy, qz, qw) and 6 in v.
            mimic: 'independent' keeps mimic joints as ordinary joints; 'coupled' makes
                them follow their leader, adding no entries to q.

        Raises:
            kstone.Error: The file can't be read or parsed.
            ValueError: Unknown root or mimic.
        """

    @staticmethod
    def from_string(
        text: str, format: str = "urdf", base_dir: _PathLike = "", root: str = "fixed", mimic: str = "independent"
    ) -> Model:
        """Load a robot model from URDF or MJCF text.

        Args:
            text: The model.
            format: 'urdf' or 'mjcf'.
            base_dir: Folder that mesh paths in the text resolve against.
            root: 'fixed' or 'free_flyer', as in from_file.
            mimic: 'independent' or 'coupled', as in from_file.
        """

    @property
    def name(self) -> str:
        """Robot name."""

    @property
    def nq(self) -> int:
        """Size of a configuration q."""

    @property
    def nv(self) -> int:
        """Size of a velocity v."""

    @property
    def joint_names(self) -> list[str]:
        """Joint names by id; index 0 is the fixed 'universe' joint."""

    @property
    def joints(self) -> list[dict[str, Any]]:
        """Joint details: name, type, id, parent, idx_q, nq, idx_v, nv."""

    @property
    def frame_names(self) -> list[str]:
        """Frame names by index."""

    def joint_id(self, name: str) -> int:
        """Joint id by name; raises kstone.Error (code 'not_found') if there is none."""

    def frame_id(self, name: str) -> int:
        """Frame index by name; raises kstone.Error (code 'not_found') if there is none."""

    def q_index(self, joint: str | int) -> int:
        """Position of the joint's first entry in q.

        Args:
            joint: Joint name or id.

        Example:
            q[model.q_index("j1")] = 0.5
        """

    def v_index(self, joint: str | int) -> int:
        """Position of the joint's first entry in v.

        Args:
            joint: Joint name or id.
        """

    def neutral(self) -> _Vector:
        """Default configuration (zeros, unit quaternions), size nq."""

    def integrate(self, q: ArrayLike, v: ArrayLike) -> _Vector:
        """Configuration reached from q by moving with velocity v for 1 s.

        Args:
            q: Configuration, size nq.
            v: Velocity, size nv; scale it by dt for a step of dt seconds.
        """

    def normalize(self, q: ArrayLike, tolerance: float = 1e-3) -> _Vector:
        """Copy of q with quaternions rescaled to unit length.

        Args:
            q: Configuration, size nq.
            tolerance: Largest accepted deviation of a norm from 1.

        Raises:
            kstone.Error: A norm is off by more than tolerance.
        """

    @property
    def lower_position_limit(self) -> _Vector:
        """Lower joint limits, size nq; see has_position_limit."""

    @property
    def upper_position_limit(self) -> _Vector:
        """Upper joint limits, size nq; see has_position_limit."""

    @property
    def velocity_limit(self) -> _Vector:
        """Velocity limits, size nv."""

    @property
    def effort_limit(self) -> _Vector:
        """Effort limits, size nv."""

    def has_position_limit(self, joint: str | int) -> bool:
        """Whether the joint has position limits.

        Args:
            joint: Joint name or id.
        """

    def check_position_limits(self, q: ArrayLike, tolerance: float = 1e-6) -> None:
        """Raise kstone.Error if q is outside the position limits.

        Args:
            q: Configuration, size nq.
            tolerance: Allowed overshoot, in rad or m.
        """

    @property
    def total_mass(self) -> float:
        """Mass of the whole robot in kg."""

    def subtree_mass(self, root: str | int) -> float:
        """Mass in kg of the joint and everything after it.

        Args:
            root: Joint name or id.
        """

class Workspace:
    """Forward kinematics for a model: call update(q), then query poses and Jacobians."""

    def __init__(self, model: Model) -> None:
        """Create a workspace for the model."""

    @property
    def model(self) -> Model:
        """The model."""

    @property
    def q(self) -> _Vector:
        """Configuration of the last update()."""

    def update(self, q: ArrayLike) -> None:
        """Compute kinematics at q; call it before the queries below.

        Args:
            q: Configuration, size nq.
        """

    def frame_pose(self, frame: str | int) -> _Matrix:
        """4x4 pose of the frame in the world, as of the last update().

        Args:
            frame: Frame name or index.
        """

    def joint_pose(self, joint: str | int) -> _Matrix:
        """4x4 pose of the joint in the world, as of the last update().

        Args:
            joint: Joint name or id.
        """

    def center_of_mass(self, root: str | int = 0) -> _Vector:
        """Centre of mass in the world, as of the last update().

        Args:
            root: Joint name or id; only it and the links after it count.
                The default 0 is the whole robot.
        """

    def center_of_mass_jacobian(self, root: str | int = 0) -> _Matrix:
        """3 x nv Jacobian of the centre of mass.

        Args:
            root: Joint name or id, as in center_of_mass.
        """

    def frame_jacobian(self, frame: str | int, reference: str = "local_world_aligned") -> _Matrix:
        """6 x nv Jacobian of the frame: rows 0-2 linear, rows 3-5 angular.

        Args:
            frame: Frame name or index.
            reference: Axes the rows are expressed in: 'local_world_aligned' (world axes
                at the frame origin), 'local' (frame axes) or 'world' (world axes at the
                world origin).
        """

class SolveResult:
    """Result of one solve."""

    @property
    def v(self) -> _Vector:
        """Joint velocity, size nv."""

    @property
    def q_next(self) -> _Vector:
        """Configuration after dt, size nq; pass it as the next q."""

    @property
    def stages(self) -> list[dict[str, Any]]:
        """Per priority level: level, status, exitflag, iterations, held."""

    @property
    def residuals(self) -> list[dict[str, Any]]:
        """Per task: name, level and remaining error."""

    @property
    def held(self) -> bool:
        """True if some level could not be met and was held back."""

    @property
    def step_scale(self) -> float:
        """1.0, or less when the step was shortened to stay within constraints."""

    @property
    def quality(self) -> str:
        """'nominal' when every task was solved as asked, else 'degraded'."""

class Module:
    """One module of an engine or solver; get it with engine.module(id)."""

    @property
    def id(self) -> str:
        """Module id from the config."""

    @property
    def type(self) -> str:
        """Module type, e.g. 'frame_task'."""

    @property
    def enabled(self) -> bool:
        """Whether the module takes part in the solve."""

    @enabled.setter
    def enabled(self, value: bool) -> None: ...
    @property
    def running(self) -> bool:
        """True if the module was active in the last step."""

    @property
    def held_by(self) -> str | None:
        """Id of the higher-priority module that blocked this one in the last step, or None."""

    @property
    def preempted(self) -> int:
        """How many times another module has blocked this one."""

    @property
    def home_state(self) -> str | None:
        """Homing progress: idle, waiting, executing, succeeded, timed_out or aborted; None if not homeable."""

    @property
    def ports(self) -> list[dict[str, Any]]:
        """Input ports as dicts with name, type, labels, lower and upper."""

    def read_port(self, name: str) -> Any:
        """Current value of an input port.

        Args:
            name: Port name; see ports.

        Returns:
            A float, bool or array, or for a pose {'translation': [x, y, z],
            'rotation': [w, x, y, z]}.
        """

    def write_port(self, name: str, value: Any) -> None:
        """Set an input port; the module uses it from the next step.

        Args:
            name: Port name; see ports.
            value: A float, bool or sequence of the port's size. A pose takes a 4x4
                matrix or {'translation': [x, y, z], 'rotation': [w, x, y, z]}.

        Example:
            engine.module("arm").write_port("target", [0.5, 0.0, 0.0])
        """

    def home(self) -> None:
        """Start homing on the next step; watch home_state."""

    @property
    def homeable(self) -> bool:
        """Whether home() is supported."""

    @property
    def tunable(self) -> bool:
        """Whether tune() is supported."""

    @property
    def tune_schema(self) -> dict[str, Any] | None:
        """JSON Schema of the tune() parameters, or None."""

    def tune(self, params: dict[str, Any]) -> None:
        """Change some parameters; on error nothing changes.

        Args:
            params: Names and new values, e.g. {'gain': 2.0}; see tune_schema.
        """

    def params(self) -> dict[str, Any] | None:
        """Current tunable parameters, or None."""

    def outputs(self, q: ArrayLike) -> dict[str, Any]:
        """Display values computed at q, keyed by output name.

        Args:
            q: Configuration, size nq.
        """

    def visuals(self) -> list[dict[str, Any]]:
        """Shapes to draw: dicts with name, frame, shape and placement (4x4, or None when hidden)."""

class Solver:
    """Solver without a robot or loop; you pass q and get the next q. See load_solver."""

    @property
    def model(self) -> Model:
        """Robot model."""

    @property
    def initial(self) -> _Vector:
        """Start configuration: robot.initial from the config, or the neutral pose."""

    @property
    def time(self) -> float:
        """Sum of dt over all successful solves."""

    @property
    def modules(self) -> list[str]:
        """Module ids in config order."""

    def module(self, id: str) -> Module:
        """Module by id."""

    def solve(
        self, q: ArrayLike, dt: float, v: ArrayLike | None = None, measured: ArrayLike | None = None
    ) -> SolveResult:
        """Solve one step and return where the robot should go next.

        Args:
            q: Current configuration, size nq.
            dt: Step length in seconds, > 0.
            v: Current velocity, size nv; pass the previous result.v. Defaults to zeros.
            measured: Measured configuration, size nq, if you have one; modules and
                closed-loop constraints use it.

        Returns:
            The result; pass result.q_next back as the next q.

        Raises:
            ValueError: A size is wrong or dt <= 0.

        Example:
            q = solver.initial
            for _ in range(100):
                q = solver.solve(q, 0.01).q_next
        """

    def resynchronize(self, rebase: bool = False) -> None:
        """Call after q jumps, e.g. after you move the robot by hand.

        Args:
            rebase: Also drop the targets the modules hold, so they hold the new q.
        """

    @property
    def solver_tune_schema(self) -> dict[str, Any]:
        """JSON Schema of the tune_solver() parameters."""

    def tune_solver(self, params: dict[str, Any]) -> None:
        """Change some solver options; on error nothing changes.

        Args:
            params: Names and new values, e.g. {'damping': 1e-4}; see solver_tune_schema.
        """

    def solver_params(self) -> dict[str, Any]:
        """Current solver options."""

    def tune_joint_limits(self, params: dict[str, Any]) -> None:
        """Narrow joint limits at run time; on error nothing changes.

        Args:
            params: {'velocity': {joint: limit}, 'position': {'lower': {joint: limit},
                'upper': {joint: limit}}}, any part optional. Limits can't exceed the
                model's. {'reset': True} removes all overrides.

        Example:
            solver.tune_joint_limits({"velocity": {"j1": 0.5}})
        """

    def joint_limits(self) -> dict[str, Any]:
        """Joint limit overrides currently in force."""

class DebugServer:
    """Debug server started by engine.serve()."""

    @property
    def port(self) -> int:
        """Port the server listens on."""

    def stop(self) -> None:
        """Stop the server; safe to call twice."""

class Engine:
    """Control loop built from a YAML config; see load_engine."""

    @property
    def dt(self) -> float:
        """Cycle period in seconds."""

    @property
    def rate_hz(self) -> float:
        """Cycle rate in Hz."""

    @property
    def realtime(self) -> bool:
        """Whether run() paces cycles to wall-clock time."""

    @property
    def feedback_timeout_s(self) -> float:
        """Feedback older than this counts as stale."""

    @property
    def max_failed_cycles(self) -> int:
        """Consecutive failed cycles that stop the engine."""

    @property
    def feedback_joints(self) -> list[str]:
        """Joints whose measured position feeds back into the solve."""

    @property
    def safety(self) -> dict[str, Any]:
        """Command check settings: mode ('reject', 'clamp' or 'off'), velocity_scale, tolerance."""

    @property
    def quality(self) -> str:
        """Quality of the last solve: 'nominal' or 'degraded'."""

    @property
    def solver_tune_schema(self) -> dict[str, Any]:
        """JSON Schema of the tune_solver() parameters."""

    def tune_solver(self, params: dict[str, Any]) -> None:
        """Change some solver options; on error nothing changes.

        Args:
            params: Names and new values, e.g. {'damping': 1e-4}; see solver_tune_schema.
        """

    def solver_params(self) -> dict[str, Any]:
        """Current solver options."""

    def tune_joint_limits(self, params: dict[str, Any]) -> None:
        """Narrow joint limits at run time; on error nothing changes.

        Args:
            params: {'velocity': {joint: limit}, 'position': {'lower': {joint: limit},
                'upper': {joint: limit}}}, any part optional. Limits can't exceed the
                model's. {'reset': True} removes all overrides.

        Example:
            engine.tune_joint_limits({"velocity": {"j1": 0.5}})
        """

    def joint_limits(self) -> dict[str, Any]:
        """Joint limit overrides currently in force."""

    @property
    def result(self) -> SolveResult:
        """Result of the last successful solve."""

    @property
    def running(self) -> bool:
        """True while run() is active."""

    @property
    def time(self) -> float:
        """Engine time in seconds."""

    @property
    def stats(self) -> dict[str, Any]:
        """Counters and cycle times, e.g. cycles, overruns, failed_cycles, cycle_p99_s."""

    @property
    def q(self) -> _Vector:
        """Robot joint positions from the last read, size nq."""

    @property
    def v(self) -> _Vector:
        """Robot joint velocities from the last read, size nv."""

    @property
    def ideal(self) -> _Vector:
        """Commanded positions the next solve starts from, size nq."""

    @property
    def fresh(self) -> bool:
        """False when robot feedback is older than feedback_timeout_s."""

    @property
    def written(self) -> bool:
        """Whether the last cycle sent a command (False while paused)."""

    @property
    def command(self) -> dict[str, Any]:
        """Last command sent to the robot: {q, v, fresh}."""

    @property
    def measured(self) -> _Vector | None:
        """Measured positions used by feedback, or None while feedback is stale."""

    @property
    def model(self) -> Model:
        """Robot model."""

    @property
    def modules(self) -> list[str]:
        """Module ids in config order."""

    def module(self, id: str) -> Module:
        """Module by id."""

    def step(self) -> None:
        """Run one cycle (read, solve, write)."""

    @property
    def paused(self) -> bool:
        """While True, cycles keep reading but do not solve or send commands."""

    @paused.setter
    def paused(self, value: bool) -> None: ...
    def step_paused(self, cycles: int = 1) -> None:
        """While paused, let some cycles run normally.

        Args:
            cycles: How many cycles.
        """

    def set_rate_hz(self, rate_hz: float) -> None:
        """Change the cycle rate from the next cycle on.

        Args:
            rate_hz: New rate in Hz, > 0.
        """

    def reset_stats(self) -> None:
        """Reset stats; time is kept."""

    def snapshot(self) -> dict[str, Any]:
        """State of the last cycle as one dict; works while run() is active."""

    def run(self, duration_s: float = 0.0) -> None:
        """Run cycles, paced to wall-clock time if realtime is set.

        Args:
            duration_s: Engine time to run in seconds; 0 runs until stop() or Ctrl-C.
        """

    def stop(self) -> None:
        """Make run() return; call it from another thread."""

    def serve(
        self,
        host: str = "127.0.0.1",
        port: int = 7311,
        urdf: _PathLike | None = None,
        publish_hz: float = 30.0,
        token: str = "",
        command_timeout_s: float = 1.0,
        assets: bool = True,
    ) -> DebugServer:
        """Start the debug server used by the viewer and remote tools.

        Args:
            host: Address to listen on. Any address other than loopback requires a token.
            port: TCP port; 0 picks a free one (see server.port).
            urdf: URDF sent to clients; defaults to the model file if it is a URDF.
            publish_hz: Highest rate of state updates sent to clients.
            token: Secret clients must send, 16 to 1024 bytes; '' disables
                authentication. It travels in clear text.
            command_timeout_s: How long a remote command stays in effect when the
                client sets no timeout.
            assets: Let clients download the URDF and the files it references.

        Returns:
            The server; it runs until server.stop().
        """
