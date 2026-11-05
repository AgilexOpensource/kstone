"""Pins runtime dependencies to the shared-library wheels present at build time."""

from __future__ import annotations

import re
from importlib.metadata import PackageNotFoundError, version
from typing import Any, Mapping

# Pinocchio 4.0 and 4.1 crash when swapped at runtime, so each library keeps the minor it was built with.
_SAME_MINOR = [
    "libpinocchio",
    "libcoal",
    "cmeel-urdfdom",
    "cmeel-octomap",
    "cmeel-tinyxml2",
    "cmeel-console-bridge",
]
# Boost puts its full version in the soname.
_SAME_RELEASE = ["cmeel-boost"]


def _release(name: str) -> tuple[int, int, int] | None:
    try:
        m = re.match(r"(\d+)(?:\.(\d+))?(?:\.(\d+))?", version(name))
    except PackageNotFoundError:
        return None
    if not m:
        return None
    major, minor, patch = (int(g or 0) for g in m.groups())
    return major, minor, patch


def dynamic_metadata(settings: Mapping[str, Any], project: Mapping[str, Any]) -> dict[str, Any]:
    deps = list(settings.get("static", []))
    for name in _SAME_MINOR:
        r = _release(name)
        if r:
            deps.append(f"{name}>={r[0]}.{r[1]}.{r[2]},=={r[0]}.{r[1]}.*")
    for name in _SAME_RELEASE:
        r = _release(name)
        if r:
            deps.append(f"{name}=={r[0]}.{r[1]}.{r[2]}.*")
    return {"dependencies": deps}
