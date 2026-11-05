"""Turns the ci workflow inputs into the job lists, checks that the docs match them, and extracts the README C++ recipe."""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
UBUNTU = "22.04"
PYTHON = "3.10"
# Pinocchio minor -> (exact version, companion libraries); docs/install.md lists the same set.
EXACT = {"4.0": ("4.0.0", "libcoal==3.0.3 cmeel-urdfdom-headers==3.0.0")}
# (ubuntu, arch, python, pin, swap); swap is a runtime requirement installed after the build, then the tests run again.
ARCHS = ["x86_64", "aarch64"]
PIP_JOBS = [(UBUNTU, a, PYTHON, "4.0", "pin==4.0.0") for a in ARCHS]
PRESETS = ["debug", "asan", "tsan"]
ROS = [("humble", a) for a in ARCHS]
# Workflow checkboxes, off by default; ALL or a tag turns every one on.
OPTIONAL = ["aarch64", *PRESETS, "isolated", "source"]

def fail(msg: str) -> None:
    print(f"::error::{msg}")
    sys.exit(1)


def runner(ubuntu: str, arch: str) -> str:
    return f"ubuntu-{ubuntu}" + ("-arm" if arch == "aarch64" else "")


def pip_job(ubuntu: str, arch: str, python: str, pin: str, swap: str) -> dict[str, object]:
    version, companions = EXACT[pin]
    return dict(ubuntu=ubuntu, arch=arch, runner=runner(ubuntu, arch), python=python, pin=pin,
                deps=f"libpinocchio=={version} {companions}", swap=swap)


def check_docs() -> None:
    rows = re.findall(r"^\| (\d+\.\d+\.\d+) \| (\d+\.\d+\.\d+) \|", (ROOT / "docs/install.md").read_text(), re.M)
    expected = [(v, re.search(r"libcoal==([0-9.]+)", c).group(1)) for v, c in EXACT.values()]
    if rows != expected:
        fail(f"docs/install.md tested Pinocchio/Coal rows {rows} differ from plan.py {expected}")
    for doc in ("README.md", "docs/install.md"):
        text = (ROOT / doc).read_text()
        if f"Ubuntu {UBUNTU}" not in text or f"Python {PYTHON}" not in text:
            fail(f"{doc} does not name the tested Ubuntu {UBUNTU} and Python {PYTHON}")
    if cpp_block("README.md") != cpp_block("README.zh-CN.md"):
        fail("the C++ recipe in README.zh-CN.md differs from README.md")


def cpp_block(readme: str) -> str:
    text = (ROOT / readme).read_text()
    match = re.search(r"^### C\+\+\n.*?^```bash\n(.*?)^```", text, re.M | re.S)
    if not match:
        fail(f"no bash block under '### C++' in {readme}")
    return match.group(1)


SOURCE_PARTS = ("apt", "deps", "kstone")


def source_script(part: str) -> str:
    """One part of the README C++ recipe: apt, deps (Coal and Pinocchio, cacheable) or kstone in the checkout, tested."""
    parts: dict[str, list[str]] = {p: [] for p in SOURCE_PARTS}
    prefix, current = None, "apt"
    for line in cpp_block("README.md").splitlines():
        if line.startswith("sudo apt install "):
            line = "sudo apt-get update && sudo apt-get install -y " + line.removeprefix("sudo apt install ")
        elif line.startswith("PREFIX="):
            prefix, current = line, "deps"
            continue
        elif line.startswith("git clone") and line.endswith("/kstone.git"):
            line, current = 'ln -s "$GITHUB_WORKSPACE" kstone', "kstone"
        parts[current].append(line)
    if not prefix or not parts["kstone"] or not any("apt-get install" in s for s in parts["apt"]):
        fail("the README C++ recipe no longer has the expected apt install, PREFIX and kstone clone lines")
    parts["kstone"] += [
        "ctest --test-dir kstone/build --output-on-failure",
        'cmake -S kstone/tests/consumer -B consumer -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PREFIX"',
        "cmake --build consumer",
        "consumer/consumer kstone/tests/fixtures/arm.urdf",
    ]
    return "\n".join(["set -euxo pipefail", prefix, *parts[part]]).strip() + "\n"


def changelog_section(version: str) -> str:
    # Same rule as the release job's awk: from "## <version>" up to the next "## " heading.
    lines, on = [], False
    for line in (ROOT / "CHANGELOG.md").read_text().splitlines():
        if on and line.startswith("## "):
            break
        if on:
            lines.append(line)
        on = on or line == f"## {version}"
    return "\n".join(lines).strip()


def plan(env: dict[str, str]) -> dict[str, object]:
    tag = env.get("TAG", "").strip()
    every = bool(tag) or env.get("ALL", "false") == "true"
    on = {k: every or env.get(k.upper(), "false") == "true" for k in OPTIONAL}
    scope = "all" if all(on.values()) else ", ".join(k for k, v in on.items() if v) or "default"

    check_docs()
    text = (ROOT / "CMakeLists.txt").read_text()
    version = re.search(r"project\(kstone VERSION ([0-9.]+)", text).group(1)
    ros_version = re.search(r"<version>([0-9.]+)</version>", (ROOT / "package.xml").read_text()).group(1)
    if ros_version != version:
        fail(f"package.xml version {ros_version} differs from the CMakeLists.txt version {version}")
    if every and not changelog_section(version):
        fail(f"CHANGELOG.md has no non-empty '## {version}' section for the release notes")
    if tag:
        if tag != f"v{version}":
            fail(f"tag {tag} does not match the CMakeLists.txt version, expected v{version}")
        if env.get("CHECK_REMOTE", "true") == "true":
            found = subprocess.run(["git", "ls-remote", "--tags", "origin", f"refs/tags/{tag}"],
                                   check=True, capture_output=True, text=True).stdout
            if found.strip():
                fail(f"tag {tag} already exists on origin")

    archs = ARCHS if on["aarch64"] else ARCHS[:1]
    return dict(
        scope=scope,
        pip=[pip_job(*j) for j in PIP_JOBS if j[1] in archs],
        presets=[p for p in PRESETS if on[p]],
        preset_runner=runner(UBUNTU, "x86_64"),
        isolated=[dict(ubuntu=UBUNTU, arch=a, runner=runner(UBUNTU, a)) for a in archs] if on["isolated"] else [],
        ros=[dict(distro=d, arch=a, runner=runner(UBUNTU, a)) for d, a in ROS if a in archs],
        source=[dict(ubuntu=UBUNTU, arch="x86_64", runner=runner(UBUNTU, "x86_64"))] if on["source"] else [],
        tag=tag,
        version=version if tag else "",
    )


def summary(out: dict[str, object]) -> str:
    def names(key: str, fmt: str) -> str:
        return ", ".join(fmt.format(**e) for e in out[key]) or "no"

    lines = [f"### Plan ({out['scope']})", "", "| job | runs |", "|---|---|",
             "| format, sdist | yes |",
             f"| presets | {', '.join(out['presets']) or 'no'} ({out['preset_runner']}) |",
             f"| isolated | {names('isolated', 'u{ubuntu} {arch}')} |",
             f"| ros | {names('ros', '{distro} {arch}')} |",
             f"| source | {names('source', 'u{ubuntu} {arch}')} |",
             f"| release | {('draft ' + out['tag']) if out['tag'] else 'no'} |", "",
             "| pip: ubuntu | arch | python | pin | swap |", "|---|---|---|---|---|"]
    lines += [f"| {e['ubuntu']} | {e['arch']} | {e['python']} | {e['pin']} | {e['swap']} |" for e in out["pip"]]
    return "\n".join(lines) + "\n"


def main() -> None:
    if len(sys.argv) == 3 and sys.argv[1] == "source-script" and sys.argv[2] in SOURCE_PARTS:
        sys.stdout.write(source_script(sys.argv[2]))
        return
    if sys.argv[1:]:
        fail(f"unknown arguments {sys.argv[1:]}; use no arguments or source-script {'|'.join(SOURCE_PARTS)}")
    out = plan(dict(os.environ))
    text = summary(out)
    print(text)
    if "GITHUB_STEP_SUMMARY" in os.environ:
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as f:
            f.write(text)
    if "GITHUB_OUTPUT" in os.environ:
        with open(os.environ["GITHUB_OUTPUT"], "a") as f:
            for key, value in out.items():
                f.write(f"{key}={json.dumps(value) if not isinstance(value, str) else value}\n")


if __name__ == "__main__":
    main()
