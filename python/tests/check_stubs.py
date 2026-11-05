"""Run mypy stubtest on kstone; exit 77 (skip) when mypy is missing."""
import importlib.util
import pathlib
import subprocess
import sys

if importlib.util.find_spec("mypy") is None:
    print("mypy not installed; skipping")
    sys.exit(77)

allow = pathlib.Path(__file__).with_name("stubtest-allowlist.txt")
cmd = [sys.executable, "-m", "mypy.stubtest", "kstone", "--allowlist", str(allow)]
sys.exit(subprocess.call(cmd))
