#!/usr/bin/env python3
"""Compile and execute the P0 control core with a native C compiler."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components" / "robot_controller"


def find_compiler(explicit: str | None) -> str | None:
    for candidate in (explicit, os.environ.get("P0_CC"), os.environ.get("CC"),
                      "gcc", "clang", "cc", "tcc"):
        if not candidate:
            continue
        resolved = shutil.which(candidate)
        if resolved:
            return resolved
        if Path(candidate).is_file():
            return str(Path(candidate).resolve())
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="native gcc/clang/tcc path (or set P0_CC/CC)")
    args = parser.parse_args()
    compiler = find_compiler(args.cc)
    if compiler is None:
        print("No native C compiler found. Install gcc/clang/tcc or set P0_CC.", file=sys.stderr)
        return 2
    if Path(compiler).name.lower() in {"cl", "cl.exe"}:
        print("MSVC is not supported; use gcc, clang, or tcc.", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix="esp32-p0-host-") as temp:
        executable = Path(temp) / ("p0-control-test.exe" if os.name == "nt" else "p0-control-test")
        command = [compiler, "-std=c11", "-Wall",
                   "-I", str(ROOT / "tests" / "host"),
                   "-I", str(COMPONENT),
                   str(ROOT / "tests" / "host" / "test_control_core.c"),
                   str(COMPONENT / "robot_state_machine.c"),
                   str(COMPONENT / "control_manager.c"),
                   "-o", str(executable)]
        print("Compiling control core with", compiler, flush=True)
        compiled = subprocess.run(command, cwd=ROOT, check=False)
        if compiled.returncode:
            return compiled.returncode
        return subprocess.run([str(executable)], cwd=ROOT, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
