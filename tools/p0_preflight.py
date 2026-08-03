#!/usr/bin/env python3
"""Run the board-free P0 acceptance gates.

Usage:
    python tools/p0_preflight.py
    python tools/p0_preflight.py --idf-build

The default path is deterministic and board-free.  ``--idf-build`` additionally
runs an ESP-IDF build using ``idf.py`` from PATH, or ``$IDF_PATH/tools/idf.py``.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Gate:
    name: str
    command: list[str]
    required: bool = True


def run_gate(gate: Gate) -> bool:
    print(f"\n== {gate.name} ==", flush=True)
    try:
        completed = subprocess.run(gate.command, cwd=ROOT, check=False)
    except OSError as error:
        print(f"[FAIL] {gate.name}: {error}")
        return False
    passed = completed.returncode == 0
    print(f"[{'PASS' if passed else 'FAIL'}] {gate.name}")
    return passed


def idf_command() -> list[str] | None:
    executable = shutil.which("idf.py")
    if executable:
        return [executable]
    idf_path = os.environ.get("IDF_PATH")
    if idf_path:
        script = Path(idf_path) / "tools" / "idf.py"
        if script.is_file():
            return [sys.executable, str(script)]
    return None


def git_command() -> str | None:
    executable = shutil.which("git")
    if executable:
        return executable
    if os.name == "nt":
        for candidate in (
            Path(r"F:\Git\cmd\git.exe"),
            Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
            / "Git"
            / "cmd"
            / "git.exe",
        ):
            if candidate.is_file():
                return str(candidate)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--idf-build",
        action="store_true",
        help="also run idf.py build (requires an activated ESP-IDF environment)",
    )
    parser.add_argument(
        "--build-dir",
        default="build",
        help="ESP-IDF build directory used with --idf-build (default: build)",
    )
    parser.add_argument(
        "--cc",
        help="native gcc/clang/tcc path for executable control-core tests",
    )
    args = parser.parse_args()

    git = git_command()
    if git is None:
        print("[FAIL] Git whitespace check: git executable not found")
        return 2

    gates = [
        Gate(
            "Python host tests",
            [
                sys.executable,
                "-m",
                "unittest",
                "discover",
                "-s",
                "tests",
                "-p",
                "test_*.py",
                "-v",
            ],
        ),
        Gate(
            "Native control-core tests",
            [
                sys.executable,
                "tools/run_p0_c_tests.py",
                *(["--cc", args.cc] if args.cc else []),
            ],
        ),
        Gate("Git whitespace check", [git, "diff", "--check"]),
    ]

    if args.idf_build:
        idf = idf_command()
        if idf is None:
            print("[FAIL] ESP-IDF build: idf.py not found; activate ESP-IDF first")
            return 2
        gates.append(Gate("ESP-IDF build", [*idf, "-B", args.build_dir, "build"]))

    results = [run_gate(gate) for gate in gates]
    print("\nP0 preflight: " + ("PASS" if all(results) else "FAIL"))
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
