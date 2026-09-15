"""Use the optional local Ubuntu Tk runtime without changing Conda or ESP-IDF."""
from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


def activate_local_tk() -> None:
    if sys.platform != "linux":
        return
    runtime = Path(__file__).resolve().parents[1] / ".gui-runtime"
    if not runtime.is_dir() or os.environ.get("ESP32_GUI_RUNTIME") == str(runtime):
        return
    python = Path("/usr/bin/python3")
    if not python.is_file():
        return
    env = os.environ.copy()
    env.pop("PYTHONHOME", None)
    env.pop("PYTHONPATH", None)
    version = subprocess.check_output(
        [str(python), "-c", "import sys; print('%d.%d' % sys.version_info[:2])"],
        env=env, text=True,
    ).strip()
    library = runtime / "usr" / "lib" / ("python" + version)
    if not (library / "tkinter").is_dir():
        return
    env.update({
        "PYTHONPATH": os.pathsep.join([str(library), str(library / "lib-dynload")]),
        "LD_LIBRARY_PATH": os.pathsep.join([str(runtime / "usr/lib/x86_64-linux-gnu"), str(runtime / "usr/lib")]),
        "TCL_LIBRARY": str(runtime / "usr/share/tcltk/tcl8.6"),
        "TK_LIBRARY": str(runtime / "usr/share/tcltk/tk8.6"),
        "ESP32_GUI_RUNTIME": str(runtime),
    })
    os.execve(str(python), [str(python), *sys.argv], env)
