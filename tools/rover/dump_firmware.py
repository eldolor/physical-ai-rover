# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/rover/dump_firmware.py

3.168 / 3.140's L0.4, the first half only: keep the Rover's stock firmware
before anything replaces it. Reads the flash; flashing is not in this tool.

    python -m tools.rover.dump_firmware --check
    python -m tools.rover.dump_firmware --port /dev/ttyTHS1 [--size 0x400000]
        [--baud 115200] [--before default_reset] [--dir ~/rover-firmware]

Reads twice and keeps the image only if the two SHA-256 agree and the size is
what was asked. The image, its `.sha256` and a manifest go to `--dir`, outside
the repository; an existing image is never overwritten. On the 40-pin UART
there are no reset lines: hold BOOT, tap EN, then run with
`--before no_reset`. It asks you to do that again before the second read;
tap EN alone afterwards to start the firmware. The motors are not driven by
the ROM bootloader.
"""

import argparse
import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional

from tools.rover.serial_ro import holder_may_hide, holders

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIR = Path.home() / "rover-firmware"
DEFAULT_SIZE = 0x400000    # 4 MB: the ESP32-WROOM on the driver board
READ_TIMEOUT_S = 900.0


class DumpError(RuntimeError):
    pass


def find_esptool(explicit: Optional[str] = None) -> Optional[List[str]]:
    if explicit:
        return [explicit]
    for name in ("esptool.py", "esptool"):
        path = shutil.which(name)
        if path:
            return [path]
    if importlib.util.find_spec("esptool"):
        return [sys.executable, "-m", "esptool"]
    return None


def read_argv(tool: List[str], port: str, baud: int, size: int, out: Path, before: str) -> List[str]:
    return [*tool, "--chip", "esp32", "--port", port, "--baud", str(baud),
            "--before", before, "--after", "no_reset" if before == "no_reset" else "hard_reset",
            "read_flash", "0", hex(size), str(out)]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read(tool, port, baud, size, out: Path, before: str) -> str:
    argv = read_argv(tool, port, baud, size, out, before)
    r = subprocess.run(argv, capture_output=True, text=True, timeout=READ_TIMEOUT_S)
    if r.returncode != 0:
        raise DumpError(f"esptool exited {r.returncode}: {(r.stderr or r.stdout)[-400:]}")
    if not out.exists() or out.stat().st_size != size:
        got = out.stat().st_size if out.exists() else 0
        raise DumpError(f"short read: {got} bytes, asked for {size}")
    return _sha256(out)


def dump(port: str, directory: Path, size: int = DEFAULT_SIZE, baud: int = 115200,
         before: str = "default_reset", tool: Optional[List[str]] = None, name: Optional[str] = None,
         prompt: Callable[[str], object] = input) -> dict:
    tool = tool or find_esptool()
    if tool is None:
        raise DumpError("esptool not found: pip install esptool (nothing was read)")
    directory = Path(directory).expanduser().resolve()
    if directory == ROOT or ROOT in directory.parents:
        raise DumpError(f"{directory} is inside the repository: a vendor image stays out of it")
    held = holders(port)
    if held:
        raise DumpError(f"{port} is held by {held}: stop it first")
    if holder_may_hide(port):
        raise DumpError(f"{port}: not root, so a holder could be hidden: run under sudo")
    directory.mkdir(parents=True, exist_ok=True)
    stem = name or f"stock-{time.strftime('%Y%m%d')}"
    image = directory / f"{stem}.bin"
    if image.exists():
        raise DumpError(f"{image} already exists: never overwritten")
    first, second = directory / f"{stem}.read1", directory / f"{stem}.read2"
    try:
        sha_a = _read(tool, port, baud, size, first, before)
        if before == "no_reset":     # no reset lines: only a person can restart the bootloader
            prompt("Read 1 done. Hold BOOT, tap EN, release BOOT, then press Enter: ")
        sha_b = _read(tool, port, baud, size, second, before)
        if sha_a != sha_b:
            raise DumpError(f"the two reads differ: {sha_a} and {sha_b}; nothing kept")
        first.replace(image)
    finally:
        for leftover in (first, second):
            leftover.unlink(missing_ok=True)
    Path(f"{image}.sha256").write_text(f"{sha_a}  {image.name}\n")
    manifest = {"image": str(image), "sha256": sha_a, "bytes": size, "port": port, "baud": baud,
                "reads": 2, "at": time.time(), "tool": tool}
    Path(f"{image}.json").write_text(json.dumps(manifest, indent=1) + "\n")
    return manifest


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[3])
    ap.add_argument("--check", action="store_true", help="say whether esptool is here; touch nothing")
    ap.add_argument("--port")
    ap.add_argument("--size", type=lambda s: int(s, 0), default=DEFAULT_SIZE)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--before", choices=("default_reset", "no_reset"), default="default_reset")
    ap.add_argument("--dir", default=str(DEFAULT_DIR))
    ap.add_argument("--esptool")
    a = ap.parse_args(argv)
    tool = find_esptool(a.esptool)
    if a.check:
        print(json.dumps({"esptool": tool, "dir": a.dir}, indent=1))
        return 0 if tool else 1
    if not a.port:
        ap.error("--port is needed unless --check")
    try:
        print(json.dumps(dump(a.port, Path(a.dir), a.size, a.baud, a.before, tool), indent=1))
    except DumpError as e:
        print(f"refused: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
