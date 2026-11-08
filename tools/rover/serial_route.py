# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/rover/serial_route.py

3.168 / 3.140's L0.3: which serial port is what on the Rover, by listening.

    python -m tools.rover.serial_route [--seconds 3] [--path /dev/pts/N ...]
                                       [--out report.json] [--rules-out 99-picar-rover.rules]

`UGV-ROVER-MOUNTING.md` expects the motor board on the 40-pin header
(`/dev/ttyTHS1`) and the D500 on a USB port of the driver board, so only the
listening decides. A port is `board` if a `T:1001` line arrives at 115200 and
`lidar` if CRC-valid LD19 packets arrive at 230400. The report names who
holds each port (Waveshare's stock app would: L0.2).

Writes a udev rules FILE for what it found; `install_udev.sh` installs one.
Nothing here is installed, and every port is opened read-only.
"""

import argparse
import glob
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from robot.lidar_ld19 import Parser
from tools.rover.serial_ro import (BOARD_BAUD, LIDAR_BAUD, PortHeld, can_see_all, chunks,
                                   holders, lines, open_readonly)

PATTERNS = ("/dev/ttyTHS*", "/dev/ttyUSB*", "/dev/ttyACM*")
MIN_PACKETS = 3
NAMES = {"board": "picar-board", "lidar": "picar-lidar"}


def candidates(extra=()) -> List[str]:
    found = [p for pat in PATTERNS for p in sorted(glob.glob(pat))]
    return found + [p for p in extra if p not in found]


def _attr(node: str, name: str) -> str:
    p = Path(node, name)
    return p.read_text().strip() if p.exists() else ""


def usb_info(path: str, sys_root: str = "/sys/class/tty") -> Optional[Dict[str, str]]:
    """The USB device behind a tty, from sysfs; None for a UART or a pty."""
    name = os.path.basename(os.path.realpath(path))
    node = os.path.realpath(os.path.join(sys_root, name, "device"))
    interface = ""
    while node not in ("/", ""):
        vendor = os.path.join(node, "idVendor")
        if os.path.exists(vendor):
            return {"vendor": _attr(node, "idVendor"), "product": _attr(node, "idProduct"),
                    "serial": _attr(node, "serial"), "usb_path": os.path.basename(node),
                    "interface": interface}
        interface = interface or _attr(node, "bInterfaceNumber")
        node = os.path.dirname(node)
    return None


def _is_board_line(line: bytes) -> bool:
    try:
        obj = json.loads(line)
    except ValueError:
        return False
    return isinstance(obj, dict) and obj.get("T") == 1001


def listen(path: str, seconds: float) -> Dict:
    """Listen at the board's baud, then the lidar's; the role heard."""
    rec = {"role": "silent", "bytes": 0, "error": None}
    try:
        fd = open_readonly(path, BOARD_BAUD)
        try:
            frames = 0
            for _t, line in lines(fd, seconds):
                rec["bytes"] += len(line) + 1
                frames += _is_board_line(line)
            if frames:
                rec.update(role="board", frames=frames, baud=BOARD_BAUD)
                return rec
        finally:
            os.close(fd)
        fd = open_readonly(path, LIDAR_BAUD)
        try:
            parser = Parser()
            for _t, data in chunks(fd, seconds):
                rec["bytes"] += len(data)
                parser.feed(data)
            if parser.packets >= MIN_PACKETS:
                rec.update(role="lidar", packets=parser.packets, crc_failed=parser.crc_failed,
                           baud=LIDAR_BAUD)
            elif rec["bytes"]:
                rec["role"] = "unknown"
        finally:
            os.close(fd)
    except PortHeld as e:
        rec.update(role="held", error=str(e))     # never listened: see open_readonly
    except OSError as e:
        rec.update(role="unreadable", error=str(e))
    return rec


def scan(seconds: float, extra=()) -> Dict:
    ports = []
    for path in candidates(extra):
        ports.append({"path": path, "usb": usb_info(path), "held_by": holders(path),
                      **listen(path, seconds)})
    warnings = [] if can_see_all() else [
        "not root: a port may be held by a process this user cannot see (run under sudo for L0.2)"]
    return {"ports": ports, "seen_all_processes": can_see_all(), "problems": problems(ports),
            "warnings": warnings}


def problems(ports: List[Dict]) -> List[str]:
    out = []
    for role in NAMES:
        found = [p["path"] for p in ports if p["role"] == role]
        if not found:
            out.append(f"no {role} heard on any port")
        elif len(found) > 1:
            out.append(f"{role} heard on several ports: {found}")
    for p in ports:
        if p["held_by"]:
            out.append(f"{p['path']} is held by {p['held_by']} (L0.2: stop it first)")
    return out


def _identity(usb: Dict) -> tuple:
    return usb["vendor"], usb["product"], usb["serial"]


def rule_for(port: Dict, others: Sequence[Dict] = ()) -> Optional[str]:
    """One udev line for a port whose role was heard; None for a pty. `others`
    are the other ports: two bridges with one identity differ only by USB port."""
    name = NAMES.get(port["role"])
    if name is None:
        return None
    usb = port["usb"]
    tail = f'SYMLINK+="{name}", GROUP="dialout", MODE="0660"'
    if usb is None:
        if not os.path.basename(port["path"]).startswith("ttyTHS"):
            return None                 # a pty has no identity udev could match
        return f'SUBSYSTEM=="tty", KERNEL=="{os.path.basename(port["path"])}", {tail}'
    match = f'ATTRS{{idVendor}}=="{usb["vendor"]}", ATTRS{{idProduct}}=="{usb["product"]}"'
    twin = any(o["usb"] and o is not port and _identity(o["usb"]) == _identity(usb) for o in others)
    match += (f', KERNELS=="{usb["usb_path"]}"' if twin or not usb["serial"]
              else f', ATTRS{{serial}}=="{usb["serial"]}"')
    if twin and usb.get("interface"):        # two channels of one bridge share a USB path
        match += f', ATTRS{{bInterfaceNumber}}=="{usb["interface"]}"'
    return f'SUBSYSTEM=="tty", {match}, {tail}'


def rules_text(report: Dict) -> str:
    lines_ = ["# 3.168: generated by tools/rover/serial_route.py; install with install_udev.sh"]
    for port in report["ports"]:
        if sum(p["role"] == port["role"] for p in report["ports"]) > 1:
            lines_.append(f"# {port['path']}: {port['role']} heard on several ports: no rule")
            continue
        rule = rule_for(port, report["ports"])
        if rule:
            lines_.append(f"# {port['path']}: {port['role']}")
            lines_.append(rule)
    return "\n".join(lines_) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[3])
    # Long enough for a board that opening the port has just reset.
    ap.add_argument("--seconds", type=float, default=8.0, help="listening time at each baud")
    ap.add_argument("--path", action="append", default=[], help="another port (a pty, in a dry run)")
    ap.add_argument("--out")
    ap.add_argument("--rules-out")
    a = ap.parse_args(argv)
    report = scan(a.seconds, a.path)
    text = json.dumps(report, indent=1)
    print(text)
    for warning in report["warnings"]:
        print(f"WARNING: {warning}", file=sys.stderr)
    if a.out:
        Path(a.out).write_text(text + "\n")
    if a.rules_out:
        Path(a.rules_out).write_text(rules_text(report))
    return 0 if not report["problems"] else 1


if __name__ == "__main__":
    sys.exit(main())
