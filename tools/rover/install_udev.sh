#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
#
# tools/rover/install_udev.sh -- 3.168 / 3.140's L0.3.
#
#   bash tools/rover/install_udev.sh RULES_FILE         # says what it would do
#   bash tools/rover/install_udev.sh RULES_FILE --yes   # does it (sudo)
#
# RULES_FILE comes from `python -m tools.rover.serial_route --rules-out FILE`,
# run on the Jetson with the Rover connected. Installs the rules and puts the
# user in dialout. Written, not installed, by 3.168.
set -eu

RULES="${1:-}"
DO="${2:-}"
DEST=/etc/udev/rules.d/99-picar-rover.rules

if [ -z "$RULES" ] || [ ! -f "$RULES" ]; then
  echo "usage: install_udev.sh RULES_FILE [--yes]" >&2
  exit 2
fi
# A rules file can run programs as root; these only ever name devices.
if grep -Eq '(RUN|PROGRAM|IMPORT)[+=:{]' "$RULES"; then
  echo "refused: $RULES has a RUN, PROGRAM or IMPORT key" >&2
  exit 3
fi
if ! grep -q 'SYMLINK+="picar-' "$RULES"; then
  echo "refused: $RULES names no picar device (did serial_route hear the board?)" >&2
  exit 3
fi

echo "would install to $DEST:"
cat "$RULES"
IN_DIALOUT=no
id -nG | tr ' ' '\n' | grep -qx dialout && IN_DIALOUT=yes
echo "user $(id -un) in dialout: $IN_DIALOUT"

if [ "$DO" != "--yes" ]; then
  echo "dry run: nothing changed (pass --yes to install)"
  exit 0
fi

sudo install -m 644 "$RULES" "$DEST"
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=tty
if [ "$IN_DIALOUT" = no ]; then
  sudo usermod -aG dialout "$(id -un)"
  echo "added to dialout: takes effect at the next login"
fi
ls -l /dev/picar-* || echo "no /dev/picar-* yet: check the rules against the report"
