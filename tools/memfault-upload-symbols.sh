#!/usr/bin/env sh
# Upload a build's symbol file to Memfault. Run this for EVERY build whose
# heartbeats you expect to see.
#
# Why this is not optional: Memfault heartbeat metrics travel as a positional
# CBOR array with no names in it, e.g.
#
#   4: {2: 0, 1: [60000, ..., 4020, 4, None, 1, None, 30500]}
#
# The names for those slots live only in the firmware's symbol file, keyed by
# the GNU build ID that the event also carries. With no symbol file for that
# build ID, the chunks endpoint still answers 202 and then discards the event.
# Nothing reports an error: the device just looks stale, its last_seen frozen at
# the last reboot, because reboot events are self-describing and survive while
# every heartbeat is dropped. That failure cost a long debugging session on
# 2026-08-27; do not rediscover it.
#
# Usage:
#   MEMFAULT_ORG_TOKEN=oat_... tools/memfault-upload-symbols.sh [build_dir]
#
# Defaults to the app image of the given build directory, or ./build.
set -eu

BUILD_DIR="${1:-build}"
ELF="${MEMFAULT_ELF:-$BUILD_DIR/app/zephyr/zephyr.elf}"
ORG="${MEMFAULT_ORG_SLUG:-nordic-semiconductorhwyp}"
PROJECT="${MEMFAULT_PROJECT_SLUG:-test}"

if [ -z "${MEMFAULT_ORG_TOKEN:-}" ]; then
	echo "MEMFAULT_ORG_TOKEN is not set (create one under Organization > Auth Tokens)" >&2
	exit 1
fi

if [ ! -f "$ELF" ]; then
	echo "No ELF at $ELF. Pass a build directory or set MEMFAULT_ELF." >&2
	exit 1
fi

if ! command -v memfault >/dev/null 2>&1; then
	echo "The memfault CLI is missing: pip install memfault-cli" >&2
	exit 1
fi

echo "Uploading $ELF to $ORG/$PROJECT"
memfault --org-token "$MEMFAULT_ORG_TOKEN" --org "$ORG" --project "$PROJECT" \
	upload-symbols "$ELF"

# Print the build ID so it can be matched against the id in a received event.
if command -v arm-zephyr-eabi-readelf >/dev/null 2>&1; then
	echo "build id:"
	arm-zephyr-eabi-readelf -x .note.gnu.build-id "$ELF" 2>/dev/null | tail -3
fi
