#!/bin/sh
# Uses the current environment; no network/system edits or package installation.
set -eu
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
command -v python3 >/dev/null || { echo 'python3 is required' >&2; exit 1; }
exec python3 -B "$script_dir/main.py" "$@"
