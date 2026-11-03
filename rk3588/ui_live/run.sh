#!/bin/sh
# Foreground service. Motion requires an explicit start and verified calibration.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export PYTHONDONTWRITEBYTECODE=1
export GST_PLUGIN_PATH="/root/gst-rawparse-1.22.2${GST_PLUGIN_PATH:+:$GST_PLUGIN_PATH}"
if [ -d /userdata/rknn-python ]; then
    export PYTHONPATH="/userdata/rknn-python${PYTHONPATH:+:$PYTHONPATH}"
fi
if [ -S /var/run/wayland-0 ]; then
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/var/run}"
    export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
fi
exec python3 -B "$root/server.py" "$@"
