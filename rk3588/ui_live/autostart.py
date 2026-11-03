#!/usr/bin/env python3
"""Detached dual-display service and scoped, recoverable board installation."""
import argparse
import hashlib
import ipaddress
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import re
import selectors
import signal
import socket
import stat
import subprocess
import sys
import threading
import time
import urllib.request

ROOT = Path("/userdata/rover_ui_live")
SCRIPT = ROOT / "autostart.py"
APP = ROOT / "server.py"
RUN = ROOT / "run.sh"
KEY = ROOT / "pairing.key"
LOG = ROOT / "service.log"
PID = Path("/var/run/rover-ui.pid")
LOCK = Path("/var/run/rover-ui.lock")
BOOT = Path("/etc/init.d/S99udp-vision")
OLD = Path("/userdata/udp_vision_20261019/autostart.py")
UPDATES = ("autostart.py", "S99udp-vision", "server.py", "app.js", "test_live.py", "README.md")
BASE_HASHES = {
    "server.py": "95c6eb7f3bcd2b0364577387af332a0202aace9714da4e9dc8e00732b4078307",
    "app.js": "5be71d2d4108fe0e780ea65b299e323f412f3927580f7e9f9dcd17e3bc49d656",
    "test_live.py": "3a6aee7847909dd5827d6f059c0c278b3ae313b9760d4e1c77e837f35e863132",
    "README.md": "cc211ff5c37f65d8420d283af66190b37fddcd736a08f816d0b08ff482de9ca2",
}
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def identity(pid):
    try:
        fields = Path("/proc/%d/stat" % pid).read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return fields[19], Path("/proc/%d/cmdline" % pid).read_bytes().split(b"\0")
    except (OSError, ValueError, IndexError):
        return None


def record():
    try:
        data = json.loads(PID.read_text())
        current = identity(int(data["pid"]))
        if (current and current[0] == data["ticks"] and os.fsencode(SCRIPT) in current[1]
                and b"run" in current[1]):
            return data
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def save_record(data):
    fd = os.open(PID, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(data, stream)


def select_wifi_address(output):
    for word in output.split():
        try:
            value = ipaddress.ip_interface(word)
            ip = value.ip
            if ip.version == 4 and ip.is_private and not (ip.is_loopback or ip.is_link_local or ip.is_unspecified):
                return str(ip)
        except ValueError:
            continue
    raise RuntimeError("waiting for wlan0 hotspot IPv4")


def wifi_address():
    result = subprocess.run(["ip", "-4", "-o", "addr", "show", "dev", "wlan0"],
                            capture_output=True, text=True, timeout=3, check=True)
    return select_wifi_address(result.stdout)


def configure_camera():
    # Preserve the previous boot supervisor's existing eth1 camera initialization.
    # No Wi-Fi settings or default routes are modified.
    device = Path("/sys/class/net/eth1")
    if not device.exists():
        raise RuntimeError("waiting for eth1")
    if not int((device / "flags").read_text().strip(), 16) & 1:
        subprocess.run(["ip", "link", "set", "dev", "eth1", "up"],
                       capture_output=True, timeout=3, check=True)
    result = subprocess.run(["ip", "-4", "addr", "show", "dev", "eth1"],
                            capture_output=True, text=True, timeout=3, check=True)
    if "192.168.50.1/24" not in result.stdout.split():
        subprocess.run(["ip", "addr", "add", "192.168.50.1/24", "dev", "eth1"],
                       capture_output=True, timeout=3, check=True)


def ready():
    for path in (RUN, APP, OLD, Path("/root/receiver.py")):
        if not path.is_file():
            raise RuntimeError("waiting for " + str(path))
    configure_camera()
    if not stat.S_ISSOCK(Path("/var/run/wayland-0").stat().st_mode):
        raise RuntimeError("waiting for Weston socket")
    address = wifi_address()
    # Never share the FPGA UDP stream or take an occupied HTTP port.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("0.0.0.0", 6102))
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind((address, 8080))
    return address


def stop_child(child):
    if child is None:
        return
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGKILL)
        child.wait(timeout=3)
    # The application owns a GStreamer descendant in the same private group.
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    if child.stdout:
        child.stdout.close()


def restart_delay(failures):
    return min(30, 2 ** min(max(1, failures), 5))


def run_service():
    import fcntl
    os.umask(0o077)
    lock_fd = os.open(LOCK, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    lock = os.fdopen(lock_fd, "r+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        return 0
    logger = logging.getLogger("rover-ui")
    logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(LOG, maxBytes=2 * 1024 * 1024, backupCount=1, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
    logger.addHandler(handler)
    stopping = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stopping.set())
    data = dict(pid=os.getpid(), ticks=identity(os.getpid())[0], phase="waiting", child_pid=None)
    child = None
    failures = 0
    previous_problem = None
    save_record(data)
    logger.info("SUPERVISOR_START")
    try:
        while not stopping.is_set():
            try:
                address = ready()
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                message = str(exc)
                if message != previous_problem:
                    logger.warning("WAITING %s", message)
                    previous_problem = message
                stopping.wait(2)
                continue
            previous_problem = None
            started = last_check = time.monotonic()
            child = subprocess.Popen(
                ["sh", str(RUN), "--bind", address, "--port", "8080",
                 "--pairing-file", str(KEY), "--supervised"],
                cwd=ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, start_new_session=True, close_fds=True,
                env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
            data.update(phase="running", child_pid=child.pid, address=address)
            save_record(data)
            logger.info("APPLICATION_START pid=%s url=http://%s:8080", child.pid, address)
            os.set_blocking(child.stdout.fileno(), False)
            pending = b""
            stride_reported = False
            with selectors.DefaultSelector() as selector:
                selector.register(child.stdout, selectors.EVENT_READ)
                while child.poll() is None and not stopping.is_set():
                    for key, _ in selector.select(.5):
                        chunk = os.read(key.fd, 65536)
                        if not chunk:
                            selector.unregister(key.fileobj)
                            continue
                        pending += chunk
                        lines = pending.split(b"\n")
                        pending = lines.pop()
                        for raw in lines:
                            line = raw.decode("utf-8", "replace").strip()
                            if "PAIRING CODE" in line:
                                continue
                            if "size_with_stride" in line:
                                if not stride_reported:
                                    logger.warning("Repeated RKNN stride advisory suppressed")
                                    stride_reported = True
                                continue
                            if line:
                                logger.info("%s", line[:4096])
                        if len(pending) > 65536:
                            pending = b""
                            logger.warning("Discarded oversized unterminated output")
                    if time.monotonic() - last_check > 10:
                        last_check = time.monotonic()
                        try:
                            configure_camera()
                            current_address = wifi_address()
                            if current_address != address:
                                logger.info("Hotspot IPv4 changed; restarting listener")
                                break
                        except (OSError, RuntimeError, subprocess.SubprocessError):
                            # Keep the board display alive during a temporary Wi-Fi loss.
                            pass
            code = child.poll()
            stop_child(child)
            child = None
            data.update(phase="waiting", child_pid=None)
            save_record(data)
            if stopping.is_set():
                break
            failures = 0 if time.monotonic() - started > 30 else failures + 1
            delay = restart_delay(failures)
            logger.warning("APPLICATION_EXIT code=%s retry_seconds=%s", code, delay)
            stopping.wait(delay)
        return 0
    finally:
        stop_child(child)
        current = record()
        if current and current["pid"] == os.getpid():
            PID.unlink()
        logger.info("SUPERVISOR_STOP")
        handler.close()
        logger.removeHandler(handler)
        lock.close()


def start_service():
    current = record()
    if current:
        print(json.dumps(dict(result="already_running", **current)))
        return 0
    child = subprocess.Popen([sys.executable, "-B", str(SCRIPT), "run"], cwd="/",
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True, close_fds=True)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        current = record()
        if current:
            print(json.dumps(dict(result="supervisor_started", **current)))
            return 0
        if child.poll() is not None:
            break
        time.sleep(.1)
    print("Supervisor did not start; inspect " + str(LOG), file=sys.stderr)
    return 1


def stop_service():
    current = record()
    if not current:
        print("ROVER service is stopped")
        return 0
    os.kill(current["pid"], signal.SIGTERM)
    deadline = time.monotonic() + 18
    while time.monotonic() < deadline:
        now = record()
        if not now or (now["pid"], now["ticks"]) != (current["pid"], current["ticks"]):
            print("ROVER service stopped")
            return 0
        time.sleep(.1)
    print("Service did not stop; no second instance started", file=sys.stderr)
    return 1


def status():
    current = record()
    if not current:
        print("ROVER service is stopped")
        return 3
    child = identity(current["child_pid"]) if current.get("child_pid") else None
    print(json.dumps(dict(**current, application_alive=bool(child and os.fsencode(APP) in child[1]))))
    return 0


def download(base):
    from urllib.parse import urlsplit
    parsed = urlsplit(base)
    ip = ipaddress.ip_address(parsed.hostname)
    if parsed.scheme != "http" or not ip.is_private or ip.is_unspecified or parsed.username or parsed.password or parsed.path not in ("", "/") or parsed.query or parsed.fragment:
        raise ValueError("expected private-PC HTTP origin")
    def get(path):
        with OPENER.open(base.rstrip("/") + path, timeout=10) as response:
            content = response.read(2 * 1024 * 1024 + 1)
        if len(content) > 2 * 1024 * 1024:
            raise ValueError("source too large")
        return content
    manifest = json.loads(get("/manifest.json"))
    bundle = {}
    for name in UPDATES:
        content = get("/files/" + name)
        if hashlib.sha256(content).hexdigest() != manifest.get(name):
            raise ValueError("source hash mismatch: " + name)
        if name.endswith(".py"):
            compile(content, name, "exec")
        bundle[name] = content
    subprocess.run(["sh", "-n"], input=bundle["S99udp-vision"], check=True)
    return bundle


def install_plan(bundle):
    """Validate every destination before backup, process changes or writes."""
    if set(bundle) != set(UPDATES):
        raise ValueError("unexpected installation files")
    if not ROOT.is_dir() or ROOT.is_symlink():
        raise ValueError("unexpected installation directory")
    if b"/root/gst-rawparse-1.22.2" not in RUN.read_bytes():
        raise ValueError("run.sh is missing the previously approved plugin-path fix")
    planned = []
    for name in UPDATES:
        target = BOOT if name == "S99udp-vision" else ROOT / name
        backup = ROOT / (name + ".before-autostart")
        if target.is_symlink() or backup.exists() or backup.is_symlink():
            raise ValueError("symlink or backup conflict: " + str(target))
        if name == "autostart.py":
            if target.exists():
                raise ValueError("new supervisor already exists; no overwrite")
            planned.append((target, bundle[name], None, None))
            continue
        old = target.read_bytes()
        if name in BASE_HASHES and hashlib.sha256(old).hexdigest() != BASE_HASHES[name]:
            raise ValueError("existing file changed; no overwrite: " + name)
        if name == "S99udp-vision":
            commands = [line.strip() for line in old.decode().splitlines()
                        if line.strip() and not line.lstrip().startswith("#")]
            expected = 'exec /usr/bin/python3 -B /userdata/udp_vision_20261019/autostart.py "${1:-start}"'
            if not commands or commands[-1] != expected or any(line not in ("set -e", "set -u", "set -eu") for line in commands[:-1]):
                raise ValueError("boot entry differs from expected old supervisor; no overwrite")
        planned.append((target, bundle[name], backup, old))
    return planned


def stop_foreground():
    """Stop only a verified instance of this exact UI script, never a broad pkill."""
    targets = []
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        pid = int(path.name)
        current = identity(pid)
        if current and os.fsencode(APP) in current[1]:
            targets.append((pid, current))
    for pid, expected in targets:
        if identity(pid) == expected:
            os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        if all(identity(pid) != expected for pid, expected in targets):
            return
        time.sleep(.2)
    raise RuntimeError("foreground UI has not stopped; refusing to compete for its devices")


def write_new(path, content):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())


def replace_content(path, content):
    # Existing paths retain their permissions (including the boot entry executable bit).
    with path.open("r+b") as stream:
        stream.write(content)
        stream.truncate()
        stream.flush()
        os.fsync(stream.fileno())


def install(base):
    bundle = download(base)
    planned = install_plan(bundle)
    subprocess.run(["sh", str(RUN), "--check"], check=True)
    for target, _, backup, old in planned:
        if backup:
            write_new(backup, old)
    # Backups are outside /etc/init.d, so boot will not execute the old entry twice.
    written = []
    switched = False
    try:
        if stop_service():
            raise RuntimeError("new supervisor did not stop")
        subprocess.run([sys.executable, "-B", str(OLD), "stop"], check=True)
        switched = True
        stop_foreground()
        for target, content, backup, old in planned:
            if old is None:
                write_new(target, content)
                written.append((target, old))
            else:
                written.append((target, old))
                replace_content(target, content)
        # Check the installed server before allowing the boot supervisor to start it.
        subprocess.run(["sh", str(RUN), "--check"], check=True)
        result = subprocess.run([sys.executable, "-B", str(SCRIPT), "start"])
        if result.returncode:
            raise RuntimeError("background supervisor failed to start")
        deadline = time.monotonic() + 25
        health = None
        while time.monotonic() < deadline:
            current = record()
            if current and current.get("address") and KEY.exists():
                # Import only the installed credential reader; never include the key in logs.
                sys.path.insert(0, str(ROOT))
                from server import pairing_token
                request = urllib.request.Request("http://%s:8080/api/state" % current["address"],
                                                 headers={"Authorization": "Bearer " + pairing_token(KEY)})
                try:
                    with OPENER.open(request, timeout=2) as response:
                        health = json.loads(response.read(1024 * 1024))
                    if health["vision"]["npu"] == "initialized" and not health["vision"].get("error"):
                        break
                except (OSError, ValueError, KeyError):
                    pass
            time.sleep(.5)
        else:
            raise RuntimeError("new service did not become ready within 25 seconds")
        print("INSTALL OK: background service and boot entry installed")
        print("URL: http://%s:8080" % current["address"])
        print("Live image: " + ("received" if health["vision"].get("connected") else "waiting; verify camera input"))
        print("PAIRING CODE (enter once in your browser; do not share): " + pairing_token(KEY))
        print("No reboot performed. Verify both displays before closing the terminal.")
        return 0
    except BaseException:
        # Do not restore code while a managed child could still have it open.
        if record() and stop_service():
            raise RuntimeError("rollback paused: new service would not stop; backups retained")
        for target, old in reversed(written):
            if old is None:
                # This exact file was created by this installation; no recursive removal.
                target.unlink(missing_ok=True)
            else:
                replace_content(target, old)
        if switched:
            subprocess.run([sys.executable, "-B", str(OLD), "start"], check=False)
        print("Installation failed; modified sources and boot entry restored. Backups retained.",
              file=sys.stderr)
        raise


def restore():
    pairs = []
    for name in (*BASE_HASHES, "S99udp-vision"):
        target = BOOT if name == "S99udp-vision" else ROOT / name
        backup = ROOT / (name + ".before-autostart")
        if target.is_symlink() or backup.is_symlink() or not backup.is_file():
            raise ValueError("missing or unsafe backup: " + str(backup))
        content = backup.read_bytes()
        if name in BASE_HASHES and hashlib.sha256(content).hexdigest() != BASE_HASHES[name]:
            raise ValueError("backup checksum differs: " + name)
        if name == "S99udp-vision" and os.fsencode(OLD) not in content:
            raise ValueError("unexpected boot backup")
        pairs.append((target, content))
    if stop_service():
        return 1
    stop_foreground()
    for target, content in pairs:
        replace_content(target, content)
    result = subprocess.run([sys.executable, "-B", str(OLD), "start"])
    print("Original boot entry and source files restored; backups and registered targets kept.")
    return result.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "restart", "status", "run", "pair", "install", "restore"))
    parser.add_argument("source", nargs="?")
    args = parser.parse_args()
    if os.name != "posix" or os.geteuid() != 0:
        raise SystemExit("Run service commands as root on the RK3588 board")
    if args.action == "install":
        if not args.source:
            parser.error("install requires the private-PC source URL")
        return install(args.source)
    if args.action == "restore":
        return restore()
    if args.action == "run":
        return run_service()
    if args.action == "status":
        return status()
    if args.action == "pair":
        if not KEY.exists():
            raise SystemExit("Pairing file not created yet; check service status")
        from server import pairing_token
        print(pairing_token(KEY))
        return 0
    if args.action in ("stop", "restart"):
        result = stop_service()
        if args.action == "stop" or result:
            return result
    return start_service()


if __name__ == "__main__":
    raise SystemExit(main())
