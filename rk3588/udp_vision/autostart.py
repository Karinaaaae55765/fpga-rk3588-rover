#!/usr/bin/env python3
"""Supervise the existing UDP recognition application on the RK3588 board.

Only eth1 gets the dedicated camera address. Wi-Fi and default routes are
untouched. The application receives camera data and never commands motors.
"""
import argparse
import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import selectors
import signal
import socket
import stat
import subprocess
import sys
import time

ROOT = Path('/userdata/udp_vision_20261019')
PIDFILE = Path('/var/run/udp-vision.pid')
LOCKFILE = Path('/var/run/udp-vision.lock')
SCRIPT = ROOT / 'autostart.py'
LAUNCHER = ROOT / 'start_vision.sh'
APP = ROOT / 'rk3588/udp_vision/main.py'
LOGFILE = ROOT / 'autostart.log'
ADDRESS = '192.168.50.1/24'


def identity(pid):
    """PID plus kernel start time prevents signalling a reused PID."""
    try:
        data = Path('/proc/%d/stat' % pid).read_text()
        fields = data[data.rfind(')') + 2:].split()
        if fields[0] == 'Z':
            return None
        args = Path('/proc/%d/cmdline' % pid).read_bytes().split(b'\0')
        return fields[19], args
    except (OSError, ValueError, IndexError):
        return None


def active_record():
    try:
        record = json.loads(PIDFILE.read_text())
        current = identity(int(record['pid']))
        if (current and current[0] == record['start_ticks']
                and os.fsencode(SCRIPT) in current[1] and b'run' in current[1]):
            return record
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def save_record(record):
    # A single small write; readers tolerate the brief truncate/write interval.
    fd = os.open(PIDFILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        os.write(fd, (json.dumps(record) + '\n').encode())
    finally:
        os.close(fd)


def configure_network():
    device = Path('/sys/class/net/eth1')
    if not device.exists():
        raise RuntimeError('waiting for camera interface eth1')
    if not int((device / 'flags').read_text().strip(), 16) & 1:
        subprocess.run(['ip', 'link', 'set', 'dev', 'eth1', 'up'],
                       check=True, capture_output=True, timeout=3)
    output = subprocess.run(['ip', '-4', 'addr', 'show', 'dev', 'eth1'],
                            check=True, capture_output=True, text=True, timeout=3).stdout
    if ADDRESS not in output.split():
        subprocess.run(['ip', 'addr', 'add', ADDRESS, 'dev', 'eth1'],
                       check=True, capture_output=True, timeout=3)


def ready():
    for path in [LAUNCHER, APP, Path('/root/receiver.py'),
                 Path('/userdata/rk3588-app/models/yolov5s_relu_rk3588.rknn')]:
        if not path.is_file():
            raise RuntimeError('waiting for ' + str(path))
    configure_network()
    path = Path('/var/run/wayland-0')
    if not path.exists() or not stat.S_ISSOCK(path.stat().st_mode):
        raise RuntimeError('waiting for Weston display socket')


def stop_child(child):
    if child is None:
        return
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    # Also remove any display process left in this child's private group.
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=2)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGKILL)
        child.wait(timeout=2)
    if child.stdout:
        child.stdout.close()


def run_service():
    lock = LOCKFILE.open('a+b')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        return 0
    logger = logging.getLogger('udp-vision')
    logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(LOGFILE, maxBytes=2 * 1024 * 1024,
                                 backupCount=1, encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    logger.addHandler(handler)
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    record = dict(pid=os.getpid(), start_ticks=identity(os.getpid())[0],
                  phase='waiting', child_pid=None)
    save_record(record)
    logger.info('SUPERVISOR_START pid=%s', os.getpid())
    child = None
    failure_count = 0
    last_problem = None
    warning_reported = False
    try:
        while not stopping:
            try:
                ready()
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    probe.bind(('0.0.0.0', 6102))
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                problem = str(exc)
                if problem != last_problem:
                    logger.warning('WAITING %s', problem)
                    last_problem = problem
                until = time.monotonic() + 2
                while not stopping and time.monotonic() < until:
                    time.sleep(.1)
                continue
            last_problem = None
            started = time.monotonic()
            child = subprocess.Popen(['sh', str(LAUNCHER)], cwd=ROOT,
                                     stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, start_new_session=True,
                                     env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
            record.update(phase='running', child_pid=child.pid)
            save_record(record)
            logger.info('APPLICATION_START pid=%s', child.pid)
            os.set_blocking(child.stdout.fileno(), False)
            pending = b''
            last_network_check = time.monotonic()
            with selectors.DefaultSelector() as selector:
                selector.register(child.stdout, selectors.EVENT_READ)
                while not stopping and child.poll() is None:
                    for key, _mask in selector.select(.5):
                        data = os.read(key.fd, 65536)
                        if not data:
                            selector.unregister(key.fileobj)
                            continue
                        pending += data
                        lines = pending.split(b'\n')
                        pending = lines.pop()
                        for raw in lines:
                            line = raw.decode('utf-8', 'replace').rstrip()
                            # The vendor runtime repeats this advisory every frame
                            # and can interleave it with the JSON event stream.
                            if 'size_with_stride' in line:
                                if not warning_reported:
                                    logger.warning('RKNN stride advisory (duplicates suppressed)')
                                    warning_reported = True
                                event_start = line.find('{"event":')
                                if event_start < 0:
                                    continue
                                line = line[event_start:]
                            if line:
                                logger.info('%s', line)
                        if len(pending) > 65536:
                            logger.warning('truncated oversized application log line')
                            pending = b''
                    if time.monotonic() - last_network_check >= 10:
                        try:
                            configure_network()
                        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                            logger.warning('NETWORK_WAIT %s', exc)
                        last_network_check = time.monotonic()
            exit_code = child.poll()
            stop_child(child)
            child = None
            record.update(phase='waiting', child_pid=None)
            save_record(record)
            if stopping:
                break
            failure_count = 0 if time.monotonic() - started >= 30 else failure_count + 1
            delay = min(30, 2 ** min(failure_count, 5))
            logger.warning('APPLICATION_EXIT code=%s retry_seconds=%s', exit_code, delay)
            until = time.monotonic() + delay
            while not stopping and time.monotonic() < until:
                time.sleep(.1)
        return 0
    finally:
        stop_child(child)
        logger.info('SUPERVISOR_STOP pid=%s', os.getpid())
        if active_record() and active_record()['pid'] == os.getpid():
            PIDFILE.unlink(missing_ok=True)
        handler.close()
        logger.removeHandler(handler)
        lock.close()


def start_service():
    current = active_record()
    if current:
        print(json.dumps(dict(result='already_running', **current)))
        return 0
    child = subprocess.Popen([sys.executable, '-B', str(SCRIPT), 'run'],
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True,
                             cwd='/', close_fds=True)
    until = time.monotonic() + 4
    while time.monotonic() < until:
        current = active_record()
        if current:
            print(json.dumps(dict(result='started', **current)))
            return 0
        if child.poll() is not None and child.returncode != 0:
            break
        time.sleep(.1)
    print('Could not start UDP vision supervisor; inspect ' + str(LOGFILE), file=sys.stderr)
    return 1


def stop_service():
    current = active_record()
    if not current:
        print('UDP vision service is stopped')
        return 0
    os.kill(current['pid'], signal.SIGTERM)
    until = time.monotonic() + 14
    while time.monotonic() < until:
        now = active_record()
        if not now or (now['pid'], now['start_ticks']) != (current['pid'], current['start_ticks']):
            print('UDP vision service stopped')
            return 0
        time.sleep(.1)
    print('Service did not stop within 14 seconds', file=sys.stderr)
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'restart', 'status', 'run'])
    action = parser.parse_args().action
    if action == 'run':
        return run_service()
    if action == 'status':
        current = active_record()
        if current:
            child = identity(current['child_pid']) if current.get('child_pid') else None
            print(json.dumps(dict(**current, application_alive=bool(child and os.fsencode(APP) in child[1]))))
            return 0
        print('UDP vision service is stopped')
        return 3
    if action in ['stop', 'restart']:
        result = stop_service()
        if action == 'stop' or result:
            return result
    return start_service()


if __name__ == '__main__':
    raise SystemExit(main())
