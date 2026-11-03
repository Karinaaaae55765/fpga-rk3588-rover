#!/usr/bin/env python3
"""Dedicated-link FUDP receiver. Standard-library only; GStreamer is a child.

No /dev/video40 or RKISP dependency. Never displays uncommitted/incomplete frames.
"""
import argparse
import array
import collections
import json
import os
import queue
import select
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time

HEADER = struct.Struct('!4sBBHIHHHHHHII')
WIDTH, HEIGHT, CHUNK = 1280, 720, 1280
FRAME_BYTES = WIDTH * HEIGHT * 2
PACKETS = FRAME_BYTES // CHUNK


def newer(a, b):
    return 0 < ((a - b) & 0xffffffff) < 0x80000000


class Assembler:
    def __init__(self, width=WIDTH, height=HEIGHT, chunk=CHUNK, timeout=0.30):
        self.width, self.height, self.chunk = width, height, chunk
        self.size = width * height * 2
        if self.size % chunk:
            raise ValueError('frame must contain an integral number of chunks')
        self.count = self.size // chunk
        self.timeout = timeout
        self.frames = collections.OrderedDict()
        self.closed = collections.deque(maxlen=32)
        self.latest = self.output_id = None
        self.last_packet = None
        self.stats = collections.Counter()

    def expire(self, now):
        for fid, frame in list(self.frames.items()):
            if now - frame['born'] > self.timeout:
                self.stats['incomplete_frames'] += 1
                self.stats['missing_packets'] += self.count - frame['received']
                self.closed.append(fid)
                del self.frames[fid]

    def feed(self, packet, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        if len(packet) < HEADER.size:
            self.stats['invalid_packets'] += 1
            return None
        magic, version, fmt, flags, fid, index, count, width, height, length, hlen, total, reserved = HEADER.unpack_from(packet)
        end = flags == 1
        if (magic != b'FUDP' or version != 1 or fmt != 1 or flags not in (0, 1)
                or (width, height, count, hlen, total, reserved) !=
                (self.width, self.height, self.count, 32, self.size, 0)
                or len(packet) != 32 + length
                or (end and (length != 0 or index != self.count))
                or (not end and (length != self.chunk or index >= self.count))):
            self.stats['invalid_packets'] += 1
            return None
        # Permit a rebooted FPGA's sequence to restart after a real input gap.
        if self.last_packet is not None and now - self.last_packet > 0.5:
            self.frames.clear()
            self.closed.clear()
            self.latest = self.output_id = None
            self.stats['input_restarts'] += 1
        self.last_packet = now
        self.stats['packets'] += 1
        self.stats['bytes'] += len(packet)
        if fid in self.closed:
            self.stats['late_packets'] += 1
            return None
        if fid not in self.frames:
            if self.latest is not None and not newer(fid, self.latest):
                self.stats['late_packets'] += 1
                return None
            self.latest = fid
            while len(self.frames) >= 3:
                old, frame = self.frames.popitem(last=False)
                self.closed.append(old)
                self.stats['incomplete_frames'] += 1
                self.stats['missing_packets'] += self.count - frame['received']
            self.frames[fid] = dict(data=bytearray(self.size), seen=bytearray(self.count),
                                    received=0, committed=False, born=now)
        frame = self.frames[fid]
        if end:
            frame['committed'] = True
        else:
            pos = index * self.chunk
            payload = packet[32:]
            if frame['seen'][index]:
                self.stats['duplicate_packets'] += 1
                if frame['data'][pos:pos + self.chunk] != payload:
                    del self.frames[fid]
                    self.closed.append(fid)
                    self.stats['conflicting_frames'] += 1
                    return None
            else:
                frame['data'][pos:pos + self.chunk] = payload
                frame['seen'][index] = 1
                frame['received'] += 1
        if frame['committed'] and frame['received'] == self.count:
            del self.frames[fid]
            self.closed.append(fid)
            if self.output_id is not None and not newer(fid, self.output_id):
                self.stats['late_frames'] += 1
                return None
            self.output_id = fid
            self.stats['frames_ok'] += 1
            return bytes(frame['data'])
        return None


def native_rgb565(frame):
    if sys.byteorder == 'big':
        return frame
    words = array.array('H')
    words.frombytes(frame)
    words.byteswap()
    return words.tobytes()


class Display:
    def __init__(self):
        for executable in ('gst-launch-1.0', 'gst-inspect-1.0'):
            if not shutil.which(executable):
                raise RuntimeError('missing executable: ' + executable)
        for element in ('fdsrc', 'rawvideoparse', 'videoconvert', 'queue', 'waylandsink'):
            if subprocess.call(['gst-inspect-1.0', element], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL):
                raise RuntimeError('missing GStreamer element: ' + element)
        command = ['gst-launch-1.0', '-q', 'fdsrc', 'fd=0', '!', 'rawvideoparse',
                   'format=rgb16', 'width=1280', 'height=720', 'framerate=15/1',
                   '!', 'videoconvert', '!', 'queue', 'max-size-buffers=2',
                   'max-size-bytes=0', 'max-size-time=0', 'leaky=downstream',
                   '!', 'waylandsink', 'fullscreen=true', 'sync=false']
        self.child = subprocess.Popen(command, stdin=subprocess.PIPE)
        os.set_blocking(self.child.stdin.fileno(), False)
        self.pending = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.error = None
        self.replaced = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def submit(self, frame):
        try:
            self.pending.put_nowait(frame)
        except queue.Full:
            try:
                self.pending.get_nowait()
                self.replaced += 1
            except queue.Empty:
                pass
            try:
                self.pending.put_nowait(frame)
            except queue.Full:
                self.replaced += 1

    def run(self):
        try:
            while not self.stop.is_set():
                try:
                    frame = self.pending.get(timeout=0.1)
                except queue.Empty:
                    continue
                view = memoryview(native_rgb565(frame))
                deadline = time.monotonic() + 2.0
                while view and not self.stop.is_set():
                    if self.child.poll() is not None:
                        raise RuntimeError('GStreamer exited with code %s' % self.child.returncode)
                    if time.monotonic() > deadline:
                        raise RuntimeError('display pipe stalled for 2 s; network reception is separate')
                    fd = self.child.stdin.fileno()
                    if select.select([], [fd], [], 0.1)[1]:
                        try:
                            view = view[os.write(fd, view):]
                        except BlockingIOError:
                            pass
        except Exception as exc:
            self.error = str(exc)
            self.stop.set()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        if self.child.poll() is None:
            self.child.terminate()
            try:
                self.child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.wait()
        self.child.stdin.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=6102)
    parser.add_argument('--source', default='192.168.50.2')
    parser.add_argument('--no-display', action='store_true')
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--frames', type=int, default=0)
    args = parser.parse_args(argv)
    assembler = Assembler()
    display = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        sock.bind((args.bind, args.port))
        sock.settimeout(0.1)
        if not args.no_display:
            display = Display()
            display.submit(bytes(FRAME_BYTES))
        start = report = time.monotonic()
        last_frame = None
        black = True
        previous_frames = 0
        print(json.dumps(dict(event='listening', port=args.port, source=args.source,
                              socket_buffer=sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))), flush=True)
        while not args.seconds or time.monotonic() - start < args.seconds:
            now = time.monotonic()
            if display and (display.error or display.child.poll() is not None):
                raise RuntimeError(display.error or 'GStreamer display exited')
            try:
                packet, source = sock.recvfrom(2048)
                if source[0] != args.source:
                    assembler.stats['foreign_packets'] += 1
                    continue
                frame = assembler.feed(packet, now)
                if frame is not None:
                    last_frame = now
                    black = False
                    if display:
                        display.submit(frame)
                    if args.frames and assembler.stats['frames_ok'] >= args.frames:
                        break
            except socket.timeout:
                assembler.expire(now)
            if last_frame is not None and now - last_frame > 1 and not black:
                black = True
                if display:
                    display.submit(bytes(FRAME_BYTES))
            if now - report >= 1:
                frames = assembler.stats['frames_ok']
                print(json.dumps(dict(event='stats', **dict(assembler.stats),
                      fps=round((frames-previous_frames)/(now-report), 2),
                      waiting=last_frame is None or now-last_frame > 1,
                      display_replaced=display.replaced if display else 0)), flush=True)
                previous_frames, report = frames, now
        print(json.dumps(dict(event='summary', **dict(assembler.stats))), flush=True)
        return 0 if assembler.stats['frames_ok'] else 2
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError) as exc:
        print('ERROR: ' + str(exc), file=sys.stderr)
        return 1
    finally:
        sock.close()
        if display:
            display.close()


if __name__ == '__main__':
    sys.exit(main())
