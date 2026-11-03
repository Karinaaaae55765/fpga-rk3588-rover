"""Live browser service or restricted, read-only wireless source transfer."""
import argparse
import hashlib
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import stat
import threading
import sys
import time
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
FILES = ("index.html", "styles.css", "app.js", "server.py", "pipeline.py",
         "telemetry_v3.py", "sensor_overlay.py", "run.sh", "test_live.py", "README.md", "autostart.py", "S99udp-vision", "task5_bridge.py", "motion_bridge.py", "pose_estimate.py")
STATIC = {"/": "index.html", "/index.html": "index.html", "/styles.css": "styles.css", "/app.js": "app.js"}


def pairing_token(path=None):
    """Persist only when explicitly requested; never expose the file over HTTP."""
    if path is None:
        return secrets.token_urlsafe(24)
    path = Path(path)
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    try:
        fd = os.open(path, flags)
    except FileNotFoundError:
        value = secrets.token_urlsafe(24)
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            return pairing_token(path)
        with os.fdopen(fd, "wb") as stream:
            stream.write((value + "\n").encode("ascii"))
            stream.flush()
            os.fsync(stream.fileno())
        return value
    with os.fdopen(fd, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise ValueError("pairing file is not regular")
        if os.name == "posix" and (info.st_uid != os.geteuid() or info.st_mode & 0o077):
            raise ValueError("pairing file must be owner-only (0600)")
        value = stream.read(256).decode("ascii").strip()
        if not re.fullmatch(r"[A-Za-z0-9_-]{32}", value):
            raise ValueError("invalid pairing file; refusing to replace it")
        return value


def make_handler(args, pipeline=None, telemetry=None, token=None):
    # Package content is captured once; a transfer has an immutable hash manifest.
    package = {name: (ROOT/name).read_bytes() for name in FILES} if args.serve_package else {}
    manifest = {name: hashlib.sha256(data).hexdigest() for name, data in package.items()}

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(4)

        def log_message(self, fmt, *values):
            # Do not log credentials or request URLs.
            pass

        def permitted(self):
            host = self.headers.get("Host", "")
            expected = f"{args.bind}:{self.server.server_port}"
            return (host == expected and
                    (not args.serve_package or self.client_address[0] == args.client))

        def authenticated(self):
            provided = self.headers.get("Authorization", "")
            return bool(token) and hmac.compare_digest(provided.encode("utf-8"), ("Bearer " + token).encode("utf-8"))

        def respond(self, status, body=b"", kind="application/json", extra=None):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header("Content-Security-Policy",
                             "default-src 'self'; script-src 'self'; style-src 'self'; "
                             "img-src 'self' blob: data:; connect-src 'self'; "
                             "object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
            for key, value in (extra or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def do_GET(self):
            if not self.permitted():
                return self.respond(403, {"error": "host_or_client_rejected"})
            path = urlsplit(self.path).path
            if args.serve_package:
                if path == "/manifest.json":
                    return self.respond(200, manifest)
                if path.startswith("/files/") and path[7:] in package:
                    return self.respond(200, package[path[7:]], "application/octet-stream")
                return self.respond(404, {"error": "not_found"})
            if path in STATIC:
                name = STATIC[path]
                mime = "text/html" if name.endswith(".html") else "text/css" if name.endswith(".css") else "text/javascript"
                return self.respond(200, (ROOT/name).read_bytes(), mime+"; charset=utf-8")
            if not self.authenticated():
                return self.respond(401, {"error": "pairing_required"})
            if path == "/api/state":
                status = pipeline.snapshot()
                status["telemetry"] = telemetry.snapshot()
                status["audio_playback_requested"] = getattr(args,"voice","off")=="play"
                status["hardware_write"] = status["audio_playback_requested"]
                status["motor_write"] = status["telemetry"].get("motion", {}).get("motor_write", False)
                status["hardware_write"] = status["hardware_write"] or status["motor_write"]
                return self.respond(200, status)
            if path == "/api/frame":
                result = pipeline.image()
                if result is None:
                    return self.respond(503, {"error": "no_fresh_frame"})
                jpeg, ttl = result
                if ttl <= 0:
                    return self.respond(503, {"error": "frame_expired"})
                return self.respond(200, jpeg, "image/jpeg", {"X-Frame-TTL-Ms": int(ttl*1000)})
            return self.respond(404, {"error": "not_found"})

        def do_POST(self):
            if args.serve_package:
                return self.respond(405, {"error": "read_only"})
            if not self.permitted() or not self.authenticated():
                return self.respond(403, {"error": "request_rejected"})
            if self.headers.get("Origin") != f"http://{args.bind}:{self.server.server_port}":
                return self.respond(403, {"error": "origin_rejected"})
            if self.headers.get("Content-Type") != "application/json" or self.headers.get("Transfer-Encoding"):
                return self.respond(415, {"error": "json_required"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024:
                    raise ValueError()
                raw = self.rfile.read(size)
                if len(raw) != size:
                    raise ValueError()
                body = json.loads(raw)
            except (ValueError, UnicodeError, OSError):
                return self.respond(400, {"error": "invalid_body"})
            path = urlsplit(self.path).path
            motion_commands = {
                "/api/motion/start": ("start", {"mode","label","color","distance","client_id"}),
                "/api/motion/stop": ("stop", set()),
                "/api/motion/heartbeat": ("heartbeat", {"mission_id","client_id"}),
                "/api/pose": ("pose", {"x","y","heading_deg"}),
            }
            if path in motion_commands:
                operation, fields = motion_commands[path]
                if not isinstance(body,dict) or set(body)!=fields:
                    return self.respond(400,{"error":"invalid_body"})
                for key,value in body.items():
                    if key in ("distance","x","y","heading_deg"):
                        valid = type(value) in (int,float) and math.isfinite(value)
                    elif key=="color":
                        valid = value is None or isinstance(value,str) and 1<=len(value)<=32
                    else:
                        valid = isinstance(value,str) and 1<=len(value)<=128
                    if not valid:
                        return self.respond(400,{"error":"invalid_body"})
                if not hasattr(telemetry,"action"):
                    return self.respond(409,{"error":"motion_bridge_unavailable"})
                result = telemetry.action(operation,body)
                return self.respond(200 if result.get("ok") else 409,result)
            commands = {"/api/confirm": ("confirm", "candidate_token"), "/api/arm": ("arm", "session_id")}
            if path not in commands:
                return self.respond(404, {"error": "no_motion_endpoint"})
            operation, field = commands[path]
            if not isinstance(body, dict) or set(body) != {field} or not isinstance(body[field], str) or not 1 <= len(body[field]) <= 128:
                return self.respond(400, {"error": "invalid_body"})
            result = pipeline.action(operation, body)
            return self.respond(200 if result.get("ok") else 409, result)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--serve-package", action="store_true")
    parser.add_argument("--client", default="172.20.10.2")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--vision-root", default="/userdata/udp_vision_20261019/rk3588/udp_vision")
    parser.add_argument("--task5-root", default="/userdata/task6_bundle_20261028_235520/task5_perception")
    parser.add_argument("--task6-root", default="/userdata/task6_bundle_20261028_235520/task6_navigation")
    parser.add_argument("--receiver", default="/root/receiver.py")
    parser.add_argument("--model", default="/userdata/rk3588-app/models/yolov5s_relu_rk3588.rknn")
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--no-display", action="store_true", help="browser only; leave board screen unused")
    parser.add_argument("--max-age", type=float, default=1.0)
    parser.add_argument("--pairing-file", type=Path, help="owner-only persistent pairing file")
    parser.add_argument("--supervised", action="store_true", help="exit on vision worker failure")
    parser.add_argument('--voice', choices=['off','dry','play'], default='off')
    parser.add_argument('--audio-device', help='ALSA playback device')
    args = parser.parse_args()
    ip = ipaddress.ip_address(args.bind)
    if not ip.is_private or ip.is_unspecified or ip.version != 4:
        parser.error("bind must be a specific private IPv4 address")
    if not 1 <= args.port <= 65535 or not math.isfinite(args.max_age) or not 0 < args.max_age <= 1:
        parser.error("invalid port or max-age")
    if args.check:
        from pipeline import dependencies
        dependencies(args.vision_root, args.receiver, args.model)
        from task5_bridge import Task5Bridge
        from motion_bridge import MotionBridge
        Task5Bridge(args.task5_root)
        MotionBridge(args.task6_root,args.serial)
        if not args.no_display:
            from display import check_display
            check_display()
        print("Dependencies and configuration OK. No UART, UDP or NPU device opened.")
        return
    pipeline = telemetry = token = voice = None
    if not args.serve_package:
        from pipeline import LivePipeline
        from motion_bridge import MotionBridge
        dependencies_args = (args.vision_root, args.receiver, args.model)
        from pipeline import dependencies
        token = pairing_token(args.pairing_file)
        if args.voice!='off':
            try:
                sys.path.append(str(ROOT.parent))
                from status_voice import Session
                voice=Session(args.voice,args.audio_device)
            except Exception as exc:print('VOICE UNAVAILABLE:',exc,flush=True)
        try:
            dependencies(*dependencies_args)
        except Exception:
            if voice:voice.failed('camera_error');voice.close()
            raise
        telemetry = MotionBridge(args.task6_root, args.serial, voice=voice)
        pipeline = LivePipeline(args, telemetry=telemetry)
        telemetry.vision = pipeline
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(args, pipeline, telemetry, token))
    server.daemon_threads = True
    server.request_queue_size = 8
    if pipeline:
        pipeline.start()
        telemetry.start()
    print(f"URL: http://{args.bind}:{args.port}", flush=True)
    if token:
        if args.pairing_file:
            print("Persistent pairing enabled; use autostart.py pair in the board terminal.", flush=True)
        else:
            print("PAIRING CODE (enter in browser; not stored): " + token, flush=True)
        print("Shared perception/UART. Motion requires explicit authenticated start and calibration.", flush=True)
    else:
        print(f"Source transfer only; allowed client: {args.client}", flush=True)
    try:
        if os.name == "posix":
            def terminate(_signum, _frame):
                raise KeyboardInterrupt()
            signal.signal(signal.SIGTERM, terminate)
        if (args.supervised or voice) and pipeline:
            server.timeout = .2
            started=time.monotonic();camera_problem=False
            while True:
                server.handle_request()
                view=pipeline.snapshot()['vision']
                problem=bool(view['error']) or (time.monotonic()-started>10 and not view['connected'])
                if voice and problem and not camera_problem:voice.failed('camera_error')
                camera_problem=problem
                if args.supervised and pipeline.error:
                    raise RuntimeError("Vision worker failed: " + str(pipeline.error))
        else:
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if pipeline:
            telemetry.close()
            pipeline.close()
        if voice:voice.close()


if __name__ == "__main__":
    main()
