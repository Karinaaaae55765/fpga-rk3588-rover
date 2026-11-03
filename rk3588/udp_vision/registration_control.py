"""Board-local registration API and command line client; never controls motors.

python3 -B registration_control.py status
python3 -B registration_control.py confirm
python3 -B registration_control.py arm     # Explicitly allow a new registration

HTTP: GET /status, POST /confirm {candidate_token}, POST /arm {session_id}.
Bind is always 127.0.0.1; a future authenticated UI backend may forward requests.
No CORS and no browser Origin requests; this is not a network-facing web UI.
"""
import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import threading
import urllib.error
import urllib.request


class ControlServer:
    def __init__(self, registration, port=8766):
        reg = registration

        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(.5)

            def log_message(self, *args):
                pass

            def respond(self, status, payload):
                data = json.dumps(payload, ensure_ascii=True).encode()
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(data)

            def permitted(self):
                host = self.headers.get('Host', '')
                allowed = {'127.0.0.1:%d' % self.server.server_port, 'localhost:%d' % self.server.server_port}
                return host in allowed and self.headers.get('Origin') is None

            def do_GET(self):
                if not self.permitted():
                    return self.respond(403, dict(ok=False, reason='local_client_required'))
                if self.path != '/status':
                    return self.respond(404, dict(ok=False, reason='unknown_endpoint'))
                self.respond(200, dict(ok=True, status=reg.status()))

            def do_POST(self):
                if not self.permitted():
                    return self.respond(403, dict(ok=False, reason='local_client_required'))
                if self.headers.get('Content-Type') != 'application/json' or self.headers.get('Transfer-Encoding'):
                    return self.respond(415, dict(ok=False, reason='json_required'))
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 2048:
                        raise ValueError('invalid size')
                    body = json.loads(self.rfile.read(size))
                    if not isinstance(body, dict):
                        raise ValueError('object required')
                except (ValueError, UnicodeError, OSError):
                    return self.respond(400, dict(ok=False, reason='invalid_request'))
                if self.path == '/confirm' and set(body) == {'candidate_token'} and isinstance(body['candidate_token'], str):
                    result = reg.confirm(body['candidate_token'])
                elif self.path == '/arm' and set(body) == {'session_id'} and isinstance(body['session_id'], str):
                    result = reg.arm(body['session_id'])
                else:
                    return self.respond(400, dict(ok=False, reason='invalid_command'))
                self.respond(200 if result['ok'] else 409, result)

        self.server = HTTPServer(('127.0.0.1', port), Handler)
        self.address = self.server.server_address
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={'poll_interval': .1}, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


def request(port, endpoint, payload=None):
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request('http://127.0.0.1:%d/%s' % (port, endpoint), data=data,
                                 headers={'Content-Type': 'application/json'} if data else {})
    # A machine-wide HTTP proxy must not receive local registration requests.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(req, timeout=5) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        with exc:
            return json.load(exc)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=['status', 'confirm', 'arm'])
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('invalid port')
    try:
        result = request(args.port, 'status')
        if args.action != 'status' and result.get('ok'):
            status = result['status']
            if args.action == 'confirm':
                token = status['candidate_token']
                result = request(args.port, 'confirm', dict(candidate_token=token)) if token else dict(ok=False, reason=status['reason'])
            else:
                result = request(args.port, 'arm', dict(session_id=status['session_id']))
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result.get('ok') else 1
    except (OSError, ValueError, urllib.error.URLError) as exc:
        print(json.dumps(dict(ok=False, reason='control_unavailable', error=str(exc))))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
