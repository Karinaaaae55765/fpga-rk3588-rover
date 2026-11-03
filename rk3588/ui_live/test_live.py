"""PC checks: no real camera, NPU or serial port is opened."""
import binascii
import json
import os
from pathlib import Path
import signal
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from telemetry_v3 import Parser, Telemetry
from pipeline import LivePipeline
from server import make_handler, FILES, pairing_token
import autostart as service


def independent_crc(raw):
    crc = 0xffff
    for value in raw:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def frame(seq=1, flags=15, uptime=1000):
    body = b"\x01\x10\x15" + struct.pack("<HBHHhhhHHI", seq, flags, 1234, 200, -123, 456, -789, 2, 3, uptime)
    return b"\xa5\x5a" + body + independent_crc(body).to_bytes(2, "little")


class ProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(independent_crc(b"123456789"), 0x29b1)
        self.assertEqual(binascii.crc_hqx(b"123456789", 0xffff), 0x29b1)

    def test_split_noise_and_signed_values(self):
        parser = Parser()
        raw = frame()
        self.assertEqual(parser.feed(b"noise"+raw[:12]), [])
        item = parser.feed(raw[12:])[0]
        self.assertEqual(item["distance_mm"], 1234)
        self.assertEqual(item["yaw"], -7.89)
        self.assertEqual(item["roll"], -1.23)
        self.assertEqual(len(raw), 28)

    def test_bad_crc_recovery(self):
        parser = Parser()
        raw = bytearray(frame())
        raw[8] ^= 1
        result = parser.feed(raw+frame(2))
        self.assertEqual(len(result), 1)
        self.assertEqual(parser.bad, 1)
        self.assertEqual(result[0]["sequence"], 2)

    def test_validity_flags(self):
        result = Parser().feed(frame(flags=3))[0]
        self.assertIsNone(result["distance_mm"])
        self.assertIsNone(result["roll"])
        self.assertEqual(Parser().feed(frame(flags=0x80)), [])

    def test_sequence_wrap_duplicate_and_reboot(self):
        parser = Parser()
        self.assertEqual(len(parser.feed(frame(65535))), 1)
        self.assertEqual(len(parser.feed(frame(0, uptime=1001))), 1)
        self.assertEqual(parser.feed(frame(0, uptime=1001)), [])
        self.assertEqual(parser.feed(frame(65535, uptime=1001)), [])
        self.assertEqual(len(parser.feed(frame(0, uptime=10))), 1)

    def test_uart_age_and_failure(self):
        now = [10.]
        telemetry = Telemetry("unused", clock=lambda: now[0])
        self.assertFalse(telemetry.snapshot()["connected"])
        telemetry.latest = Parser().feed(frame())[0]
        telemetry.received = now[0]
        self.assertTrue(telemetry.snapshot()["connected"])
        now[0] += .6
        self.assertIsNone(telemetry.snapshot()["values"])
        now[0] = 10.
        telemetry.error = "disconnected"
        self.assertFalse(telemetry.snapshot()["connected"])

    def test_no_uart_write_calls(self):
        import ast
        tree = ast.parse(Path(__file__).with_name("telemetry_v3.py").read_text())
        self.assertFalse(any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                             and node.func.attr == "write" for node in ast.walk(tree)))


class FrameTests(unittest.TestCase):
    def test_expiry_epoch_and_capture_failure(self):
        pipeline = LivePipeline(SimpleNamespace(max_age=1.))
        pipeline.capture = SimpleNamespace(epoch=1, error=None)
        pipeline.jpeg, pipeline.received, pipeline.epoch = b"jpeg", time.monotonic(), 1
        self.assertIsNotNone(pipeline.image())
        pipeline.received -= 2
        self.assertIsNone(pipeline.image())
        pipeline.received = time.monotonic()
        pipeline.capture.epoch = 2
        self.assertIsNone(pipeline.image())
        pipeline.capture.epoch = 1
        pipeline.capture.error = "lost input"
        self.assertIsNone(pipeline.image())


class FakePipeline:
    def __init__(self):
        self.calls = []

    def snapshot(self):
        return dict(hardware_write=False, vision={"connected": False})

    def image(self):
        return None

    def action(self, op, body):
        self.calls.append((op, body))
        return dict(ok=True)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.pipeline = FakePipeline()
        self.args = SimpleNamespace(bind="127.0.0.1", serve_package=False, client="127.0.0.1")
        self.token = "unit-test-only"
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
            self.args, self.pipeline, SimpleNamespace(snapshot=lambda: {"connected": False}), self.token))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = "http://127.0.0.1:"+str(self.server.server_port)
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def request(self, path, token=True, body=None, origin=True, host=None):
        headers = {"Authorization": "Bearer "+self.token} if token else {}
        if origin:
            headers["Origin"] = self.base
        if host:
            headers["Host"] = host
        raw = None
        if body is not None:
            raw = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(self.base+path, data=raw, headers=headers)
        try:
            with self.opener.open(req, timeout=2) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as exc:
            with exc:
                return exc.code, exc.read()

    def test_pairing_required_for_data(self):
        self.assertEqual(self.request("/", token=False)[0], 200)
        self.assertEqual(self.request("/api/state", token=False)[0], 401)
        self.assertEqual(self.request("/api/state")[0], 200)
        self.assertEqual(self.request("/api/frame")[0], 503)

    def test_no_motion_or_arbitrary_file_access(self):
        for path in ("/server.py", "/../README.md", "/targets/any/object.jpg",
                     "/pairing.key", "/service.log", "/server.py.before-autostart"):
            self.assertEqual(self.request(path)[0], 404)
        self.assertEqual(self.request("/api/execute", body={})[0], 404)
        self.assertEqual(self.pipeline.calls, [])

    def test_registration_auth_origin_and_schema(self):
        body = {"candidate_token": "current-token"}
        self.assertEqual(self.request("/api/confirm", token=False, body=body)[0], 403)
        self.assertEqual(self.request("/api/confirm", origin=False, body=body)[0], 403)
        self.assertEqual(self.request("/api/confirm", body={"candidate_token": 123})[0], 400)
        self.assertEqual(self.request("/api/confirm", body=body)[0], 200)
        self.assertEqual(len(self.pipeline.calls), 1)

    def test_host_rejected(self):
        self.assertEqual(self.request("/api/state", host="example.com")[0], 403)

    def package_handler(self, args):
        # Source distribution and installed layout differ (init script lives in /etc).
        # Exercise the whitelist with a complete isolated source package, not deployment files.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in FILES:
                (root/name).write_bytes(('package fixture '+name).encode())
            with patch('server.ROOT', root):
                return make_handler(args)

    def test_package_manifest_and_readonly(self):
        args = SimpleNamespace(bind="127.0.0.1", serve_package=True, client="127.0.0.1")
        self.server.RequestHandlerClass = self.package_handler(args)
        code, raw = self.request("/manifest.json", token=False)
        self.assertEqual(code, 200)
        manifest = json.loads(raw)
        self.assertEqual(set(manifest), set(FILES))
        self.assertNotIn("pairing.key", manifest)
        self.assertNotIn("service.log", manifest)
        import hashlib
        for name, digest in manifest.items():
            code, data = self.request("/files/"+name, token=False)
            self.assertEqual(code, 200)
            self.assertEqual(data, ('package fixture '+name).encode())
            self.assertEqual(hashlib.sha256(data).hexdigest(), digest)
        self.assertEqual(self.request("/api/confirm", body={})[0], 405)
        self.assertEqual(self.request("/files/../server.py")[0], 404)

    def test_package_rejects_other_clients(self):
        args = SimpleNamespace(bind="127.0.0.1", serve_package=True, client="192.0.2.1")
        self.server.RequestHandlerClass = self.package_handler(args)
        self.assertEqual(self.request("/manifest.json")[0], 403)


class PairingTests(unittest.TestCase):
    def test_persistent_pairing_survives_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pairing.key"
            first = pairing_token(path)
            self.assertEqual(first, pairing_token(path))
            self.assertEqual(len(first), 32)
            if os.name == "posix":
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_bad_key_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pairing.key"
            path.write_bytes(b"broken")
            path.chmod(0o600)
            with self.assertRaises(ValueError):
                pairing_token(path)
            self.assertEqual(path.read_bytes(), b"broken")

    @unittest.skipUnless(os.name == "posix", "Linux permissions")
    def test_public_or_symlink_key_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pairing.key"
            pairing_token(path)
            path.chmod(0o644)
            with self.assertRaises(ValueError):
                pairing_token(path)
            link = Path(directory) / "link"
            link.symlink_to(path)
            with self.assertRaises(OSError):
                pairing_token(link)


class BrowserPairingTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("node"), "Node.js required for browser-state harness")
    def test_remember_reload_outage_invalid_code_and_forget(self):
        # Runs actual app.js with DOM/network adapters; no real credentials or board traffic.
        script = r"""
const assert=require("assert/strict"),fs=require("fs"),vm=require("vm");
const source=fs.readFileSync(process.argv[1],"utf8");
const saved=new Map(),key="rover.live.pairing.v1",code="test-browser-pairing";
function create(mode="online",storageBlocked=false){
  const elements=new Map(),timers=[],calls=[];
  const api={mode,elements,timers,calls};
  function element(id){
    if(!elements.has(id))elements.set(id,{
      value:"",textContent:"",hidden:false,disabled:false,handlers:{},
      addEventListener(name,fn){this.handlers[name]=fn;},
      replaceChildren(){},append(){},removeAttribute(){},
      getContext(){return new Proxy({}, {get:()=>()=>{},set:()=>true});}
    });
    return elements.get(id);
  }
  const context={
    document:{getElementById:element,createElement:()=>element("li"),addEventListener(){},hidden:false},
    location:{host:"127.0.0.1:8080"},performance:{now:()=>100},
    localStorage:{
      getItem(k){if(storageBlocked)throw Error("blocked");return saved.get(k)||null;},
      setItem(k,v){if(storageBlocked)throw Error("blocked");saved.set(k,v);},
      removeItem(k){if(storageBlocked)throw Error("blocked");saved.delete(k);}
    },
    setTimeout(fn,ms){if(ms!==1800)timers.push(fn);return 1;},clearTimeout(){},setInterval(){},
    AbortController,URL:{revokeObjectURL(){},createObjectURL(){return "blob:fixture";}},
    fetch:async(path,opts)=>{
      calls.push([path,opts.headers.Authorization]);
      if(api.mode==="offline")throw Error("network unavailable");
      if(api.mode==="invalid")return {status:401,ok:false,json:async()=>({error:"pairing_required"})};
      if(path==="/api/frame")return {status:503,ok:false,json:async()=>({error:"no_fresh_frame"})};
      return {status:200,ok:true,json:async()=>({
        vision:{connected:false,npu:"initialized"},registration:null,
        telemetry:{connected:false,frames_ok:0,crc_errors:0,ignored_sequence:0}
      })};
    },Image:class {},console
  };
  vm.runInNewContext(source,context);
  return api;
}
const settle=async()=>{for(let i=0;i<8;i++)await new Promise(resolve=>setImmediate(resolve));};
(async()=>{
  const first=create();
  first.elements.get("pair-code").value=code;
  first.elements.get("pair-form").handlers.submit({preventDefault(){}});
  await settle();assert.equal(saved.get(key),code);
  const reloaded=create();await settle();
  assert(reloaded.calls.some(([p,a])=>p==="/api/state"&&a==="Bearer "+code));
  const outage=create("offline");await settle();
  assert.equal(saved.get(key),code);assert(outage.timers.length>0);
  outage.mode="online";for(const fn of outage.timers.splice(0))fn();await settle();
  assert.equal(outage.elements.get("connection").textContent,"设备服务已连接");
  outage.elements.get("disconnect").handlers.click();
  assert.equal(saved.has(key),false);
  const before=outage.calls.length;
  for(const fn of outage.timers.splice(0))fn();await settle();
  assert.equal(outage.calls.length,before);
  saved.set(key,code);const invalid=create("invalid");await settle();
  assert.equal(saved.has(key),false);assert.equal(invalid.timers.length,0);
  const blocked=create("online",true);blocked.elements.get("pair-code").value=code;
  blocked.elements.get("pair-form").handlers.submit({preventDefault(){}});
  await settle();assert.equal(saved.has(key),false);
  assert.equal(blocked.elements.get("disconnect").disabled,false);
  console.log("Browser pairing, reload, reconnect, rejection and forget OK");
})().catch(error=>{console.error(error);process.exitCode=1;});
"""
        result = subprocess.run(["node", "-e", script, str(Path(__file__).with_name("app.js"))],
                                text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class InstallTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.boot = self.root / "boot-entry"
        self.boot.write_bytes(b'#!/bin/sh\nexec /usr/bin/python3 -B /userdata/udp_vision_20261019/autostart.py "${1:-start}"\n')
        (self.root / "run.sh").write_text("# /root/gst-rawparse-1.22.2\n")
        self.original = {}
        import hashlib
        hashes = {}
        for name in service.BASE_HASHES:
            raw = ("original " + name).encode()
            (self.root / name).write_bytes(raw)
            self.original[name] = raw
            hashes[name] = hashlib.sha256(raw).hexdigest()
        self.patches = [
            patch.object(service, "ROOT", self.root),
            patch.object(service, "RUN", self.root / "run.sh"),
            patch.object(service, "SCRIPT", self.root / "autostart.py"),
            patch.object(service, "BOOT", self.boot),
            patch.object(service, "BASE_HASHES", hashes),
        ]
        for item in self.patches:
            item.start()
        self.bundle = {name: ("new " + name).encode() for name in service.UPDATES}

    def tearDown(self):
        for item in reversed(self.patches):
            item.stop()
        self.temp.cleanup()

    def test_conflicts_prevent_all_modifications(self):
        (self.root / "app.js.before-autostart").write_bytes(b"keep")
        with self.assertRaisesRegex(ValueError, "conflict"):
            service.install_plan(self.bundle)
        self.assertEqual((self.root / "server.py").read_bytes(), self.original["server.py"])
        self.assertFalse((self.root / "autostart.py").exists())

    def test_changed_existing_file_rejected(self):
        (self.root / "app.js").write_bytes(b"user edit")
        with self.assertRaisesRegex(ValueError, "changed"):
            service.install_plan(self.bundle)
        self.assertEqual((self.root / "app.js").read_bytes(), b"user edit")

    def test_unrecognized_boot_entry_rejected(self):
        self.boot.write_text("#!/bin/sh\nstart-something-else\n")
        with self.assertRaisesRegex(ValueError, "boot entry"):
            service.install_plan(self.bundle)

    def test_existing_strict_shell_boot_entry_allowed(self):
        self.boot.write_bytes(self.boot.read_bytes().replace(b"#!/bin/sh\n", b"#!/bin/sh\nset -eu\n"))
        plan = service.install_plan(self.bundle)
        self.assertEqual(len(plan), 6)
        self.assertTrue(all(backup.parent == self.root for _, _, backup, _ in plan if backup))

    def test_failed_start_restores_sources_and_boot(self):
        original_boot = self.boot.read_bytes()
        def run(command, **kwargs):
            code = 1 if command[-1] == "start" and str(self.root / "autostart.py") in command else 0
            return SimpleNamespace(returncode=code)
        with patch.object(service, "download", return_value=self.bundle), \
                patch.object(service, "stop_service", return_value=0), \
                patch.object(service, "stop_foreground"), \
                patch.object(service, "record", return_value=None), \
                patch.object(service.subprocess, "run", side_effect=run):
            with self.assertRaisesRegex(RuntimeError, "failed to start"):
                service.install("http://127.0.0.1:8091")
        for name, content in self.original.items():
            self.assertEqual((self.root / name).read_bytes(), content)
            self.assertEqual((self.root / (name + ".before-autostart")).read_bytes(), content)
        self.assertEqual(self.boot.read_bytes(), original_boot)
        self.assertEqual((self.root / "S99udp-vision.before-autostart").read_bytes(), original_boot)
        self.assertFalse((self.root / "autostart.py").exists())

    def test_wifi_address_and_retry_backoff(self):
        self.assertEqual(service.select_wifi_address("5: wlan0 inet 172.20.10.7/28 brd 172.20.10.15"), "172.20.10.7")
        for raw in ("", "inet 127.0.0.1/8", "inet 169.254.1.1/16", "inet 8.8.8.8/24"):
            with self.assertRaises(RuntimeError):
                service.select_wifi_address(raw)
        self.assertEqual([service.restart_delay(n) for n in (1, 2, 3, 4, 5, 100)], [2, 4, 8, 16, 30, 30])


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux process sessions and flock")
class LinuxLifecycleTests(unittest.TestCase):
    def test_detach_single_instance_restart_and_clean_stop(self):
        # Isolated /tmp fixture: no board paths, network, serial or NPU opened.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = str(Path(__file__).resolve().parent)
            wrapper = root / "autostart.py"
            wrapper.write_text(
                "import sys\nfrom pathlib import Path\nsys.path.insert(0, %r)\n"
                "import autostart as m\nm.ROOT=Path(%r)\n"
                "m.SCRIPT=m.ROOT/'autostart.py'\nm.APP=m.ROOT/'worker.py'\n"
                "m.RUN=m.ROOT/'run.sh'\nm.KEY=m.ROOT/'pairing.key'\n"
                "m.PID=m.ROOT/'pid'\nm.LOCK=m.ROOT/'lock'\nm.LOG=m.ROOT/'service.log'\n"
                "m.ready=lambda:'127.0.0.1'\nm.configure_camera=lambda:None\n"
                "m.wifi_address=lambda:'127.0.0.1'\n"
                "raise SystemExit({'start':m.start_service,'run':m.run_service,'stop':m.stop_service}[sys.argv[1]]())\n"
                % (source, str(root)))
            (root / "run.sh").write_text('exec "%s" -B "%s"\n' % (sys.executable, root / "worker.py"))
            (root / "worker.py").write_text(
                "import time\nprint('W RKNN size_with_stride advisory',flush=True)\n"
                "print('W RKNN size_with_stride advisory',flush=True)\n"
                "print('worker ready',flush=True)\ntime.sleep(120)\n")
            def command(action):
                return subprocess.run([sys.executable, "-B", str(wrapper), action],
                                      capture_output=True, text=True, timeout=22, check=True)
            def current():
                try:
                    return json.loads((root / "pid").read_text())
                except (OSError, ValueError):
                    return {}
            def wait_for(predicate):
                until = time.monotonic() + 10
                while time.monotonic() < until:
                    value = current()
                    if predicate(value):
                        return value
                    time.sleep(.1)
                self.fail("isolated supervisor did not reach expected state")
            try:
                command("start")  # launcher has exited; detached supervisor must stay alive
                first = wait_for(lambda value: value.get("child_pid") is not None)
                time.sleep(.2)
                command("start")
                self.assertEqual(current()["pid"], first["pid"])
                self.assertEqual(current()["child_pid"], first["child_pid"])
                os.kill(first["child_pid"], signal.SIGTERM)
                second = wait_for(lambda value: value.get("child_pid") not in (None, first["child_pid"]))
                self.assertEqual(second["pid"], first["pid"])
                self.assertNotEqual(second["child_pid"], first["child_pid"])
                command("stop")
                self.assertFalse((root / "pid").exists())
                self.assertIsNone(service.identity(second["child_pid"]))
                text = (root / "service.log").read_text()
                self.assertIn("APPLICATION_EXIT", text)
                self.assertIn("worker ready", text)
                self.assertNotIn("W RKNN size_with_stride advisory", text)
            finally:
                command("stop")



class MotionIntegrationTests(unittest.TestCase):
    def setUp(self):
        from motion_bridge import MotionBridge
        root = Path(__file__).parent.parent/'task6_navigation'
        if not root.is_dir():
            root = Path('/userdata/task6_bundle_20261028_235520/task6_navigation')
        self.now = [10.]
        self.bridge = MotionBridge(root, '/dev/ttyUSB0', clock=lambda:self.now[0])
        self.sent = []
        class Link:
            execute = False
            armed = False
            seq = 0
            session = 0
            def send(link, command, **kwargs):
                self.assertTrue(link.execute)
                link.seq += 1
                self.sent.append((command,kwargs))
                return link.seq
            def velocity(link, left, right):
                self.assertTrue(link.armed)
                self.sent.append((2,dict(left=left,right=right)))
        self.bridge.link = Link()
        self.bridge.latest = self.sample()
        self.row = dict(generation=1,frame_id=1,stream_epoch=0,observed_at_monotonic=10.,objects=[],
                        tracking=dict(status='not_found',confirmed=False,track_id=None,observation_index=None))
        task5 = SimpleNamespace(select=lambda *args:1)
        self.bridge.vision = SimpleNamespace(task5=task5,perception_snapshot=lambda *args:self.row)
        self.body = dict(mode='approach',label='cup',color='red',distance=.5,client_id='test-client')

    def sample(self, **kw):
        row = dict(received_at=self.now[0],uptime_ms=int(self.now[0]*1000),sequence=1,
            challenge=7,session=0,accepted_sequence=0,state=0,reason=0,flags=39,
            distance_mm=2000,yaw_cdeg=0,left_count=0,right_count=0,left_rpm=0,right_rpm=0)
        row.update(kw)
        return row

    def allow_synthetic_calibration(self):
        import copy
        original = self.bridge.load
        cal = copy.deepcopy(original('calibration.json'))
        for k in ('vehicle_verified','ground_projection_verified','recognition_envelope_verified','tof_extrinsics_verified','start_pose_verified'):
            cal[k] = True
        cal.update(evidence='SYNTHETIC TEST ONLY',pixel_to_robot_ground_h=[[0,0,200],[-.4,0,256],[0,1,-180]],
            max_holdout_error_m=.01,valid_ground_polygon_px=[[4,190],[1276,190],[1276,716],[4,716]])
        self.bridge.load = lambda name:cal if name=='calibration.json' else original(name)

    def begin(self):
        self.allow_synthetic_calibration()
        result = self.bridge.action('start',self.body)
        self.assertTrue(result['ok'],result)
        self.assertEqual(self.sent,[])
        self.bridge._tick(self.now[0])
        self.assertEqual(self.sent[0][0],1)
        self.assertEqual(self.bridge.phase,'ARMING')
        return result

    def acknowledge(self):
        self.now[0] += .05
        self.bridge.latest = self.sample(state=1,session=self.bridge.link.session,
                                         accepted_sequence=self.bridge.arm_seq)
        self.bridge._tick(self.now[0])
        self.assertEqual(self.bridge.phase,'RUNNING')

    def test_unmeasured_calibration_cannot_write_or_arm(self):
        result = self.bridge.action('start',self.body)
        self.assertFalse(result['ok'])
        self.assertIn('ground_projection_verified',result['blockers'])
        self.bridge._tick(self.now[0])
        self.assertEqual(self.sent,[])
        self.assertFalse(self.bridge.link.execute)

    def test_arm_ack_required_before_velocity(self):
        self.begin()
        self.now[0] += .05
        self.bridge.latest = self.sample()
        self.bridge._tick(self.now[0])
        self.assertEqual([c for c,k in self.sent],[1])
        self.acknowledge()
        self.assertEqual(self.sent[-1],(2,dict(left=0,right=0)))

    def test_missing_arm_ack_stops_and_explains_TX(self):
        self.begin()
        self.now[0] += .21
        self.bridge.latest = self.sample()
        self.bridge._tick(self.now[0])
        self.assertEqual(self.bridge.reason,'ARM_ack_timeout_check_TX_wiring')
        self.assertEqual(self.sent[-1][0],3)
        self.assertFalse(self.bridge.link.execute)

    def test_stop_waits_for_new_stationary_telemetry(self):
        self.begin();self.acknowledge()
        self.bridge.action('stop',{})
        self.bridge._tick(self.now[0])
        self.assertEqual(self.bridge.phase,'STOPPING')
        self.assertFalse(self.bridge.stop_confirmed)
        self.bridge._tick(self.now[0]+.02)
        self.assertTrue(self.bridge.active)
        self.now[0] += .05
        self.bridge.latest = self.sample()
        self.bridge._tick(self.now[0])
        self.assertFalse(self.bridge.active)
        self.assertTrue(self.bridge.stop_confirmed)

    def test_browser_expiry_cannot_be_revived(self):
        result=self.begin();self.acknowledge()
        self.now[0] += 2
        r=self.bridge.action('heartbeat',dict(mission_id=result['mission_id'],client_id='test-client'))
        self.assertFalse(r['ok'])
        self.bridge._tick(self.now[0])
        self.assertEqual(self.sent[-1][0],3)
        self.assertEqual(self.bridge.phase,'STOPPING')

    def test_heartbeat_from_different_browser_rejected(self):
        result=self.begin()
        deadline=self.bridge.lease
        self.now[0] += .1
        self.assertFalse(self.bridge.action('heartbeat',dict(mission_id=result['mission_id'],client_id='another'))['ok'])
        self.assertEqual(deadline,self.bridge.lease)

    def test_lost_vision_does_not_renew_velocity(self):
        self.begin();self.acknowledge()
        self.row=None
        self.now[0] += .05
        self.bridge.latest=self.sample(state=1,session=self.bridge.link.session)
        self.bridge._tick(self.now[0])
        self.assertEqual(self.bridge.reason,'vision_stale')
        self.assertEqual(self.sent[-1][0],3)

    def test_stream_restart_and_control_stall_stop(self):
        self.begin();self.acknowledge()
        self.row['stream_epoch']=2
        self.bridge._tick(self.now[0]+.01)
        self.assertEqual(self.bridge.reason,'camera_restarted')

    def test_loop_stall_before_arm_writes_no_velocity(self):
        self.begin()
        self.bridge.last_control=self.now[0]-.3
        self.bridge._tick(self.now[0])
        self.assertEqual(self.bridge.reason,'control_loop_stalled')
        self.assertEqual([c for c,k in self.sent],[1,3])

    def test_global_search_rejects_simulation_map(self):
        self.allow_synthetic_calibration()
        self.bridge.pose.reset(.13,1.14,0)
        self.bridge.pose.update(self.sample())
        result=self.bridge.action('start',dict(self.body,mode='search'))
        self.assertFalse(result['ok'])
        self.assertIn('search_map_not_printed_map',result['blockers'])
        self.assertEqual(self.sent,[])

    def test_pose_requires_stationary_explicit_origin_and_expires(self):
        p=self.bridge.pose
        self.assertFalse(p.snapshot()['available'])
        self.bridge.latest=self.sample(left_rpm=5)
        self.assertFalse(self.bridge.action('pose',dict(x=.13,y=1.14,heading_deg=0))['ok'])
        self.bridge.latest=self.sample()
        self.assertTrue(self.bridge.action('pose',dict(x=.13,y=1.14,heading_deg=0))['ok'])
        p.update(self.sample());self.now[0]+=.1
        p.update(self.sample(left_count=100,right_count=100))
        self.assertGreater(p.snapshot()['pose']['x'],.13)
        self.now[0]+=.3
        self.assertFalse(p.snapshot()['available'])
        self.now[0]+=.3;p.update(self.sample(left_count=101,right_count=101))
        self.assertIsNone(p.localizer)
        self.assertFalse(p.snapshot()['available'])

    def test_shared_v3_and_task6_bytes_do_not_open_second_port(self):
        from fpga_link import Parser,STATUS,KEYS
        sample=self.sample(tof_sequence=1,imu_sequence=1)
        raw=b'\xa5\x5a\x01\x36\x27'+STATUS.pack(*(sample[k] for k in KEYS))
        raw+=independent_crc(raw[2:]).to_bytes(2,'little')
        parser=Parser();rows=[]
        stream=frame()+raw
        for i in range(0,len(stream),7):
            data=stream[i:i+7]
            self.bridge.sensor.accept(data)
            rows.extend(parser.feed(data,self.now[0]))
        self.assertEqual(len(rows),1)
        self.assertTrue(self.bridge.sensor.snapshot()['connected'])
        self.assertIsNone(self.bridge.sensor.thread)
        self.assertEqual(self.sent,[])


class MotionHTTPTests(HTTPTests):
    def setUp(self):
        super().setUp()
        self.motion_calls=[]
        def action(op,body):
            self.motion_calls.append((op,body));return dict(ok=True)
        telemetry=SimpleNamespace(snapshot=lambda:{'connected':False},action=action)
        self.server.RequestHandlerClass=make_handler(self.args,self.pipeline,telemetry,self.token)

    def test_motion_auth_origin_and_numeric_schema(self):
        body=dict(mode='approach',label='cup',color='red',distance=.5,client_id='test')
        self.assertEqual(self.request('/api/motion/start',token=False,body=body)[0],403)
        self.assertEqual(self.request('/api/motion/start',origin=False,body=body)[0],403)
        self.assertEqual(self.request('/api/motion/start',body=dict(body,distance=True))[0],400)
        self.assertEqual(self.request('/api/motion/start',body=dict(body,execute=True))[0],400)
        self.assertEqual(self.motion_calls,[])
        self.assertEqual(self.request('/api/motion/start',body=body)[0],200)
        self.assertEqual(self.request('/api/motion/stop',body={})[0],200)
        self.assertEqual([op for op,_ in self.motion_calls],['start','stop'])


class Task5SharingTests(unittest.TestCase):
    @unittest.skipUnless(__import__('importlib.util').util.find_spec('cv2'), 'OpenCV required; run on RK3588')
    def test_color_correspondence_generation_and_stale_frames(self):
        import numpy as np
        from task5_bridge import Task5Bridge
        root=Path(__file__).parent.parent/'task5_perception'
        if not root.is_dir():root=Path('/userdata/task6_bundle_20261028_235520/task5_perception')
        bridge=Task5Bridge(root);generation=bridge.select('cup','red')
        image=np.zeros((150,300,3),dtype=np.uint8)
        image[:,:150]=(0,0,255);image[:,150:]=(255,0,0)
        boxes=[SimpleNamespace(label='cup',score=.9,left=5,top=5,right=145,bottom=145),
               SimpleNamespace(label='cup',score=.9,left=155,top=5,right=295,bottom=145)]
        for i in range(3):
            now=time.monotonic()
            annotated=bridge.process(image,boxes,SimpleNamespace(frame_id=i,epoch=0,received_at=now))
        row=bridge.snapshot(time.monotonic(),1,generation)
        self.assertEqual([o['colors']['body']['name'] for o in row['objects']],['red','blue'])
        self.assertTrue(row['tracking']['confirmed']);self.assertEqual(row['tracking']['observation_index'],0)
        self.assertIsNotNone(annotated)
        self.assertIsNone(bridge.snapshot(time.monotonic()+2,1,generation))
        bridge.select('cup','blue')
        self.assertIsNone(bridge.snapshot(time.monotonic(),1,generation))
        self.assertEqual(tuple(image[100,100]),(0,0,255))


class RegistrationHighlightTests(unittest.TestCase):
    def setUp(self):
        from task5_bridge import RegistrationHighlight
        self.highlight=RegistrationHighlight()
        self.record=dict(target_id='saved-A',epoch=0,frame_id=1,
                         box=dict(label='cup',left=10,top=80,right=110,bottom=180))

    def item(self,index=0,x=10,color='orange'):
        return SimpleNamespace(index=index,label='cup',box=(x,80,x+100,180),
            colors={'body':{'name':color,'reliable':True}})

    def update(self,i,items=None,record=None,epoch=0,at=None):
        at=10+i*.1 if at is None else at
        return self.highlight.update(record,items if items is not None else [self.item()],
            SimpleNamespace(epoch=epoch,frame_id=i,received_at=at),at+.01)

    def acquire(self):
        self.assertIsNone(self.update(1))
        self.assertEqual(self.update(2,record=self.record),0)

    def test_registration_uses_saved_frame_then_tracks_current_box(self):
        self.acquire()
        self.assertEqual(self.update(3,[self.item(2,20)],self.record),2)

    def test_loss_does_not_reacquire_same_saved_record(self):
        self.acquire()
        self.assertIsNone(self.update(3,[],self.record))
        self.assertIsNone(self.update(4,record=self.record))

    def test_two_overlapping_matches_cancel_green(self):
        self.acquire()
        self.assertIsNone(self.update(3,[self.item(),self.item(1,15)],self.record))
        self.assertIsNone(self.update(4,record=self.record))

    def test_far_object_not_mistaken_for_registered_object(self):
        self.acquire()
        self.assertEqual(self.update(3,[self.item(),self.item(1,200)],self.record),0)
        self.assertIsNone(self.update(4,[self.item(1,200)],self.record))

    def test_color_change_stream_restart_gap_and_duplicate_clear(self):
        self.acquire()
        self.assertIsNone(self.update(3,[self.item(color='blue')],self.record))
        self.setUp();self.acquire()
        self.assertIsNone(self.update(3,record=self.record,epoch=1))
        self.setUp();self.acquire()
        self.assertIsNone(self.update(3,record=self.record,at=12))
        self.setUp();self.acquire()
        self.assertIsNone(self.update(2,record=self.record))

    def test_old_registration_without_frame_history_is_not_identification(self):
        self.assertIsNone(self.update(10,record=self.record))
        self.assertIsNone(self.update(11,record=self.record))

    def test_saving_gap_replayed_and_new_registration_restores(self):
        self.update(1)
        self.update(2,[])
        self.assertIsNone(self.update(3,record=self.record))
        self.update(4)
        new=dict(self.record,target_id='saved-B',frame_id=4)
        self.assertEqual(self.update(5,record=new),0)
        self.assertIsNone(self.update(6))

    @unittest.skipUnless(__import__('importlib.util').util.find_spec('cv2'), 'OpenCV required; run on RK3588')
    def test_green_pixels_without_enabling_motion_tracker(self):
        import numpy as np
        from task5_bridge import Task5Bridge
        root=Path(__file__).parent.parent/'task5_perception'
        if not root.is_dir():root=Path('/userdata/task6_bundle_20261028_235520/task5_perception')
        bridge=Task5Bridge(root)
        image=np.zeros((240,320,3),dtype=np.uint8);image[:]=(0,100,255)
        box=SimpleNamespace(label='cup',score=.9,left=10,top=80,right=110,bottom=180)
        at=time.monotonic()
        bridge.process(image,[box],SimpleNamespace(frame_id=1,epoch=0,received_at=at))
        out=bridge.process(image,[box],SimpleNamespace(frame_id=2,epoch=0,received_at=time.monotonic()),self.record)
        row=bridge.snapshot(time.monotonic(),1)
        self.assertEqual(tuple(out[130,10]),(0,255,0))
        self.assertTrue(row['registration_highlight']['visible'])
        self.assertFalse(row['tracking']['confirmed'])
        self.assertEqual(row['tracking']['status'],'observe_only')
        bridge.process(image,[],SimpleNamespace(frame_id=3,epoch=0,received_at=time.monotonic()),self.record)
        out=bridge.process(image,[box],SimpleNamespace(frame_id=4,epoch=0,received_at=time.monotonic()),self.record)
        self.assertEqual(tuple(out[130,10]),(0,255,255))
        self.assertFalse(bridge.snapshot(time.monotonic(),1)['registration_highlight']['visible'])
        self.assertEqual(tuple(image[130,10]),(0,100,255))

if __name__ == "__main__":
    unittest.main(verbosity=2)
