"""Loopback-only status dashboard. POST /stop requests a latched mission stop."""
import json
import threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

HTML='''<!doctype html><meta charset="utf-8"><title>任务六</title>
<style>body{background:#152128;color:#e4eded;font:18px sans-serif;margin:30px}button{padding:18px;background:#c33;color:white}pre{white-space:pre-wrap}</style>
<h1>任务六 · 实际运行状态</h1><button onclick="fetch('/stop',{method:'POST'})">停止任务</button>
<p id="description">等待任务状态。此页面不是硬件急停。</p>
<canvas id="map" width="900" height="600" style="background:#23343c;max-width:100%"></canvas><pre id="s"></pre>
<script>
function draw(d){let c=map.getContext('2d');c.clearRect(0,0,900,600);
if(d.coordinate_frame==='robot'){
 description.textContent='可见目标模式：前向为上、车身左侧为左；没有全局位置或场地边界信息。';
 let k=200,point=p=>[450-p[1]*k,550-p[0]*k];
 c.strokeStyle='#5bd6ff';c.beginPath();c.moveTo(450,550);c.lineTo(450,510);c.stroke();
 for(let p of d.obstacles_relative||[]){let [x,y]=point(p);c.beginPath();c.arc(x,y,p[2]*k,0,7);c.fillStyle='#d66';c.fill()}
 if(d.target_relative){let t=d.target_relative,[x,y]=point([t.x,t.y]);c.beginPath();c.arc(x,y,7,0,7);c.fillStyle='#ffd562';c.fill();
 c.font='20px sans-serif';c.fillText('目标 '+t.distance.toFixed(2)+' m / '+(t.bearing*180/Math.PI).toFixed(1)+'°',20,30)}return;}
 description.textContent='地图坐标为里程计估计；绿色路线、黄色目标、红色观测障碍。此页面不是硬件急停。';
 if(!d.map||!d.pose)return;let m=d.map,k=Math.min(860/m.width_m,560/m.height_m);
let point=p=>[20+p[0]*k,580-p[1]*k];c.clearRect(0,0,900,600);c.strokeStyle='#aab';c.strokeRect(20,580-m.height_m*k,m.width_m*k,m.height_m*k);
for(let p of m.stations||[]){let [x,y]=point(p);c.fillStyle='#718695';c.fillRect(x-2,y-2,4,4)}
for(let p of m.dynamic||[]){let [x,y]=point(p);c.beginPath();c.arc(x,y,p[2]*k,0,7);c.fillStyle='#d66';c.fill()}
c.beginPath();c.strokeStyle='#6ed99e';c.lineWidth=2;for(let [i,p] of (d.route||[]).entries()){let [x,y]=point(p);i?c.lineTo(x,y):c.moveTo(x,y)}c.stroke();
if(d.target){let [x,y]=point(d.target);c.beginPath();c.arc(x,y,7,0,7);c.fillStyle='#ffd562';c.fill()}
let [x,y]=point([d.pose.x,d.pose.y]);c.beginPath();c.arc(x,y,m.footprint_radius_m*k,0,7);c.strokeStyle='#5bd6ff';c.stroke();
c.beginPath();c.moveTo(x,y);c.lineTo(x+20*Math.cos(d.pose.yaw),y-20*Math.sin(d.pose.yaw));c.stroke();}
setInterval(async()=>{try{let r=await fetch('/state'),d=await r.json();s.textContent=JSON.stringify(d,null,2);draw(d)}catch(e){s.textContent='状态连接中断'}},300)
</script>'''

class Dashboard:
    def __init__(self,port):
        self.state={'state':'STARTING'};self.stop=threading.Event();outer=self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                data=json.dumps(outer.state,ensure_ascii=False).encode() if self.path=='/state' else HTML.encode()
                self.send_response(200);self.send_header('Content-Type','application/json' if self.path=='/state' else 'text/html; charset=utf-8')
                self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
            def do_POST(self):
                if self.path!='/stop':self.send_error(404);return
                outer.stop.set();self.send_response(204);self.end_headers()
            def log_message(self,*args):pass
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
    def close(self):self.server.shutdown();self.server.server_close();self.thread.join(timeout=1)
