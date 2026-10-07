"""Local infrared preview with calibrated, frame-matched raw Leap 21-point overlays.

Run with the same Python environment as the workbench. This diagnostic opens no
robot connection and does not modify tracking mode, orientation, or landmarks.
"""
import argparse
import base64
from collections import OrderedDict, deque
import json
import math
import signal
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')


def camera_point(point, matrix):
    """Invert the column-major camera-to-Leap rigid transform (millimetres)."""
    offset = [point[i] - matrix[12+i] for i in range(3)]
    return [sum(matrix[4*axis+i] * offset[i] for i in range(3)) for axis in range(3)]


def png_gray(width, height, pixels):
    def chunk(name, data):
        return (struct.pack('!I', len(data)) + name + data
                + struct.pack('!I', zlib.crc32(name + data) & 0xffffffff))
    rows = b''.join(b'\0' + pixels[y*width:(y+1)*width] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!IIBBBBB', width, height, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows, 1)) + chunk(b'IEND', b''))


HTML = r"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Leap 红外与21点诊断</title>
<style>
body{background:#11151b;color:#e9eef5;font:15px system-ui,sans-serif;max-width:1180px;margin:24px auto;padding:0 18px}
h1{font-size:25px;margin-bottom:8px}p{line-height:1.6}.controls{display:flex;gap:18px;align-items:center;flex-wrap:wrap;margin:18px 0}
button,select{font:inherit;background:#27313f;color:inherit;border:1px solid #53647a;padding:5px 10px;border-radius:5px}
.views{display:grid;grid-template-columns:1fr 1fr;gap:16px}canvas{width:100%;background:#000;border:1px solid #394658;border-radius:8px}
#status{padding:10px;background:#202a36;border-radius:6px}#metrics{white-space:pre-line;font-variant-numeric:tabular-nums;color:#92dfef}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;margin-top:12px}
th,td{padding:5px 10px;border-bottom:1px solid #303b49;text-align:right}th:first-child,td:first-child{text-align:left}
.tip{color:#ffcf69;font-weight:600}small{color:#aab5c4}.error{color:#ff9999!important}@media(max-width:700px){.views{grid-template-columns:1fr}}
</style>
<h1>Leap Motion 2 · 红外与21点诊断</h1>
<p>将原始三维骨架按相机标定投影到红外画面。<span class="tip">4：拇指尖（橙色）；8：食指尖（黄色）</span>。虚线连接两点。</p>
<div class="controls">
<label><input id="overlay" type="checkbox" checked>显示骨架</label>
<label><input id="labels" type="checkbox" checked>显示编号</label>
<label>手别 <select id="side"><option value="all">全部</option><option value="Left">左手</option><option value="Right">右手</option></select></label>
<button id="pause">暂停画面</button>
</div>
<div id="status">等待红外图像…</div>
<p id="metrics"></p>
<div class="views"><section><p>相机 0</p><canvas id="c0" width="512" height="512"></canvas></section>
<section><p>相机 1</p><canvas id="c1" width="512" height="512"></canvas></section></div>
<small>4–8 距离按三维坐标计算，单位 mm；SDK 捏合距离的定义不同，两者不一定相等。预览上限 20 FPS，追踪帧率单独显示。过期图像会变暗，过期骨架会清除。</small>
<details open><summary>关键点三维坐标（设备坐标，mm）</summary>
<table><thead><tr><th>手别 / 点号</th><th>X</th><th>Y</th><th>Z</th></tr></thead><tbody id="coords"></tbody></table></details>
<script>
const $=id=>document.getElementById(id),edges=[];
for(let base=1;base<=17;base+=4){edges.push([0,base],[base,base+1],[base+1,base+2],[base+2,base+3]);}
edges.push([5,9],[9,13],[13,17]);
let paused=false,last=null,bitmaps=[],lastGood=0,watchdogExpired=false;
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'继续画面':'暂停画面';if(last)render(last,bitmaps);};
for(const id of ['overlay','labels','side'])$(id).onchange=()=>{if(last)render(last,bitmaps);};
function selected(data){return data.hands.filter(h=>$('side').value==='all'||h.side===$('side').value);}
function render(data,imgs){
 const expired=data.image_age_ms===null||data.image_age_ms>350;
 const hands=expired?[]:selected(data);
 $('status').textContent=(paused?'已暂停 · ':'')+(expired?'图像已过期／等待新帧':'实时')
 +' · 追踪 '+data.tracking_fps.toFixed(1)+' FPS · 图像龄期 '+(data.image_age_ms??'—')+' ms'
 +' · 图像帧 '+(data.frame_id??'—')+' · '+(data.matched?'骨架同帧匹配':'没有同帧骨架');
 $('status').className=expired?'error':'';
 for(let camera=0;camera<2;camera++){
  const canvas=$('c'+camera),ctx=canvas.getContext('2d'),meta=data.cameras[camera];
  if(meta){canvas.width=meta.width;canvas.height=meta.height;}
  ctx.clearRect(0,0,canvas.width,canvas.height);
  if(imgs[camera]){ctx.globalAlpha=expired?.3:1;ctx.drawImage(imgs[camera],0,0);ctx.globalAlpha=1;}
  if(expired){ctx.fillStyle='#ffbbbb';ctx.font='bold 20px sans-serif';ctx.fillText('图像已过期',20,35);}
  if(!$('overlay').checked)continue;
  for(const hand of hands){
   const points=hand.pixels[camera],color=hand.side==='Left'?'#56e6bd':'#73b7ff';
   if(!points)continue;
   ctx.lineWidth=2;ctx.strokeStyle=color;
   for(const [a,b] of edges){if(!points[a]||!points[b])continue;ctx.beginPath();ctx.moveTo(...points[a]);ctx.lineTo(...points[b]);ctx.stroke();}
   if(points[4]&&points[8]){ctx.setLineDash([5,4]);ctx.strokeStyle='#ffcf69';ctx.beginPath();ctx.moveTo(...points[4]);ctx.lineTo(...points[8]);ctx.stroke();ctx.setLineDash([]);}
   points.forEach((p,i)=>{if(!p)return;ctx.fillStyle=i===4?'#ff984c':i===8?'#ffe164':color;ctx.beginPath();ctx.arc(p[0],p[1],i===4||i===8?5:3,0,Math.PI*2);ctx.fill();
    if($('labels').checked){ctx.font='bold 13px sans-serif';ctx.lineWidth=3;ctx.strokeStyle='#101010';ctx.strokeText(String(i),p[0]+6,p[1]-5);ctx.fillText(String(i),p[0]+6,p[1]-5);}});
  }
 }
 $('metrics').textContent=hands.map(h=>(h.side==='Left'?'左手':'右手')+' · 4–8 三维距离 '+h.tip_gap_mm.toFixed(1)+' mm · SDK 捏合距离 '+h.pinch_distance_mm.toFixed(1)+' mm · 捏合强度 '+h.pinch_strength.toFixed(2)).join('\n')||(expired?'等待有效图像':'当前帧未检测到所选手');
 $('coords').replaceChildren();
 for(const h of hands)h.points_mm.forEach((p,i)=>{const tr=document.createElement('tr');if(i===4||i===8)tr.className='tip';
 const values=[(h.side==='Left'?'左手':'右手')+' / '+i,...p.map(v=>v.toFixed(1))];for(const value of values){const td=document.createElement('td');td.textContent=value;tr.append(td);}$('coords').append(tr);});
}
async function poll(){
 if(!paused)try{
  const res=await fetch('/frame',{cache:'no-store',signal:AbortSignal.timeout(1500)});if(!res.ok)throw Error('HTTP '+res.status);
  const data=await res.json();
  const imgs=await Promise.all(data.cameras.map(c=>c?new Promise((resolve,reject)=>{const im=new Image();im.onload=()=>resolve(im);im.onerror=reject;im.src='data:image/png;base64,'+c.png;}):null));
  last=data;bitmaps=imgs;lastGood=performance.now();watchdogExpired=false;render(data,imgs);
 }catch(e){$('status').textContent='诊断连接失败：'+e.message;$('status').className='error';
  if(last&&performance.now()-lastGood>350)render({...last,image_age_ms:9999},bitmaps);
 }
 setTimeout(poll,50);
}
setInterval(()=>{
 if(paused||!last||last.image_age_ms===null||watchdogExpired)return;
 const age=last.image_age_ms+performance.now()-lastGood;
 if(age>350){watchdogExpired=true;render({...last,image_age_ms:Math.round(age)},bitmaps);}
},100);
poll();
</script></html>"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--fps', type=float, default=20, help='Preview limit; does not change tracking FPS')
    args = parser.parse_args()
    if not 1 <= args.fps <= 30 or not 1 <= args.port <= 65535:
        parser.error('fps must be 1..30; port must be 1..65535')
    import leap
    from leap.enums import PolicyFlag
    from leap.exceptions import LeapTimeoutError
    from leapc_cffi import ffi, libleapc

    lock = threading.Lock()
    frames = OrderedDict()
    pending = None
    bundle = dict(frame_id=None, cameras=[None, None], hands=[], matched=False, timestamp_us=None)
    counts = dict(frames=0, started=time.monotonic(), last_frame=None, error=None)
    tracking_times = deque(maxlen=240)
    policy_needed = threading.Event()
    stopping = threading.Event()
    calibration = {}
    connection = leap.Connection(poll_timeout=.1)

    def vector(v):
        return [float(v.x), float(v.y), float(v.z)]

    class Listener(leap.Listener):
        def on_tracking_event(self, event):
            hands = []
            for hand in event.hands:
                points = [vector(hand.arm.next_joint)]
                for name in FINGERS:
                    finger = getattr(hand, name)
                    points += [vector(finger.proximal.prev_joint), vector(finger.proximal.next_joint),
                               vector(finger.intermediate.next_joint), vector(finger.distal.next_joint)]
                hands.append(dict(side=hand.type.name, points_mm=points,
                                  tip_gap_mm=math.dist(points[4], points[8]),
                                  pinch_distance_mm=float(hand.pinch_distance),
                                  pinch_strength=float(hand.pinch_strength)))
            frame_id = int(event.c_data.info.frame_id)
            with lock:
                counts['frames'] += 1
                counts['last_frame'] = time.monotonic()
                tracking_times.append(counts['last_frame'])
                frames[frame_id] = hands
                while len(frames) > 120:
                    frames.popitem(last=False)

        def on_image_event(self, event):
            nonlocal pending
            now = time.monotonic()
            if now - getattr(self, 'last_image', 0) < 1/args.fps:
                return
            self.last_image = now
            cameras = []
            for camera, image in enumerate(event.image):
                data = image.c_data
                prop = data.properties
                if prop.bpp != 1:
                    return
                key = (camera, int(data.matrix_version))
                if key not in calibration:
                    dest = ffi.new('float[16]')
                    libleapc.LeapExtrinsicCameraMatrixByIndex(connection.get_connection_ptr(), camera, dest)
                    calibration[key] = list(dest)
                pixels = bytes(ffi.buffer(ffi.cast('char *', data.data) + data.offset,
                                          int(prop.width) * int(prop.height)))
                cameras.append(dict(width=int(prop.width), height=int(prop.height),
                                    pixels=pixels, matrix=calibration[key]))
            # All native buffers above are copied before the callback returns.
            with lock:
                pending = dict(frame_id=int(event.c_data.info.frame_id),
                               timestamp_us=int(event.c_data.info.timestamp),
                               cameras=cameras, received_at=now)

        def on_error(self, error):
            if not isinstance(error, LeapTimeoutError):
                with lock:
                    counts['error'] = str(error)

        def on_connection_lost_event(self, event):
            with lock:
                frames.clear()
                counts['error'] = 'Ultraleap 服务连接断开'

        def on_connection_event(self, event):
            with lock:
                counts['error'] = None
                calibration.clear()
            policy_needed.set()

    def publish():
        nonlocal bundle, pending
        while not stopping.wait(.008):
            if policy_needed.is_set():
                policy_needed.clear()
                try:
                    connection.set_policy_flags([PolicyFlag.Images])
                except Exception as exc:
                    with lock:
                        counts['error'] = str(exc)
            with lock:
                item = pending
                if item is None or time.monotonic() - item['received_at'] < .04:
                    continue
                pending = None
                hands = frames.get(item['frame_id'])
            # Only exact matching frames are overlaid; never reuse an old hand.
            projected = []
            for hand in hands or []:
                pixels = []
                for camera in range(2):
                    meta = item['cameras'][camera]
                    view = []
                    for point in hand['points_mm']:
                        p = camera_point([-point[0], point[1], point[2]], meta['matrix'])
                        # Desktop tracking +X is opposite raw image horizontal +X.
                        # Convert that basis before the stereo camera transform,
                        # so its baseline translation is not reflected. +Y is
                        # depth; +Z increases down the raw image.
                        if p[1] <= 0 or not all(math.isfinite(v) for v in p):
                            view.append(None)
                            continue
                        ray = ffi.new('LEAP_VECTOR *', {'x': p[0]/p[1], 'y': p[2]/p[1], 'z': 1})
                        pixel = libleapc.LeapRectilinearToPixelByIndex(connection.get_connection_ptr(), camera, ray[0])
                        view.append([float(pixel.x), float(pixel.y)]
                                    if math.isfinite(pixel.x) and math.isfinite(pixel.y) else None)
                    pixels.append(view)
                projected.append(dict(hand, pixels=pixels))
            cameras = [dict(width=c['width'], height=c['height'],
                            png=base64.b64encode(png_gray(c['width'], c['height'], c['pixels'])).decode('ascii'))
                       for c in item['cameras']]
            with lock:
                bundle = dict(frame_id=item['frame_id'], timestamp_us=item['timestamp_us'],
                              cameras=cameras, hands=projected, matched=hands is not None)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split('?')[0] == '/':
                payload, kind = HTML.encode('utf-8'), 'text/html;charset=utf-8'
            elif self.path.split('?')[0] in ('/frame', '/status'):
                with lock:
                    snapshot = dict(bundle)
                    stats = dict(counts)
                    recent = list(tracking_times)
                timestamp = snapshot.pop('timestamp_us')
                snapshot['image_age_ms'] = None if timestamp is None else max(0, round((leap.get_now()-timestamp)/1000))
                span = recent[-1]-recent[0] if len(recent)>1 else 0
                snapshot['tracking_fps'] = (len(recent)-1)/span if span else 0
                if stats['last_frame'] is None or time.monotonic()-stats['last_frame'] > .35:
                    snapshot['tracking_fps'] = 0
                snapshot['frames'] = stats['frames']
                snapshot['error'] = stats['error']
                if snapshot['image_age_ms'] is None or snapshot['image_age_ms'] > 350:
                    snapshot['hands'] = []
                if self.path.split('?')[0] == '/status':
                    snapshot['cameras'] = [{k:v for k,v in c.items() if k!='png'} if c else None
                                           for c in snapshot['cameras']]
                payload, kind = json.dumps(snapshot, allow_nan=False).encode(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    connection.add_listener(Listener())
    worker = threading.Thread(target=publish, daemon=True)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        connection.connect(timeout=3)
        connection.set_policy_flags([PolicyFlag.Images])
        policy_needed.clear()
        worker.start()
        print('Leap IR 21-point preview: http://127.0.0.1:'+str(args.port)+'/', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        stopping.set()
        if worker.is_alive():
            worker.join(timeout=2)
        connection.disconnect()


if __name__ == '__main__':
    main()
