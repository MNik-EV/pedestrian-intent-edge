#!/usr/bin/env python3
"""
One-command live demo: real LD19 + Pi/IMX219 camera + YOLO + dashboard.

  python app.py

No mocks. No SLAM. No ROS2. No navigation.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from contextlib import asynccontextmanager
from pathlib import Path

import psutil
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from demo.defaults import (
    DEFAULT_EXTRINSICS_PATH,
    DEFAULT_INTRINSICS_PATH,
    default_lidar_port,
)
from demo.logger import DemoLogger
from demo.perception import DemoPerception

PERCEPTION: DemoPerception | None = None
LOGGER: DemoLogger | None = None
LATEST: dict = {}
START_MONO = time.monotonic()
CLIENTS: set[WebSocket] = set()
ARGS: argparse.Namespace | None = None


DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>AMP Live Demo</title>
<style>
:root{--bg:#0b1014;--panel:#141c22;--line:#24303a;--text:#e8eef3;--muted:#8fa3b3;--ok:#3ecf8e;--accent:#2eb7c9;--warn:#e6b84d}
*{box-sizing:border-box} body{margin:0;background:radial-gradient(900px 500px at 15% -10%,#1a2830,var(--bg));
color:var(--text);font-family:Segoe UI,Arial,sans-serif}
header{display:flex;justify-content:space-between;align-items:center;padding:14px 18px;border-bottom:1px solid var(--line);position:sticky;top:0;background:rgba(11,16,20,.9);backdrop-filter:blur(8px);z-index:5}
.brand{font-size:1.25rem;font-weight:700;letter-spacing:.02em}.brand span{color:var(--accent)}
.pills{display:flex;gap:8px;flex-wrap:wrap}.pill{background:var(--panel);border:1px solid var(--line);padding:8px 12px;border-radius:8px;font-size:14px;font-weight:600}
.pill.ok{color:var(--ok)}.pill.warn{color:var(--warn)}
main{display:grid;grid-template-columns:1.2fr 1fr;gap:14px;padding:14px;max-width:1400px;margin:0 auto}
@media(max-width:980px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;overflow:hidden;min-height:340px;display:flex;flex-direction:column}
.card h2{margin:0;padding:12px 14px;font-size:13px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);border-bottom:1px solid var(--line)}
.viewport{flex:1;background:#070b0e;position:relative;min-height:300px}
img#cam{width:100%;height:100%;object-fit:contain;display:block;background:#000}
canvas#radar{width:100%;height:100%;display:block}
.bottom{display:grid;grid-template-columns:1.2fr .8fr;gap:14px;padding:0 14px 18px;max-width:1400px;margin:0 auto}
@media(max-width:980px){.bottom{grid-template-columns:1fr}}
table{width:100%;border-collapse:collapse;font-size:15px}
th,td{padding:10px 8px;border-bottom:1px solid var(--line);text-align:left}
th{color:var(--muted);font-size:12px;text-transform:uppercase;letter-spacing:.06em}
.note{padding:12px 14px;color:var(--muted);font-size:13px;line-height:1.45}
</style>
</head>
<body>
<header>
  <div class="brand">AMP <span>LIVE DEMO</span> · LD19 + Camera</div>
  <div class="pills" id="pills"><div class="pill">BOOT…</div></div>
</header>
<main>
  <section class="card"><h2>Annotated Camera (LiDAR overlay + objects)</h2>
    <div class="viewport"><img id="cam" alt="camera"/></div>
  </section>
  <section class="card"><h2>LiDAR Radar</h2>
    <div class="viewport"><canvas id="radar"></canvas></div>
  </section>
</main>
<div class="bottom">
  <section class="card" style="min-height:220px"><h2>Objects</h2>
    <div style="overflow:auto;max-height:280px;padding:0 8px">
      <table><thead><tr><th>Class</th><th>Conf</th><th>Distance</th><th>Bearing</th><th>LiDAR pts</th></tr></thead>
      <tbody id="tbody"></tbody></table>
    </div>
  </section>
  <section class="card" style="min-height:220px"><h2>System / Calib Notes</h2>
    <div class="note" id="notes">Connecting…</div>
  </section>
</div>
<script>
const cam=document.getElementById('cam');
const radar=document.getElementById('radar');
const tbody=document.getElementById('tbody');
const pills=document.getElementById('pills');
const notes=document.getElementById('notes');
let latest=null;

function fit(){
  const r=radar.parentElement.getBoundingClientRect();
  radar.width=r.width*devicePixelRatio; radar.height=r.height*devicePixelRatio;
  return radar.getContext('2d');
}
function drawRadar(d){
  const ctx=fit(); const W=radar.width/devicePixelRatio, H=radar.height/devicePixelRatio;
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  ctx.fillStyle='#070b0e'; ctx.fillRect(0,0,W,H);
  const cx=W/2, cy=H/2, scale=Math.min(W,H)/(2*5);
  [1,2,3,4].forEach(m=>{ctx.beginPath();ctx.arc(cx,cy,m*scale,0,Math.PI*2);ctx.strokeStyle='#24303a';ctx.stroke();});
  (d.lidar_xy||[]).forEach(p=>{
    ctx.fillStyle='#9fd7e0';
    ctx.fillRect(cx+p[0]*scale, cy-p[1]*scale, 2, 2);
  });
  if(d.closest_m!=null){
    ctx.strokeStyle='#e85d5d'; ctx.beginPath(); ctx.arc(cx,cy,d.closest_m*scale,0,Math.PI*2); ctx.stroke();
  }
  ctx.fillStyle='#c9a227'; ctx.beginPath(); ctx.arc(cx,cy,5,0,Math.PI*2); ctx.fill();
  ctx.fillStyle='#8fa3b3'; ctx.font='14px Segoe UI';
  ctx.fillText('closest: '+(d.closest_m!=null?d.closest_m.toFixed(2)+' m':'—'), 12, 22);
}
function render(d){
  cam.src='/frame.jpg?t='+Date.now();
  tbody.innerHTML='';
  (d.objects||[]).forEach(o=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${o.class}</td><td>${o.confidence.toFixed(2)}</td><td>${o.distance_m!=null?o.distance_m.toFixed(2)+' m':'—'}</td><td>${o.bearing_deg.toFixed(1)}°</td><td>${o.lidar_points}</td>`;
    tbody.appendChild(tr);
  });
  pills.innerHTML=`
    <div class="pill ${d.camera_connected?'ok':'err'}">CAM ${d.camera_connected?'LIVE':'RECONNECT'} ${d.camera_fps.toFixed(1)} FPS</div>
    <div class="pill ok">LIDAR ${d.lidar_hz.toFixed(1)} Hz</div>
    <div class="pill">CPU ${d.cpu_percent.toFixed(0)}%</div>
    <div class="pill">DET ${d.detect_ms.toFixed(0)} ms</div>
    <div class="pill">UP ${d.uptime_s.toFixed(0)}s</div>`;
  notes.innerHTML = (d.calib_notes||[]).map(x=>'• '+x).join('<br/>') +
    `<br/>experiment: <b>${d.experiment_id||''}</b>` +
    `<br/>detector: ${d.detector||''}`;
  drawRadar(d);
}
function connectWs(){
  const ws=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws');
  ws.onmessage=(ev)=>{ try{ render(JSON.parse(ev.data)); }catch(e){} };
  ws.onclose=()=>setTimeout(connectWs, 1000);
  setInterval(()=>{ if(ws.readyState===1) ws.send('ping'); }, 15000);
}
connectWs();
</script>
</body>
</html>
"""


async def _loop() -> None:
    assert PERCEPTION is not None and LOGGER is not None
    while True:
        t0 = time.perf_counter()
        try:
            snap = await asyncio.to_thread(PERCEPTION.step)
        except Exception as exc:  # noqa: BLE001
            print("step error:", exc)
            await asyncio.sleep(0.25)
            continue
        cpu = psutil.cpu_percent(interval=None)
        payload = {
            "objects": snap.objects_dict(),
            "lidar_xy": [
                [round(x, 3), round(y, 3), round(r, 3)] for x, y, r in snap.lidar_xy
            ],
            "closest_m": None if snap.closest_m is None else round(snap.closest_m, 3),
            "camera_fps": round(snap.camera_fps, 2),
            "camera_age_s": round(snap.camera_age_s, 3),
            "camera_connected": snap.camera_connected,
            "lidar_hz": round(snap.lidar_hz, 2),
            "detect_ms": round(snap.detect_ms, 1),
            "cpu_percent": cpu,
            "uptime_s": time.monotonic() - START_MONO,
            "projected_count": snap.projected_count,
            "calib_notes": snap.calib_notes,
            "experiment_id": LOGGER.exp_id,
            "detector": PERCEPTION.detector.name(),
        }
        LATEST.clear()
        LATEST.update(payload)
        LATEST["jpeg_bytes"] = snap.jpeg or b""
        LOGGER.log(
            {
                "objects": payload["objects"],
                "closest_m": payload["closest_m"],
                "camera_fps": payload["camera_fps"],
                "camera_age_s": payload["camera_age_s"],
                "camera_connected": payload["camera_connected"],
                "lidar_hz": payload["lidar_hz"],
                "detect_ms": payload["detect_ms"],
                "cpu_percent": payload["cpu_percent"],
                "projected_count": payload["projected_count"],
            }
        )
        dead = []
        for ws in list(CLIENTS):
            try:
                await ws.send_json(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            CLIENTS.discard(ws)
        # Aim ~8–10 Hz dashboard
        elapsed = time.perf_counter() - t0
        await asyncio.sleep(max(0.0, 0.12 - elapsed))


@asynccontextmanager
async def lifespan(app: FastAPI):
    global PERCEPTION, LOGGER
    args = ARGS
    assert args is not None
    print("Starting REAL sensors (no mocks)...")
    LOGGER = DemoLogger()
    print("Experiment folder:", LOGGER.dir.resolve())
    PERCEPTION = DemoPerception(
        camera_index=args.camera,
        camera_url=args.camera_url,
        lidar_port=args.lidar_port,
        intrinsics_path=args.intrinsics,
        extrinsics_path=args.extrinsics,
    )
    LOGGER.update_context(
        camera_source=type(PERCEPTION.camera).__name__,
        camera_url=getattr(PERCEPTION.camera, "url", None),
        lidar_port=PERCEPTION.lidar.port,
        detector=PERCEPTION.detector.name(),
        intrinsics=args.intrinsics,
        extrinsics=args.extrinsics,
    )
    print("Detector:", PERCEPTION.detector.name())
    task = asyncio.create_task(_loop())
    yield
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    if PERCEPTION:
        PERCEPTION.close()
    if LOGGER:
        LOGGER.close()


app = FastAPI(title="AMP Live Demo", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
def index() -> HTMLResponse:
    return HTMLResponse(DASHBOARD_HTML)


@app.get("/api/status")
def status() -> dict:
    return {
        "ok": bool(LATEST),
        "experiment_id": None if LOGGER is None else LOGGER.exp_id,
        "uptime_s": time.monotonic() - START_MONO,
        "latest_keys": list(LATEST.keys()),
    }


@app.get("/frame.jpg")
def frame_jpg() -> Response:
    data = LATEST.get("jpeg_bytes")
    if not data:
        return Response(status_code=503)
    return Response(
        content=data,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    CLIENTS.add(ws)
    try:
        while True:
            try:
                msg = await asyncio.wait_for(ws.receive(), timeout=30.0)
            except asyncio.TimeoutError:
                continue
            if msg.get("type") == "websocket.disconnect":
                break
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        CLIENTS.discard(ws)


def main() -> None:
    global ARGS
    parser = argparse.ArgumentParser(description="AMP live hardware demo")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--camera",
        type=int,
        default=None,
        help="Force USB camera.mode with this OpenCV index (overrides config/demo_hardware.yaml)",
    )
    parser.add_argument(
        "--camera-url",
        default=None,
        help="Force network camera.mode with this MJPEG URL (overrides config/demo_hardware.yaml, "
        "e.g. http://raspberrypi.local:8000/stream.mjpg)",
    )
    parser.add_argument(
        "--lidar-port",
        default=default_lidar_port(),
        help="Serial port (default from config/demo_hardware.yaml, usually COM17)",
    )
    parser.add_argument("--intrinsics", default=DEFAULT_INTRINSICS_PATH)
    parser.add_argument("--extrinsics", default=DEFAULT_EXTRINSICS_PATH)
    ARGS = parser.parse_args()
    print(f"Dashboard: http://{ARGS.host}:{ARGS.port}")
    uvicorn.run(app, host=ARGS.host, port=ARGS.port, log_level="info")


if __name__ == "__main__":
    main()
