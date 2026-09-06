#!/usr/bin/env python3
"""Professor showcase demo — polished live LD19 + camera perception.

Camera source is the Pi Zero 2W + IMX219-120 network stream by default (see
config/demo_hardware.yaml), with a directly-attached USB camera (e.g. the
PS3 Eye used during bench development) available as a fallback.

  python demo_show.py

Opens http://127.0.0.1:8000 with a presentation-ready dashboard.
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
import yaml
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

LATEST: dict = {}
START = time.monotonic()
CLIENTS: set[WebSocket] = set()
ENGINE = None
LOGGER: DemoLogger | None = None
ARGS: argparse.Namespace | None = None


SHOW_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>AMP · Live Multimodal Perception</title>
<link rel="preconnect" href="https://fonts.googleapis.com"/>
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600;700&display=swap" rel="stylesheet"/>
<style>
:root{
  --bg:#071018; --panel:rgba(16,28,38,.88); --line:rgba(120,170,190,.18);
  --text:#eef5f8; --muted:#8aa3b3; --ok:#3dd68c; --accent:#3ec6d8; --warn:#f0c35a; --bad:#ff6b6b;
}
*{box-sizing:border-box}
body{margin:0;color:var(--text);font-family:"IBM Plex Sans",Segoe UI,sans-serif;
background:
  radial-gradient(1000px 520px at 10% -20%, #163247 0%, transparent 55%),
  radial-gradient(800px 480px at 90% 0%, #1a2a22 0%, transparent 50%),
  linear-gradient(180deg,#081018,#0a141c 40%,#071018);
min-height:100vh}
header{display:flex;justify-content:space-between;gap:16px;align-items:flex-start;padding:18px 22px 10px;border-bottom:1px solid var(--line)}
.brand h1{margin:0;font-size:clamp(1.35rem,2.2vw,1.85rem);font-weight:700;letter-spacing:.01em}
.brand h1 span{color:var(--accent)}
.brand p{margin:6px 0 0;color:var(--muted);font-size:.95rem;max-width:46rem;line-height:1.55}
.pills{display:flex;flex-wrap:wrap;gap:8px;justify-content:flex-end}
.pill{background:var(--panel);border:1px solid var(--line);padding:8px 12px;border-radius:999px;font-size:.86rem;font-weight:600;backdrop-filter:blur(8px)}
.pill.ok{color:var(--ok);box-shadow:0 0 0 1px rgba(61,214,140,.15)}
.pill.warn{color:var(--warn)}
main{display:grid;grid-template-columns:1.35fr .95fr;gap:14px;padding:14px 18px;max-width:1600px;margin:0 auto}
@media(max-width:1100px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:18px;overflow:hidden;backdrop-filter:blur(10px);box-shadow:0 10px 40px rgba(0,0,0,.25)}
.card h2{margin:0;padding:12px 16px;font-size:.78rem;letter-spacing:.12em;text-transform:uppercase;color:var(--muted);border-bottom:1px solid var(--line)}
.viewport{position:relative;background:#05090d;min-height:360px}
#cam{width:100%;display:block;aspect-ratio:4/3;object-fit:contain;background:#000}
#radar{width:100%;height:100%;min-height:360px;display:block}
.hero-metric{position:absolute;left:14px;bottom:14px;background:rgba(5,12,18,.72);border:1px solid var(--line);border-radius:14px;padding:12px 14px;min-width:180px;backdrop-filter:blur(8px)}
.hero-metric .label{color:var(--muted);font-size:.78rem}
.hero-metric .value{font-size:1.85rem;font-weight:700;color:var(--accent);line-height:1.1;margin-top:2px}
.hero-metric .sub{color:var(--muted);font-size:.8rem;margin-top:4px}
.bottom{display:grid;grid-template-columns:1.1fr .9fr .9fr;gap:14px;padding:0 18px 20px;max-width:1600px;margin:0 auto}
@media(max-width:1100px){.bottom{grid-template-columns:1fr}}
table{width:100%;border-collapse:collapse;font-size:.95rem}
th,td{padding:11px 10px;border-bottom:1px solid var(--line);text-align:left}
th{color:var(--muted);font-size:.75rem;letter-spacing:.08em;text-transform:uppercase}
td.num{font-variant-numeric:tabular-nums}
.note{padding:14px 16px;color:var(--muted);font-size:.9rem;line-height:1.7}
.note b{color:var(--text)}
.oktxt{color:var(--ok)}.badtxt{color:var(--bad)}.warntxt{color:var(--warn)}
.footer{padding:0 22px 22px;color:var(--muted);font-size:.8rem;max-width:1600px;margin:0 auto}
</style>
</head>
<body>
<header>
  <div class="brand">
    <h1>AMP <span>Live Perception</span></h1>
    <p>Adaptive multimodal perception for low-cost indoor robots — live LD19 LiDAR + Pi Zero 2W/IMX219-120 camera fusion with object detection and real distance estimation.</p>
  </div>
  <div class="pills" id="pills"><div class="pill">Starting…</div></div>
</header>
<main>
  <section class="card">
    <h2>Camera · LiDAR Overlay · Detections</h2>
    <div class="viewport">
      <img id="cam" alt="camera"/>
      <div class="hero-metric">
        <div class="label">Closest obstacle</div>
        <div class="value" id="closest">—</div>
        <div class="sub" id="closestSub">Waiting for LiDAR</div>
      </div>
    </div>
  </section>
  <section class="card">
    <h2>LiDAR Radar · Top View</h2>
    <div class="viewport"><canvas id="radar"></canvas></div>
  </section>
</main>
<div class="bottom">
  <section class="card">
    <h2>Objects · Distance from LiDAR-in-Box</h2>
    <div style="overflow:auto;max-height:300px;padding:0 6px 8px">
      <table>
        <thead><tr><th>Class</th><th>Conf</th><th>Distance</th><th>Bearing</th><th>LiDAR pts</th><th>Fusion</th><th>Status</th></tr></thead>
        <tbody id="tbody"></tbody>
      </table>
    </div>
  </section>
  <section class="card">
    <h2>Calibration · Research Honesty</h2>
    <div class="note" id="calib">…</div>
  </section>
  <section class="card">
    <h2>System Health</h2>
    <div class="note" id="sys">…</div>
  </section>
</div>
<div class="footer">Adaptive Multimodal Perception · Real sensors only · No mock LiDAR · Practical field extrinsics (not formal multi-pose metrology)</div>
<script>
const cam=document.getElementById('cam');
const radar=document.getElementById('radar');
const tbody=document.getElementById('tbody');
const pills=document.getElementById('pills');
const calib=document.getElementById('calib');
const sys=document.getElementById('sys');
const closest=document.getElementById('closest');
const closestSub=document.getElementById('closestSub');

function fit(){
  const parent=radar.parentElement.getBoundingClientRect();
  radar.width=parent.width*devicePixelRatio;
  radar.height=parent.height*devicePixelRatio;
  return radar.getContext('2d');
}
function drawRadar(d){
  const ctx=fit(); const W=radar.width/devicePixelRatio, H=radar.height/devicePixelRatio;
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  const g=ctx.createRadialGradient(W/2,H/2,10,W/2,H/2,Math.min(W,H)/2);
  g.addColorStop(0,'#0b1820'); g.addColorStop(1,'#05090d');
  ctx.fillStyle=g; ctx.fillRect(0,0,W,H);
  const cx=W/2, cy=H*0.58, scale=Math.min(W,H)/(2*4.2);
  for(const m of [1,2,3,4]){
    ctx.beginPath(); ctx.arc(cx,cy,m*scale,Math.PI,0); ctx.strokeStyle='rgba(120,170,190,.18)'; ctx.stroke();
    ctx.fillStyle='rgba(143,163,179,.55)'; ctx.font='12px IBM Plex Sans'; ctx.fillText(m+'m', cx+m*scale-10, cy+16);
  }
  ctx.strokeStyle='rgba(62,198,216,.25)'; ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(cx,cy-4*scale); ctx.stroke();
  (d.lidar_xy||[]).forEach(p=>{
    const x=cx+p[1]*scale, y=cy-p[0]*scale;
    const t=Math.max(0,Math.min(1,p[2]/4));
    ctx.fillStyle=`rgba(${Math.floor(80+160*t)},${Math.floor(200-80*t)},${Math.floor(220-40*t)},0.95)`;
    ctx.fillRect(x,y,2.2,2.2);
  });
  if(d.closest_m!=null){
    ctx.strokeStyle='rgba(255,107,107,.85)'; ctx.setLineDash([4,4]);
    ctx.beginPath(); ctx.arc(cx,cy,d.closest_m*scale,Math.PI,0); ctx.stroke(); ctx.setLineDash([]);
  }
  ctx.fillStyle='#f0c35a'; ctx.beginPath(); ctx.arc(cx,cy,5,0,Math.PI*2); ctx.fill();
}
function fmt(m){ return m==null? '—' : m.toFixed(2)+' m'; }
let frameTick=0;
function render(d){
  // Image via HTTP — avoids huge base64 JSON that freezes/closes the WebSocket
  cam.src='/frame.jpg?t='+(++frameTick);
  closest.textContent = d.closest_m==null? '—' : d.closest_m.toFixed(2)+' m';
  closestSub.textContent = d.objects?.length? (d.objects.length+' object(s) detected') : 'No detections';
  tbody.innerHTML='';
  (d.objects||[]).forEach(o=>{
    const ok = o.distance_m!=null && o.lidar_points>=3;
    const st = ok? '<span class="oktxt">OK</span>' : '<span class="warntxt">weak</span>';
    const fus = o.fusion? String(o.fusion).replace('bearing_cluster_','') : '—';
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${o.class}</td><td class="num">${o.confidence.toFixed(2)}</td><td class="num"><b>${fmt(o.distance_m)}</b></td><td class="num">${o.bearing_deg.toFixed(1)}°</td><td class="num">${o.lidar_points}</td><td class="num">${fus}</td><td>${st}</td>`;
    tbody.appendChild(tr);
  });
  pills.innerHTML=`
    <div class="pill ${d.camera_connected?'ok':'err'}">CAM ${d.camera_connected?'LIVE':'RECONNECT'} ${d.camera_fps.toFixed(1)} FPS</div>
    <div class="pill ok">LIDAR ${d.lidar_hz.toFixed(1)} Hz</div>
    <div class="pill">YOLO ${d.detect_ms.toFixed(0)} ms</div>
    <div class="pill">CPU ${d.cpu_percent.toFixed(0)}%</div>
    <div class="pill">PROJ ${d.projected_count}</div>`;
  calib.innerHTML = (d.calib_html||'');
  sys.innerHTML = `
    <b>Experiment</b>: ${d.experiment_id||''}<br/>
    <b>Detector</b>: ${d.detector}<br/>
    <b>Camera source</b>: ${d.camera_source}<br/>
    <b>LiDAR port</b>: ${d.lidar_port}<br/>
    <b>Uptime</b>: ${d.uptime_s.toFixed(0)} s<br/>
    <b>Intrinsics RMS</b>: ${d.intrinsics_rms??'—'} px
  `;
  drawRadar(d);
}
let ws=null;
let pingTimer=null;
function connectWs(){
  if(pingTimer){ clearInterval(pingTimer); pingTimer=null; }
  ws=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws');
  ws.onopen=()=>{
    pills.innerHTML='<div class="pill ok">Connected</div>';
    pingTimer=setInterval(()=>{ if(ws && ws.readyState===1) ws.send('ping'); }, 15000);
  };
  ws.onmessage=(ev)=>{
    try{ render(JSON.parse(ev.data)); }
    catch(e){ console.warn('bad ws payload', e); }
  };
  ws.onclose=()=>{
    pills.innerHTML='<div class="pill warn">Reconnecting…</div>';
    if(pingTimer){ clearInterval(pingTimer); pingTimer=null; }
    setTimeout(connectWs, 1000);
  };
  ws.onerror=()=>{ try{ ws.close(); }catch(_){} };
}
connectWs();
</script>
</body>
</html>
"""


class ShowcaseEngine:
    def __init__(
        self,
        camera_index: int | None,
        camera_url: str | None,
        lidar_port: str,
        intrinsics: str,
        extrinsics: str,
    ) -> None:
        self.intrinsics_path = Path(intrinsics)
        self.extrinsics_path = Path(extrinsics)
        self.camera_index = camera_index
        self.lidar_port = lidar_port
        self.perception = DemoPerception(
            camera_index=camera_index,
            camera_url=camera_url,
            lidar_port=lidar_port,
            intrinsics_path=intrinsics,
            extrinsics_path=extrinsics,
        )
        self.intrinsics_rms = None
        if self.intrinsics_path.exists():
            data = yaml.safe_load(self.intrinsics_path.read_text(encoding="utf-8"))
            self.intrinsics_rms = float(
                data.get("mean_reprojection_error_px", data.get("rms_opencv", 0.0))
            )
        self.ext_data = {}
        if self.extrinsics_path.exists():
            self.ext_data = (
                yaml.safe_load(self.extrinsics_path.read_text(encoding="utf-8")) or {}
            )

    @property
    def detector(self):
        return self.perception.detector

    def close(self) -> None:
        self.perception.close()

    def calib_html(self) -> str:
        e = self.ext_data
        method = e.get("method", "practical_field_calibration")
        metrics = e.get("metrics") or {}
        range_err = metrics.get("range_err_m")
        frac = metrics.get("frac_inside")
        bits = [
            (
                f"<b>Intrinsics</b>: chessboard RMS <span class='oktxt'>{self.intrinsics_rms:.3f} px</span>"
                if self.intrinsics_rms is not None
                else "<b>Intrinsics</b>: missing"
            ),
            f"<b>Extrinsics</b>: {method}",
            (
                f"t=({e.get('x', 0):.3f}, {e.get('y', 0):.3f}, {e.get('z', 0):.3f}) m · "
                f"rpy=({e.get('roll_deg', 0):.1f}, {e.get('pitch_deg', 0):.1f}, {e.get('yaw_deg', 0):.1f})°"
            ),
            "<b>Distance</b>: bearing-gated LiDAR fusion + person height prior (to 4.5 m)",
        ]
        if range_err is not None:
            cls = (
                "oktxt"
                if range_err <= 0.08
                else "warntxt"
                if range_err <= 0.15
                else "badtxt"
            )
            bits.append(
                f"<b>Box-target residual</b>: <span class='{cls}'>{range_err:.3f} m</span>"
                f" · overlay inside {(frac or 0) * 100:.0f}%"
            )
        bits.append(
            "<span class='warntxt'>Honest scope</span>: practical field calibration "
            "(not formal multi-pose hand-eye metrology)."
        )
        return "<br/>".join(bits)

    def step(self) -> dict:
        snap = self.perception.step()
        return {
            "jpeg": snap.jpeg,
            "objects": snap.objects_dict(),
            "lidar_xy": snap.lidar_xy,
            "closest_m": snap.closest_m,
            "camera_fps": snap.camera_fps,
            "camera_age_s": snap.camera_age_s,
            "camera_connected": snap.camera_connected,
            "lidar_hz": snap.lidar_hz,
            "detect_ms": snap.detect_ms,
            "projected_count": snap.projected_count,
        }


@asynccontextmanager
async def lifespan(app: FastAPI):
    global ENGINE, LOGGER
    assert ARGS is not None
    LOGGER = DemoLogger(prefix="SHOW")
    ENGINE = ShowcaseEngine(
        ARGS.camera, ARGS.camera_url, ARGS.lidar_port, ARGS.intrinsics, ARGS.extrinsics
    )
    LOGGER.update_context(
        camera_source=type(ENGINE.perception.camera).__name__,
        camera_url=getattr(ENGINE.perception.camera, "url", None),
        lidar_port=ENGINE.perception.lidar.port,
        detector=ENGINE.detector.name(),
        intrinsics=ARGS.intrinsics,
        extrinsics=ARGS.extrinsics,
    )
    print("SHOWCASE experiment:", LOGGER.dir.resolve())
    print("Detector:", ENGINE.detector.name())
    task = asyncio.create_task(_loop())
    yield
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    ENGINE.close()
    LOGGER.close()


app = FastAPI(title="AMP Showcase", lifespan=lifespan)


async def _loop() -> None:
    assert ENGINE is not None and LOGGER is not None
    while True:
        t0 = time.perf_counter()
        try:
            snap = await asyncio.to_thread(ENGINE.step)
        except Exception as exc:  # noqa: BLE001
            print("step error:", exc)
            await asyncio.sleep(0.25)
            continue
        # Keep JPEG out of WebSocket (browser freeze / disconnect on huge JSON)
        jpeg = snap["jpeg"] or b""
        payload = {
            "objects": snap["objects"],
            "lidar_xy": [
                [round(x, 3), round(y, 3), round(r, 3)] for x, y, r in snap["lidar_xy"]
            ],
            "closest_m": None
            if snap["closest_m"] is None
            else round(snap["closest_m"], 3),
            "camera_fps": round(snap["camera_fps"], 2),
            "camera_age_s": round(snap["camera_age_s"], 3),
            "camera_connected": snap["camera_connected"],
            "lidar_hz": round(snap["lidar_hz"], 2),
            "detect_ms": round(snap["detect_ms"], 1),
            "cpu_percent": psutil.cpu_percent(interval=None),
            "uptime_s": time.monotonic() - START,
            "projected_count": snap["projected_count"],
            "experiment_id": LOGGER.exp_id,
            "detector": ENGINE.detector.name(),
            "camera_source": type(ENGINE.perception.camera).__name__,
            "lidar_port": ENGINE.lidar_port,
            "intrinsics_rms": ENGINE.intrinsics_rms,
            "calib_html": ENGINE.calib_html(),
            "has_frame": bool(jpeg),
        }
        LATEST.clear()
        LATEST.update(payload)
        LATEST["jpeg_bytes"] = jpeg
        try:
            LOGGER.log(
                {
                    "objects": payload["objects"],
                    "closest_m": payload["closest_m"],
                    "camera_fps": payload["camera_fps"],
                    "camera_age_s": payload["camera_age_s"],
                    "camera_connected": payload["camera_connected"],
                    "lidar_hz": payload["lidar_hz"],
                    "detect_ms": payload["detect_ms"],
                    "projected_count": payload["projected_count"],
                }
            )
        except Exception as exc:  # noqa: BLE001
            print("log error:", exc)
        dead = []
        for ws in list(CLIENTS):
            try:
                await ws.send_json(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            CLIENTS.discard(ws)
        await asyncio.sleep(max(0.0, 0.12 - (time.perf_counter() - t0)))


@app.get("/", response_class=HTMLResponse)
def index() -> HTMLResponse:
    return HTMLResponse(SHOW_HTML)


@app.get("/frame.jpg")
def frame() -> Response:
    data = LATEST.get("jpeg_bytes")
    if not data:
        return Response(status_code=503)
    return Response(
        content=data,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    CLIENTS.add(ws)
    try:
        # Broadcast-only socket: stay open until client disconnects.
        # Timeouts must NOT close the connection (browser may send nothing).
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
    p = argparse.ArgumentParser(description="AMP professor showcase demo")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument(
        "--camera",
        type=int,
        default=None,
        help="Force USB camera.mode with this OpenCV index (overrides config/demo_hardware.yaml)",
    )
    p.add_argument(
        "--camera-url",
        default=None,
        help="Force network camera.mode with this MJPEG URL (overrides config/demo_hardware.yaml)",
    )
    p.add_argument("--lidar-port", default=default_lidar_port())
    p.add_argument("--intrinsics", default=DEFAULT_INTRINSICS_PATH)
    p.add_argument("--extrinsics", default=DEFAULT_EXTRINSICS_PATH)
    ARGS = p.parse_args()
    print(f"Showcase dashboard: http://{ARGS.host}:{ARGS.port}")
    uvicorn.run(app, host=ARGS.host, port=ARGS.port, log_level="info")


if __name__ == "__main__":
    main()
