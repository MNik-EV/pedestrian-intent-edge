"""AMP Robot FastAPI + WebSocket dashboard backend."""

from __future__ import annotations

import asyncio
import os
import time
from contextlib import asynccontextmanager
from typing import Any

from fastapi import (
    Depends,
    FastAPI,
    Header,
    HTTPException,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

from amp_core import __version__
from amp_core.common.types import FusionMode
from amp_core.pipeline import AmpPipeline, PipelineConfig

# ---------------------------------------------------------------------------
# Global pipeline (Pi runtime or PC mock)
# ---------------------------------------------------------------------------
PIPELINE: AmpPipeline | None = None
TELEMETRY_HZ = 10.0
CONTROL_TOKEN = os.environ.get("AMP_CONTROL_TOKEN", "dev-token-change-me")


def get_pipeline() -> AmpPipeline:
    global PIPELINE
    if PIPELINE is None:
        mode = os.environ.get("AMP_FUSION_MODE", "adaptive_fusion")
        force_mock = os.environ.get("AMP_FORCE_MOCK", "0") == "1"
        PIPELINE = AmpPipeline(
            PipelineConfig(
                fusion_mode=FusionMode(mode),
                dynamic_filtering=os.environ.get("AMP_DYN_FILTER", "1") == "1",
                enable_navigation=True,
                detector_backend=os.environ.get("AMP_DETECTOR", "auto"),
                force_mock_camera=force_mock,
                force_mock_lidar=force_mock,
            )
        )
    return PIPELINE


class GoalRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class CmdVelRequest(BaseModel):
    linear: float = Field(..., ge=-1.0, le=1.0)
    angular: float = Field(..., ge=-1.5, le=1.5)


class ExperimentRequest(BaseModel):
    name: str = "manual"
    scenario: str = "normal"


def require_control(authorization: str | None = Header(default=None)) -> None:
    if os.environ.get("AMP_AUTH_DISABLED", "0") == "1":
        return
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing Bearer token")
    token = authorization.split(" ", 1)[1].strip()
    if token != CONTROL_TOKEN:
        raise HTTPException(status_code=403, detail="Invalid control token")


@asynccontextmanager
async def lifespan(app: FastAPI):
    get_pipeline()
    task = asyncio.create_task(_tick_loop())
    yield
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    try:
        get_pipeline().close()
    except Exception:
        pass


app = FastAPI(
    title="AMP Robot API",
    version=__version__,
    description="Adaptive Multimodal Perception robot control & telemetry API",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

_latest: dict[str, Any] = {}
_clients: set[WebSocket] = set()
_rate_bucket: dict[str, float] = {}


async def _tick_loop() -> None:
    pipe = get_pipeline()
    period = 1.0 / TELEMETRY_HZ
    while True:
        t0 = time.perf_counter()
        snap = await asyncio.to_thread(pipe.step)
        _latest.clear()
        _latest.update(snap.to_dict())
        dead: list[WebSocket] = []
        for ws in list(_clients):
            try:
                await ws.send_json({"channel": "telemetry", "data": _latest})
            except Exception:
                dead.append(ws)
        for ws in dead:
            _clients.discard(ws)
        elapsed = time.perf_counter() - t0
        await asyncio.sleep(max(0.0, period - elapsed))


def _rate_limit(key: str, min_interval: float = 0.05) -> None:
    now = time.monotonic()
    last = _rate_bucket.get(key, 0.0)
    if now - last < min_interval:
        raise HTTPException(status_code=429, detail="Rate limited")
    _rate_bucket[key] = now


@app.get("/api/status")
def api_status() -> dict[str, Any]:
    pipe = get_pipeline()
    return {
        "status": "ok",
        "version": __version__,
        "fusion_mode": pipe.ekf.cfg.mode.value,
        "dynamic_filtering": pipe.cfg.dynamic_filtering,
        "has_telemetry": bool(_latest),
        "camera_live": pipe.camera_live,
        "lidar_live": pipe.lidar_live,
        "camera_source": "local_usb"
        if pipe.camera_live
        else ("mock" if pipe.camera_mock else "none"),
        "lidar_source": "mock" if pipe.lidar_mock else "none",
        "detector": pipe.detector.name(),
    }


@app.get("/api/hardware")
def api_hardware() -> dict[str, Any]:
    pipe = get_pipeline()
    return {
        "camera_live": pipe.camera_live,
        "lidar_live": pipe.lidar_live,
        "camera_source": "local_usb"
        if pipe.camera_live
        else ("mock" if pipe.camera_mock else "none"),
        "lidar_source": "mock" if pipe.lidar_mock else "none",
        "detector": pipe.detector.name(),
        "inventory": pipe.inventory.to_dict(),
        "recommendations": pipe.inventory.recommendations,
    }


@app.get("/api/robot")
def api_robot() -> dict[str, Any]:
    return {
        "pose": _latest.get("pose"),
        "twist": _latest.get("twist"),
        "safety": _latest.get("safety"),
    }


@app.get("/api/sensors")
def api_sensors() -> dict[str, Any]:
    return {
        "confidence": _latest.get("confidence"),
        "sectors": _latest.get("sectors"),
        "camera": _latest.get("camera"),
        "system": _latest.get("system"),
    }


@app.get("/api/objects")
def api_objects() -> dict[str, Any]:
    return {"objects": _latest.get("objects", [])}


@app.get("/api/map")
def api_map() -> dict[str, Any]:
    return {"map": _latest.get("map")}


@app.get("/api/logs")
def api_logs() -> dict[str, Any]:
    return {"events": _latest.get("events", [])}


@app.get("/api/experiments")
def api_experiments() -> dict[str, Any]:
    root = os.path.join(os.path.dirname(os.path.dirname(__file__)), "..", "experiments")
    root = os.path.abspath(root)
    ids = []
    if os.path.isdir(root):
        ids = sorted(
            [d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]
        )
    return {"experiments": ids}


@app.post("/api/navigation/goal")
def api_goal(req: GoalRequest, _: None = Depends(require_control)) -> dict[str, str]:
    _rate_limit("goal")
    get_pipeline().set_goal(req.x, req.y, req.yaw)
    get_pipeline().events.append(f"goal_set:{req.x:.2f},{req.y:.2f}")
    return {"status": "accepted"}


@app.post("/api/navigation/stop")
def api_stop(_: None = Depends(require_control)) -> dict[str, str]:
    pipe = get_pipeline()
    pipe.nav.stop()
    pipe.set_cmd(0.0, 0.0)
    pipe.safety.force_estop("api_stop")
    pipe.events.append("navigation_stop")
    return {"status": "stopped"}


@app.post("/api/navigation/clear_estop")
def api_clear_estop(_: None = Depends(require_control)) -> dict[str, str]:
    get_pipeline().safety.clear_estop()
    get_pipeline().events.append("estop_cleared")
    return {"status": "cleared"}


@app.post("/api/teleop")
def api_teleop(
    req: CmdVelRequest, _: None = Depends(require_control)
) -> dict[str, Any]:
    """Web joystick. Safety supervisor ALWAYS filters commands locally."""
    _rate_limit("teleop", 0.03)
    pipe = get_pipeline()
    pipe.set_cmd(req.linear, req.angular)
    return {"status": "ok", "note": "filtered_by_safety_supervisor"}


@app.post("/api/experiment/start")
def api_exp_start(
    req: ExperimentRequest, _: None = Depends(require_control)
) -> dict[str, str]:
    get_pipeline().events.append(f"experiment_start:{req.name}:{req.scenario}")
    return {"status": "started", "name": req.name, "scenario": req.scenario}


@app.post("/api/experiment/stop")
def api_exp_stop(_: None = Depends(require_control)) -> dict[str, str]:
    get_pipeline().events.append("experiment_stop")
    return {"status": "stopped"}


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket) -> None:
    await ws.accept()
    _clients.add(ws)
    try:
        while True:
            # Keepalive / optional client pings
            try:
                await asyncio.wait_for(ws.receive_text(), timeout=30.0)
            except asyncio.TimeoutError:
                await ws.send_json({"channel": "ping"})
    except WebSocketDisconnect:
        pass
    finally:
        _clients.discard(ws)


@app.websocket("/ws/events")
async def ws_events(ws: WebSocket) -> None:
    await ws.accept()
    try:
        while True:
            await ws.send_json({"channel": "events", "data": _latest.get("events", [])})
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass


FRONTEND_INDEX = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>AMP Robot Control Center</title>
<link rel="preconnect" href="https://fonts.googleapis.com"/>
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet"/>
<style>
:root{
  --bg:#0e1419; --panel:#162028; --line:#243440; --text:#e7eef4; --muted:#8aa0b2;
  --ok:#3ecf8e; --warn:#e6b84d; --err:#e85d5d; --accent:#2eb7c9; --accent2:#c9a227;
}
*{box-sizing:border-box} body{margin:0;background:radial-gradient(1200px 600px at 10% -10%,#1a2a33 0%,var(--bg) 55%);
color:var(--text);font-family:"IBM Plex Sans",sans-serif;min-height:100vh}
header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between;
padding:14px 18px;border-bottom:1px solid var(--line);backdrop-filter:blur(8px);position:sticky;top:0;z-index:5;background:rgba(14,20,25,.86)}
.brand{font-weight:600;letter-spacing:.02em;font-size:1.05rem}
.brand span{color:var(--accent)}
.pills{display:flex;flex-wrap:wrap;gap:8px}
.pill{font-family:"IBM Plex Mono",monospace;font-size:12px;padding:6px 10px;border:1px solid var(--line);
border-radius:6px;background:var(--panel)}
.pill.ok{border-color:color-mix(in srgb,var(--ok) 50%,var(--line));color:var(--ok)}
.pill.warn{color:var(--warn)} .pill.err{color:var(--err)}
main{display:grid;grid-template-columns:1.1fr 1fr;gap:14px;padding:14px;max-width:1400px;margin:0 auto}
@media(max-width:980px){main{grid-template-columns:1fr}}
.card{background:linear-gradient(180deg,rgba(22,32,40,.95),rgba(18,26,32,.95));border:1px solid var(--line);
border-radius:10px;overflow:hidden;min-height:280px;display:flex;flex-direction:column}
.card h2{margin:0;padding:10px 12px;font-size:13px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);
border-bottom:1px solid var(--line);font-weight:500}
.viewport{flex:1;position:relative;background:#0a1014}
canvas{width:100%;height:100%;display:block}
.mapcard{grid-column:1/-1;min-height:320px}
.bottom{display:grid;grid-template-columns:repeat(5,1fr);gap:14px;padding:0 14px 18px;max-width:1400px;margin:0 auto}
@media(max-width:1100px){.bottom{grid-template-columns:1fr 1fr}}
@media(max-width:700px){.bottom{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px;min-height:220px}
.panel h3{margin:0 0 10px;font-size:13px;color:var(--muted);letter-spacing:.06em;text-transform:uppercase}
table{width:100%;border-collapse:collapse;font-size:12px;font-family:"IBM Plex Mono",monospace}
th,td{padding:6px 4px;border-bottom:1px solid var(--line);text-align:left}
.controls{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
button,input{font:inherit} button{background:var(--accent);color:#042028;border:0;border-radius:6px;padding:8px 12px;font-weight:600;cursor:pointer}
button.danger{background:var(--err);color:#fff} button.ghost{background:transparent;color:var(--text);border:1px solid var(--line)}
input{background:#0e151b;border:1px solid var(--line);color:var(--text);border-radius:6px;padding:8px}
.log{font-family:"IBM Plex Mono",monospace;font-size:11px;max-height:160px;overflow:auto;color:var(--muted);white-space:pre-wrap}
.joy{display:grid;grid-template-columns:repeat(3,48px);gap:6px;justify-content:center;margin-top:8px}
.joy button{width:48px;height:48px;padding:0}
</style>
</head>
<body>
<header>
  <div class="brand">AMP <span>Robot</span> · Adaptive Multimodal Perception</div>
  <div class="pills" id="statusPills">
    <div class="pill">STATUS …</div>
  </div>
</header>
<main>
  <section class="card"><h2>Live Camera</h2><div class="viewport"><canvas id="cam"></canvas></div></section>
  <section class="card"><h2>LiDAR Radar</h2><div class="viewport"><canvas id="lidar"></canvas></div></section>
  <section class="card mapcard"><h2>SLAM Map · Pose</h2><div class="viewport"><canvas id="map"></canvas></div></section>
</main>
<div class="bottom">
  <div class="panel"><h3>Objects</h3><div style="overflow:auto;max-height:180px"><table><thead><tr><th>ID</th><th>Class</th><th>Conf</th><th>Dist</th><th>Brg</th><th>Dyn</th></tr></thead><tbody id="objBody"></tbody></table></div></div>
  <div class="panel"><h3>Distances</h3><div id="sectors" class="log"></div></div>
  <div class="panel"><h3>Fusion</h3><div id="fusion" class="log"></div>
    <div class="controls"><input id="token" placeholder="control token" style="flex:1"/><button class="ghost" onclick="saveToken()">Save</button></div>
  </div>
  <div class="panel"><h3>Teleop / Safety</h3>
    <div class="joy">
      <span></span><button onmousedown="tele(0.2,0)" onmouseup="tele(0,0)">↑</button><span></span>
      <button onmousedown="tele(0,0.5)" onmouseup="tele(0,0)">←</button>
      <button class="danger" onclick="stopAll()">■</button>
      <button onmousedown="tele(0,-0.5)" onmouseup="tele(0,0)">→</button>
      <span></span><button onmousedown="tele(-0.15,0)" onmouseup="tele(0,0)">↓</button><span></span>
    </div>
    <div class="controls"><button onclick="clearEstop()">Clear E-Stop</button><button class="ghost" onclick="setGoal()">Goal (2,0)</button></div>
  </div>
  <div class="panel"><h3>Logs / Events</h3><div id="logs" class="log"></div></div>
</div>
<script>
const tokenKey='amp_token';
document.getElementById('token').value=localStorage.getItem(tokenKey)||'dev-token-change-me';
function saveToken(){localStorage.setItem(tokenKey,document.getElementById('token').value)}
function authHeaders(){return {'Authorization':'Bearer '+ (localStorage.getItem(tokenKey)||''),'Content-Type':'application/json'}}
async function tele(l,a){await fetch('/api/teleop',{method:'POST',headers:authHeaders(),body:JSON.stringify({linear:l,angular:a})})}
async function stopAll(){await fetch('/api/navigation/stop',{method:'POST',headers:authHeaders()})}
async function clearEstop(){await fetch('/api/navigation/clear_estop',{method:'POST',headers:authHeaders()})}
async function setGoal(){await fetch('/api/navigation/goal',{method:'POST',headers:authHeaders(),body:JSON.stringify({x:2,y:0,yaw:0})})}

const cam=document.getElementById('cam'), lidar=document.getElementById('lidar'), mapc=document.getElementById('map');
function fit(c){const r=c.parentElement.getBoundingClientRect(); c.width=r.width*devicePixelRatio; c.height=r.height*devicePixelRatio; return c.getContext('2d')}
let latest=null;
const camImg=new Image();
let camImgReady=false;
camImg.onload=()=>{camImgReady=true};
function draw(){
  if(!latest){requestAnimationFrame(draw);return}
  let ctx=fit(cam); ctx.scale(devicePixelRatio,devicePixelRatio);
  const w=cam.width/devicePixelRatio, h=cam.height/devicePixelRatio;
  ctx.fillStyle='#101820'; ctx.fillRect(0,0,w,h);
  const cw=(latest.camera&&latest.camera.width)||640;
  const ch=(latest.camera&&latest.camera.height)||480;
  if(latest.camera_jpeg_b64){
    const src='data:image/jpeg;base64,'+latest.camera_jpeg_b64;
    if(camImg.src!==src){ camImgReady=false; camImg.src=src; }
    if(camImgReady){ ctx.drawImage(camImg,0,0,w,h); }
  } else {
    ctx.fillStyle='#8aa0b2'; ctx.font='14px IBM Plex Mono';
    ctx.fillText('No live camera frame (mock or permission denied)', 16, 28);
  }
  (latest.projected_lidar||[]).forEach(p=>{ctx.fillStyle='rgba(46,183,201,.85)'; ctx.fillRect(p.u/cw*w, p.v/ch*h, 2,2)});
  (latest.objects||[]).forEach(o=>{
    const b=o.bbox; const sx=w/cw, sy=h/ch;
    ctx.strokeStyle=o.is_dynamic?'#e85d5d':'#3ecf8e'; ctx.lineWidth=2;
    ctx.strokeRect(b.x1*sx,b.y1*sy,(b.x2-b.x1)*sx,(b.y2-b.y1)*sy);
    ctx.fillStyle='#e7eef4'; ctx.font='12px IBM Plex Mono';
    const dist=o.distance_m!=null?o.distance_m.toFixed(2)+' m':'—';
    const br=o.bearing_deg!=null?((o.bearing_deg>=0?'+':'')+o.bearing_deg.toFixed(1)+'°'):'';
    ctx.fillText(`${o.class_name.toUpperCase()} #${o.track_id}  ${dist}  ${br}  ${o.confidence.toFixed(2)}  ${o.is_dynamic?'DYNAMIC':'STATIC'}`, b.x1*sx, Math.max(12,b.y1*sy-4));
  });
  // LiDAR radar
  ctx=fit(lidar); ctx.scale(devicePixelRatio,devicePixelRatio);
  const W=lidar.width/devicePixelRatio, H=lidar.height/devicePixelRatio, cx=W/2, cy=H/2, scale=Math.min(W,H)/(2*6);
  ctx.fillStyle='#0a1014'; ctx.fillRect(0,0,W,H);
  const hw=latest.hardware||{};
  if(hw.lidar_source==='none'){
    ctx.fillStyle='#e6b84d'; ctx.font='14px IBM Plex Mono';
    ctx.fillText('LiDAR NOT CONNECTED', 16, 28);
    ctx.fillStyle='#8aa0b2'; ctx.font='12px IBM Plex Mono';
    ctx.fillText('Showing vision-based distance sectors only', 16, 48);
    // draw object bearings as rays
    (latest.objects||[]).forEach(o=>{
      if(o.distance_m==null||o.bearing_deg==null) return;
      const rad=o.bearing_deg*Math.PI/180;
      const rr=o.distance_m*scale;
      ctx.strokeStyle=o.is_dynamic?'#e85d5d':'#3ecf8e';
      ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(cx+Math.sin(rad)*rr, cy-Math.cos(rad)*rr); ctx.stroke();
      ctx.beginPath(); ctx.arc(cx+Math.sin(rad)*rr, cy-Math.cos(rad)*rr, 5, 0, Math.PI*2); ctx.fillStyle=ctx.strokeStyle; ctx.fill();
    });
  } else {
    [['#e85d5d',0.3],['#e6b84d',0.6],['#2eb7c9',1.0]].forEach(([col,r])=>{ctx.beginPath();ctx.arc(cx,cy,r*scale,0,Math.PI*2);ctx.strokeStyle=col;ctx.globalAlpha=.5;ctx.stroke();ctx.globalAlpha=1});
    (latest.lidar_points||[]).forEach(p=>{ctx.fillStyle='#9fd7e0'; ctx.fillRect(cx+p.x*scale, cy-p.y*scale, 2,2)});
  }
  ctx.fillStyle='#c9a227'; ctx.beginPath(); ctx.arc(cx,cy,4,0,Math.PI*2); ctx.fill();
  // Map
  ctx=fit(mapc); ctx.scale(devicePixelRatio,devicePixelRatio);
  const mw=mapc.width/devicePixelRatio, mh=mapc.height/devicePixelRatio;
  ctx.fillStyle='#0a1014'; ctx.fillRect(0,0,mw,mh);
  const m=latest.map||{}; const data=m.data||[];
  if(data.length){
    const gw=data[0].length, gh=data.length; const cs=Math.min(mw/gw, mh/gh);
    for(let y=0;y<gh;y++) for(let x=0;x<gw;x++){
      const v=data[y][x]; if(!v) continue; ctx.fillStyle=v===2?'#3a5160':'#1a2a33'; ctx.fillRect(x*cs,y*cs,cs+0.5,cs+0.5);
    }
    const pose=m.pose||latest.pose||{};
    ctx.fillStyle='#c9a227';
    const px = mw/2 + (pose.x||0)*20; const py = mh/2 - (pose.y||0)*20;
    ctx.beginPath(); ctx.arc(px,py,5,0,Math.PI*2); ctx.fill();
    ctx.fillStyle='#8aa0b2'; ctx.font='12px IBM Plex Mono';
    ctx.fillText(`x=${(pose.x||0).toFixed(2)} y=${(pose.y||0).toFixed(2)} yaw=${((pose.yaw||0)*180/Math.PI).toFixed(1)}°`, 12, 20);
  }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

function renderTables(d){
  const body=document.getElementById('objBody'); body.innerHTML='';
  (d.objects||[]).forEach(o=>{
    const tr=document.createElement('tr');
    const br=o.bearing_deg!=null?o.bearing_deg.toFixed(1)+'°':'—';
    tr.innerHTML=`<td>${o.track_id}</td><td>${o.class_name}</td><td>${o.confidence.toFixed(2)}</td><td>${o.distance_m!=null?o.distance_m.toFixed(2)+'m':'—'}</td><td>${br}</td><td>${o.is_dynamic?'YES':'NO'}</td>`;
    body.appendChild(tr);
  });
  const s=d.sectors||{};
  const src=s.source||((d.hardware||{}).lidar_source||'camera');
  document.getElementById('sectors').textContent =
`source: ${src}
FRONT       ${(s.front??0).toFixed(2)} m
FRONT_LEFT  ${(s.front_left??0).toFixed(2)} m
FRONT_RIGHT ${(s.front_right??0).toFixed(2)} m
LEFT        ${(s.left??0).toFixed(2)} m
RIGHT       ${(s.right??0).toFixed(2)} m
REAR_LEFT   ${(s.rear_left??0).toFixed(2)} m
REAR        ${(s.rear??0).toFixed(2)} m
REAR_RIGHT  ${(s.rear_right??0).toFixed(2)} m`;
  const f=d.fusion||{}, c=d.confidence||{}, sys=d.system||{}, saf=d.safety||{}, hw=d.hardware||{};
  document.getElementById('fusion').textContent =
`mode=${f.mode||'—'}  hz=${(f.update_rate_hz||0).toFixed(1)}
detector=${sys.detector||hw.detector||'—'}
weights lidar=${(f.sensor_weights||{}).lidar?.toFixed?.(2)??'—'} cam=${(f.sensor_weights||{}).camera?.toFixed?.(2)??'—'}
conf L=${(c.lidar??0).toFixed(2)} C=${(c.camera??0).toFixed(2)}
safety=${saf.action||'—'} (${saf.reason||''})
camFPS=${(sys.camera_fps||0).toFixed(1)} detFPS=${(sys.detection_fps||0).toFixed(1)}`;
  document.getElementById('logs').textContent=(d.events||[]).join('\\n');
  const pills=document.getElementById('statusPills');
  const ok = (saf.action||'ALLOW')!=='ESTOP';
  pills.innerHTML=`
    <div class="pill ${ok?'ok':'err'}">ROBOT ${saf.action||'…'}</div>
    <div class="pill ${hw.camera_source!=='none'?'ok':'warn'}">CAM ${(hw.camera_source||'none').toUpperCase()}</div>
    <div class="pill ${hw.lidar_source!=='none'?'ok':'warn'}">LIDAR ${(hw.lidar_source||'none').toUpperCase()}</div>
    <div class="pill">DET ${(sys.detection_fps||0).toFixed(1)} FPS</div>
    <div class="pill ok">WS LIVE</div>`;
}
const wsProto = location.protocol==='https:'?'wss':'ws';
const ws=new WebSocket(`${wsProto}://${location.host}/ws/telemetry`);
ws.onmessage=(ev)=>{ const msg=JSON.parse(ev.data); if(msg.channel==='telemetry'){ latest=msg.data; renderTables(latest);} };
</script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
def index() -> HTMLResponse:
    return HTMLResponse(FRONTEND_INDEX)


def main() -> None:
    import uvicorn

    host = os.environ.get("AMP_HOST", "0.0.0.0")
    port = int(os.environ.get("AMP_PORT", "8000"))
    # Pass app object so `python -m web_dashboard.backend.app` works reliably.
    uvicorn.run(app, host=host, port=port, reload=False)


if __name__ == "__main__":
    main()
