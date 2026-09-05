"""Draw a recorded run as a 3D scene you can fly around.

`tools/make_replay.py` draws the run flat, which is the right tool for asking
"did bot_03 take the correct turn". This one is for looking at the fleet: a lit
warehouse, lanes on the floor, robots that face where they are going, and a
camera you can orbit.

It shares that tool's reason for existing. Webots' own streaming viewer renders
in the browser, so it must first download every texture in the world -- 924 MB
of marker tiles for the full warehouse -- and a tab will not do that. Nothing
here is textured: the floor, the lanes, the nodes and the robots are all
geometry generated from `warehouse.json`, so the whole 881-node warehouse costs
about as much as the 83-node chunk. The run's speed does not matter either,
because this is drawing a recording, not a simulation.

It is ground truth. The poses come from `robot/supervisor.py --replay`, which is
the only thing in the system that knows where a robot really is -- not where it
believes it is. What you are watching is what actually happened.

Robots are drawn larger than life by default (`--robot-scale`); a 15 cm chassis
in a 38 m hall is about two pixels, and a fleet of specks is not worth
rendering. Everything else is to scale.

Usage:
    uv run python -m tools.make_replay3d              # out/replay.csv -> out/replay3d.html
    uv run python -m tools.make_replay3d --open
    uv run python -m tools.make_replay3d --stride 2   # halve the frames
"""

import argparse
import base64
import csv
import json
import pathlib
import struct
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

# Pinned: r128 is the last three.js that ships a global `THREE` build, which is
# what lets this be one script tag and no module loader. Orbit controls are
# written by hand below rather than pulled from the addons, so this is the only
# thing fetched from the network.
THREE_URL = "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"


def f32(values):
    """Pack floats as base64 little-endian float32.

    A long run is 150k poses; as JSON that is megabytes of decimal text the
    browser then has to parse into numbers one at a time. This is the same
    numbers at 4 bytes each, decoded in one pass into a Float32Array.
    """
    raw = struct.pack("<{}f".format(len(values)), *values)
    return base64.b64encode(raw).decode("ascii")


def load_poses(path, stride=1):
    """Frames of {robot: (x, y, theta)}, in recorded order.

    Rows arrive grouped by sample time; anything out of order lands in the frame
    nearest its own timestamp, so a truncated recording still plays.
    """
    frames = {}
    with path.open() as handle:
        for row in csv.DictReader(handle):
            t = round(float(row["t"]), 3)
            frames.setdefault(t, {})[row["robot"]] = (
                float(row["x"]), float(row["y"]), float(row["theta"]))
    times = sorted(frames)[::stride]
    return times, [frames[t] for t in times]


def load_graph(path):
    """Nodes and lanes in world metres, sharing the poses' frame.

    `warehouse.json` is in centimetres with the origin at a corner; the
    supervisor reports metres about the centre, so the map is shifted to match
    rather than the poses -- the poses are the ground truth.
    """
    document = json.loads(path.read_text())
    width = float(document["dimensions"]["width"])
    height = float(document["dimensions"]["height"])
    nodes = {
        int(n["id"]): {
            "x": round(float(n["position"]["x"]) / 100.0 - width / 200.0, 4),
            "y": round(float(n["position"]["y"]) / 100.0 - height / 200.0, 4),
            "kind": n.get("node_type", "PT"),
        }
        for n in document["nodes"]
    }
    edges = [(int(e["a"]), int(e["b"])) for e in document["edges"]]
    return nodes, edges


PAGE = r"""<title>Fleet replay 3D</title>
<style>
  :root {
    --ink:#e8e6e3; --dim:#8b9199; --bg:#0d1014; --panel:#161b21;
    --line:#2b333b; --accent:#6aa9ff;
  }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--ink); overflow:hidden;
         font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace; }
  canvas { display:block; }
  #hud { position:fixed; left:0; right:0; bottom:0; padding:10px 14px;
         background:linear-gradient(transparent,rgba(13,16,20,.92) 34%);
         display:flex; align-items:center; gap:12px; flex-wrap:wrap; }
  #top { position:fixed; top:0; left:0; padding:12px 14px; pointer-events:none; }
  h1 { font-size:13px; margin:0 0 2px; font-weight:600; letter-spacing:.02em; }
  .sub { color:var(--dim); font-size:12px; }
  button, select { background:#232b33; color:var(--ink); border:1px solid var(--line);
           border-radius:6px; padding:5px 12px; cursor:pointer; font:inherit; }
  button:hover, select:hover { background:#2d3740; }
  input[type=range] { accent-color:var(--accent); }
  #scrub { flex:1; min-width:200px; }
  .clock { color:var(--dim); font-variant-numeric:tabular-nums; min-width:150px; }
  #legend { position:fixed; top:12px; right:14px; background:rgba(22,27,33,.86);
            border:1px solid var(--line); border-radius:8px; padding:9px 12px;
            font-size:12px; }
  #legend div { display:flex; align-items:center; gap:7px; margin:2px 0; }
  .sw { width:9px; height:9px; border-radius:50%; }
  #err { position:fixed; inset:0; display:none; place-items:center;
         text-align:center; padding:30px; }
</style>
<div id="err">
  <div>
    <h1>three.js did not load</h1>
    <p class="sub">This page pulls one script from cdnjs and needs a network
    connection the first time.<br>Everything else is embedded.</p>
  </div>
</div>
<div id="top">
  <h1>Fleet replay &mdash; ground truth</h1>
  <div class="sub"><b>__NROBOTS__</b> robots &middot; <b>__NNODES__</b> nodes
    &middot; drag to orbit, scroll to zoom, right-drag to pan</div>
</div>
<div id="legend"></div>
<div id="hud">
  <button id="play">pause</button>
  <input type="range" id="scrub" min="0" value="0">
  <span class="clock" id="clock"></span>
  <label class="sub">speed
    <select id="speed">
      <option value="0.25">0.25x</option><option value="1" selected>1x</option>
      <option value="4">4x</option><option value="16">16x</option>
      <option value="64">64x</option>
    </select></label>
  <label class="sub">follow
    <select id="follow"><option value="-1">free camera</option></select></label>
  <label class="sub">size
    <input type="range" id="scale" min="1" max="12" step="0.5" value="__SCALE__"></label>
  <label class="sub">lanes
    <input type="range" id="lanew" min="0.02" max="0.5" step="0.01" value="0.13"></label>
  <label class="sub"><input type="checkbox" id="grid" checked> grid</label>
  <label class="sub"><input type="checkbox" id="trails" checked> trails</label>
</div>
<script src="__THREE__"></script>
<script>
const NODES=__NODES__, EDGES=__EDGES__, NAMES=__NAMES__;
const NFRAMES=__NFRAMES__, ROBOT_SCALE=__SCALE__;
const KINDS={CH:"#1ae5e5",PK:"#b44dff",TR:"#2a80ff",YI:"#ff2a1a",PT:"#39424b"};
const KIND_NAMES={CH:"charging",PK:"parking",TR:"transfer",YI:"yield",PT:"pass-through"};
// Distinct hues, not a gradient: you are tracking individuals, and neighbouring
// shades of one colour are exactly what you cannot tell apart.
const COLORS=[0xff6b6b,0xffd93d,0x6aa9ff,0x51cf66,0xff9f43,
              0xc77dff,0x4ecdc4,0xf06595,0xa0e548,0x74c0fc];

function decode(b64){
  const bin=atob(b64), buf=new ArrayBuffer(bin.length), view=new Uint8Array(buf);
  for(let i=0;i<bin.length;i++) view[i]=bin.charCodeAt(i);
  return new Float32Array(buf);
}
const TIMES=decode("__TIMES__");
// One flat [x,y,theta] run per robot, so a frame lookup is an index, not a
// dictionary of dictionaries.
const POSES=__POSES__.map(decode);

if(typeof THREE==="undefined"){ document.getElementById("err").style.display="grid"; }
else { main(); }

function main(){
// World +y is the plan view's "up"; three.js reserves +y for vertical, so the
// map lies in the xz-plane with world y running along -z. Doing it here once
// keeps every coordinate below in the same frame the supervisor recorded.
const W=v=>v;                       // world x  -> three x
const D=v=>-v;                      // world y  -> three z

const scene=new THREE.Scene();
scene.background=new THREE.Color(0x0d1014);
scene.fog=new THREE.Fog(0x0d1014, 30, 130);

const camera=new THREE.PerspectiveCamera(55, innerWidth/innerHeight, 0.1, 500);
const renderer=new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled=true;
renderer.shadowMap.type=THREE.PCFSoftShadowMap;
document.body.appendChild(renderer.domElement);

// ---- bounds, so the camera frames the map whatever its size ----------------
const xs=Object.values(NODES).map(n=>n.x), ys=Object.values(NODES).map(n=>n.y);
const minX=Math.min(...xs), maxX=Math.max(...xs);
const minY=Math.min(...ys), maxY=Math.max(...ys);
const cx=(minX+maxX)/2, cy=(minY+maxY)/2;
const span=Math.max(maxX-minX, maxY-minY)+4;

// ---- lighting --------------------------------------------------------------
scene.add(new THREE.HemisphereLight(0x9fb4cc, 0x0a0d10, 0.55));
const sun=new THREE.DirectionalLight(0xfff2e0, 1.5);
sun.position.set(cx+span*0.35, span*0.8, D(cy)-span*0.35);
sun.castShadow=true;
sun.shadow.mapSize.set(2048,2048);
const s=span*0.7, sc=sun.shadow.camera;
sc.left=-s; sc.right=s; sc.top=s; sc.bottom=-s; sc.near=1; sc.far=span*3;
scene.add(sun);

// ---- floor -----------------------------------------------------------------
const floor=new THREE.Mesh(
  new THREE.PlaneGeometry(span*2.2, span*2.2),
  new THREE.MeshStandardMaterial({color:0x171c22, roughness:0.95, metalness:0.0}));
floor.rotation.x=-Math.PI/2;
floor.position.set(cx, 0, D(cy));
floor.receiveShadow=true;
scene.add(floor);

// 10 m cells, not 2 m: at node spacing the grid competes with the lane graph
// and you end up reading the floor instead of the map. Dim enough to give
// depth and nothing else.
const grid=new THREE.GridHelper(span*2.2, Math.max(2,Math.round(span*2.2/10)),
                                0x1b2229, 0x181e24);
grid.position.set(cx, 0.002, D(cy));
scene.add(grid);

// ---- lanes: one merged geometry, so 952 lanes are one draw call ------------
// The world generator paints these at 20 mm in near-black ink. Reproducing that
// faithfully gives you an invisible map, so they are drawn wide and bright --
// exaggerated for the same reason the robots are. Emissive, so a lane reads the
// same on the far side of the hall as it does under the light.
//
// Rebuilt on the width slider rather than fixed at generation time: how wide a
// lane needs to be to read depends on how far out the camera is, and that is a
// thing to judge by looking rather than to guess and regenerate.
const laneMaterial=new THREE.MeshStandardMaterial({
  color:0x9fb6cd, emissive:0x53708c, emissiveIntensity:1.0, roughness:0.55});
let lanes=null;

function buildLanes(LW){
  const pos=[];
  for(const [a,b] of EDGES){
    const A=NODES[a], B=NODES[b]; if(!A||!B) continue;
    const dx=B.x-A.x, dy=B.y-A.y, len=Math.hypot(dx,dy)||1;
    // Normal in the floor plane, to give the line width.
    const nx=-dy/len*LW/2, ny=dx/len*LW/2;
    const p=[[A.x+nx,A.y+ny],[B.x+nx,B.y+ny],[B.x-nx,B.y-ny],[A.x-nx,A.y-ny]];
    for(const [i,j,k] of [[0,1,2],[0,2,3]])
      for(const q of [p[i],p[j],p[k]]) pos.push(W(q[0]), 0.006, D(q[1]));
  }
  const g=new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos,3));
  g.computeVertexNormals();
  if(lanes){ scene.remove(lanes); lanes.geometry.dispose(); }
  lanes=new THREE.Mesh(g, laneMaterial);
  lanes.receiveShadow=true;
  scene.add(lanes);
}
buildLanes(0.13);

// A junction is where the eye goes, and four lane ends meeting leave a notch.
// A disc under each node fills it, so the graph reads as continuous track.
{
  const cap=new THREE.CircleGeometry(0.075, 12);
  const m=new THREE.Matrix4(), rot=new THREE.Matrix4().makeRotationX(-Math.PI/2);
  const ids=Object.keys(NODES);
  const caps=new THREE.InstancedMesh(cap, laneMaterial, ids.length);
  ids.forEach((id,i)=>{
    const n=NODES[id];
    caps.setMatrixAt(i, m.makeTranslation(W(n.x), 0.0055, D(n.y)).multiply(rot));
  });
  scene.add(caps);
}

// ---- nodes -----------------------------------------------------------------
{
  const disc=new THREE.CylinderGeometry(0.16,0.16,0.012,14);
  const mats={};
  for(const k in KINDS) mats[k]=new THREE.MeshStandardMaterial({
    color:new THREE.Color(KINDS[k]),
    emissive:new THREE.Color(KINDS[k]).multiplyScalar(k==="PT"?0.05:0.45),
    roughness:0.5});
  const byKind={};
  for(const id in NODES){ const n=NODES[id]; (byKind[n.kind]=byKind[n.kind]||[]).push(n); }
  for(const kind in byKind){
    const list=byKind[kind];
    const mesh=new THREE.InstancedMesh(disc, mats[kind]||mats.PT, list.length);
    const m=new THREE.Matrix4();
    list.forEach((n,i)=>{ m.makeTranslation(W(n.x), 0.010, D(n.y)); mesh.setMatrixAt(i,m); });
    mesh.castShadow=false; mesh.receiveShadow=true;
    scene.add(mesh);
  }
}

// ---- robots ----------------------------------------------------------------
// Built once and cloned. A LineBot is ~150 mm long with a 99 mm track and 20 mm
// wheels; those are the real numbers, multiplied by the size slider so the
// fleet is visible from across a 38 m hall.
function makeRobot(color){
  const g=new THREE.Group();
  const body=new THREE.Mesh(
    new THREE.BoxGeometry(0.15,0.055,0.11),
    new THREE.MeshStandardMaterial({color:0x2a3138, roughness:0.45, metalness:0.35}));
  body.position.y=0.042; body.castShadow=true; g.add(body);

  // Beacon: the robot's identity at a glance, and the only emissive part, so it
  // reads as a light rather than as paint.
  const beacon=new THREE.Mesh(new THREE.SphereGeometry(0.028,14,10),
    new THREE.MeshStandardMaterial({color, emissive:color,
      emissiveIntensity:1.5, roughness:0.3}));
  beacon.position.set(-0.02,0.082,0); g.add(beacon);

  // Nose wedge: heading is half of what a robot is doing, and a box hides which
  // way it is about to leave a junction.
  const nose=new THREE.Mesh(new THREE.ConeGeometry(0.032,0.06,4),
    new THREE.MeshStandardMaterial({color, roughness:0.4}));
  nose.rotation.z=-Math.PI/2; nose.rotation.y=Math.PI/4;
  nose.position.set(0.088,0.045,0); nose.castShadow=true; g.add(nose);

  const wheel=new THREE.CylinderGeometry(0.02,0.02,0.012,12);
  const wm=new THREE.MeshStandardMaterial({color:0x11151a, roughness:0.85});
  for(const side of [-1,1]){
    const w=new THREE.Mesh(wheel,wm);
    w.rotation.x=Math.PI/2;
    w.position.set(0,0.02,side*0.0497);   // track_width 0.0994 m, calibrated
    w.castShadow=true; g.add(w);
  }
  return g;
}

const robots=NAMES.map((name,i)=>{
  const r=makeRobot(COLORS[i%COLORS.length]);
  r.scale.setScalar(ROBOT_SCALE);
  scene.add(r);
  return r;
});

// ---- trails ----------------------------------------------------------------
// A ring buffer per robot drawn as a fading line: where a robot has just been
// is most of what tells a jam from a robot merely going slowly.
const TRAIL=140;
const trails=NAMES.map((name,i)=>{
  const g=new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(TRAIL*3),3));
  const colour=new THREE.Color(COLORS[i%COLORS.length]);
  const cols=new Float32Array(TRAIL*3);
  for(let k=0;k<TRAIL;k++){
    const f=k/TRAIL;                      // older -> dimmer
    cols[k*3]=colour.r*f; cols[k*3+1]=colour.g*f; cols[k*3+2]=colour.b*f;
  }
  g.setAttribute("color", new THREE.Float32BufferAttribute(cols,3));
  const line=new THREE.Line(g, new THREE.LineBasicMaterial({vertexColors:true,
    transparent:true, opacity:0.85}));
  line.frustumCulled=false;
  scene.add(line);
  return {line, buf:new Float32Array(TRAIL*3), n:0};
});

// ---- camera: hand-rolled orbit, so the addons are not a second dependency ---
let dist=span*0.95, yaw=-Math.PI/4, pitch=0.62;
const target=new THREE.Vector3(cx, 0, D(cy));
let followIndex=-1;

function placeCamera(){
  if(followIndex>=0 && robots[followIndex]) target.lerp(robots[followIndex].position, 0.12);
  camera.position.set(
    target.x + dist*Math.cos(pitch)*Math.sin(yaw),
    target.y + dist*Math.sin(pitch),
    target.z + dist*Math.cos(pitch)*Math.cos(yaw));
  camera.lookAt(target);
}

let drag=null;
renderer.domElement.addEventListener("mousedown", e=>{
  drag={x:e.clientX, y:e.clientY, pan:e.button===2}; });
addEventListener("mouseup", ()=>drag=null);
addEventListener("mousemove", e=>{
  if(!drag) return;
  const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
  drag.x=e.clientX; drag.y=e.clientY;
  if(drag.pan){
    // Pan along the ground plane, in the direction the camera is facing.
    const k=dist*0.0012;
    target.x -= (dx*Math.cos(yaw) - dy*Math.sin(yaw))*k;
    target.z += (dx*Math.sin(yaw) + dy*Math.cos(yaw))*k;
    followIndex=-1; document.getElementById("follow").value="-1";
  } else {
    yaw   -= dx*0.005;
    pitch  = Math.max(0.06, Math.min(1.5, pitch + dy*0.005));
  }
});
renderer.domElement.addEventListener("contextmenu", e=>e.preventDefault());
renderer.domElement.addEventListener("wheel", e=>{
  e.preventDefault();
  dist = Math.max(1.2, Math.min(span*2.2, dist*(1+Math.sign(e.deltaY)*0.09)));
}, {passive:false});

addEventListener("resize", ()=>{
  camera.aspect=innerWidth/innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

// ---- UI --------------------------------------------------------------------
const $=id=>document.getElementById(id);
const legend=$("legend");
legend.innerHTML=Object.keys(KIND_NAMES)
  .filter(k=>Object.values(NODES).some(n=>n.kind===k))
  .map(k=>`<div><span class="sw" style="background:${KINDS[k]}"></span>${KIND_NAMES[k]}</div>`)
  .join("");

const followSel=$("follow");
NAMES.forEach((n,i)=>followSel.insertAdjacentHTML("beforeend",
  `<option value="${i}">${n}</option>`));
followSel.addEventListener("change", e=>{
  followIndex=+e.target.value;
  if(followIndex>=0) dist=Math.min(dist, 6);
});

let frame=0, playing=true, speed=1;
$("play").addEventListener("click", e=>{
  playing=!playing; e.target.textContent=playing?"pause":"play"; });
$("speed").addEventListener("change", e=>speed=+e.target.value);
$("scale").addEventListener("input", e=>{
  const v=+e.target.value; robots.forEach(r=>r.scale.setScalar(v)); });
$("trails").addEventListener("change", e=>{
  trails.forEach(t=>t.line.visible=e.target.checked); });
$("lanew").addEventListener("input", e=>buildLanes(+e.target.value));
$("grid").addEventListener("change", e=>grid.visible=e.target.checked);

const scrub=$("scrub");
scrub.max=Math.max(0,NFRAMES-1);
scrub.addEventListener("input", ()=>{
  frame=+scrub.value; playing=false; $("play").textContent="play";
  trails.forEach(t=>t.n=0);            // a jump makes the old trail a lie
});

// ---- the loop --------------------------------------------------------------
let last=performance.now(), acc=0;
const PERIOD = NFRAMES>1 ? (TIMES[1]-TIMES[0]) : 0.1;   // recorded sample period

function applyFrame(){
  for(let i=0;i<NAMES.length;i++){
    const p=POSES[i], o=frame*3;
    if(o+2>=p.length) continue;
    const x=p[o], y=p[o+1], th=p[o+2];
    const r=robots[i];
    r.position.set(W(x), 0, D(y));
    // Rotating about three's +y by theta takes local +x to (cos, 0, -sin),
    // which is the world heading (cos, sin) under the same mapping.
    r.rotation.y=th;

    const t=trails[i];
    if(t.n<TRAIL){ t.n++; } else { t.buf.copyWithin(0, 3); }
    const k=(t.n-1)*3;
    t.buf[k]=W(x); t.buf[k+1]=0.03; t.buf[k+2]=D(y);
    const attr=t.line.geometry.attributes.position;
    attr.array.set(t.buf); attr.needsUpdate=true;
    t.line.geometry.setDrawRange(0, t.n);
  }
  $("clock").textContent =
    "t = "+(TIMES[frame]||0).toFixed(1)+" / "+(TIMES[NFRAMES-1]||0).toFixed(1)+" s";
}

function tick(now){
  requestAnimationFrame(tick);
  const dt=(now-last)/1000; last=now;
  if(playing && NFRAMES){
    acc += dt*speed;
    while(acc >= PERIOD){
      acc -= PERIOD;
      frame=(frame+1)%NFRAMES;
      if(frame===0) trails.forEach(t=>t.n=0);
    }
    scrub.value=frame;
  }
  applyFrame();
  placeCamera();
  renderer.render(scene,camera);
}
applyFrame();
requestAnimationFrame(tick);
}
</script>
"""


def build(times, frames, nodes, edges, names, scale):
    # One flat [x, y, theta] run per robot. A robot missing from a frame holds
    # its previous pose rather than jumping to the origin -- a gap in the
    # recording should look like a pause, not like a teleport.
    runs = []
    for name in names:
        flat = []
        last = (0.0, 0.0, 0.0)
        for frame in frames:
            last = frame.get(name, last)
            flat.extend(last)
        runs.append(f32(flat))

    return (PAGE
            .replace("__THREE__", THREE_URL)
            .replace("__NODES__", json.dumps(nodes, separators=(",", ":")))
            .replace("__EDGES__", json.dumps(edges, separators=(",", ":")))
            .replace("__NAMES__", json.dumps(names, separators=(",", ":")))
            .replace("__TIMES__", f32([round(t, 3) for t in times]))
            .replace("__POSES__", json.dumps(runs, separators=(",", ":")))
            .replace("__NFRAMES__", str(len(times)))
            .replace("__NROBOTS__", str(len(names)))
            .replace("__NNODES__", str(len(nodes)))
            .replace("__SCALE__", "{:g}".format(scale)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=pathlib.Path,
                        default=ROOT / "out" / "replay.csv")
    parser.add_argument("--map", type=pathlib.Path,
                        default=ROOT / "config" / "warehouse.json")
    parser.add_argument("--out", type=pathlib.Path,
                        default=ROOT / "out" / "replay3d.html")
    parser.add_argument("--stride", type=int, default=1,
                        help="keep every Nth frame; halves the file at 2")
    parser.add_argument("--robot-scale", type=float, default=4.0,
                        help="draw robots this many times life size")
    parser.add_argument("--open", action="store_true",
                        help="open the result in a browser")
    args = parser.parse_args(argv)

    if not args.replay.exists():
        print("no recording at {} -- run the fleet first".format(args.replay))
        return 1

    times, frames = load_poses(args.replay, max(1, args.stride))
    if not times:
        print("{} is empty".format(args.replay))
        return 1

    nodes, edges = load_graph(args.map)
    names = sorted({name for frame in frames for name in frame})
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(build(times, frames, nodes, edges, names,
                              args.robot_scale))

    print("{}  --  {} robots, {} nodes, {} frames, {:.0f} s of run, {:.1f} MB".format(
        args.out, len(names), len(nodes), len(times), times[-1] - times[0],
        args.out.stat().st_size / 1e6))
    if args.open:
        import webbrowser
        webbrowser.open(args.out.resolve().as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
