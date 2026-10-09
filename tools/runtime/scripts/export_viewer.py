#!/usr/bin/env python3
"""Standalone 3D viewer (one HTML file) of the pallets built by the runtime
loop, from ``runtime_physics.json`` (``physics_replay.py --report``).

    python tools/runtime/scripts/export_viewer.py docs/taehyeon/reports/runtime_physics.json \
        docs/taehyeon/reports/runtime_viewer.html

Open the HTML in a browser: pick a pallet, step through the placement order,
boxes coloured by mass; the frame shows the stacking-height limit and the
marker the load centre (CoG projected on the deck).
"""

import json
from pathlib import Path
import sys

TEMPLATE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>AHEAD 런타임 적재 결과</title>
<style>
 :root{--bg:#f6f7f9;--panel:#fff;--text:#1d2329;--muted:#5b6672;--line:#d9dee4;--accent:#2f6fde}
 @media (prefers-color-scheme: dark){:root{--bg:#14171b;--panel:#1d2127;--text:#e7ebef;--muted:#9aa5b1;--line:#2d333b;--accent:#6ea0ff}}
 *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--text);font:14px/1.5 system-ui,sans-serif}
 header{padding:12px 16px;border-bottom:1px solid var(--line);background:var(--panel)}
 h1{font-size:17px;margin:0} .sub{color:var(--muted);font-size:13px}
 main{display:grid;grid-template-columns:300px 1fr;min-height:calc(100vh - 62px)}
 aside{padding:16px;border-right:1px solid var(--line);background:var(--panel);display:flex;flex-direction:column;gap:12px}
 label{font-size:12px;color:var(--muted)} select,input[type=range]{width:100%}
 select{padding:6px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--text)}
 .stats{display:grid;grid-template-columns:1fr 1fr;gap:8px}
 .stat{border:1px solid var(--line);border-radius:8px;padding:8px} .stat b{display:block;font-size:16px}
 .stat span{font-size:12px;color:var(--muted)} #view{position:relative;min-height:420px}
 canvas{display:block;width:100%;height:100%} .legend{font-size:12px;color:var(--muted)}
 .bar{height:10px;border-radius:5px;background:linear-gradient(90deg,#cfe1ff,#2f6fde,#0b2f73)}
 .ok{color:#1f9d55}.bad{color:#d64545}
 @media (max-width:760px){main{grid-template-columns:1fr} aside{border-right:0;border-bottom:1px solid var(--line)} #view{height:70vh}}
</style></head><body>
<header><h1>AHEAD 런타임 적재 결과 (1→8 루프, 가상 셀)</h1>
<div class="sub">실제 크기·실제 자세 · 4단계 Rule · 6단계 HDR50-22 검사 통과 자리만 · PyBullet 재현 결과 포함</div></header>
<main><aside>
 <div><label for="pal">팔레트</label><select id="pal"></select></div>
 <div><label for="step">적재 순서 <span id="stepv"></span></label><input id="step" type="range" min="0" value="0"></div>
 <div class="stats">
  <div class="stat"><b id="sBoxes">-</b><span>박스</span></div>
  <div class="stat"><b id="sFill">-</b><span>실제 채움률</span></div>
  <div class="stat"><b id="sTop">-</b><span>최고 높이</span></div>
  <div class="stat"><b id="sPhys">-</b><span>PyBullet</span></div>
 </div>
 <div class="legend">박스 색: 가벼움 → 무거움<div class="bar"></div></div>
 <div class="legend">회색 틀: 적재 높이 한계 · 빨간 선: 하중 중심(무게중심의 수직선)</div>
 <div class="legend">드래그: 회전 · 휠: 확대 · 오른쪽 드래그: 이동</div>
</aside><div id="view"></div></main>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
const DATA = __DATA__;
const view = document.getElementById('view');
const dark = matchMedia('(prefers-color-scheme: dark)').matches;
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(devicePixelRatio); view.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(dark ? 0x14171b : 0xf6f7f9);
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50); camera.up.set(0,0,1);
const controls = new THREE.OrbitControls(camera, renderer.domElement);
scene.add(new THREE.AmbientLight(0xffffff, 0.65));
const sun = new THREE.DirectionalLight(0xffffff, 0.6); sun.position.set(2,-3,5); scene.add(sun);
let group = new THREE.Group(); scene.add(group);
const sel = document.getElementById('pal'), step = document.getElementById('step');
DATA.pallets.forEach((p,i)=>{const o=document.createElement('option');o.value=i;
  o.textContent=`${p.scenario} · ${p.pallet_id} · ${p.layout.length}박스`;sel.appendChild(o);});
const mmax = Math.max(...DATA.pallets.flatMap(p=>p.layout.map(b=>b.mass_kg||1)));
function color(m){const t=Math.min(1,(m||1)/mmax);const a=[207,225,255],b=[47,111,222],c=[11,47,115];
  const lerp=(u,v,s)=>u.map((x,k)=>x+(v[k]-x)*s);const rgb=t<0.5?lerp(a,b,t*2):lerp(b,c,(t-0.5)*2);
  return new THREE.Color(rgb[0]/255,rgb[1]/255,rgb[2]/255);}
function dims(b){let [sx,sy,sz]=b.size;if(Math.round(b.pose[3]/(Math.PI/2))%2)[sx,sy]=[sy,sx];return [sx,sy,sz];}
function draw(){
  scene.remove(group); group = new THREE.Group(); scene.add(group);
  const p = DATA.pallets[+sel.value], n = +step.value; const [X,Y,H] = p.pallet_size;
  const deck = new THREE.Mesh(new THREE.BoxGeometry(X,Y,0.15), new THREE.MeshLambertMaterial({color:0xb8895a}));
  deck.position.set(X/2,Y/2,-0.075); group.add(deck);
  const lim = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(X,Y,H)),
    new THREE.LineBasicMaterial({color:dark?0x5b6672:0x9aa5b1})); lim.position.set(X/2,Y/2,H/2); group.add(lim);
  let mass=0,cx=0,cy=0,top=0,vol=0;
  p.layout.slice(0,n).forEach(b=>{const [sx,sy,sz]=dims(b);const [x,y,z]=b.pose;
    const m=new THREE.Mesh(new THREE.BoxGeometry(sx,sy,sz), new THREE.MeshLambertMaterial({color:color(b.mass_kg)}));
    m.position.set(x+sx/2,y+sy/2,z+sz/2); group.add(m);
    const e=new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({color:0x10151c}));
    e.position.copy(m.position); group.add(e);
    const w=b.mass_kg||1; mass+=w; cx+=w*(x+sx/2); cy+=w*(y+sy/2); top=Math.max(top,z+sz); vol+=sx*sy*sz;});
  if(mass>0){const gx=cx/mass, gy=cy/mass;
    const g=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(gx,gy,0),new THREE.Vector3(gx,gy,top+0.15)]);
    group.add(new THREE.Line(g,new THREE.LineBasicMaterial({color:0xd64545})));
    const c=new THREE.Mesh(new THREE.SphereGeometry(0.025,16,12), new THREE.MeshBasicMaterial({color:0xd64545}));
    c.position.set(gx,gy,top+0.15); group.add(c);}
  document.getElementById('stepv').textContent=`${n} / ${p.layout.length}`;
  document.getElementById('sBoxes').textContent=n;
  document.getElementById('sFill').textContent=(100*vol/(X*Y*H)).toFixed(1)+' %';
  document.getElementById('sTop').textContent=top.toFixed(2)+' m';
  const ph=p.physics; const el=document.getElementById('sPhys');
  el.textContent = ph ? (ph.stable ? '안정' : '불안정') : '-'; el.className = ph ? (ph.stable?'ok':'bad') : '';
  controls.target.set(X/2,Y/2,0.4);
}
function resetCam(){const p=DATA.pallets[+sel.value];const [X,Y]=p.pallet_size;
  camera.position.set(X/2+1.9,Y/2-2.3,2.1); controls.update();}
function resize(){const w=view.clientWidth,h=Math.max(420,view.clientHeight||window.innerHeight-62);
  renderer.setSize(w,h); camera.aspect=w/h; camera.updateProjectionMatrix();}
sel.onchange=()=>{const p=DATA.pallets[+sel.value];step.max=p.layout.length;step.value=p.layout.length;draw();resetCam();};
step.oninput=draw; addEventListener('resize',resize);
sel.onchange(); resize();
(function loop(){requestAnimationFrame(loop);controls.update();renderer.render(scene,camera);})();
</script></body></html>
"""


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    rep = json.loads(src.read_text(encoding="utf-8"))
    phys = {(r["scenario"], r["pallet_id"]): r for r in rep.get("pallets", [])}
    pallets = []
    for p in rep["layouts"]:
        pallets.append({"scenario": p["scenario"], "pallet_id": p["pallet_id"], "pallet_size": p["pallet_size"],
                        "fill_true": p["fill_true"], "physics": phys.get((p["scenario"], p["pallet_id"])),
                        "layout": [{"size": [round(v, 4) for v in b["size"]], "mass_kg": b["mass_kg"],
                                    "pose": [round(v, 4) for v in b["pose"]]} for b in p["layout"]]})
    html = TEMPLATE.replace("__DATA__", json.dumps({"pallets": pallets}, separators=(",", ":")))
    dst.write_text(html, encoding="utf-8")
    print("wrote", dst, len(html) // 1024, "KB,", len(pallets), "pallets")


if __name__ == "__main__":
    main()
