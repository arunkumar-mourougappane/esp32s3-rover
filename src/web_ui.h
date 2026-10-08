#pragma once
#include <Arduino.h>

// Dashboard served from the rover at http://<ip>/. Single self-contained page.
const char HTML_INDEX[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Rover Dashboard</title>
<style>
:root{--bg:#0f1115;--card:#181b22;--line:#2a2f3a;--fg:#e4e7ee;--dim:#8a93a6;--ok:#3ddc84;--warn:#ffb74d;--bad:#ff5c5c;--acc:#4da6ff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;touch-action:manipulation}
header{display:flex;flex-wrap:wrap;gap:10px;align-items:center;padding:10px 16px;border-bottom:1px solid var(--line)}
header h1{font-size:18px;margin:0;flex:1;color:var(--acc)}
.badge{padding:3px 9px;border-radius:5px;font-weight:600;font-size:12px}
.on{background:#12331f;color:var(--ok)}.off{background:#3a1616;color:var(--bad)}.neutral{background:#222733;color:var(--dim)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px;padding:12px 16px;max-width:1200px;margin:0 auto}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{margin:0 0 8px;font-size:12px;letter-spacing:.08em;color:var(--dim);font-weight:600;display:flex;justify-content:space-between;align-items:center}
canvas{width:100%;display:block;border-radius:6px;background:#0b0d11}
.row{display:flex;justify-content:space-between;font:13px ui-monospace,Menlo,monospace;margin:4px 0}
.row b{color:var(--ok);font-weight:600}
.wide{grid-column:1/-1}
button{background:#222733;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 12px;font-size:13px;cursor:pointer}
button:active{background:#2e3545}
button.stop{background:#5a1a1a;border-color:#8a2a2a;font-weight:700;width:100%;padding:12px;margin-top:8px}
.tilts{display:grid;grid-template-columns:1fr 1fr;gap:8px}
pre{margin:0;background:#0b0d11;color:#7fe3a5;border-radius:6px;padding:8px;font-size:11px;max-height:150px;overflow:auto}
input[type=range]{width:100%}
</style></head><body>
<header>
  <h1>ESP32-S3 Rover</h1>
  <span id="batt" class="badge neutral">BAT --</span>
  <span id="pair" class="badge neutral">PAIR --</span>
  <span id="link" class="badge off">OFFLINE</span>
</header>
<div class="grid">
  <div class="card"><h2>TOP VIEW &middot; HEADING <button onclick="send({cmd:'zero_yaw'})">Zero yaw</button></h2>
    <canvas id="top" width="300" height="260"></canvas>
    <div class="row"><span>Yaw (relative)</span><b id="yaw">0.0&deg;</b></div>
    <div class="row"><span>Turn rate</span><b id="gz">0.0 &deg;/s</b></div>
  </div>
  <div class="card"><h2>TILT <span id="tiltwarn"></span></h2>
    <div class="tilts"><canvas id="side" width="200" height="150"></canvas><canvas id="rear" width="200" height="150"></canvas></div>
    <div class="row"><span>Pitch (nose up +)</span><b id="pitch">0.0&deg;</b></div>
    <div class="row"><span>Roll (left side up +)</span><b id="roll">0.0&deg;</b></div>
  </div>
  <div class="card"><h2>G-METER (linear accel)</h2>
    <canvas id="gm" width="300" height="260"></canvas>
    <div class="row"><span>Long. / Lat.</span><b id="gtxt">0.00 / 0.00 g</b></div>
  </div>
  <div class="card"><h2>MOTORS</h2>
    <canvas id="mot" width="300" height="190"></canvas>
    <div class="row"><span>Left  D5/D6</span><b id="pl">0 / 0</b></div>
    <div class="row"><span>Right D9/D10</span><b id="pr">0 / 0</b></div>
    <div class="row"><span>Failsafe</span><b id="fs">ok</b></div>
  </div>
  <div class="card"><h2>DRIVE</h2>
    <canvas id="joy" width="300" height="220" style="touch-action:none"></canvas>
    <div class="row"><span>Speed limit</span><b id="limtxt">60%</b></div>
    <input id="lim" type="range" min="10" max="100" value="60">
    <div style="font-size:12px;color:var(--dim)">Drag the pad or use WASD / arrow keys. Space = stop.</div>
    <button class="stop" onclick="stopNow()">STOP</button>
  </div>
  <div class="card"><h2>SYSTEM <button onclick="send({cmd:'calibrate'})">Cal gyro</button></h2>
    <div class="row"><span>Temp</span><b id="temp">--</b></div>
    <div class="row"><span>Battery</span><b id="vbat">--</b></div>
    <div class="row"><span>Free heap</span><b id="heap">--</b></div>
    <div class="row"><span>Uptime</span><b id="up">--</b></div>
    <div class="row"><span>WS clients</span><b id="cl">--</b></div>
    <div class="row"><span>Frames / rate</span><b id="fr">--</b></div>
  </div>
  <div class="card wide"><h2>YAW RATE &middot; PITCH &middot; ROLL (10 s)</h2>
    <canvas id="chart" width="900" height="160"></canvas>
    <pre id="raw" style="margin-top:8px">waiting for data...</pre>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);
const D2R=Math.PI/180, G=9.80665;
let ws, last=null, frames=0, rate=0, rateT=Date.now(), lastMsg=0;
const hist={t:[],gz:[],pitch:[],roll:[]}; const trail=[];
let linf={x:0,y:0};

// ---------- WebSocket ----------
function connect(){
  ws=new WebSocket(`ws://${location.hostname}:81/`);
  ws.onopen=()=>{$('link').className='badge on';$('link').textContent='LIVE'};
  ws.onclose=()=>{$('link').className='badge off';$('link').textContent='OFFLINE';setTimeout(connect,1500)};
  ws.onmessage=e=>{try{last=JSON.parse(e.data)}catch(_){return}
    frames++;lastMsg=Date.now();render(last)};
}
function send(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o))}
connect();
setInterval(()=>{const n=Date.now();rate=frames*1000/(n-rateT);frames=0;rateT=n;
  if(last)$('fr').textContent='#'+last.seq+' / '+rate.toFixed(0)+' Hz';
  if(Date.now()-lastMsg>1500&&ws&&ws.readyState===1){$('link').className='badge off';$('link').textContent='STALE'}},1000);

// ---------- helpers ----------
// Draw in a fixed logical space (w x h) regardless of pixel size.
function ctxFor(id,w,h){const c=$(id);const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;
  const pw=Math.max(1,Math.round(r.width*d)),ph=Math.round(pw*h/w);
  if(c.width!==pw||c.height!==ph){c.width=pw;c.height=ph;c.style.aspectRatio=w+'/'+h}
  const x=c.getContext('2d');x.setTransform(pw/w,0,0,ph/h,0,0);x.clearRect(0,0,w,h);return x}
function col(v,a,b){return Math.abs(v)>b?'#ff5c5c':Math.abs(v)>a?'#ffb74d':'#3ddc84'}

// ---------- rover drawing ----------
const TIRE='#2b2f3a',RIM='#9aa3b5',BODY='#2a3b55',DECK='#3f5a85',LAMP='#ffe08a',TAIL='#ff5c5c';
function poly(x,pts,fill,stroke){x.beginPath();pts.forEach(([a,b],i)=>i?x.lineTo(a,b):x.moveTo(a,b));x.closePath();
  if(fill){x.fillStyle=fill;x.fill()}if(stroke){x.strokeStyle=stroke;x.lineWidth=2;x.stroke()}}
function dot(x,cx,cy,r,fill,stroke){x.beginPath();x.arc(cx,cy,r,0,7);
  if(fill){x.fillStyle=fill;x.fill()}if(stroke){x.strokeStyle=stroke;x.lineWidth=2;x.stroke()}}
function mast(x,bx,tipy,c){x.strokeStyle=RIM;x.lineWidth=2;x.beginPath();x.moveTo(bx,-40);x.lineTo(bx,tipy);x.stroke();dot(x,bx,tipy,3,c)}

// ---------- side / rear tilt ----------
function drawTilt(id,angle,label,rear){
  const x=ctxFor(id,200,150),c=col(angle,20,35);
  x.strokeStyle='#555';x.lineWidth=1;x.beginPath();x.moveTo(8,125);x.lineTo(192,125);x.stroke();
  x.strokeStyle='#3a3f4b';for(let g=14;g<192;g+=10){x.beginPath();x.moveTo(g,125);x.lineTo(g-4,130);x.stroke()}
  x.save();x.translate(100,125);x.scale(1.2,1.2);
  // pivot on the tyre that stays on the ground (rear tyre nose-up / right tyre left-side-up)
  const pv=rear?(angle>0?40:-40):(angle>0?-28:28);
  x.translate(pv,0);x.rotate((rear?angle:-angle)*D2R);x.translate(-pv,0);
  if(rear){
    [[-46,-34],[34,46]].forEach(([a,b])=>poly(x,[[a,-22],[b,-22],[b,0],[a,0]],TIRE,RIM));
    poly(x,[[-32,-24],[-28,-30],[28,-30],[32,-24],[32,-16],[28,-14],[-28,-14],[-32,-16]],BODY,c);
    poly(x,[[-20,-30],[-16,-40],[16,-40],[20,-30]],DECK,c);
    [-26,26].forEach(sx=>poly(x,[[sx-3,-26],[sx+3,-26],[sx+3,-20],[sx-3,-20]],TAIL));
    mast(x,0,-50,c);
  }else{
    poly(x,[[-46,-24],[-42,-30],[42,-30],[46,-24],[46,-16],[42,-14],[-42,-14],[-46,-16]],BODY,c);
    poly(x,[[-24,-30],[-19,-40],[15,-40],[20,-30]],DECK,c);
    poly(x,[[44,-26],[48,-26],[48,-20],[44,-20]],LAMP);
    poly(x,[[-48,-26],[-44,-26],[-44,-20],[-48,-20]],TAIL);
    mast(x,-6,-50,c);
    [-28,28].forEach(wx=>{dot(x,wx,-11,11,TIRE,RIM);dot(x,wx,-11,3.9,RIM)});
  }
  x.restore();
  x.fillStyle='#8a93a6';x.font='11px sans-serif';x.fillText(label,6,14);
  x.fillStyle=c;x.font='bold 16px sans-serif';x.fillText(angle.toFixed(1)+'°',6,34);
}

// ---------- top view ----------
function drawTop(yaw,l,r){
  const x=ctxFor('top',300,260),cx=150,cy=130;
  x.strokeStyle='#2a2f3a';x.lineWidth=1;x.beginPath();x.arc(cx,cy,115,0,7);x.stroke();
  for(let a=0;a<360;a+=30){const t=(a+yaw)*D2R;const r0=a%90?108:100;
    x.beginPath();x.moveTo(cx+Math.sin(t)*r0,cy-Math.cos(t)*r0);x.lineTo(cx+Math.sin(t)*115,cy-Math.cos(t)*115);x.stroke()}
  // reference marker: where yaw==0 points
  x.fillStyle='#4da6ff';x.font='11px sans-serif';x.textAlign='center';
  const t0=yaw*D2R; // a left (CCW) turn moves the reference clockwise on screen
  x.fillText('0°',cx+Math.sin(t0)*127,cy-Math.cos(t0)*127+4);x.textAlign='left';
  // rover (always pointing up, the world rotates)
  x.save();x.translate(cx,cy);x.scale(1.5,1.5);
  [[-26,l],[18,r]].forEach(([px,v])=>{            // tyres, speed fill: green fwd / amber reverse
    x.fillStyle=TIRE;x.fillRect(px,-22,8,44);x.strokeStyle=RIM;x.lineWidth=1.2;x.strokeRect(px,-22,8,44);
    x.strokeStyle='#444b5a';x.lineWidth=1;
    for(let ty=-18;ty<22;ty+=8){x.beginPath();x.moveTo(px+1,ty);x.lineTo(px+7,ty);x.stroke()}
    const len=Math.abs(v)*20;x.fillStyle=v>=0?'#3ddc84':'#ffb74d';
    if(v>=0)x.fillRect(px+2,-len,4,len);else x.fillRect(px+2,0,4,len)});
  poly(x,[[-15,-24],[-11,-30],[11,-30],[15,-24],[15,24],[11,30],[-11,30],[-15,24]],BODY,'#4da6ff');
  x.fillStyle=DECK;x.fillRect(-9,-14,18,32);
  [[-9,-27,LAMP],[9,-27,LAMP],[-9,27,TAIL],[9,27,TAIL]].forEach(([a,b,f])=>dot(x,a,b,2.5,f));
  poly(x,[[0,-46],[-6,-36],[6,-36]],'#4da6ff');   // heading arrow
  x.restore();
}

// ---------- g-meter ----------
function drawG(lx,ly){
  const x=ctxFor('gm',300,260),cx=150,cy=130,R=110,S=R/0.6; // 0.6 g full scale
  x.strokeStyle='#2a2f3a';[0.2,0.4,0.6].forEach(g=>{x.beginPath();x.arc(cx,cy,g*S,0,7);x.stroke()});
  x.beginPath();x.moveTo(cx-R,cy);x.lineTo(cx+R,cy);x.moveTo(cx,cy-R);x.lineTo(cx,cy+R);x.stroke();
  x.fillStyle='#8a93a6';x.font='10px sans-serif';x.fillText('accel',cx-9,cy-R-3);x.fillText('brake',cx-10,cy+R+11);
  const px=g=>[cx-g.y*S,cy-g.x*S]; // lateral: left=+ shown on left
  trail.forEach((g,i)=>{const [a,b]=px(g);x.fillStyle=`rgba(77,166,255,${i/trail.length*0.5})`;x.beginPath();x.arc(a,b,3,0,7);x.fill()});
  const [a,b]=px({x:lx,y:ly});x.fillStyle='#3ddc84';x.beginPath();x.arc(a,b,6,0,7);x.fill();
}

// ---------- motors ----------
function drawMotors(l,r){
  const x=ctxFor('mot',300,190);
  [[70,l,'LEFT'],[190,r,'RIGHT']].forEach(([px,v,n])=>{
    x.fillStyle='#0b0d11';x.fillRect(px,10,40,150);x.strokeStyle='#2a2f3a';x.strokeRect(px,10,40,150);
    x.strokeStyle='#666';x.beginPath();x.moveTo(px-6,85);x.lineTo(px+46,85);x.stroke();
    const h=Math.abs(v)*75;x.fillStyle=v>=0?'#3ddc84':'#ffb74d';
    if(v>=0)x.fillRect(px+3,85-h,34,h);else x.fillRect(px+3,85,34,h);
    x.fillStyle='#e4e7ee';x.font='bold 13px sans-serif';x.textAlign='center';
    x.fillText((v*100).toFixed(0)+'%',px+20,180);x.fillStyle='#8a93a6';x.font='10px sans-serif';x.fillText(n,px+20,8);
  });x.textAlign='left';
}

// ---------- strip chart ----------
function drawChart(){
  const x=ctxFor('chart',900,160),n=hist.t.length;if(n<2)return;
  const t1=hist.t[n-1],span=10000;
  x.strokeStyle='#2a2f3a';x.beginPath();x.moveTo(0,80);x.lineTo(900,80);x.stroke();
  x.fillStyle='#8a93a6';x.font='10px sans-serif';x.fillText('±80° / ±80°/s',4,10);
  [['gz','#ffb74d','yaw rate'],['pitch','#4da6ff','pitch'],['roll','#3ddc84','roll']].forEach(([k,c,name],i)=>{
    x.strokeStyle=c;x.lineWidth=1.5;x.beginPath();
    for(let j=0;j<n;j++){const px=900-(t1-hist.t[j])/span*900,py=80-Math.max(-80,Math.min(80,hist[k][j]));
      j?x.lineTo(px,py):x.moveTo(px,py)}x.stroke();
    x.fillStyle=c;x.fillText(name,820,12+i*12)});
}

// ---------- render telemetry ----------
function fmtUp(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+(s%60)+'s'}
function render(d){
  const a=d.att,i=d.imu,m=d.motors;
  $('pitch').textContent=a.pitch.toFixed(1)+'°';$('roll').textContent=a.roll.toFixed(1)+'°';
  $('yaw').textContent=a.yaw.toFixed(1)+'°';$('gz').textContent=i.gz.toFixed(1)+' °/s';
  $('temp').textContent=d.temp==null?'n/a':d.temp.toFixed(1)+' °C';
  $('vbat').textContent=d.vbat==null?'n/a':d.vbat.toFixed(2)+' V';
  $('batt').textContent=d.vbat==null?'BAT n/a':'BAT '+d.vbat.toFixed(2)+'V';
  $('heap').textContent=(d.heap/1024).toFixed(0)+' KB';$('up').textContent=fmtUp(d.uptime_s);$('cl').textContent=d.clients;
  $('pl').textContent=m.pwm[0]+' / '+m.pwm[1];$('pr').textContent=m.pwm[2]+' / '+m.pwm[3];
  $('fs').textContent=d.failsafe?'TRIPPED (no cmd)':'ok';$('fs').style.color=d.failsafe?'#ff5c5c':'';
  $('pair').textContent=d.pairing.open?'PAIRING '+d.pairing.secs:'PAIR closed';
  $('pair').className='badge '+(d.pairing.open?'on':'neutral');
  $('tiltwarn').textContent=(Math.abs(a.pitch)>35||Math.abs(a.roll)>35)?'⚠ TIP RISK':'';
  $('tiltwarn').style.color='#ff5c5c';
  // gravity-compensated linear acceleration in g (see firmware axis convention)
  const p=a.pitch*D2R,r=a.roll*D2R;
  const lx=(i.ax-G*Math.sin(p))/G, ly=(i.ay-G*Math.cos(p)*Math.sin(r))/G;
  linf.x+=0.3*(lx-linf.x);linf.y+=0.3*(ly-linf.y);
  trail.push({x:linf.x,y:linf.y});if(trail.length>40)trail.shift();
  $('gtxt').textContent=linf.x.toFixed(2)+' / '+linf.y.toFixed(2)+' g';
  const t=Date.now();hist.t.push(t);hist.gz.push(i.gz);hist.pitch.push(a.pitch);hist.roll.push(a.roll);
  while(hist.t.length&&t-hist.t[0]>10000)for(const k in hist)hist[k].shift();
  drawTop(a.yaw,m.left,m.right);drawTilt('side',a.pitch,'SIDE (pitch)',false);drawTilt('rear',a.roll,'REAR (roll)',true);
  drawG(linf.x,linf.y);drawMotors(m.left,m.right);drawChart();
  $('raw').textContent=JSON.stringify(d);
}

// ---------- driving ----------
let jx=0,jy=0,dragging=false,keys=new Set(),lim=0.6;
$('lim').oninput=e=>{lim=e.target.value/100;$('limtxt').textContent=e.target.value+'%'};
function mix(t,s){let l=t+s,r=t-s;const m=Math.max(1,Math.abs(l),Math.abs(r));return[l/m*lim,r/m*lim]}
function drawJoy(){
  const x=ctxFor('joy',300,220),cx=150,cy=110,R=90;
  x.strokeStyle='#2a2f3a';x.lineWidth=2;x.beginPath();x.arc(cx,cy,R,0,7);x.stroke();
  x.lineWidth=1;x.beginPath();x.moveTo(cx-R,cy);x.lineTo(cx+R,cy);x.moveTo(cx,cy-R);x.lineTo(cx,cy+R);x.stroke();
  x.fillStyle=dragging||keys.size?'#3ddc84':'#4da6ff';x.beginPath();x.arc(cx+jx*R,cy-jy*R,18,0,7);x.fill();
}
function joyPos(e){const c=$('joy'),r=c.getBoundingClientRect();
  const sx=300/r.width,sy=220/r.height;
  let x=((e.clientX-r.left)*sx-150)/90,y=-((e.clientY-r.top)*sy-110)/90;
  const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}jx=x;jy=y}
$('joy').addEventListener('pointerdown',e=>{dragging=true;$('joy').setPointerCapture(e.pointerId);joyPos(e);drawJoy()});
$('joy').addEventListener('pointermove',e=>{if(dragging){joyPos(e);drawJoy()}});
['pointerup','pointercancel'].forEach(n=>$('joy').addEventListener(n,()=>{dragging=false;jx=jy=0;drawJoy();stopNow()}));
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;const k=e.key.toLowerCase();
  if(k===' '){stopNow();e.preventDefault();return}
  if('wasd'.includes(k)||k.startsWith('arrow')){keys.add(k);e.preventDefault()}});
addEventListener('keyup',e=>{keys.delete(e.key.toLowerCase());if(!keys.size&&!dragging){jx=jy=0;stopNow()}});
addEventListener('blur',()=>{keys.clear();dragging=false;jx=jy=0;stopNow()});
function stopNow(){jx=jy=0;keys.clear();send({cmd:'stop'});drawJoy()}
setInterval(()=>{
  if(keys.size){jy=(keys.has('w')||keys.has('arrowup')?1:0)-(keys.has('s')||keys.has('arrowdown')?1:0);
    jx=(keys.has('d')||keys.has('arrowright')?1:0)-(keys.has('a')||keys.has('arrowleft')?1:0);drawJoy()}
  if(dragging||keys.size){const[l,r]=mix(jy,jx);send({cmd:'drive',left:+l.toFixed(3),right:+r.toFixed(3)})}
},100);
drawJoy();
addEventListener('resize',()=>{drawJoy();if(last)render(last)});
</script></body></html>
)rawliteral";
