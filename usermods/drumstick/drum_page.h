#pragma once
// Embedded /drum web interface — 2-zone kinematic engine (DOWN / UP) + BNO055 diagnostics.
// Do not edit directly; maintained as part of drumstick usermod source.

static const char DRUM_PAGE[] PROGMEM = R"RAWHTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drumstick</title>
<style>
  :root{--bg:#09141b;--panel:#102431;--text:#edf6ff;--muted:#9fc0d4;
        --red:#cf453d;--cyan:#2ea9d0;--good:#68d391;}
  *{box-sizing:border-box} body{background:radial-gradient(circle at top,#163447 0,#09141b 52%,#050b0f 100%);
      color:var(--text);font-family:Segoe UI,system-ui,sans-serif;margin:0;padding:16px;}
  .wrap{max-width:1280px;margin:0 auto;display:grid;gap:12px}
  .hero{display:flex;justify-content:space-between;gap:12px;align-items:end;flex-wrap:wrap}
  h1{margin:0;font-size:1.6rem;letter-spacing:.08em}
  #status{font-size:.85rem;color:var(--muted)}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}
  .panel{background:linear-gradient(180deg,rgba(21,46,61,.92),rgba(10,24,34,.96));
         border:1px solid rgba(92,149,184,.35);border-radius:14px;padding:12px;
         box-shadow:0 14px 40px rgba(0,0,0,.22)}
  .panel h2{margin:0 0 8px;font-size:.95rem;letter-spacing:.05em;color:#dff0ff}
  #pads{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .pad{border-radius:10px;height:80px;display:flex;align-items:center;justify-content:center;
       font-size:1.1rem;font-weight:700;letter-spacing:.04em;
       transition:transform .08s,filter .08s;filter:brightness(.28)}
  .pad.hit{filter:brightness(1.0);transform:translateY(-3px)}
  #p0{background:var(--red)} #p1{background:var(--cyan)}
  .kv{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:6px;font-size:.78rem}
  .kv2{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:6px;font-size:.78rem}
  .mini{background:rgba(4,11,16,.24);border:1px solid rgba(92,149,184,.18);border-radius:8px;padding:7px}
  .mini b{display:block;font-size:.68rem;color:var(--muted);font-weight:600;margin-bottom:3px}
  .controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px}
  label{display:flex;flex-direction:column;gap:4px;font-size:.75rem;color:var(--muted)}
  input,select,button{border-radius:8px;border:1px solid rgba(92,149,184,.35);
                      background:#0a1a24;color:var(--text);padding:7px 9px;font:inherit}
  button{cursor:pointer;background:linear-gradient(180deg,#17354a,#102633)}
  button.on{background:linear-gradient(180deg,#1a5c2a,#0e3a19);border-color:#2d8a46}
  button.secondary{background:linear-gradient(180deg,#13242f,#0d1921)}
  .badge{display:inline-block;padding:2px 7px;border-radius:5px;font-size:.7rem;font-weight:700}
  .badge.ok{background:#1a4a2e;color:var(--good)} .badge.err{background:#4a1a1a;color:var(--red)}
  .badge.streaming{background:#1a3a4a;color:var(--cyan);animation:pulse 1s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
  .meters{display:grid;gap:8px}
  .meter{display:grid;gap:4px}
  .meterRow{display:flex;justify-content:space-between;font-size:.75rem;color:var(--muted)}
  .track{height:10px;border-radius:999px;background:#08131a;
         border:1px solid rgba(92,149,184,.25);position:relative;overflow:hidden}
  .fill{height:100%;background:linear-gradient(90deg,#2ea9d0,#68d391);width:0%;transition:width .08s}
  .calFill{height:100%;width:0%;border-radius:999px;transition:width .3s,background .6s}
  .marker{position:absolute;top:0;bottom:0;width:2px;background:#ffdc73;opacity:.95}
  canvas{width:100%;background:radial-gradient(circle at center,#112533 0,#0a1720 65%,#081118 100%);
         border-radius:10px;border:1px solid rgba(92,149,184,.22)}
  canvas.sq{aspect-ratio:1/1}
  .row{display:flex;gap:7px;align-items:center;flex-wrap:wrap}
  .muted{color:var(--muted);font-size:.75rem}
  #log{height:130px;overflow-y:auto;font-size:.72rem;color:#a8c2d3;
       border-top:1px solid rgba(92,149,184,.2);padding-top:7px;margin-top:8px}
  @media(max-width:700px){.kv{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style></head><body>
<div class="wrap">
<div class="hero">
  <div><h1>&#x1F941; Drumstick</h1><div id="status">connecting&#x2026;</div></div>
  <div class="row muted"><span>Vol</span>
    <input id="vr" type="range" min="0" max="1" step=".05" value="0.8" style="width:90px">
  </div>
</div>

<div class="grid">
  <section class="panel">
    <h2>Live Pads</h2>
    <div id="pads">
      <div class="pad" id="p0">&#x2B07; DOWN</div>
      <div class="pad" id="p1">&#x2B06; UP</div>
    </div>
    <div class="kv2" style="margin-top:8px">
      <div class="mini"><b>Cal (Sys)</b><span id="calSysInline">0/3</span></div>
      <div class="mini"><b>Last hit</b><span id="zonePreview">&#x2014;</span></div>
    </div>
  </section>

  <section class="panel">
    <h2>Kinematic Engine <span id="streamBadge" class="badge err">off</span></h2>
    <div class="row" style="margin-bottom:10px">
      <button id="btnStream">Start stream (100&#x202F;Hz)</button>
    </div>
    <div class="meters">
      <div class="meter">
        <div class="meterRow"><span>|velocity| m/s</span><span id="velText">0.00</span></div>
        <div class="track"><div class="fill" id="velFill"></div><div class="marker" id="velMark"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>|accel| m/s&#xB2;</span><span id="accText">0.00</span></div>
        <div class="track"><div class="fill" id="accFill"></div></div>
      </div>
    </div>
    <div class="kv" style="margin-top:8px">
      <div class="mini"><b>Peak vel m/s</b><span id="peakVelVal">0.00</span></div>
      <div class="mini"><b>Vert vel m/s</b><span id="vzVal">0.00</span></div>
      <div class="mini"><b>ZVU</b><span id="zvuState">&#x2014;</span></div>
    </div>
    <div class="controls" style="margin-top:8px">
      <label>Vel onset m/s<input id="velOnset" type="number" min="0.05" step="0.05" value="0.35"></label>
      <label>ZVU thresh m/s&#xB2;<input id="zvuThresh" type="number" min="0.05" step="0.05" value="0.25"></label>
      <label>ZVU window<input id="zvuWin" type="number" min="1" max="20" step="1" value="5"></label>
    </div>
    <div class="muted" style="margin-top:6px">
      Zone = sign of peak world-frame vertical velocity:
      &#x2B07; DOWN (v&#x2093;&lt;0) &#xB7; &#x2B06; UP (v&#x2093;&#x2265;0).
      No calibration required &#x2014; BNO055 NDOF gives absolute ENU orientation.
    </div>
  </section>
</div>

<div class="grid">
  <section class="panel">
    <h2>Trajectory (side view)</h2>
    <canvas id="posCanvas" class="sq"></canvas>
    <div class="muted">Side view: horizontal distance vs vertical height.
    Red&#x202F;=&#x202F;DOWN stroke, Cyan&#x202F;=&#x202F;UP stroke.
    Circle&#x202F;=&#x202F;velocity-onset radius.</div>
  </section>

  <section class="panel">
    <h2>Velocity History</h2>
    <canvas id="velCanvas" style="aspect-ratio:3/1;height:120px"></canvas>
    <div class="kv2" style="margin-top:8px">
      <div class="mini"><b>World accel (m/s&#xB2;)</b><span id="worldAccVal">0 0 0</span></div>
      <div class="mini"><b>Velocity (m/s)</b><span id="velXYZ">0 0 0</span></div>
    </div>
  </section>
</div>

<div class="grid">
  <section class="panel">
    <h2>BNO055 Diagnostics</h2>
    <div class="kv" style="margin-bottom:8px">
      <div class="mini"><b>IMU</b><span id="bnoStatus"><span class="badge err">&#x2014;</span></span></div>
      <div class="mini"><b>Mode</b><span id="bnoMode">&#x2014;</span></div>
      <div class="mini"><b>Temp</b><span id="bnoTemp">&#x2014;</span></div>
      <div class="mini"><b>Battery</b><span id="battVal">&#x2014;</span></div>
    </div>
    <div class="meters">
      <div class="meter">
        <div class="meterRow"><span>Sys cal</span><span id="calSysT">0/3</span></div>
        <div class="track"><div class="calFill" id="calSysF"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Gyro cal</span><span id="calGyrT">0/3</span></div>
        <div class="track"><div class="calFill" id="calGyrF"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Accel cal</span><span id="calAccT">0/3</span></div>
        <div class="track"><div class="calFill" id="calAccF"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Mag cal</span><span id="calMagT">0/3</span></div>
        <div class="track"><div class="calFill" id="calMagF"></div></div>
      </div>
    </div>
    <div class="kv2" style="margin-top:8px">
      <div class="mini"><b>Quat W X Y Z</b><span id="quatVal">&#x2014;</span></div>
      <div class="mini"><b>Euler H/R/P &#xB0;</b><span id="eulerVal">&#x2014;</span></div>
    </div>
    <div class="row" style="margin-top:8px">
      <button id="selfTest" class="secondary">Run self-test</button>
    </div>
    <div class="muted" style="margin-top:6px">
      Calibration guide: keep still 30&#x202F;s (gyro&#x2192;3), wave slowly (accel&#x2192;3),
      figure-8 rotation (mag&#x2192;3). Sys&#x202F;3&#x202F;=&#x202F;full NDOF fusion active.
    </div>
  </section>

  <section class="panel">
    <h2>Sensor Debug</h2>
    <div class="kv">
      <div class="mini"><b>Accel m/s&#xB2;</b><span id="accelVal">&#x2014;</span></div>
      <div class="mini"><b>LinAcc m/s&#xB2;</b><span id="linVal">&#x2014;</span></div>
      <div class="mini"><b>Gyro &#xB0;/s</b><span id="gyroVal">&#x2014;</span></div>
      <div class="mini"><b>Mag &#xB5;T</b><span id="magVal">&#x2014;</span></div>
    </div>
    <div id="log"></div>
  </section>
</div>
</div>

<script>
// ── Web Audio synth engine ─────────────────────────────────────────────────
let ctx=null;
function getCtx(){
  if(!ctx)ctx=new(window.AudioContext||window.webkitAudioContext)();
  if(ctx.state==='suspended')ctx.resume();return ctx;
}
function masterGain(){
  if(!masterGain._n){masterGain._n=getCtx().createGain();masterGain._n.connect(getCtx().destination);}
  masterGain._n.gain.value=parseFloat(document.getElementById('vr').value);
  return masterGain._n;
}
function adsr(g,a,d,s,r,now){
  g.gain.cancelScheduledValues(now);g.gain.setValueAtTime(0,now);
  g.gain.linearRampToValueAtTime(1,now+a);g.gain.linearRampToValueAtTime(s,now+a+d);
  g.gain.setValueAtTime(s,now+a+d+.001);g.gain.linearRampToValueAtTime(0,now+a+d+r);
}
// Zone 0 = DOWN: tight snare-like hit
// Zone 1 = UP:   open hi-hat / cymbal shimmer
const SYNTHS=[
  function down(vel){const C=getCtx(),t=C.currentTime,v=vel/127,dur=.18;
    const buf=C.createBuffer(1,C.sampleRate*dur,C.sampleRate);const d=buf.getChannelData(0);
    for(let i=0;i<d.length;i++)d[i]=(Math.random()*2-1);
    const ns=C.createBufferSource();ns.buffer=buf;
    const nf=C.createBiquadFilter();nf.type='bandpass';nf.frequency.value=2200;nf.Q.value=.9;
    const ng=C.createGain();adsr(ng,.002,.04,.1,.09,t);
    ns.connect(nf);nf.connect(ng);ng.connect(masterGain());ns.start(t);ns.stop(t+dur);
    const osc=C.createOscillator();osc.type='triangle';osc.frequency.value=200;
    osc.frequency.linearRampToValueAtTime(80,t+.05);
    const og=C.createGain();og.gain.setValueAtTime(.6*v,t);og.gain.linearRampToValueAtTime(0,t+.12);
    osc.connect(og);og.connect(masterGain());osc.start(t);osc.stop(t+.14);},
  function up(vel){const C=getCtx(),t=C.currentTime,v=vel/127,dur=.28;
    const buf=C.createBuffer(1,C.sampleRate*dur,C.sampleRate);const d=buf.getChannelData(0);
    for(let i=0;i<d.length;i++)d[i]=(Math.random()*2-1)*Math.exp(-i/(C.sampleRate*0.18));
    const ns=C.createBufferSource();ns.buffer=buf;
    const hf=C.createBiquadFilter();hf.type='highpass';hf.frequency.value=6000;
    const rf=C.createBiquadFilter();rf.type='peaking';rf.frequency.value=9000;rf.gain.value=8;
    const g=C.createGain();g.gain.setValueAtTime(.8*v,t);g.gain.linearRampToValueAtTime(0,t+dur);
    ns.connect(hf);hf.connect(rf);rf.connect(g);g.connect(masterGain());ns.start(t);ns.stop(t+dur);}
];

const NAMES=['DOWN','UP'];
// RGB components for zone colours — used in canvas rgba() strings
const ZONE_RGB=['207,69,61','46,169,208'];
const ZONE_CSS=['#cf453d','#2ea9d0'];
const log=document.getElementById('log');
const st=document.getElementById('status');
function addLog(msg){
  const l=document.createElement('div');l.textContent=new Date().toLocaleTimeString()+' '+msg;
  log.prepend(l);while(log.childNodes.length>80)log.removeChild(log.lastChild);
}
function send(obj){if(ws&&ws.readyState===1)ws.send(JSON.stringify(obj));}
function fmt(v,d=2){return(typeof v==='number'&&isFinite(v))?v.toFixed(d):'0';}
function fmti(v){return(typeof v==='number'&&isFinite(v))?Math.round(v):'0';}
function clamp(v,a,b){return Math.min(b,Math.max(a,v));}

// ── State ──────────────────────────────────────────────────────────────────
const state={
  tele:{ok:0,ax:0,ay:0,az:0,lax:0,lay:0,laz:0,gx:0,gy:0,gz:0,mx:0,my:0,mz:0,
        cal:0,cg:0,ca:0,cm:0,qw:1,qx:0,qy:0,qz:0,eh:0,er:0,ep:0,temp:0,batt:0,battp:0},
  streaming:false
};

// ── Kinematic Engine ───────────────────────────────────────────────────────
// AI: below section was generated by an AI
//
// Rotates body-frame linear acceleration (BNO055 VECTOR_LINEARACCEL) into
// world (ENU) frame using the absolute orientation quaternion.
// Uses Rodrigues sandwich: t = 2*(q.xyz x v);  v' = v + q.w*t + q.xyz x t
// Reference: https://en.wikipedia.org/wiki/Quaternion#Hamilton_product
function rotByQuat(qw,qx,qy,qz, vx,vy,vz){
  const tx=2*(qy*vz-qz*vy), ty=2*(qz*vx-qx*vz), tz=2*(qx*vy-qy*vx);
  return [vx+qw*tx+(qy*tz-qz*ty), vy+qw*ty+(qz*tx-qx*tz), vz+qw*tz+(qx*ty-qy*tx)];
}

class KinematicEngine {
  constructor(){
    this.vel=[0,0,0]; this.pos=[0,0,0];
    this.worldAcc=[0,0,0];
    this.prevT=0; this.restCount=0; this.isResting=false;
    this.params={zvuThresh:0.25, zvuWin:5, velOnset:0.35};
    this.trace=[]; // {x,y,z,zone} position history for canvas
  }
  update(pkt){
    const dt=(this.prevT>0)?((pkt.t-this.prevT)/1000.0):0;
    this.prevT=pkt.t;
    if(dt<=0||dt>0.5)return;
    this.worldAcc=rotByQuat(pkt.qw,pkt.qx,pkt.qy,pkt.qz, pkt.lax,pkt.lay,pkt.laz);
    const aMag=Math.sqrt(this.worldAcc[0]**2+this.worldAcc[1]**2+this.worldAcc[2]**2);
    // Zero-velocity update: reset velocity when nearly at rest to stop drift
    if(aMag<this.params.zvuThresh){
      this.restCount++;
      if(this.restCount>=this.params.zvuWin){this.vel=[0,0,0];this.isResting=true;}
    }else{this.restCount=0;this.isResting=false;}
    this.vel[0]+=this.worldAcc[0]*dt;
    this.vel[1]+=this.worldAcc[1]*dt;
    this.vel[2]+=this.worldAcc[2]*dt;
    this.pos[0]+=this.vel[0]*dt;
    this.pos[1]+=this.vel[1]*dt;
    this.pos[2]+=this.vel[2]*dt;
    for(let i=0;i<3;i++) this.pos[i]=clamp(this.pos[i],-5,5);
    this.trace.push({x:this.pos[0],y:this.pos[1],z:this.pos[2],zone:0});
    if(this.trace.length>300)this.trace.shift();
  }
  velMag(){return Math.sqrt(this.vel.reduce((s,v)=>s+v*v,0));}
  accMag(){return Math.sqrt(this.worldAcc.reduce((s,v)=>s+v*v,0));}
}

class BrowserSwingFSM {
  constructor(engine){
    this.eng=engine; this.state='IDLE';
    this.peakVel=0; this.peakVelVec=[0,0,0];
    this.samples=0; this.lastHit=0;
    this.DECAY=0.4; this.MAX_SAMPLES=150; this.COOLDOWN_MS=120;
  }
  // Returns hit object {zone, vel, vz} or null
  // AI: below section was generated by an AI
  // Kinematic swing detector:
  //   IDLE  → velocity onset threshold       → SWING
  //   SWING → peak tracking + wrist-stop     → DECAY
  //   DECAY → classify zone by vertical sign → emit hit
  // Zone 0 = DOWN (peak vz < 0), Zone 1 = UP (peak vz >= 0).
  process(pkt){
    const vm=this.eng.velMag();
    const p=this.eng.params;
    const now=pkt.t;
    switch(this.state){
      case 'IDLE':
        if(vm>=p.velOnset){this.state='SWING';this.peakVel=vm;this.peakVelVec=[...this.eng.vel];this.samples=1;}
        break;
      case 'SWING':
        this.samples++;
        if(vm>this.peakVel){this.peakVel=vm;this.peakVelVec=[...this.eng.vel];}
        if(this.samples>=3&&vm<this.peakVel*this.DECAY){this.state='DECAY';break;}
        if(this.samples>this.MAX_SAMPLES)this.state='IDLE';
        break;
      case 'DECAY':
        this.state='IDLE';
        if(now-this.lastHit<this.COOLDOWN_MS)return null;
        if(this.peakVel<p.velOnset*0.8)return null;
        {const hit=this._classify();this.lastHit=now;return hit;}
    }
    return null;
  }
  // AI: end
  _classify(){
    const [vx,vy,vz]=this.peakVelVec;
    // Zone is purely determined by the vertical (Z) component of peak velocity.
    // ENU world frame: Z = Up, so vz >= 0 means the dominant motion was upward.
    const zone=vz>=0?1:0;
    const midiVel=this._calcVel(this.peakVel);
    return {zone, vel:midiVel, peakVel:this.peakVel, vz};
  }
  _calcVel(v){
    const onset=this.eng.params.velOnset,top=onset*6;
    if(v<=onset)return 1;
    const t=Math.min(1,(v-onset)/(top-onset));
    return Math.round(1+t*126);
  }
}
// AI: end

// ── Velocity history canvas ────────────────────────────────────────────────
const VEL_HIST_LEN=80;
const velHistory=new Float32Array(VEL_HIST_LEN);
let velHistIdx=0;
function pushVelHist(v){velHistory[velHistIdx%VEL_HIST_LEN]=v;velHistIdx++;}

function drawVelCanvas(){
  const cv=document.getElementById('velCanvas');
  const w=cv.width=cv.clientWidth,h=cv.height=cv.clientHeight||120;
  const c=cv.getContext('2d');
  c.clearRect(0,0,w,h);
  const maxV=Math.max(2,...velHistory);
  const sw=w/VEL_HIST_LEN;
  c.strokeStyle='rgba(92,149,184,.18)';c.lineWidth=1;
  c.beginPath();c.moveTo(0,h/2);c.lineTo(w,h/2);c.stroke();
  const onsetY=h*(1-parseFloat(document.getElementById('velOnset').value)/maxV);
  c.strokeStyle='rgba(255,220,115,.5)';c.beginPath();c.moveTo(0,onsetY);c.lineTo(w,onsetY);c.stroke();
  c.beginPath();c.strokeStyle='#2ea9d0';c.lineWidth=1.5;
  for(let i=0;i<VEL_HIST_LEN;i++){
    const idx=(velHistIdx+i)%VEL_HIST_LEN;
    const x=i*sw,y=h*(1-velHistory[idx]/maxV);
    if(i===0)c.moveTo(x,y);else c.lineTo(x,y);
  }
  c.stroke();
}

// ── Trajectory canvas — side view (horizontal dist vs vertical) ───────────
function drawPosCanvas(engine){
  const cv=document.getElementById('posCanvas');
  const w=cv.width=cv.clientWidth,h=cv.height=cv.clientWidth;
  const c=cv.getContext('2d');
  const cx=w/2,cy=h/2,scale=w*0.25;
  c.clearRect(0,0,w,h);
  // grid lines
  c.strokeStyle='rgba(120,175,209,.12)';c.lineWidth=1;
  for(let r=1;r<=3;r++){c.beginPath();c.arc(cx,cy,scale*r,0,Math.PI*2);c.stroke();}
  c.beginPath();c.moveTo(cx,14);c.lineTo(cx,h-14);c.moveTo(14,cy);c.lineTo(w-14,cy);c.stroke();
  // axis labels
  c.fillStyle='rgba(159,192,212,.7)';c.font='11px Segoe UI';
  c.fillText('UP',cx+5,18);c.fillText('DN',cx+5,h-6);
  c.fillText('AWAY',w-38,cy-4);
  // velocity-onset ring
  const onsetR=Math.min(parseFloat(document.getElementById('velOnset').value)*scale,w*0.45);
  c.strokeStyle='rgba(255,220,115,.5)';c.beginPath();c.arc(cx,cy,onsetR,0,Math.PI*2);c.stroke();
  // trace — x axis = horizontal distance, y axis = vertical (z)
  const tr=engine.trace;
  if(tr.length<2)return;
  for(let i=1;i<tr.length;i++){
    const alpha=i/tr.length;
    const p=tr[i-1],q=tr[i];
    const ph=Math.sqrt(p.x**2+p.y**2);
    const qh=Math.sqrt(q.x**2+q.y**2);
    c.strokeStyle=`rgba(${ZONE_RGB[q.zone||0]},${(alpha*0.75).toFixed(2)})`;
    c.lineWidth=alpha*2+0.5;
    c.beginPath();c.moveTo(cx+ph*scale,cy-p.z*scale);c.lineTo(cx+qh*scale,cy-q.z*scale);c.stroke();
  }
  const last=tr[tr.length-1];
  const lh=Math.sqrt(last.x**2+last.y**2);
  c.fillStyle=ZONE_CSS[last.zone||0];
  c.beginPath();c.arc(cx+lh*scale,cy-last.z*scale,5,0,Math.PI*2);c.fill();
}

// ── Engine & FSM instances ─────────────────────────────────────────────────
const engine=new KinematicEngine();
const swingFSM=new BrowserSwingFSM(engine);

function syncEngineParams(){
  engine.params.zvuThresh=parseFloat(document.getElementById('zvuThresh').value)||0.25;
  engine.params.zvuWin   =parseInt(document.getElementById('zvuWin').value)||5;
  engine.params.velOnset =parseFloat(document.getElementById('velOnset').value)||0.35;
}
['zvuThresh','zvuWin','velOnset'].forEach(id=>
  document.getElementById(id).addEventListener('change',syncEngineParams));

// ── Handle raw stream packet ───────────────────────────────────────────────
function handleRaw(pkt){
  engine.update(pkt);
  const vm=engine.velMag();
  const am=engine.accMag();
  pushVelHist(vm);
  const maxVm=Math.max(vm*2,engine.params.velOnset*3,1);
  document.getElementById('velFill').style.width=(100*clamp(vm/maxVm,0,1)).toFixed(1)+'%';
  document.getElementById('velMark').style.left=(100*clamp(engine.params.velOnset/maxVm,0,1)).toFixed(1)+'%';
  document.getElementById('velText').textContent=fmt(vm);
  document.getElementById('accFill').style.width=(100*clamp(am/20,0,1)).toFixed(1)+'%';
  document.getElementById('accText').textContent=fmt(am);
  document.getElementById('peakVelVal').textContent=fmt(swingFSM.peakVel);
  document.getElementById('vzVal').textContent=fmt(engine.vel[2]);
  document.getElementById('zvuState').textContent=engine.isResting?'resting':'moving';
  const [wax,way,waz]=engine.worldAcc;
  document.getElementById('worldAccVal').textContent=[fmt(wax),fmt(way),fmt(waz)].join(' ');
  document.getElementById('velXYZ').textContent=engine.vel.map(v=>fmt(v)).join(' ');
  // Swing FSM
  const hit=swingFSM.process(pkt);
  if(hit){
    document.getElementById('zonePreview').textContent=NAMES[hit.zone];
    getCtx();SYNTHS[hit.zone](hit.vel);
    const pad=document.getElementById('p'+hit.zone);
    pad.classList.add('hit');setTimeout(()=>pad.classList.remove('hit'),150);
    addLog(NAMES[hit.zone]+' vel='+hit.vel+' vz='+fmt(hit.vz)+' pv='+fmt(hit.peakVel));
    send({cmd:'browserHit',z:hit.zone,vel:hit.vel});
    if(engine.trace.length>0)engine.trace[engine.trace.length-1].zone=hit.zone;
  }
  drawPosCanvas(engine);drawVelCanvas();
}

// ── Calibration bars helper ────────────────────────────────────────────────
function calBar(idF,idT,val){
  const el=document.getElementById(idF);
  el.style.width=(val/3*100).toFixed(0)+'%';
  el.style.background=val>=3?'#68d391':val>=1?'#e08a2e':'#cf453d';
  document.getElementById(idT).textContent=val+'/3';
}

// ── Telemetry refresh (20 Hz when stream off) ──────────────────────────────
function refreshTele(tele){
  document.getElementById('calSysInline').textContent=(tele.cal||0)+'/3';
  document.getElementById('bnoStatus').innerHTML=(tele.ok===1)?
    '<span class="badge ok">connected</span>':'<span class="badge err">not found</span>';
  document.getElementById('bnoMode').textContent=(tele.cal||0)>=1?'NDOF':'warming up';
  document.getElementById('bnoTemp').textContent=(tele.temp||0)+' \xb0C';
  document.getElementById('battVal').textContent=fmt(tele.batt||0,2)+'V ('+((tele.battp||0))+'%)';
  calBar('calSysF','calSysT',tele.cal||0);
  calBar('calGyrF','calGyrT',tele.cg||0);
  calBar('calAccF','calAccT',tele.ca||0);
  calBar('calMagF','calMagT',tele.cm||0);
  document.getElementById('quatVal').textContent=
    [tele.qw,tele.qx,tele.qy,tele.qz].map(v=>fmt(v||0,3)).join(' ');
  document.getElementById('eulerVal').textContent=
    'H:'+fmt(tele.eh||0,1)+' R:'+fmt(tele.er||0,1)+' P:'+fmt(tele.ep||0,1);
  document.getElementById('accelVal').textContent=
    [fmt(tele.ax||0),fmt(tele.ay||0),fmt(tele.az||0)].join(' ');
  document.getElementById('linVal').textContent=
    [fmt(tele.lax||0),fmt(tele.lay||0),fmt(tele.laz||0)].join(' ');
  document.getElementById('gyroVal').textContent=
    [fmt(tele.gx||0,1),fmt(tele.gy||0,1),fmt(tele.gz||0,1)].join(' ');
  document.getElementById('magVal').textContent=
    [fmt(tele.mx||0,1),fmt(tele.my||0,1),fmt(tele.mz||0,1)].join(' ');
}

// ── Button wiring ──────────────────────────────────────────────────────────
document.getElementById('selfTest').onclick=()=>send({cmd:'selftest'});

const btnStream=document.getElementById('btnStream');
btnStream.onclick=()=>{
  if(!state.streaming){
    syncEngineParams();
    send({cmd:'startStream'});
    state.streaming=true;
    btnStream.textContent='Stop stream';
    btnStream.classList.add('on');
    document.getElementById('streamBadge').className='badge streaming';
    document.getElementById('streamBadge').textContent='100\u202FHz';
  }else{
    send({cmd:'stopStream'});
    state.streaming=false;
    btnStream.textContent='Start stream (100\u202FHz)';
    btnStream.classList.remove('on');
    document.getElementById('streamBadge').className='badge err';
    document.getElementById('streamBadge').textContent='off';
  }
};

// ── WebSocket ──────────────────────────────────────────────────────────────
let ws,reconnTimer;
function connect(){
  ws=new WebSocket('ws://'+location.host+'/drum/ws');
  ws.onopen=()=>{st.textContent='connected';clearTimeout(reconnTimer);};
  ws.onclose=()=>{
    st.textContent='disconnected \u2014 retrying\u2026';
    state.streaming=false;
    btnStream.textContent='Start stream (100\u202FHz)';btnStream.classList.remove('on');
    document.getElementById('streamBadge').className='badge err';
    document.getElementById('streamBadge').textContent='off';
    reconnTimer=setTimeout(connect,2000);
  };
  ws.onmessage=e=>{
    try{
      const m=JSON.parse(e.data);
      if(m.type==='r'){
        // High-rate raw packet — update quaternion cache and feed kinematic engine
        state.tele.qw=m.qw;state.tele.qx=m.qx;state.tele.qy=m.qy;state.tele.qz=m.qz;
        state.tele.lax=m.lax;state.tele.lay=m.lay;state.tele.laz=m.laz;
        state.tele.gx=m.gx;state.tele.gy=m.gy;state.tele.gz=m.gz;
        handleRaw(m);
        return;
      }
      if(m.type==='cfg'){return;}
      if(m.type==='tele'){state.tele=m;refreshTele(m);return;}
      if(m.type==='info'&&m.msg){addLog(m.msg);return;}
    }catch(_){}
  };
}
document.body.addEventListener('pointerdown',()=>getCtx(),{once:true});
connect();
</script></body></html>

)RAWHTML";
