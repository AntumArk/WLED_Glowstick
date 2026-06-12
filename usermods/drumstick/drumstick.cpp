/*
 * Drumstick usermod for WLED
 *
 * Detects drumming motion via MPU6050 accelerometer and streams hit events
 * over UDP to an external receiver that plays audio samples (e.g. to
 * Bluetooth headphones). Uses the ESP32-C3 friendly BLE/WiFi stack only;
 * no Bluetooth Classic / A2DP required on the microcontroller side.
 *
 * Hardware: ESP32-C3 mini + MPU6050 on I2C (uses WLED's shared Wire bus)
 *
 * UDP packet schema v1 (compact JSON):
 *   {"v":1,"seq":<n>,"t":<millis>,"z":<zone>,"vel":<1-127>,"conf":<0-100>,"s":"MPU"}
 *   Heartbeat: {"v":1,"seq":<n>,"t":<millis>,"hb":1,"imu":<0|1>,"sda":<pin>,"scl":<pin>}
 *
 * Zone mapping (configurable per stick):  0=Snare, 1=Kick, 2=Hi-Hat, 3=Tom
 *
 * Sensor: MPU6050, ±8g full-scale range.  1g = 4096 LSB in this range.
 * Hit threshold is expressed as raw delta from the 1g rest magnitude.
 * A threshold of 20000 corresponds to roughly 5g of dynamic acceleration.
 *
 * References:
 *   MPU6050 datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *   electroniccats MPU6050 library: https://github.com/electroniccats/mpu6050
 */

#include "wled.h"
#include "MPU6050.h"
#include <WiFiUdp.h>

// ── constants ──────────────────────────────────────────────────────────────

static constexpr uint16_t DS_POLL_MS          = 2;      // sensor poll interval → ~500 Hz
static constexpr uint16_t DS_HEARTBEAT_MS     = 5000;   // ms between heartbeat packets
static constexpr uint8_t  DS_UDP_BUF          = 140;    // max UDP payload bytes
static constexpr uint16_t DS_MAG_POLL_MS      = 20;     // HMC5883 read interval (~50 Hz)
static constexpr uint16_t DS_RAW_UDP_BUF      = 180;    // raw sensor packet max bytes
static constexpr uint16_t DS_WS_TELEM_MS      = 50;     // browser telemetry cadence (~20 Hz)
static constexpr float    DS_MAG_LPF          = 0.82f;  // low-pass for noisy magnetometer axes
static constexpr float    DS_LINACC_LPF       = 0.65f;  // low-pass for linear acceleration peaks
static constexpr float    DS_OMEGA_LPF        = 0.60f;  // low-pass for gyro magnitude used by FSM

static constexpr uint8_t  DS_MPU_ADDR_0       = 0x68;
static constexpr uint8_t  DS_MPU_ADDR_1       = 0x69;
static constexpr uint8_t  DS_HMC5883_ADDR     = 0x1E;

// Gyro stroke FSM constants.
// At ±500°/s FS: 1 LSB ≈ 0.01526°/s → 65.5 LSB per °/s.
// DS_GYRO_DECAY: ωmag must drop to this fraction of stroke peak to mark the
//   wrist-stop end-point (the "hit" moment in air drumming).
// DS_SWING_MAX_SAMPLES: abort unclean stroke after 300ms @ 500Hz = 150 samples.
static constexpr float    DS_GYRO_DECAY        = 0.45f;  // decay ratio to detect stroke end
static constexpr uint16_t DS_SWING_MAX_SAMPLES = 150;    // ~300ms max stroke duration

// ── Embedded drum-synth page served at /drum ──────────────────────────────
// Self-contained: no files on LittleFS required.
// Browser opens WS to /drum/ws and plays Web Audio synth sounds on each hit.
// Kept compact so it fits in IRAM-safe flash string storage.
static const char DRUM_PAGE[] PROGMEM = R"RAWHTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drumstick</title>
<style>
  :root{--bg:#09141b;--panel:#102431;--panel2:#0c1b25;--line:#28506a;--text:#edf6ff;
        --muted:#9fc0d4;--warm:#e08a2e;--red:#cf453d;--cyan:#2ea9d0;--violet:#8460e8;--good:#68d391;}
  *{box-sizing:border-box} body{background:radial-gradient(circle at top,#163447 0,#09141b 52%,#050b0f 100%);
      color:var(--text);font-family:Segoe UI,system-ui,sans-serif;margin:0;padding:18px;}
  .wrap{max-width:1180px;margin:0 auto;display:grid;gap:14px}
  .hero{display:flex;justify-content:space-between;gap:12px;align-items:end;flex-wrap:wrap}
  h1{margin:0;font-size:1.7rem;letter-spacing:.08em}
  #status{font-size:.88rem;color:var(--muted)}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px}
  .panel{background:linear-gradient(180deg,rgba(21,46,61,.92),rgba(10,24,34,.96));border:1px solid rgba(92,149,184,.35);
         border-radius:16px;padding:14px;box-shadow:0 18px 48px rgba(0,0,0,.24)}
  .panel h2{margin:0 0 10px;font-size:1rem;letter-spacing:.05em;color:#dff0ff}
  .panel h3{margin:0 0 8px;font-size:.88rem;color:#d2e8f8}
  #pads{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .pad{border-radius:12px;height:84px;display:flex;align-items:center;justify-content:center;font-size:1rem;font-weight:700;
       transition:transform .08s,filter .08s;filter:brightness(.35)}
  .pad.hit{filter:brightness(1.0);transform:translateY(-2px)}
  #p0{background:var(--warm)} #p1{background:var(--red)} #p2{background:var(--cyan)} #p3{background:var(--violet)}
  .kv{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;font-size:.82rem}
  .kv div,.mini{background:rgba(4,11,16,.24);border:1px solid rgba(92,149,184,.18);border-radius:10px;padding:8px}
  .mini b{display:block;font-size:.72rem;color:var(--muted);font-weight:600;margin-bottom:4px}
  .controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
  label{display:flex;flex-direction:column;gap:5px;font-size:.78rem;color:var(--muted)}
  input,select,button{border-radius:10px;border:1px solid rgba(92,149,184,.35);background:#0a1a24;color:var(--text);padding:9px 10px;font:inherit}
  button{cursor:pointer;background:linear-gradient(180deg,#17354a,#102633)}
  button.secondary{background:linear-gradient(180deg,#13242f,#0d1921)}
  button.warn{background:linear-gradient(180deg,#704026,#4d2a18)}
  .meters{display:grid;gap:10px}
  .meter{display:grid;gap:5px}
  .meterRow{display:flex;justify-content:space-between;font-size:.78rem;color:var(--muted)}
  .track{height:12px;border-radius:999px;background:#08131a;border:1px solid rgba(92,149,184,.25);position:relative;overflow:hidden}
  .fill{height:100%;background:linear-gradient(90deg,#2ea9d0,#68d391);width:0%}
  .marker{position:absolute;top:0;bottom:0;width:2px;background:#ffdc73;opacity:.95}
  .projWrap{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  canvas{width:100%;aspect-ratio:1/1;background:radial-gradient(circle at center,#112533 0,#0a1720 65%,#081118 100%);
         border-radius:12px;border:1px solid rgba(92,149,184,.24)}
  .mapGrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
  .mapCell{background:rgba(4,11,16,.24);border:1px solid rgba(92,149,184,.18);border-radius:10px;padding:10px}
  .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
  .muted{color:var(--muted);font-size:.78rem}
  #log{height:140px;overflow-y:auto;font-size:.74rem;color:#a8c2d3;border-top:1px solid rgba(92,149,184,.2);padding-top:8px}
  @media (max-width:760px){.kv{grid-template-columns:repeat(2,minmax(0,1fr))}.projWrap{grid-template-columns:1fr}}
</style></head><body>
<div class="wrap">
<div class="hero">
  <div>
    <h1>&#x1F941; Drumstick Standalone</h1>
    <div id="status">connecting…</div>
  </div>
  <div class="row muted">
    <span>Volume</span><input id="vr" type="range" min="0" max="1" step=".05" value="0.8">
  </div>
</div>

<div class="grid">
  <section class="panel">
    <h2>Live Pads</h2>
    <div id="pads">
      <div class="pad" id="p0">Snare</div>
      <div class="pad" id="p1">Kick</div>
      <div class="pad" id="p2">Hi-Hat</div>
      <div class="pad" id="p3">Tom</div>
    </div>
    <div class="kv" style="margin-top:10px">
      <div class="mini"><b>Swing</b><span id="swingState">IDLE</span></div>
      <div class="mini"><b>Orientation</b><span id="orientationVal">warming up</span></div>
      <div class="mini"><b>Learn</b><span id="learnState">off</span></div>
      <div class="mini"><b>Zone Preview</b><span id="zonePreview">Snare</span></div>
    </div>
  </section>

  <section class="panel">
    <h2>Thresholds</h2>
    <div class="meters">
      <div class="meter">
        <div class="meterRow"><span>Linear accel peak</span><span id="linText">0 / 0</span></div>
        <div class="track"><div class="fill" id="linFill"></div><div class="marker" id="linMark"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Gyro swing envelope</span><span id="gyroText">0 / 0</span></div>
        <div class="track"><div class="fill" id="gyroFill"></div><div class="marker" id="gyroMark"></div></div>
      </div>
    </div>
    <div class="kv" style="margin-top:10px">
      <div class="mini"><b>Current World Vec</b><span id="worldNow">0 0 0</span></div>
      <div class="mini"><b>Peak World Vec</b><span id="worldPeak">0 0 0</span></div>
      <div class="mini"><b>Peak Accel</b><span id="peakAccel">0</span></div>
      <div class="mini"><b>Peak Omega</b><span id="peakOmega">0</span></div>
    </div>
  </section>
</div>

<div class="grid">
  <section class="panel">
    <h2>Debug Projections</h2>
    <div class="projWrap">
      <div>
        <h3>World horizontal</h3>
        <canvas id="xy"></canvas>
        <div class="muted">Right/forward projection after full 3D mag-plus-gravity alignment. The ring reflects the accel floor.</div>
      </div>
      <div>
        <h3>Right vs vertical</h3>
        <canvas id="xz"></canvas>
        <div class="muted">Shows how the same swing moves across horizontal and vertical axes.</div>
      </div>
    </div>
  </section>

  <section class="panel">
    <h2>Tuning</h2>
    <div class="controls">
      <label>Accel floor
        <input id="threshold" type="number" min="512" step="128">
      </label>
      <label>Gyro onset
        <input id="gyroOnset" type="number" min="500" step="100">
      </label>
      <label>Cooldown ms
        <input id="cooldown" type="number" min="30" max="1000" step="10">
      </label>
      <label>Zone mode
        <select id="autoZone">
          <option value="1">Auto</option>
          <option value="0">Manual</option>
        </select>
      </label>
      <label>Manual zone
        <select id="manualZone"></select>
      </label>
    </div>
    <div class="row" style="margin-top:10px">
      <button id="applyTune">Apply</button>
      <button id="resetZones" class="secondary">Reset taught zones</button>
      <button id="cancelLearn" class="warn">Cancel learn</button>
    </div>
    <div class="muted" style="margin-top:8px">Changes apply immediately and are persisted into WLED config.</div>
  </section>
</div>

<div class="grid">
  <section class="panel">
    <h2>Zone Teaching</h2>
    <div class="mapGrid" id="mapGrid"></div>
    <div class="muted" style="margin-top:8px">Pick a zone, point the stick toward that area, and play one clean stroke. The firmware stores that 3D world-space aim vector and uses the closest taught zone during auto mode.</div>
  </section>

  <section class="panel">
    <h2>Sensor Debug</h2>
    <div class="kv">
      <div class="mini"><b>Accel</b><span id="accelVal">0 0 0</span></div>
      <div class="mini"><b>Linear accel</b><span id="linVal">0 0 0</span></div>
      <div class="mini"><b>Gyro</b><span id="gyroVal">0 0 0</span></div>
      <div class="mini"><b>Mag</b><span id="magVal">0 0 0</span></div>
    </div>
    <div id="log"></div>
  </section>
</div>
</div>
<script>
// ── Web Audio synth engine ──────────────────────────────────────────────
let ctx=null;
function getCtx(){
  if(!ctx)ctx=new(window.AudioContext||window.webkitAudioContext)();
  if(ctx.state==='suspended')ctx.resume();
  return ctx;
}

function masterGain(){
  if(!masterGain._n){
    masterGain._n=getCtx().createGain();
    masterGain._n.connect(getCtx().destination);
  }
  masterGain._n.gain.value=parseFloat(document.getElementById('vr').value);
  return masterGain._n;
}

// Envelope helper: attack→decay→sustain→release shape on a GainNode
function adsr(g,a,d,s,r,now){
  g.gain.cancelScheduledValues(now);
  g.gain.setValueAtTime(0,now);
  g.gain.linearRampToValueAtTime(1,now+a);
  g.gain.linearRampToValueAtTime(s,now+a+d);
  g.gain.setValueAtTime(s,now+a+d+.001);
  g.gain.linearRampToValueAtTime(0,now+a+d+r);
}

const SYNTHS=[
  // z0 Snare — noise burst + pitched body
  function snare(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127;
    const dur=.18;
    // Noise component
    const buf=C.createBuffer(1,C.sampleRate*dur,C.sampleRate);
    const d=buf.getChannelData(0);
    for(let i=0;i<d.length;i++)d[i]=(Math.random()*2-1);
    const ns=C.createBufferSource();ns.buffer=buf;
    const nf=C.createBiquadFilter();nf.type='bandpass';nf.frequency.value=2400;nf.Q.value=.8;
    const ng=C.createGain();adsr(ng,.002,.04,.1,.09,t);
    ng.gain.setValueAtTime(ng.gain.value,t);// keep
    ns.connect(nf);nf.connect(ng);ng.connect(masterGain());
    ns.start(t);ns.stop(t+dur);
    // Tonal body
    const osc=C.createOscillator();osc.type='triangle';osc.frequency.value=200;
    osc.frequency.linearRampToValueAtTime(80,t+.05);
    const og=C.createGain();og.gain.setValueAtTime(.6*v,t);
    og.gain.linearRampToValueAtTime(0,t+.12);
    osc.connect(og);og.connect(masterGain());
    osc.start(t);osc.stop(t+.14);
  },
  // z1 Kick — sine with pitch drop
  function kick(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127;
    const osc=C.createOscillator();osc.type='sine';
    osc.frequency.setValueAtTime(160,t);
    osc.frequency.exponentialRampToValueAtTime(40,t+.08);
    const g=C.createGain();g.gain.setValueAtTime(1.2*v,t);
    g.gain.linearRampToValueAtTime(0,t+.32);
    // Add click transient
    const click=C.createOscillator();click.type='square';click.frequency.value=600;
    const cg=C.createGain();cg.gain.setValueAtTime(.4*v,t);cg.gain.linearRampToValueAtTime(0,t+.02);
    click.connect(cg);cg.connect(masterGain());
    click.start(t);click.stop(t+.025);
    osc.connect(g);g.connect(masterGain());
    osc.start(t);osc.stop(t+.35);
  },
  // z2 Hi-Hat — filtered noise, short decay
  function hihat(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127;
    const dur=.12;
    const buf=C.createBuffer(1,C.sampleRate*dur,C.sampleRate);
    const d=buf.getChannelData(0);
    for(let i=0;i<d.length;i++)d[i]=(Math.random()*2-1);
    const ns=C.createBufferSource();ns.buffer=buf;
    const f=C.createBiquadFilter();f.type='highpass';f.frequency.value=8000;
    const g=C.createGain();g.gain.setValueAtTime(.7*v,t);g.gain.linearRampToValueAtTime(0,t+.10);
    ns.connect(f);f.connect(g);g.connect(masterGain());
    ns.start(t);ns.stop(t+dur);
  },
  // z3 Tom — low sine, medium decay
  function tom(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127;
    const osc=C.createOscillator();osc.type='sine';
    osc.frequency.setValueAtTime(120,t);
    osc.frequency.exponentialRampToValueAtTime(55,t+.12);
    const g=C.createGain();g.gain.setValueAtTime(v,t);
    g.gain.linearRampToValueAtTime(0,t+.28);
    osc.connect(g);g.connect(masterGain());
    osc.start(t);osc.stop(t+.3);
  }
];

const NAMES=['Snare','Kick','Hi-Hat','Tom'];
const COLORS=['#e08a2e','#cf453d','#2ea9d0','#8460e8'];
const log=document.getElementById('log');
const st=document.getElementById('status');
const xy=document.getElementById('xy');
const xz=document.getElementById('xz');
const mapGrid=document.getElementById('mapGrid');
const state={cfg:{th:20000,ath:2500,gth:5000,cd:150,az:1,zone:0,learn:-1,
  z0x:-707,z0y:707,z0z:0,z1x:-707,z1y:-707,z1z:0,z2x:707,z2y:707,z2z:0,z3x:707,z3y:-707,z3z:0},
  tele:{wr:0,wf:0,wu:0,pwr:0,pwf:0,pwu:0,la:0,pla:0,om:0,ori:0,st:'IDLE',zone:0,
        ax:0,ay:0,az:0,lax:0,lay:0,laz:0,gx:0,gy:0,gz:0,mx:0,my:0,mz:0}};

function clamp(v,a,b){return Math.min(b,Math.max(a,v));}
function fmt(v){return (typeof v==='number'&&isFinite(v))?Math.round(v):0;}

function addLog(msg){
  const l=document.createElement('div');
  l.textContent=new Date().toLocaleTimeString()+' '+msg;
  log.prepend(l);
  while(log.childNodes.length>60)log.removeChild(log.lastChild);
}

function send(obj){
  if(ws&&ws.readyState===1)ws.send(JSON.stringify(obj));
}

function metricFill(idFill,idMarker,val,threshold,maxVal){
  document.getElementById(idFill).style.width=(100*clamp(val/maxVal,0,1)).toFixed(1)+'%';
  document.getElementById(idMarker).style.left=(100*clamp(threshold/maxVal,0,1)).toFixed(1)+'%';
}

function drawProjection(cv,mode){
  const c=cv.getContext('2d');
  const w=cv.width=cv.clientWidth;
  const h=cv.height=cv.clientWidth;
  const cx=w/2,cy=h/2,s=w*0.36;
  c.clearRect(0,0,w,h);
  c.strokeStyle='rgba(120,175,209,.18)'; c.lineWidth=1;
  for(let i=1;i<=3;i++){
    c.beginPath(); c.arc(cx,cy,s*i/3,0,Math.PI*2); c.stroke();
  }
  c.beginPath(); c.moveTo(cx,12); c.lineTo(cx,h-12); c.moveTo(12,cy); c.lineTo(w-12,cy); c.stroke();
  c.fillStyle='rgba(159,192,212,.8)'; c.font='12px Segoe UI'; c.fillText(mode==='xy'?'F':'Up',cx+6,18); c.fillText('R',w-20,cy-6);

  const t=clamp((state.cfg.ath||1)/6000,0,.95)*s;
  c.strokeStyle='rgba(255,220,115,.65)'; c.beginPath(); c.arc(cx,cy,t,0,Math.PI*2); c.stroke();

  const curX=mode==='xy'?state.tele.wr:state.tele.wr;
  const curY=mode==='xy'?state.tele.wf:state.tele.wu;
  const peakX=mode==='xy'?state.tele.pwr:state.tele.pwr;
  const peakY=mode==='xy'?state.tele.pwf:state.tele.pwu;
  const scale=1/6000;

  function dot(x,y,color,r){
    const px=cx+clamp(x*scale,-1,1)*s;
    const py=cy-clamp(y*scale,-1,1)*s;
    c.fillStyle=color; c.beginPath(); c.arc(px,py,r,0,Math.PI*2); c.fill();
  }
  dot(peakX,peakY,'rgba(255,220,115,.95)',7);
  dot(curX,curY,COLORS[state.tele.zone||0],5);
}

function refreshUI(){
  const tele=state.tele,cfg=state.cfg;
  document.getElementById('swingState').textContent=tele.st;
  document.getElementById('orientationVal').textContent=tele.ori?'3D fused':'warming up';
  document.getElementById('learnState').textContent=cfg.learn>=0?'waiting for '+NAMES[cfg.learn]:'off';
  document.getElementById('zonePreview').textContent=NAMES[tele.zone||0];
  document.getElementById('worldNow').textContent=[fmt(tele.wr),fmt(tele.wf),fmt(tele.wu)].join(' ');
  document.getElementById('worldPeak').textContent=[fmt(tele.pwr),fmt(tele.pwf),fmt(tele.pwu)].join(' ');
  document.getElementById('peakAccel').textContent=fmt(tele.pla);
  document.getElementById('peakOmega').textContent=fmt(tele.om);
  document.getElementById('linText').textContent=fmt(tele.la)+' / '+fmt(cfg.ath);
  document.getElementById('gyroText').textContent=fmt(tele.om)+' / '+fmt(cfg.gth);
  metricFill('linFill','linMark',tele.la,cfg.ath,Math.max(cfg.ath*3,6000));
  metricFill('gyroFill','gyroMark',tele.om,cfg.gth,Math.max(cfg.gth*3,12000));
  document.getElementById('accelVal').textContent=[fmt(tele.ax),fmt(tele.ay),fmt(tele.az)].join(' ');
  document.getElementById('linVal').textContent=[fmt(tele.lax),fmt(tele.lay),fmt(tele.laz)].join(' ');
  document.getElementById('gyroVal').textContent=[fmt(tele.gx),fmt(tele.gy),fmt(tele.gz)].join(' ');
  document.getElementById('magVal').textContent=[fmt(tele.mx),fmt(tele.my),fmt(tele.mz)].join(' ');
  drawProjection(xy,'xy');
  drawProjection(xz,'xz');
}

function buildMapGrid(){
  mapGrid.innerHTML='';
  NAMES.forEach((label,idx)=>{
    const card=document.createElement('div'); card.className='mapCell';
    card.innerHTML='<div class="muted">'+label+'</div><div class="muted" style="margin:6px 0">aim: '+
      [fmt(state.cfg['z'+idx+'x']),fmt(state.cfg['z'+idx+'y']),fmt(state.cfg['z'+idx+'z'])].join(' ')+'</div>';
    const learn=document.createElement('button'); learn.textContent='Teach from next stroke'; learn.className='secondary';
    learn.onclick=()=>send({cmd:'learn',zone:idx});
    card.appendChild(learn);
    mapGrid.appendChild(card);
  });
}

function syncControls(){
  ['threshold','gyroOnset','cooldown'].forEach(id=>document.getElementById(id).value='');
  document.getElementById('threshold').value=state.cfg.th;
  document.getElementById('gyroOnset').value=state.cfg.gth;
  document.getElementById('cooldown').value=state.cfg.cd;
  document.getElementById('autoZone').value=state.cfg.az?1:0;
  document.getElementById('manualZone').value=state.cfg.zone;
  buildMapGrid();
  refreshUI();
}

NAMES.forEach((name,z)=>{
  const opt=document.createElement('option'); opt.value=z; opt.textContent=name;
  document.getElementById('manualZone').appendChild(opt);
});

document.getElementById('applyTune').onclick=()=>send({cmd:'set',
  threshold:parseInt(document.getElementById('threshold').value||state.cfg.th,10),
  gyroOnset:parseInt(document.getElementById('gyroOnset').value||state.cfg.gth,10),
  cooldown:parseInt(document.getElementById('cooldown').value||state.cfg.cd,10),
  autoZone:parseInt(document.getElementById('autoZone').value,10),
  zone:parseInt(document.getElementById('manualZone').value,10)
});
document.getElementById('resetZones').onclick=()=>send({cmd:'resetZones'});
document.getElementById('cancelLearn').onclick=()=>send({cmd:'cancelLearn'});

// ── WebSocket connection ────────────────────────────────────────────────
let ws,reconnTimer;

function connect(){
  ws=new WebSocket('ws://'+location.host+'/drum/ws');
  ws.onopen=()=>{st.textContent='connected';clearTimeout(reconnTimer);};
  ws.onclose=()=>{st.textContent='disconnected — retrying…';
    reconnTimer=setTimeout(connect,2000);};
  ws.onmessage=e=>{
    try{
      const m=JSON.parse(e.data);
      if(m.type==='cfg'){
        state.cfg=m; syncControls(); return;
      }
      if(m.type==='tele'){
        state.tele=m; refreshUI(); return;
      }
      if(m.type==='info'&&m.msg){
        addLog(m.msg); return;
      }
      if(m.z===undefined)return;
      const z=m.z&3,vel=m.vel||64;
      state.tele.zone=z;
      getCtx();
      SYNTHS[z](vel);
      const pad=document.getElementById('p'+z);
      pad.classList.add('hit');
      setTimeout(()=>pad.classList.remove('hit'),120);
      addLog(NAMES[z]+' vel='+vel);
      refreshUI();
    }catch(_){}
  };
}

// Unlock AudioContext on first user interaction
document.body.addEventListener('pointerdown',()=>getCtx(),{once:true});
connect();
</script></body></html>
)RAWHTML";

// ── class ──────────────────────────────────────────────────────────────────

class DrumstickUsermod : public Usermod {
private:

  // -- sensor --
  MPU6050       mpu;
  bool          sensorOk   = false;
  bool          magOk      = false;
  bool          initDone   = false;
  unsigned long lastPoll   = 0;    // last sensor read timestamp
  unsigned long lastMagPoll = 0;
    int8_t        interruptPin = -1;
    bool          irqEnabled = false;
    bool          irqBound = false;
  uint8_t       mpuAddr = DS_MPU_ADDR_0;

  // -- optional HMC5883 via MPU XDA/XCL pass-through --
  int16_t       magX = 0;
  int16_t       magY = 0;
  int16_t       magZ = 0;
  bool          magVecInit = false;
  float         magFilt[3] = {0.0f, 0.0f, 0.0f};

    static volatile bool _irqFired;
    static void IRAM_ATTR onSensorInterrupt() { _irqFired = true; }

  // -- gravity model (maintained for raw-stream mode and fallback) --
  bool          gravInit    = false;
  float         grav[3]     = {0.0f, 0.0f, 0.0f};

  // -- swing FSM --
  // Stroke lifecycle: IDLE → (ωmag ≥ onset) → SWING → (ωmag < peak×decay) → DECAY → emit → IDLE.
  // Gyro tracks swing onset/decay; zone and hit strength come from the strongest
  // linear-acceleration peak observed inside that swing.
  enum class SwingState : uint8_t { IDLE, SWING, DECAY };
  SwingState    swingState   = SwingState::IDLE;
  float         peakOmega    = 0.0f;   // peak ωmag this stroke (raw LSBs)
  float         peakLinAccel = 0.0f;   // strongest linear accel magnitude this stroke
  float         peakLinAx    = 0.0f;   // linear accel vector at accel peak (signed, for zone)
  float         peakLinAy    = 0.0f;
  float         peakLinAz    = 0.0f;
  float         peakWorldRight = 0.0f;
  float         peakWorldForward = 0.0f;
  float         peakWorldUp = 0.0f;
  float         currWorldRight = 0.0f;
  float         currWorldForward = 0.0f;
  float         currWorldUp = 0.0f;
  float         currLinAccel = 0.0f;
  float         currOmega = 0.0f;
  float         linFilt[3] = {0.0f, 0.0f, 0.0f};
  float         omegaFilt = 0.0f;
  int16_t       lastAx = 0;
  int16_t       lastAy = 0;
  int16_t       lastAz = 0;
  int16_t       lastGx = 0;
  int16_t       lastGy = 0;
  int16_t       lastGz = 0;
  uint16_t      swingSamples = 0;      // samples accumulated since swing onset
  unsigned long lastOnsetTs  = 0;
  uint8_t       detectedZone = 0;
  unsigned long lastWsTelemetry = 0;

  // -- UDP transport --
  WiFiUDP       udp;
  uint32_t      seqNum     = 0;    // monotonic sequence counter (packet-loss detection)
  unsigned long lastHbeat  = 0;

  // -- WebSocket drum page --
  AsyncWebSocket _drumWs{"/drum/ws"};   // separate from WLED's main /ws

  // -- stats shown in /json/info --
  unsigned long lastHitTs  = 0;    // millis() of the last emitted hit
  uint8_t       lastVel    = 0;    // velocity byte of the last emitted hit
  uint16_t      peakSeen   = 0;    // running maximum motion ratio seen (permille)

  // -- config (persisted via cfg.json) --
  bool          enabled        = false;
  uint16_t      threshold      = 20000;  // retained for backwards compatibility
  uint16_t      cooldown_ms    = 150;    // minimum ms between two consecutive hits
    int8_t        irqPin         = 3;      // optional MPU6050 INT pin
  uint8_t       activeZone     = 0;      // 0=Snare, 1=Kick, 2=Hi-Hat, 3=Tom
  bool          autoZone       = true;   // infer zone from dominant movement axis/sign
  uint16_t      motionSensPermille = 2200; // trigger when motion/noise ratio exceeds this value
  uint16_t      repeatMinMs    = 70;     // shortest interval considered part of repeated motion
  uint16_t      repeatMaxMs    = 450;    // longest interval considered part of repeated motion
  uint8_t       repeatNeeded   = 1;      // required repeat score before emitting events
  uint8_t       velocityCurve  = 0;      // 0=linear, 1=log, 2=squared
  bool          rawStream       = false; // stream raw sensor data to PC instead of processed hits
  bool          useMagnetometer = true;  // use HMC5883 if present on MPU aux bus
  uint16_t      gyroOnsetLSB   = 5000;  // \u03c9mag onset threshold (raw LSBs at \u00b1500\u00b0/s FS \u2248 76\u00b0/s)
  int8_t        learnZone = -1;
  int16_t       zoneAim[4][3] = {
    {-707,  707,    0},
    {-707, -707,    0},
    { 707,  707,    0},
    { 707, -707,    0}
  };
  char          udpHost[40]    = {0};    // destination host (IP or hostname string)
  uint16_t      udpPort        = 9000;   // destination UDP port

  static const char _name[];
  static const char _enabled[];

  bool detectMpuAddr() {
    if (probeI2c(DS_MPU_ADDR_0)) { mpuAddr = DS_MPU_ADDR_0; return true; }
    if (probeI2c(DS_MPU_ADDR_1)) { mpuAddr = DS_MPU_ADDR_1; return true; }
    return false;
  }

  uint16_t accelPeakThreshold() const {
    return max<uint16_t>(512, (uint16_t)(threshold / 8));
  }

  const char* swingStateName() const {
    return (swingState == SwingState::SWING) ? "SWING" :
           (swingState == SwingState::DECAY) ? "DECAY" : "IDLE";
  }

  static float vecLen3(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
  }

  static bool normalize3(float& x, float& y, float& z) {
    const float len = vecLen3(x, y, z);
    if (len < 1e-3f) return false;
    x /= len;
    y /= len;
    z /= len;
    return true;
  }

  static void cross3(float ax, float ay, float az,
                     float bx, float by, float bz,
                     float& rx, float& ry, float& rz) {
    rx = ay * bz - az * by;
    ry = az * bx - ax * bz;
    rz = ax * by - ay * bx;
  }

  static float dot3(float ax, float ay, float az,
                    float bx, float by, float bz) {
    return ax * bx + ay * by + az * bz;
  }

  // ── velocity computation ──────────────────────────────────────────────

  // Convert peak angular velocity (raw LSBs, ±500°/s FS) to MIDI velocity 1–127.
  // Maps gyroOnsetLSB (onset) → velocity 1, and 6× onset → velocity 127.
  // At ±500°/s FS: onset default 5000 LSBs ≈ 76°/s, ceiling ≈ 460°/s.
  uint8_t calcVelocity(float omega) const {
    const float minO = (float)gyroOnsetLSB;
    const float maxO = minO * 6.0f;
    if (omega <= minO) return 1;

    float t = (omega - minO) / (maxO - minO);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float v;
    switch (velocityCurve) {
      case 1:  v = log10f(1.0f + 9.0f * t); break;  // logarithmic (easier to reach high vel)
      case 2:  v = t * t;                   break;  // squared (requires more force for high vel)
      default: v = t;                        break;  // linear
    }
    return (uint8_t)(1u + (uint8_t)(v * 126.0f));
  }

  bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    if (!data || len == 0) return false;
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    const uint8_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
    if (got != len) return false;
    for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
    return true;
  }

  bool probeI2c(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }

  bool setupMpuBypassForAuxBus() {
    if (!detectMpuAddr()) return false;

    // Disable MPU internal I2C master and enable bypass so host can reach XDA/XCL devices.
    uint8_t intCfg = 0;
    i2cReadRegs(mpuAddr, 0x37, &intCfg, 1);
    if (!i2cWriteReg(mpuAddr, 0x6A, 0x00)) return false;          // USER_CTRL
    if (!i2cWriteReg(mpuAddr, 0x37, (uint8_t)(intCfg | 0x02U))) return false; // INT_PIN_CFG.BYPASS_EN
    return true;
  }

  bool enableMpuDataReadyInterrupt() {
    if (!detectMpuAddr()) return false;

    uint8_t intCfg = 0;
    if (!i2cReadRegs(mpuAddr, 0x37, &intCfg, 1)) return false;
    // Keep BYPASS_EN if already configured and clear interrupt on any register read.
    intCfg = (uint8_t)(intCfg | 0x10U);
    if (!i2cWriteReg(mpuAddr, 0x37, intCfg)) return false;
    if (!i2cWriteReg(mpuAddr, 0x38, 0x01U)) return false; // INT_ENABLE: DATA_RDY_EN

    uint8_t dummy = 0;
    i2cReadRegs(mpuAddr, 0x3A, &dummy, 1); // clear stale interrupt state
    return true;
  }

  bool initHmc5883() {
    if (!useMagnetometer) return false;
    if (!setupMpuBypassForAuxBus()) return false;
    if (!probeI2c(DS_HMC5883_ADDR)) return false;

    // CRA: 8-average, 15 Hz output, normal measurement.
    // CRB: gain = 1.3 Ga.
    // MODE: continuous conversion.
    if (!i2cWriteReg(DS_HMC5883_ADDR, 0x00, 0x70)) return false;
    if (!i2cWriteReg(DS_HMC5883_ADDR, 0x01, 0x20)) return false;
    if (!i2cWriteReg(DS_HMC5883_ADDR, 0x02, 0x00)) return false;
    return true;
  }

  bool readHmc5883() {
    uint8_t raw[6];
    if (!i2cReadRegs(DS_HMC5883_ADDR, 0x03, raw, 6)) return false;

    // HMC5883 order: X, Z, Y.
    magX = (int16_t)((raw[0] << 8) | raw[1]);
    magZ = (int16_t)((raw[2] << 8) | raw[3]);
    magY = (int16_t)((raw[4] << 8) | raw[5]);

    // Magnetometer is mounted 180° around the forward Y axis relative to the MPU,
    // so X and Z are reversed to line up with the accelerometer frame.
    const float corrX = -(float)magX;
    const float corrY =  (float)magY;
    const float corrZ = -(float)magZ;

    if (!magVecInit) {
      magFilt[0] = corrX;
      magFilt[1] = corrY;
      magFilt[2] = corrZ;
      magVecInit = true;
    } else {
      magFilt[0] = magFilt[0] * DS_MAG_LPF + (1.0f - DS_MAG_LPF) * corrX;
      magFilt[1] = magFilt[1] * DS_MAG_LPF + (1.0f - DS_MAG_LPF) * corrY;
      magFilt[2] = magFilt[2] * DS_MAG_LPF + (1.0f - DS_MAG_LPF) * corrZ;
    }

    return true;
  }

  bool rotateToWorld(float localX, float localY, float localZ,
                     float& worldRight, float& worldForward, float& worldUp) const {
    worldRight = localX;
    worldForward = localY;
    worldUp = localZ;

    if (!(magOk && gravInit && magVecInit)) return false;

    float downX = grav[0];
    float downY = grav[1];
    float downZ = grav[2];
    if (!normalize3(downX, downY, downZ)) return false;

    float eastX = 0.0f, eastY = 0.0f, eastZ = 0.0f;
    cross3(magFilt[0], magFilt[1], magFilt[2], downX, downY, downZ, eastX, eastY, eastZ);
    if (!normalize3(eastX, eastY, eastZ)) return false;

    float northX = 0.0f, northY = 0.0f, northZ = 0.0f;
    cross3(downX, downY, downZ, eastX, eastY, eastZ, northX, northY, northZ);
    if (!normalize3(northX, northY, northZ)) return false;

    const float upX = -downX;
    const float upY = -downY;
    const float upZ = -downZ;

    worldRight = dot3(localX, localY, localZ, eastX, eastY, eastZ);
    worldForward = dot3(localX, localY, localZ, northX, northY, northZ);
    worldUp = dot3(localX, localY, localZ, upX, upY, upZ);
    return true;
  }

  bool setZoneAim(uint8_t zone, float worldRight, float worldForward, float worldUp) {
    if (zone > 3) return false;
    if (!normalize3(worldRight, worldForward, worldUp)) return false;
    zoneAim[zone][0] = (int16_t)(worldRight * 1000.0f);
    zoneAim[zone][1] = (int16_t)(worldForward * 1000.0f);
    zoneAim[zone][2] = (int16_t)(worldUp * 1000.0f);
    return true;
  }

  // Classify zone from the strongest linear-acceleration vector inside the swing.
  // The vector is rotated into a world frame using full 3-axis magnetometer data
  // and the gravity vector, then matched against the learned zone aim vectors.
  uint8_t inferZoneFromAccelPeak(float axF, float ayF, float azF,
                                 float& worldRight, float& worldForward, float& worldUp) const {
    if (!autoZone) return activeZone;

    rotateToWorld(axF, ayF, azF, worldRight, worldForward, worldUp);
    float qx = worldRight;
    float qy = worldForward;
    float qz = worldUp;
    if (!normalize3(qx, qy, qz)) return activeZone;

    uint8_t bestZone = 0;
    float bestScore = -2.0f;
    for (uint8_t zone = 0; zone < 4; zone++) {
      float zx = zoneAim[zone][0] / 1000.0f;
      float zy = zoneAim[zone][1] / 1000.0f;
      float zz = zoneAim[zone][2] / 1000.0f;
      if (!normalize3(zx, zy, zz)) continue;
      const float score = dot3(qx, qy, qz, zx, zy, zz);
      if (score > bestScore) {
        bestScore = score;
        bestZone = zone;
      }
    }
    return bestZone;
  }

  void sendWsInfo(const char* msg) {
    if (_drumWs.count() == 0 || !msg) return;
    char buf[160];
    int len = snprintf(buf, sizeof(buf), "{\"type\":\"info\",\"msg\":\"%s\"}", msg);
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    _drumWs.textAll(buf);
  }

  void sendWsConfig(AsyncWebSocketClient* client = nullptr) {
    char buf[320];
    int len = snprintf(buf, sizeof(buf),
      "{\"type\":\"cfg\",\"th\":%u,\"ath\":%u,\"gth\":%u,\"cd\":%u,\"az\":%u,\"zone\":%u,\"learn\":%d,\"z0x\":%d,\"z0y\":%d,\"z0z\":%d,\"z1x\":%d,\"z1y\":%d,\"z1z\":%d,\"z2x\":%d,\"z2y\":%d,\"z2z\":%d,\"z3x\":%d,\"z3y\":%d,\"z3z\":%d}",
      (unsigned)threshold,
      (unsigned)accelPeakThreshold(),
      (unsigned)gyroOnsetLSB,
      (unsigned)cooldown_ms,
      (unsigned)(autoZone ? 1 : 0),
      (unsigned)activeZone,
      (int)learnZone,
      (int)zoneAim[0][0], (int)zoneAim[0][1], (int)zoneAim[0][2],
      (int)zoneAim[1][0], (int)zoneAim[1][1], (int)zoneAim[1][2],
      (int)zoneAim[2][0], (int)zoneAim[2][1], (int)zoneAim[2][2],
      (int)zoneAim[3][0], (int)zoneAim[3][1], (int)zoneAim[3][2]
    );
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    if (client) client->text(buf);
    else _drumWs.textAll(buf);
  }

  void sendWsTelemetry() {
    if (_drumWs.count() == 0) return;
    char buf[512];
    int len = snprintf(buf, sizeof(buf),
      "{\"type\":\"tele\",\"st\":\"%s\",\"ori\":%u,\"wr\":%.1f,\"wf\":%.1f,\"wu\":%.1f,\"pwr\":%.1f,\"pwf\":%.1f,\"pwu\":%.1f,\"la\":%.1f,\"pla\":%.1f,\"om\":%.1f,\"zone\":%u,\"ax\":%d,\"ay\":%d,\"az\":%d,\"lax\":%.1f,\"lay\":%.1f,\"laz\":%.1f,\"gx\":%d,\"gy\":%d,\"gz\":%d,\"mx\":%d,\"my\":%d,\"mz\":%d}",
      swingStateName(),
      (unsigned)((magOk && gravInit && magVecInit) ? 1 : 0),
      currWorldRight,
      currWorldForward,
      currWorldUp,
      peakWorldRight,
      peakWorldForward,
      peakWorldUp,
      currLinAccel,
      peakLinAccel,
      currOmega,
      (unsigned)(autoZone ? detectedZone : activeZone),
      (int)lastAx,
      (int)lastAy,
      (int)lastAz,
      peakLinAx == peakLinAx ? ((float)lastAx - grav[0]) : 0.0f,
      peakLinAy == peakLinAy ? ((float)lastAy - grav[1]) : 0.0f,
      peakLinAz == peakLinAz ? ((float)lastAz - grav[2]) : 0.0f,
      (int)lastGx,
      (int)lastGy,
      (int)lastGz,
      (int)magX,
      (int)magY,
      (int)magZ
    );
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    _drumWs.textAll(buf);
  }

  void handleWsCommand(AsyncWebSocketClient* client, uint8_t* data, size_t len) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, data, len) != DeserializationError::Ok) return;
    const char* cmd = doc["cmd"] | "";
    bool changed = false;

    if (!strcmp(cmd, "cancelLearn")) {
      learnZone = -1;
      sendWsInfo("Learn mode cancelled.");
    } else if (!strcmp(cmd, "resetZones")) {
      setZoneAim(0, -0.707f,  0.707f, 0.0f);
      setZoneAim(1, -0.707f, -0.707f, 0.0f);
      setZoneAim(2,  0.707f,  0.707f, 0.0f);
      setZoneAim(3,  0.707f, -0.707f, 0.0f);
      changed = true;
      sendWsInfo("Zone aims reset to defaults.");
    } else if (!strcmp(cmd, "learn")) {
      int zone = doc["zone"] | -1;
      if (zone >= 0 && zone < 4) {
        learnZone = (int8_t)zone;
        sendWsInfo("Point at the target area and play one stroke to teach that zone.");
      }
    } else if (!strcmp(cmd, "set")) {
      if (doc.containsKey("threshold")) {
        uint16_t v = (uint16_t)max(512, (int)(doc["threshold"] | (int)threshold));
        if (threshold != v) { threshold = v; changed = true; }
      }
      if (doc.containsKey("gyroOnset")) {
        uint16_t v = (uint16_t)max(500, (int)(doc["gyroOnset"] | (int)gyroOnsetLSB));
        if (gyroOnsetLSB != v) { gyroOnsetLSB = v; changed = true; }
      }
      if (doc.containsKey("cooldown")) {
        uint16_t v = (uint16_t)constrain((int)(doc["cooldown"] | (int)cooldown_ms), 30, 1000);
        if (cooldown_ms != v) { cooldown_ms = v; changed = true; }
      }
      if (doc.containsKey("autoZone")) {
        bool v = (doc["autoZone"] | 1) != 0;
        if (autoZone != v) { autoZone = v; changed = true; }
      }
      if (doc.containsKey("zone")) {
        uint8_t v = (uint8_t)constrain((int)(doc["zone"] | (int)activeZone), 0, 3);
        if (activeZone != v) { activeZone = v; changed = true; }
      }
      if (changed) sendWsInfo("Drumstick tuning updated.");
    }

    if (changed) configNeedsWrite = true;
    sendWsConfig(client);
  }

  // ── UDP helpers ───────────────────────────────────────────────────────

  // Broadcast a hit to all connected WebSocket clients.
  void broadcastWsHit(uint8_t zone, uint8_t velocity) {
    if (_drumWs.count() == 0) return;
    char buf[36];
    snprintf(buf, sizeof(buf), "{\"type\":\"hit\",\"z\":%u,\"vel\":%u}", (unsigned)zone, (unsigned)velocity);
    _drumWs.textAll(buf);
  }

  void sendHitPacket(uint8_t zone, uint8_t velocity, uint8_t confidence) {
    if (!WLED_CONNECTED || udpHost[0] == '\0') return;
    char buf[DS_UDP_BUF];
    int len = snprintf(buf, sizeof(buf),
      "{\"v\":1,\"seq\":%lu,\"t\":%lu,\"z\":%u,\"vel\":%u,\"conf\":%u,\"s\":\"MPU\"}",
      (unsigned long)seqNum++,
      (unsigned long)millis(),
      (unsigned)zone,
      (unsigned)velocity,
      (unsigned)confidence
    );
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    udp.beginPacket(udpHost, udpPort);
    udp.write((const uint8_t*)buf, (size_t)len);
    udp.endPacket();
  }

  void sendHeartbeat() {
    if (!WLED_CONNECTED || udpHost[0] == '\0') return;
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
      "{\"v\":1,\"seq\":%lu,\"t\":%lu,\"hb\":1,\"imu\":%u,\"mag\":%u,\"sda\":%d,\"scl\":%d}",
      (unsigned long)seqNum++,
      (unsigned long)millis(),
      (unsigned)(sensorOk ? 1 : 0),
      (unsigned)(magOk ? 1 : 0),
      (int)i2c_sda,
      (int)i2c_scl
    );
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    udp.beginPacket(udpHost, udpPort);
    udp.write((const uint8_t*)buf, (size_t)len);
    udp.endPacket();
  }

  // Send raw sensor data for PC-side processing.  Includes accel, gyro, and last
  // known magnetometer values (mag updates at ~50 Hz; repeated values between reads).
  void sendRawPacket(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz) {
    if (!WLED_CONNECTED || udpHost[0] == '\0') return;
    char buf[DS_RAW_UDP_BUF];
    int len = snprintf(buf, sizeof(buf),
      "{\"v\":2,\"seq\":%lu,\"t\":%lu,"
      "\"ax\":%d,\"ay\":%d,\"az\":%d,"
      "\"gx\":%d,\"gy\":%d,\"gz\":%d,"
      "\"mx\":%d,\"my\":%d,\"mz\":%d}",
      (unsigned long)seqNum++,
      (unsigned long)millis(),
      (int)ax, (int)ay, (int)az,
      (int)gx, (int)gy, (int)gz,
      (int)magX, (int)magY, (int)magZ
    );
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    udp.beginPacket(udpHost, udpPort);
    udp.write((const uint8_t*)buf, (size_t)len);
    udp.endPacket();
  }

public:

  // ── WLED lifecycle hooks ──────────────────────────────────────────────

  // setup() is called after readFromConfig() and before the main loop.
  // Wire (I2C) is already initialised by WLED core (cfg.cpp) before this point.
  void setup() override {
    DEBUG_PRINTF("[Drumstick] I2C pins SDA=%d SCL=%d\n", (int)i2c_sda, (int)i2c_scl);

    // ── WebSocket + HTTP routes ───────────────────────────────────────
    _drumWs.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient* c,
                           AwsEventType t, void* arg, uint8_t* data, size_t len) {
      if (t == WS_EVT_CONNECT) {
        DEBUG_PRINTF("[Drumstick] WS client #%u connected\n", c->id());
        sendWsConfig(c);
      } else if (t == WS_EVT_DISCONNECT) {
        DEBUG_PRINTF("[Drumstick] WS client #%u disconnected\n", c->id());
      } else if (t == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info && info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
          handleWsCommand(c, data, len);
        }
      }
    });
    server.addHandler(&_drumWs);

    server.on("/drum", HTTP_GET, [](AsyncWebServerRequest *req) {
      req->send_P(200, "text/html", DRUM_PAGE);
    });

      interruptPin = irqPin;
      irqEnabled = false;
      _irqFired = false;

      if (interruptPin >= 0) {
        irqBound = PinManager::allocatePin(interruptPin, false, PinOwner::UM_IMU);
        if (!irqBound) {
          DEBUG_PRINTF("[Drumstick] IRQ pin %d already in use, falling back to polling\n", (int)interruptPin);
        } else {
          const int8_t irqNum = digitalPinToInterrupt(interruptPin);
          if (irqNum < 0) {
            DEBUG_PRINTF("[Drumstick] IRQ pin %d has no interrupt capability, using polling\n", (int)interruptPin);
          } else {
            pinMode(interruptPin, INPUT);
            attachInterrupt(irqNum, onSensorInterrupt, RISING);
            irqEnabled = true;
            DEBUG_PRINTF("[Drumstick] IRQ line attached on pin %d\n", (int)interruptPin);
          }
        }
      }

    mpu.initialize();
    sensorOk = mpu.testConnection();
    if (sensorOk) {
      // ±8g scale gives better dynamic range for drum impacts
      mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_8);
      // DLPF at 188 Hz bandwidth: internal gyro rate = 1 kHz, accel bandwidth = 184 Hz.
      // Sharp enough to capture drum impact spikes; clean enough to suppress RF noise.
      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_188);
      // SMPLRT_DIV = 1 → sample rate = 1 kHz / (1 + 1) = 500 Hz, matching DS_POLL_MS=2.
      mpu.setRate(1);
      // ±500°/s gyro range: headroom for fast wrist snaps (~500°/s) while retaining
      // resolution for gentle air-drum swings (50–150°/s).  1 LSB ≈ 0.015°/s.
      mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_500);
      if (irqEnabled && !enableMpuDataReadyInterrupt()) {
        irqEnabled = false;
        DEBUG_PRINTLN(F("[Drumstick] MPU DATA_RDY interrupt config failed, using polling"));
      } else if (irqEnabled) {
        DEBUG_PRINTLN(F("[Drumstick] MPU DATA_RDY interrupt enabled"));
      }
    }

    magOk = initHmc5883();
    DEBUG_PRINTF("[Drumstick] HMC5883 %s\n", magOk ? "OK (via MPU XDA/XCL)" : "NOT FOUND");

    initDone = true;
    DEBUG_PRINTF("[Drumstick] MPU6050 %s\n", sensorOk ? "OK" : "NOT FOUND (check power/wiring/pins)");
  }

  void loop() override {
    if (!enabled || !initDone) return;

    // Clean up stale WebSocket clients periodically (required by ESPAsyncWebServer).
    _drumWs.cleanupClients();

    unsigned long now = millis();

    // Periodic heartbeat so the receiver can detect connection loss
    if (WLED_CONNECTED && (now - lastHbeat >= DS_HEARTBEAT_MS)) {
      sendHeartbeat();
      lastHbeat = now;
    }

    // If IRQ is configured, read on data-ready edges; otherwise use polling.
    if (irqEnabled) {
      if (!_irqFired) return;
      _irqFired = false;
      lastPoll = now;
    } else {
      if (now - lastPoll < DS_POLL_MS) return;
      lastPoll = now;
    }

    if (!sensorOk) return;

    if (magOk && (now - lastMagPoll >= DS_MAG_POLL_MS)) {
      lastMagPoll = now;
      if (!readHmc5883()) {
        magOk = false;
        DEBUG_PRINTLN(F("[Drumstick] HMC5883 read failed, disabled"));
      }
    }

    // ── read accelerometer + gyroscope ───────────────────────────────────
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    lastAx = ax; lastAy = ay; lastAz = az;
    lastGx = gx; lastGy = gy; lastGz = gz;

    // In raw stream mode, forward all sensor data to the PC for processing.
    if (rawStream) {
      sendRawPacket(ax, ay, az, gx, gy, gz);
      return;
    }

    // Maintain a slow gravity estimate (used for tilt-awareness and raw stream).
    if (!gravInit) {
      grav[0] = (float)ax; grav[1] = (float)ay; grav[2] = (float)az;
      gravInit = true;
      return;
    }
    const float lpf = 0.92f;
    grav[0] = grav[0] * lpf + (1.0f - lpf) * (float)ax;
    grav[1] = grav[1] * lpf + (1.0f - lpf) * (float)ay;
    grav[2] = grav[2] * lpf + (1.0f - lpf) * (float)az;

    // ── Swing detector with accel-peak classification ─────────────────
    // Gyro supplies a clean swing envelope even for soft air-drumming strokes.
    // The emitted hit, however, comes from the strongest linear-acceleration peak
    // inside that envelope so standalone mode reacts to real swing peaks instead of
    // sign changes on a single axis.
    // Air-drum strokes are detected by:
    //   1. \u03c9mag rising above gyroOnsetLSB  (swing begins)
    //   2. linear acceleration peaking         (peak vector captured for zone)
    //   3. \u03c9mag dropping to DS_GYRO_DECAY of peak (swing complete)
    const float fgx   = (float)gx;
    const float fgy   = (float)gy;
    const float fgz   = (float)gz;
    const float omegaRaw = sqrtf(fgx*fgx + fgy*fgy + fgz*fgz);
    omegaFilt = omegaFilt * DS_OMEGA_LPF + (1.0f - DS_OMEGA_LPF) * omegaRaw;
    const float omega = omegaFilt;
    const float linRawX = (float)ax - grav[0];
    const float linRawY = (float)ay - grav[1];
    const float linRawZ = (float)az - grav[2];
    linFilt[0] = linFilt[0] * DS_LINACC_LPF + (1.0f - DS_LINACC_LPF) * linRawX;
    linFilt[1] = linFilt[1] * DS_LINACC_LPF + (1.0f - DS_LINACC_LPF) * linRawY;
    linFilt[2] = linFilt[2] * DS_LINACC_LPF + (1.0f - DS_LINACC_LPF) * linRawZ;
    const float linAx = linFilt[0];
    const float linAy = linFilt[1];
    const float linAz = linFilt[2];
    const float linAccel = sqrtf(linAx*linAx + linAy*linAy + linAz*linAz);
    currLinAccel = linAccel;
    currOmega = omega;
    rotateToWorld(linAx, linAy, linAz, currWorldRight, currWorldForward, currWorldUp);

    // Track running peak for calibration display.
    if ((uint16_t)min((uint32_t)omega, (uint32_t)65535u) > peakSeen)
      peakSeen = (uint16_t)min((uint32_t)omega, (uint32_t)65535u);

    switch (swingState) {
      case SwingState::IDLE:
        if (omega >= (float)gyroOnsetLSB) {
          swingState   = SwingState::SWING;
          peakOmega    = omega;
          peakLinAccel = linAccel;
          peakLinAx = linAx; peakLinAy = linAy; peakLinAz = linAz;
          peakWorldRight = currWorldRight; peakWorldForward = currWorldForward; peakWorldUp = currWorldUp;
          swingSamples = 1;
          lastOnsetTs  = now;
        }
        break;

      case SwingState::SWING:
        swingSamples++;
        if (omega > peakOmega) peakOmega = omega;
        if (linAccel > peakLinAccel) {
          peakLinAccel = linAccel;
          peakLinAx = linAx; peakLinAy = linAy; peakLinAz = linAz;
          peakWorldRight = currWorldRight; peakWorldForward = currWorldForward; peakWorldUp = currWorldUp;
        }
        // Wrist-stop detected: \u03c9mag has decayed to DS_GYRO_DECAY of peak.
        // Require at least 3 samples (6ms) to reject single-sample noise spikes.
        if (swingSamples >= 3 && omega < peakOmega * DS_GYRO_DECAY) {
          swingState = SwingState::DECAY;
          break;
        }
        // Abort strokes that run too long (arm movement, not a clean hit gesture).
        if (swingSamples > DS_SWING_MAX_SAMPLES) {
          swingState = SwingState::IDLE;
        }
        break;

      case SwingState::DECAY:
        // Stroke is complete.  Classify and emit, then return to IDLE.
        swingState = SwingState::IDLE;
        {
          if (lastHitTs > 0 && (now - lastHitTs) < (unsigned long)cooldown_ms) break;
          if (peakLinAccel < (float)accelPeakThreshold()) break;

          if (learnZone >= 0 && learnZone < 4) {
            if (setZoneAim((uint8_t)learnZone, peakWorldRight, peakWorldForward, peakWorldUp)) configNeedsWrite = true;
            detectedZone = (uint8_t)learnZone;
            char msg[96];
            snprintf(msg, sizeof(msg), "Learned %u from the last stroke.", (unsigned)learnZone);
            sendWsInfo(msg);
            learnZone = -1;
            sendWsConfig();
          }

          detectedZone = inferZoneFromAccelPeak(peakLinAx, peakLinAy, peakLinAz,
                                                peakWorldRight, peakWorldForward, peakWorldUp);
          const uint8_t zone = autoZone ? detectedZone : activeZone;
          const uint8_t vel  = calcVelocity(peakOmega);

          // Confidence: how far above the accel-peak floor the swing reached.
          // 1× floor = 0, 4× floor = 100.
          const float accelFloor = (float)accelPeakThreshold();
          int32_t conf = (int32_t)((peakLinAccel / accelFloor - 1.0f) * 33.3f);
          if (conf < 0)   conf = 0;
          if (conf > 100) conf = 100;

          sendHitPacket(zone, vel, (uint8_t)conf);
          broadcastWsHit(zone, vel);
          lastHitTs = now;
          lastVel   = vel;
        }
        break;
    }

    if (now - lastWsTelemetry >= DS_WS_TELEM_MS) {
      lastWsTelemetry = now;
      sendWsTelemetry();
    }
  }

  // ── Info page (read-only runtime data) ───────────────────────────────

  void addToJsonInfo(JsonObject& root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    // Sensor connection status
    JsonArray sArr = user.createNestedArray(F("Drumstick sensor"));
    sArr.add(sensorOk ? F("connected") : F("not found"));

    JsonArray mArr2 = user.createNestedArray(F("Drumstick magnetometer"));
    mArr2.add(magOk ? F("connected") : F("not found"));
    mArr2.add(magVecInit ? F("3D fused") : F("warming up"));

    // Last hit info
    JsonArray hArr = user.createNestedArray(F("Last drum hit"));
    if (lastHitTs > 0) {
      hArr.add((unsigned long)((millis() - lastHitTs) / 1000));
      hArr.add(F("s ago  vel="));
      hArr.add(lastVel);
    } else {
      hArr.add(F("none yet"));
    }

    JsonArray zArr = user.createNestedArray(F("Detected zone"));
    zArr.add(autoZone ? detectedZone : activeZone);
    zArr.add(autoZone ? F("auto") : F("manual"));

    JsonArray wArr = user.createNestedArray(F("World vector"));
    wArr.add((int)currWorldRight);
    wArr.add((int)currWorldForward);
    wArr.add((int)currWorldUp);

      JsonArray iArr = user.createNestedArray(F("Sensor IRQ"));
      iArr.add(irqEnabled ? F("enabled") : F("polling"));
      iArr.add(interruptPin);

    JsonArray rArr = user.createNestedArray(F("Swing state"));
    const char* stateStr = (swingState == SwingState::SWING) ? "SWING" :
                           (swingState == SwingState::DECAY) ? "DECAY" : "IDLE";
    rArr.add(stateStr);

    JsonArray mArr = user.createNestedArray(F("Peak \u03c9mag (this stroke)"));
    mArr.add((int)peakOmega);
    mArr.add(F(" LSBs"));

    JsonArray aArr = user.createNestedArray(F("Peak linear accel"));
    aArr.add((int)peakLinAccel);
    aArr.add(F(" LSBs"));

    // Running peak across all strokes — useful for tuning gyroOnsetLSB.
    JsonArray pArr = user.createNestedArray(F("Peak \u03c9mag (all time)"));
    pArr.add(peakSeen);
    pArr.add(F(" LSBs"));
  }

  // ── Config persistence ────────────────────────────────────────────────

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = enabled;
    top["threshold"]     = threshold;
    top["cooldown"]      = cooldown_ms;
    top["irqPin"]        = irqPin;
    top["zone"]          = activeZone;
    top["autoZone"]      = autoZone;
    top["motionSens"]    = motionSensPermille;
    top["repeatMin"]     = repeatMinMs;
    top["repeatMax"]     = repeatMaxMs;
    top["repeatNeed"]    = repeatNeeded;
    top["velCurve"]      = velocityCurve;
    top["rawStream"]     = rawStream;
    top["useMag"]        = useMagnetometer;
    top["gyroOnset"]     = gyroOnsetLSB;
    top["z0x"]           = zoneAim[0][0];
    top["z0y"]           = zoneAim[0][1];
    top["z0z"]           = zoneAim[0][2];
    top["z1x"]           = zoneAim[1][0];
    top["z1y"]           = zoneAim[1][1];
    top["z1z"]           = zoneAim[1][2];
    top["z2x"]           = zoneAim[2][0];
    top["z2y"]           = zoneAim[2][1];
    top["z2z"]           = zoneAim[2][2];
    top["z3x"]           = zoneAim[3][0];
    top["z3y"]           = zoneAim[3][1];
    top["z3z"]           = zoneAim[3][2];
    top["udpHost"]       = udpHost;
    top["udpPort"]       = udpPort;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[FPSTR(_name)];
    bool ok = !top.isNull();
    ok &= getJsonValue(top[FPSTR(_enabled)], enabled,       false);
    ok &= getJsonValue(top["threshold"],     threshold,     (uint16_t)20000);
    ok &= getJsonValue(top["cooldown"],      cooldown_ms,   (uint16_t)150);
    ok &= getJsonValue(top["irqPin"],        irqPin,        (int8_t)3);
    ok &= getJsonValue(top["zone"],          activeZone,    (uint8_t)0);
    ok &= getJsonValue(top["autoZone"],      autoZone,      true);
    ok &= getJsonValue(top["motionSens"],    motionSensPermille, (uint16_t)2200);
    ok &= getJsonValue(top["repeatMin"],     repeatMinMs,   (uint16_t)70);
    ok &= getJsonValue(top["repeatMax"],     repeatMaxMs,   (uint16_t)450);
    ok &= getJsonValue(top["repeatNeed"],    repeatNeeded,  (uint8_t)1);
    ok &= getJsonValue(top["velCurve"],      velocityCurve, (uint8_t)0);
    ok &= getJsonValue(top["rawStream"],     rawStream,     false);
    ok &= getJsonValue(top["useMag"],        useMagnetometer, true);
    ok &= getJsonValue(top["gyroOnset"],     gyroOnsetLSB,  (uint16_t)5000);
    ok &= getJsonValue(top["z0x"],           zoneAim[0][0], (int16_t)-707);
    ok &= getJsonValue(top["z0y"],           zoneAim[0][1], (int16_t) 707);
    ok &= getJsonValue(top["z0z"],           zoneAim[0][2], (int16_t)   0);
    ok &= getJsonValue(top["z1x"],           zoneAim[1][0], (int16_t)-707);
    ok &= getJsonValue(top["z1y"],           zoneAim[1][1], (int16_t)-707);
    ok &= getJsonValue(top["z1z"],           zoneAim[1][2], (int16_t)   0);
    ok &= getJsonValue(top["z2x"],           zoneAim[2][0], (int16_t) 707);
    ok &= getJsonValue(top["z2y"],           zoneAim[2][1], (int16_t) 707);
    ok &= getJsonValue(top["z2z"],           zoneAim[2][2], (int16_t)   0);
    ok &= getJsonValue(top["z3x"],           zoneAim[3][0], (int16_t) 707);
    ok &= getJsonValue(top["z3y"],           zoneAim[3][1], (int16_t)-707);
    ok &= getJsonValue(top["z3z"],           zoneAim[3][2], (int16_t)   0);
    ok &= getJsonValue(top["udpPort"],       udpPort,       (uint16_t)9000);
    // Char array needs manual copy; tolerate missing key (returns false)
    if (top.containsKey("udpHost") && top["udpHost"].is<const char*>()) {
      strlcpy(udpHost, top["udpHost"] | "", sizeof(udpHost));
    }
    return ok;
  }

  // ── Settings UI enhancements ──────────────────────────────────────────

  void appendConfigData() override {
    // Zone mode selector
    oappend(F("dd=addDropdown('Drumstick','autoZone');"));
    oappend(F("addOption(dd,'Auto zone from movement',1);"));
    oappend(F("addOption(dd,'Manual fixed zone',0);"));
    oappend(F("dd=addDropdown('Drumstick','useMag');"));
    oappend(F("addOption(dd,'Use HMC5883 on MPU XDA/XCL',1);"));
    oappend(F("addOption(dd,'Disable magnetometer fusion',0);"));
    // Zone selector
    oappend(F("dd=addDropdown('Drumstick','zone');"));
    oappend(F("addOption(dd,'Snare',0);"));
    oappend(F("addOption(dd,'Kick',1);"));
    oappend(F("addOption(dd,'Hi-Hat',2);"));
    oappend(F("addOption(dd,'Tom',3);"));
    // Velocity curve
    oappend(F("dd=addDropdown('Drumstick','velCurve');"));
    oappend(F("addOption(dd,'Linear',0);"));
    oappend(F("addOption(dd,'Logarithmic',1);"));
    oappend(F("addOption(dd,'Squared',2);"));
    // Hints
    oappend(F("addInfo('Drumstick:autoZone',1,'auto matches the filtered accel swing vector against learned world-space zone aims from gravity plus all 3 magnetometer axes');"));
    oappend(F("addInfo('Drumstick:useMag',1,'enable HMC5883 fusion over MPU auxiliary bus (XDA/XCL)');"));
    oappend(F("addInfo('Drumstick:gyroOnset',1,'angular velocity onset threshold in raw LSBs (±500dps FS); 5000≈76dps; lower = more sensitive');"));
    oappend(F("addInfo('Drumstick:repeatMin',1,'(legacy, unused with gyro FSM)');"));
    oappend(F("addInfo('Drumstick:repeatMax',1,'(legacy, unused with gyro FSM)');"));
    oappend(F("addInfo('Drumstick:repeatNeed',1,'(legacy, unused with gyro FSM)');"));
      oappend(F("addInfo('Drumstick:irqPin',1,'optional MPU6050 INT pin; set -1 to disable and poll');"));
    oappend(F("addInfo('Drumstick:threshold',1,'minimum accel-peak floor; effective floor is max(512, threshold/8)');"));
    oappend(F("addInfo('Drumstick:cooldown',1,'ms between hits (50\u2013500)');"));
    oappend(F("addInfo('Drumstick:udpHost',1,'receiver IP or hostname');"));
    oappend(F("addInfo('Drumstick:udpPort',1,'default 9000');"));
    // Raw stream mode toggle
    oappend(F("dd=addDropdown('Drumstick','rawStream');"));
    oappend(F("addOption(dd,'Stream raw data to PC (v2 packets)',1);"));
    oappend(F("addOption(dd,'Processed hit events (v1 packets)',0);"));
    oappend(F("addInfo('Drumstick:rawStream',1,'raw mode sends accel+gyro+mag at 200Hz; PC does all detection');"));
    oappend(F("addInfo('Drumstick:zone',1,'manual zone for non-auto mode; /drum exposes live sensor debug and one-stroke zone teaching');"));
  }

  uint16_t getId() override { return USERMOD_ID_DRUMSTICK; }
};

// ── PROGMEM string storage ────────────────────────────────────────────────

const char DrumstickUsermod::_name[]    PROGMEM = "Drumstick";
const char DrumstickUsermod::_enabled[] PROGMEM = "enabled";
volatile bool DrumstickUsermod::_irqFired = false;

// ── Static instance + auto-registration ──────────────────────────────────

static DrumstickUsermod drumstick_usermod;
REGISTER_USERMOD(drumstick_usermod);
