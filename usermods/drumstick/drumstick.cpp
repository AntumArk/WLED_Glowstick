/*
 * Drumstick usermod for WLED — BNO055 edition
 *
 * Detects drumming motion via BNO055 9-DOF AHRS (NDOF fusion mode) and
 * streams hit events over UDP / WebSocket. Also manages user button and
 * optional 1S LiPo battery monitor.
 *
 * Hardware (Seeed XIAO ESP32-C6):
 *   BNO055 : I2C addr 0x28 (ADR=GND), SDA=D4, SCL=D5, INT=GPIO3 (optional)
 *   Battery: D1 ADC input by default (set battPin in config to override)
 *   Button : D0 input (active-HIGH, pull-down)
 *              short press (<3500 ms) → cycle LED effect
 *   LED    : D10 (set in WLED settings, not here)
 *
 * BNO055 NDOF provides directly:
 *   getVector(VECTOR_LINEARACCEL)  — gravity-removed acceleration (m/s²)
 *   getVector(VECTOR_GYROSCOPE)    — calibrated angular rate (deg/s)
 *   getQuat()                      — absolute ENU orientation quaternion
 *   getCalibration()               — system/gyro/accel/mag quality 0–3
 *   This replaces the manual gravity LPF + HMC5883 cross-product AHRS.
 *
 * Swing detection (gyro FSM, unchanged in structure):
 *   IDLE → (|ω| ≥ gyroOnsetDps deg/s) → SWING → (|ω| < peak×0.45) → DECAY → emit
 *   World-frame rotation uses the BNO055 absolute quaternion (ENU frame).
 *
 * UDP packet schema v1 (hit):
 *   {"v":1,"seq":N,"t":ms,"z":zone,"vel":1-127,"conf":0-100,"s":"BNO"}
 * UDP packet schema v2 (raw stream, accel/gyro in m/s²·100 / deg/s·10 scaled int):
 *   {"v":2,"seq":N,"t":ms,"ax":…,"ay":…,"az":…,"gx":…,"gy":…,"gz":…,"mx":…,"my":…,"mz":…}
 * Heartbeat:
 *   {"v":1,"seq":N,"t":ms,"hb":1,"imu":0|1,"cal":0-3,"sda":pin,"scl":pin}
 *
 * References:
 *   BNO055 datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
 *   Adafruit BNO055 library: https://github.com/adafruit/Adafruit_BNO055
 */

#include "wled.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFiUdp.h>

// ── constants ──────────────────────────────────────────────────────────────

static constexpr uint16_t DS_POLL_MS          = 10;     // sensor poll → 100 Hz (BNO055 NDOF ODR)
static constexpr uint16_t DS_HEARTBEAT_MS     = 5000;   // ms between heartbeat UDP packets
static constexpr uint8_t  DS_UDP_BUF          = 140;    // max UDP payload bytes
static constexpr uint16_t DS_RAW_UDP_BUF      = 200;    // raw sensor packet max bytes
static constexpr uint16_t DS_WS_TELEM_MS      = 50;     // browser telemetry cadence (~20 Hz)

static constexpr uint8_t  DS_BNO_ADDR         = 0x28;   // ADR pin tied to GND → address LOW

// Gyro stroke FSM.
// BNO055 NDOF gyro output is calibrated deg/s.
// DS_GYRO_DECAY : |ω| must fall to this fraction of stroke peak to mark wrist-stop.
// DS_SWING_MAX  : abort stroke after 1500 ms @ 100 Hz = 150 samples.
static constexpr float    DS_GYRO_DECAY        = 0.45f;
static constexpr uint16_t DS_SWING_MAX_SAMPLES = 150;

// Low-pass filter coefficients (higher = smoother, more lag)
static constexpr float    DS_LINACC_LPF        = 0.55f;
static constexpr float    DS_OMEGA_LPF         = 0.50f;

// Battery 1S LiPo: linear curve 3.4 V = 0%, 4.2 V = 100%
static constexpr float    DS_BATT_MIN_V        = 3.4f;
static constexpr float    DS_BATT_MAX_V        = 4.2f;
static constexpr float    DS_BATT_LPF          = 0.97f; // slow LPF ~5 s TC at 10 ms poll rate
// Divider ratio: 100k / (100k+100k) = 0.5 → multiply ADC by 2 to recover Vin
static constexpr float    DS_BATT_DIV          = 2.0f;
static constexpr float    DS_ADC_VREF          = 3.3f;
static constexpr float    DS_ADC_COUNTS        = 4095.0f;

// ── Embedded drum-synth page served at /drum ─────────────────────────────────
// Self-contained: no LittleFS files needed.
// Browser connects to /drum/ws (WebSocket) for live telemetry, config, and hit events.
// Web Audio synth plays sounds for each hit zone.
// BNO055 diagnostics panel shows calibration, quaternion, euler and self-test results.
static const char DRUM_PAGE[] PROGMEM = R"RAWHTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drumstick</title>
<style>
  :root{--bg:#09141b;--panel:#102431;--panel2:#0c1b25;--line:#28506a;--text:#edf6ff;
        --muted:#9fc0d4;--warm:#e08a2e;--red:#cf453d;--cyan:#2ea9d0;--violet:#8460e8;
        --good:#68d391;--amber:#e08a2e;}
  *{box-sizing:border-box} body{background:radial-gradient(circle at top,#163447 0,#09141b 52%,#050b0f 100%);
      color:var(--text);font-family:Segoe UI,system-ui,sans-serif;margin:0;padding:18px;}
  .wrap{max-width:1280px;margin:0 auto;display:grid;gap:14px}
  .hero{display:flex;justify-content:space-between;gap:12px;align-items:end;flex-wrap:wrap}
  h1{margin:0;font-size:1.7rem;letter-spacing:.08em}
  #status{font-size:.88rem;color:var(--muted)}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px}
  .panel{background:linear-gradient(180deg,rgba(21,46,61,.92),rgba(10,24,34,.96));
         border:1px solid rgba(92,149,184,.35);border-radius:16px;padding:14px;
         box-shadow:0 18px 48px rgba(0,0,0,.24)}
  .panel h2{margin:0 0 10px;font-size:1rem;letter-spacing:.05em;color:#dff0ff}
  .panel h3{margin:0 0 8px;font-size:.88rem;color:#d2e8f8}
  #pads{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .pad{border-radius:12px;height:84px;display:flex;align-items:center;justify-content:center;
       font-size:1rem;font-weight:700;transition:transform .08s,filter .08s;filter:brightness(.35)}
  .pad.hit{filter:brightness(1.0);transform:translateY(-2px)}
  #p0{background:var(--warm)} #p1{background:var(--red)} #p2{background:var(--cyan)} #p3{background:var(--violet)}
  .kv{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;font-size:.82rem}
  .kv div,.mini{background:rgba(4,11,16,.24);border:1px solid rgba(92,149,184,.18);
                border-radius:10px;padding:8px}
  .kv2{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;font-size:.82rem}
  .mini b{display:block;font-size:.72rem;color:var(--muted);font-weight:600;margin-bottom:4px}
  .controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
  label{display:flex;flex-direction:column;gap:5px;font-size:.78rem;color:var(--muted)}
  input,select,button{border-radius:10px;border:1px solid rgba(92,149,184,.35);
                      background:#0a1a24;color:var(--text);padding:9px 10px;font:inherit}
  button{cursor:pointer;background:linear-gradient(180deg,#17354a,#102633)}
  button.secondary{background:linear-gradient(180deg,#13242f,#0d1921)}
  button.warn{background:linear-gradient(180deg,#704026,#4d2a18)}
  .meters{display:grid;gap:10px}
  .meter{display:grid;gap:5px}
  .meterRow{display:flex;justify-content:space-between;font-size:.78rem;color:var(--muted)}
  .track{height:12px;border-radius:999px;background:#08131a;
         border:1px solid rgba(92,149,184,.25);position:relative;overflow:hidden}
  .fill{height:100%;background:linear-gradient(90deg,#2ea9d0,#68d391);width:0%;transition:width .1s}
  .calFill{height:100%;width:0%;border-radius:999px;transition:width .3s,background .6s}
  .marker{position:absolute;top:0;bottom:0;width:2px;background:#ffdc73;opacity:.95}
  .projWrap{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  canvas{width:100%;aspect-ratio:1/1;background:radial-gradient(circle at center,#112533 0,#0a1720 65%,#081118 100%);
         border-radius:12px;border:1px solid rgba(92,149,184,.24)}
  .mapGrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
  .mapCell{background:rgba(4,11,16,.24);border:1px solid rgba(92,149,184,.18);
           border-radius:10px;padding:10px}
  .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
  .muted{color:var(--muted);font-size:.78rem}
  .badge{display:inline-block;padding:2px 8px;border-radius:6px;font-size:.72rem;font-weight:700}
  .badge.ok{background:#1a4a2e;color:var(--good)} .badge.err{background:#4a1a1a;color:var(--red)}
  #log{height:140px;overflow-y:auto;font-size:.74rem;color:#a8c2d3;
       border-top:1px solid rgba(92,149,184,.2);padding-top:8px}
  @media(max-width:760px){.kv{grid-template-columns:repeat(2,minmax(0,1fr))}
    .projWrap{grid-template-columns:1fr}}
</style></head><body>
<div class="wrap">
<div class="hero">
  <div><h1>&#x1F941; Drumstick</h1><div id="status">connecting…</div></div>
  <div class="row muted"><span>Volume</span>
    <input id="vr" type="range" min="0" max="1" step=".05" value="0.8">
  </div>
</div>

<div class="grid">
  <section class="panel">
    <h2>Live Pads</h2>
    <div id="pads">
      <div class="pad" id="p0">Snare</div><div class="pad" id="p1">Kick</div>
      <div class="pad" id="p2">Hi-Hat</div><div class="pad" id="p3">Tom</div>
    </div>
    <div class="kv" style="margin-top:10px">
      <div class="mini"><b>Swing</b><span id="swingState">IDLE</span></div>
      <div class="mini"><b>Cal (Sys)</b><span id="orientationVal">0/3</span></div>
      <div class="mini"><b>Learn</b><span id="learnState">off</span></div>
      <div class="mini"><b>Zone</b><span id="zonePreview">Snare</span></div>
    </div>
  </section>

  <section class="panel">
    <h2>Thresholds</h2>
    <div class="meters">
      <div class="meter">
        <div class="meterRow"><span>Linear accel peak (m/s²)</span><span id="linText">0 / 0</span></div>
        <div class="track"><div class="fill" id="linFill"></div><div class="marker" id="linMark"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Gyro swing envelope (deg/s)</span><span id="gyroText">0 / 0</span></div>
        <div class="track"><div class="fill" id="gyroFill"></div><div class="marker" id="gyroMark"></div></div>
      </div>
    </div>
    <div class="kv" style="margin-top:10px">
      <div class="mini"><b>World vec now</b><span id="worldNow">0 0 0</span></div>
      <div class="mini"><b>World vec peak</b><span id="worldPeak">0 0 0</span></div>
      <div class="mini"><b>Peak accel m/s²</b><span id="peakAccel">0</span></div>
      <div class="mini"><b>Peak omega deg/s</b><span id="peakOmega">0</span></div>
    </div>
  </section>
</div>

<div class="grid">
  <section class="panel">
    <h2>BNO055 Diagnostics</h2>
    <div class="kv" style="margin-bottom:10px">
      <div class="mini"><b>IMU status</b><span id="bnoStatus"><span class="badge err">—</span></span></div>
      <div class="mini"><b>Fusion mode</b><span id="bnoMode">—</span></div>
      <div class="mini"><b>Chip temp</b><span id="bnoTemp">—</span></div>
      <div class="mini"><b>Battery</b><span id="battVal">—</span></div>
    </div>
    <div class="meters">
      <div class="meter">
        <div class="meterRow"><span>Sys calibration</span><span id="calSysT">0/3</span></div>
        <div class="track"><div class="calFill" id="calSysF"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Gyro calibration</span><span id="calGyrT">0/3</span></div>
        <div class="track"><div class="calFill" id="calGyrF"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Accel calibration</span><span id="calAccT">0/3</span></div>
        <div class="track"><div class="calFill" id="calAccF"></div></div>
      </div>
      <div class="meter">
        <div class="meterRow"><span>Mag calibration</span><span id="calMagT">0/3</span></div>
        <div class="track"><div class="calFill" id="calMagF"></div></div>
      </div>
    </div>
    <div class="kv2" style="margin-top:10px">
      <div class="mini"><b>Quaternion W X Y Z</b><span id="quatVal">—</span></div>
      <div class="mini"><b>Euler H / R / P (deg)</b><span id="eulerVal">—</span></div>
    </div>
    <div class="row" style="margin-top:10px">
      <button id="selfTest" class="secondary">Run BNO055 self-test</button>
    </div>
    <div class="muted" style="margin-top:8px">
      Keep still (gyro cal 3) then wave slowly (accel cal 3) then rotate in
      figure-8 (mag cal 3). Sys cal 3 enables reliable world-frame zone detection.
    </div>
  </section>

  <section class="panel">
    <h2>Tuning</h2>
    <div class="controls">
      <label>Accel floor ×100 (m/s²)
        <input id="threshold" type="number" min="50" step="50">
      </label>
      <label>Gyro onset (deg/s)
        <input id="gyroOnset" type="number" min="10" step="5">
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
    <div class="muted" style="margin-top:8px">
      Accel floor ×100: value 500 = 5.0 m/s² threshold.
      Gyro onset: typical 60–100 deg/s. Changes are persisted to WLED config.
    </div>
  </section>
</div>

<div class="grid">
  <section class="panel">
    <h2>Debug Projections</h2>
    <div class="projWrap">
      <div>
        <h3>World horizontal (E/N)</h3>
        <canvas id="xy"></canvas>
        <div class="muted">East/North projection of linear accel in BNO055 ENU world frame.</div>
      </div>
      <div>
        <h3>East vs Up</h3>
        <canvas id="xz"></canvas>
        <div class="muted">East/Up projection — shows vertical swing component.</div>
      </div>
    </div>
  </section>

  <section class="panel">
    <h2>Zone Teaching</h2>
    <div class="mapGrid" id="mapGrid"></div>
    <div class="muted" style="margin-top:8px">
      Pick a zone, aim the stick, play one clean stroke. The firmware stores
      the BNO055 world-space direction vector. Requires Sys cal ≥ 1.
    </div>
  </section>
</div>

<div class="grid">
  <section class="panel">
    <h2>Sensor Debug</h2>
    <div class="kv">
      <div class="mini"><b>Accel raw (m/s²)</b><span id="accelVal">—</span></div>
      <div class="mini"><b>Linear accel (m/s²)</b><span id="linVal">—</span></div>
      <div class="mini"><b>Gyro (deg/s)</b><span id="gyroVal">—</span></div>
      <div class="mini"><b>Mag (µT)</b><span id="magVal">—</span></div>
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
function adsr(g,a,d,s,r,now){
  g.gain.cancelScheduledValues(now);
  g.gain.setValueAtTime(0,now);
  g.gain.linearRampToValueAtTime(1,now+a);
  g.gain.linearRampToValueAtTime(s,now+a+d);
  g.gain.setValueAtTime(s,now+a+d+.001);
  g.gain.linearRampToValueAtTime(0,now+a+d+r);
}
const SYNTHS=[
  function snare(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127,dur=.18;
    const buf=C.createBuffer(1,C.sampleRate*dur,C.sampleRate);
    const d=buf.getChannelData(0);for(let i=0;i<d.length;i++)d[i]=(Math.random()*2-1);
    const ns=C.createBufferSource();ns.buffer=buf;
    const nf=C.createBiquadFilter();nf.type='bandpass';nf.frequency.value=2400;nf.Q.value=.8;
    const ng=C.createGain();adsr(ng,.002,.04,.1,.09,t);
    ns.connect(nf);nf.connect(ng);ng.connect(masterGain());ns.start(t);ns.stop(t+dur);
    const osc=C.createOscillator();osc.type='triangle';osc.frequency.value=200;
    osc.frequency.linearRampToValueAtTime(80,t+.05);
    const og=C.createGain();og.gain.setValueAtTime(.6*v,t);og.gain.linearRampToValueAtTime(0,t+.12);
    osc.connect(og);og.connect(masterGain());osc.start(t);osc.stop(t+.14);
  },
  function kick(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127;
    const osc=C.createOscillator();osc.type='sine';
    osc.frequency.setValueAtTime(160,t);osc.frequency.exponentialRampToValueAtTime(40,t+.08);
    const g=C.createGain();g.gain.setValueAtTime(1.2*v,t);g.gain.linearRampToValueAtTime(0,t+.32);
    const click=C.createOscillator();click.type='square';click.frequency.value=600;
    const cg=C.createGain();cg.gain.setValueAtTime(.4*v,t);cg.gain.linearRampToValueAtTime(0,t+.02);
    click.connect(cg);cg.connect(masterGain());click.start(t);click.stop(t+.025);
    osc.connect(g);g.connect(masterGain());osc.start(t);osc.stop(t+.35);
  },
  function hihat(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127,dur=.12;
    const buf=C.createBuffer(1,C.sampleRate*dur,C.sampleRate);
    const d=buf.getChannelData(0);for(let i=0;i<d.length;i++)d[i]=(Math.random()*2-1);
    const ns=C.createBufferSource();ns.buffer=buf;
    const f=C.createBiquadFilter();f.type='highpass';f.frequency.value=8000;
    const g=C.createGain();g.gain.setValueAtTime(.7*v,t);g.gain.linearRampToValueAtTime(0,t+.10);
    ns.connect(f);f.connect(g);g.connect(masterGain());ns.start(t);ns.stop(t+dur);
  },
  function tom(vel){
    const C=getCtx(),t=C.currentTime,v=vel/127;
    const osc=C.createOscillator();osc.type='sine';
    osc.frequency.setValueAtTime(120,t);osc.frequency.exponentialRampToValueAtTime(55,t+.12);
    const g=C.createGain();g.gain.setValueAtTime(v,t);g.gain.linearRampToValueAtTime(0,t+.28);
    osc.connect(g);g.connect(masterGain());osc.start(t);osc.stop(t+.3);
  }
];

// ── state ──────────────────────────────────────────────────────────────────
const NAMES=['Snare','Kick','Hi-Hat','Tom'];
const COLORS=['#e08a2e','#cf453d','#2ea9d0','#8460e8'];
const log=document.getElementById('log');
const st=document.getElementById('status');
const state={
  cfg:{th:500,ath:5,gth:75,cd:150,az:1,zone:0,learn:-1,
       z0x:-707,z0y:707,z0z:0,z1x:-707,z1y:-707,z1z:0,
       z2x:707,z2y:707,z2z:0,z3x:707,z3y:-707,z3z:0},
  tele:{wr:0,wf:0,wu:0,pwr:0,pwf:0,pwu:0,la:0,pla:0,om:0,
        ori:0,st:'IDLE',zone:0,
        ax:0,ay:0,az:0,lax:0,lay:0,laz:0,gx:0,gy:0,gz:0,mx:0,my:0,mz:0,
        ok:0,cal:0,cg:0,ca:0,cm:0,
        qw:1,qx:0,qy:0,qz:0,eh:0,er:0,ep:0,temp:0,batt:0,battp:0}
};

function clamp(v,a,b){return Math.min(b,Math.max(a,v));}
function fmt(v,d=1){return(typeof v==='number'&&isFinite(v))?v.toFixed(d):'0';}
function fmti(v){return(typeof v==='number'&&isFinite(v))?Math.round(v):'0';}

function addLog(msg){
  const l=document.createElement('div');
  l.textContent=new Date().toLocaleTimeString()+' '+msg;
  log.prepend(l);
  while(log.childNodes.length>80)log.removeChild(log.lastChild);
}
function send(obj){if(ws&&ws.readyState===1)ws.send(JSON.stringify(obj));}

function calBar(idFill,idText,val){
  const pct=(val/3*100).toFixed(0)+'%';
  const el=document.getElementById(idFill);
  el.style.width=pct;
  el.style.background=val>=3?'#68d391':val>=1?'#e08a2e':'#cf453d';
  document.getElementById(idText).textContent=val+'/3';
}

function metricFill(idFill,idMarker,val,threshold,maxVal){
  document.getElementById(idFill).style.width=(100*clamp(val/maxVal,0,1)).toFixed(1)+'%';
  document.getElementById(idMarker).style.left=(100*clamp(threshold/maxVal,0,1)).toFixed(1)+'%';
}

function drawProjection(cv,mode){
  const c=cv.getContext('2d');
  const w=cv.width=cv.clientWidth,h=cv.height=cv.clientWidth;
  const cx=w/2,cy=h/2,s=w*0.36;
  c.clearRect(0,0,w,h);
  c.strokeStyle='rgba(120,175,209,.18)';c.lineWidth=1;
  for(let i=1;i<=3;i++){c.beginPath();c.arc(cx,cy,s*i/3,0,Math.PI*2);c.stroke();}
  c.beginPath();c.moveTo(cx,12);c.lineTo(cx,h-12);c.moveTo(12,cy);c.lineTo(w-12,cy);c.stroke();
  c.fillStyle='rgba(159,192,212,.8)';c.font='12px Segoe UI';
  c.fillText(mode==='xy'?'N':'Up',cx+6,18);c.fillText('E',w-20,cy-6);
  // threshold ring at accel floor (ath m/s², full scale ~30 m/s²)
  const t=clamp((state.cfg.ath||1)/30,0,.95)*s;
  c.strokeStyle='rgba(255,220,115,.65)';c.beginPath();c.arc(cx,cy,t,0,Math.PI*2);c.stroke();
  const curX=state.tele.wr, curY=mode==='xy'?state.tele.wf:state.tele.wu;
  const peakX=state.tele.pwr,peakY=mode==='xy'?state.tele.pwf:state.tele.pwu;
  const scale=1/30;
  function dot(x,y,color,r){
    const px=cx+clamp(x*scale,-1,1)*s,py=cy-clamp(y*scale,-1,1)*s;
    c.fillStyle=color;c.beginPath();c.arc(px,py,r,0,Math.PI*2);c.fill();
  }
  dot(peakX,peakY,'rgba(255,220,115,.95)',7);
  dot(curX,curY,COLORS[state.tele.zone||0],5);
}

function refreshUI(){
  const tele=state.tele,cfg=state.cfg;
  document.getElementById('swingState').textContent=tele.st;
  document.getElementById('orientationVal').textContent=(tele.cal||0)+'/3';
  document.getElementById('learnState').textContent=cfg.learn>=0?'waiting for '+NAMES[cfg.learn]:'off';
  document.getElementById('zonePreview').textContent=NAMES[tele.zone||0];
  document.getElementById('worldNow').textContent=[fmt(tele.wr),fmt(tele.wf),fmt(tele.wu)].join(' ');
  document.getElementById('worldPeak').textContent=[fmt(tele.pwr),fmt(tele.pwf),fmt(tele.pwu)].join(' ');
  document.getElementById('peakAccel').textContent=fmt(tele.pla,2);
  document.getElementById('peakOmega').textContent=fmt(tele.om,1);
  const linMax=Math.max((cfg.ath||1)*4,20);
  const gyroMax=Math.max((cfg.gth||1)*4,500);
  document.getElementById('linText').textContent=fmt(tele.la,2)+' / '+fmt(cfg.ath,2);
  document.getElementById('gyroText').textContent=fmt(tele.om,1)+' / '+(cfg.gth||75);
  metricFill('linFill','linMark',tele.la,cfg.ath,linMax);
  metricFill('gyroFill','gyroMark',tele.om,cfg.gth,gyroMax);
  document.getElementById('accelVal').textContent=[fmt(tele.ax,2),fmt(tele.ay,2),fmt(tele.az,2)].join(' ');
  document.getElementById('linVal').textContent=[fmt(tele.lax,2),fmt(tele.lay,2),fmt(tele.laz,2)].join(' ');
  document.getElementById('gyroVal').textContent=[fmt(tele.gx,1),fmt(tele.gy,1),fmt(tele.gz,1)].join(' ');
  document.getElementById('magVal').textContent=[fmt(tele.mx,1),fmt(tele.my,1),fmt(tele.mz,1)].join(' ');
  // BNO055 diagnostics
  const connected=tele.ok===1;
  document.getElementById('bnoStatus').innerHTML=connected?
    '<span class="badge ok">connected</span>':'<span class="badge err">not found</span>';
  document.getElementById('bnoMode').textContent=(tele.cal||0)>=1?'NDOF':'warming up';
  document.getElementById('bnoTemp').textContent=(tele.temp||0)+' \u00b0C';
  document.getElementById('battVal').textContent=
    fmt(tele.batt||0,2)+'V ('+(tele.battp||0)+'%)';
  calBar('calSysF','calSysT',tele.cal||0);
  calBar('calGyrF','calGyrT',tele.cg||0);
  calBar('calAccF','calAccT',tele.ca||0);
  calBar('calMagF','calMagT',tele.cm||0);
  document.getElementById('quatVal').textContent=
    [tele.qw,tele.qx,tele.qy,tele.qz].map(v=>fmt(v||0,3)).join(' ');
  document.getElementById('eulerVal').textContent=
    'H:'+fmt(tele.eh||0,1)+' R:'+fmt(tele.er||0,1)+' P:'+fmt(tele.ep||0,1);
  // canvas projections
  drawProjection(document.getElementById('xy'),'xy');
  drawProjection(document.getElementById('xz'),'xz');
}

function buildMapGrid(){
  const mapGrid=document.getElementById('mapGrid');
  mapGrid.innerHTML='';
  NAMES.forEach((label,idx)=>{
    const card=document.createElement('div');card.className='mapCell';
    card.innerHTML='<div class="muted">'+label+'</div><div class="muted" style="margin:6px 0">aim: '+
      [fmti(state.cfg['z'+idx+'x']),fmti(state.cfg['z'+idx+'y']),fmti(state.cfg['z'+idx+'z'])].join(' ')+'</div>';
    const learn=document.createElement('button');learn.textContent='Teach from next stroke';
    learn.className='secondary';learn.onclick=()=>send({cmd:'learn',zone:idx});
    card.appendChild(learn);mapGrid.appendChild(card);
  });
}

function syncControls(){
  document.getElementById('threshold').value=state.cfg.th;
  document.getElementById('gyroOnset').value=state.cfg.gth;
  document.getElementById('cooldown').value=state.cfg.cd;
  document.getElementById('autoZone').value=state.cfg.az?1:0;
  document.getElementById('manualZone').value=state.cfg.zone;
  buildMapGrid();refreshUI();
}

NAMES.forEach((name,z)=>{
  const opt=document.createElement('option');opt.value=z;opt.textContent=name;
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
document.getElementById('selfTest').onclick=()=>send({cmd:'selftest'});

// ── WebSocket ──────────────────────────────────────────────────────────────
let ws,reconnTimer;
function connect(){
  ws=new WebSocket('ws://'+location.host+'/drum/ws');
  ws.onopen=()=>{st.textContent='connected';clearTimeout(reconnTimer);};
  ws.onclose=()=>{st.textContent='disconnected — retrying…';reconnTimer=setTimeout(connect,2000);};
  ws.onmessage=e=>{
    try{
      const m=JSON.parse(e.data);
      if(m.type==='cfg'){state.cfg=m;syncControls();return;}
      if(m.type==='tele'){state.tele=m;refreshUI();return;}
      if(m.type==='info'&&m.msg){addLog(m.msg);return;}
      if(m.z===undefined)return;
      const z=m.z&3,vel=m.vel||64;
      state.tele.zone=z;getCtx();SYNTHS[z](vel);
      const pad=document.getElementById('p'+z);
      pad.classList.add('hit');setTimeout(()=>pad.classList.remove('hit'),120);
      addLog(NAMES[z]+' vel='+vel);refreshUI();
    }catch(_){}
  };
}
document.body.addEventListener('pointerdown',()=>getCtx(),{once:true});
connect();
</script></body></html>
)RAWHTML";

// ── class ──────────────────────────────────────────────────────────────────

class DrumstickUsermod : public Usermod {
private:

  // ── BNO055 sensor ─────────────────────────────────────────────────────
  // Address is user-configurable: 0x28 (ADR=GND) or 0x29 (ADR=3V3).
  // _bno is heap-allocated in setup() once bnoI2cAddr is read from config.
  Adafruit_BNO055* _bno         = nullptr;
  bool             sensorOk     = false;
  bool             initDone     = false;
  unsigned long    lastPoll     = 0;
  unsigned long    lastSlowRead = 0; // tracks euler/temp (read less often)

  imu::Quaternion  _quat;
  uint8_t          _calSys      = 0;
  uint8_t          _calGyro     = 0;
  uint8_t          _calAccel    = 0;
  uint8_t          _calMag      = 0;
  float            _eulerH      = 0.0f;  // heading (yaw), degrees
  float            _eulerR      = 0.0f;  // roll, degrees
  float            _eulerP      = 0.0f;  // pitch, degrees
  int8_t           _bnoTemp     = 0;
  int8_t           irqPin       = 3;
  bool             irqBound     = false;
  bool             irqEnabled   = false;  // true when attachInterrupt succeeded

  static volatile bool _irqFired;
  static void IRAM_ATTR onBnoInterrupt() { _irqFired = true; }

  // Last raw sensor values (floats; BNO055 output is already calibrated SI units)
  float _lastAx = 0, _lastAy = 0, _lastAz = 0;    // raw accel, m/s²
  float _lastLax = 0, _lastLay = 0, _lastLaz = 0;  // linear accel (gravity-free), m/s²
  float _lastGx = 0, _lastGy = 0, _lastGz = 0;    // gyro, deg/s
  float _lastMx = 0, _lastMy = 0, _lastMz = 0;    // magnetometer, µT

  // ── Battery ADC ────────────────────────────────────────────────────────
  // D1: 100kΩ/100kΩ voltage divider from 1S LiPo positive terminal.
  // Vmeas = analogRead(battPin) * DS_ADC_VREF / DS_ADC_COUNTS * DS_BATT_DIV
  uint8_t bnoI2cAddr   = 0x28;  // 0x28 = ADR pin GND, 0x29 = ADR pin 3V3
  int8_t  battPin      = 1;     // D1 by default
  bool    _battPinOk   = false;
  float   _battV       = 0.0f;
  uint8_t _battPct     = 0;
  bool    _battInit    = false;

  // ── User button ────────────────────────────────────────────────────────
  // D0: active-HIGH with INPUT_PULLDOWN.
  int8_t        btnPin         = 0;
  bool          _btnPinOk      = false;
  bool          _btnWasPressed = false;
  unsigned long _btnPressedTs  = 0;
  bool          _longFired     = false;

  // ── Swing FSM ──────────────────────────────────────────────────────────
  // Stroke lifecycle: IDLE → (|ω| ≥ onset) → SWING → (|ω| < peak×decay) → DECAY → emit → IDLE
  enum class SwingState : uint8_t { IDLE, SWING, DECAY };
  SwingState    swingState       = SwingState::IDLE;
  float         peakOmega        = 0.0f;  // peak |ω| this stroke, deg/s
  float         peakLinAccel     = 0.0f;  // peak |linear accel| this stroke, m/s²
  float         peakLinAx        = 0.0f;  // linear accel vector at accel peak
  float         peakLinAy        = 0.0f;
  float         peakLinAz        = 0.0f;
  float         peakWorldRight   = 0.0f;  // world-frame accel at peak (ENU: E, N, Up)
  float         peakWorldForward = 0.0f;
  float         peakWorldUp      = 0.0f;
  float         currWorldRight   = 0.0f;
  float         currWorldForward = 0.0f;
  float         currWorldUp      = 0.0f;
  float         currLinAccel     = 0.0f;
  float         currOmega        = 0.0f;
  float         linFilt[3]       = {0.0f, 0.0f, 0.0f};
  float         omegaFilt        = 0.0f;
  uint16_t      swingSamples     = 0;
  unsigned long lastOnsetTs      = 0;
  uint8_t       detectedZone     = 0;
  unsigned long lastWsTelemetry  = 0;

  // ── UDP transport ──────────────────────────────────────────────────────
  WiFiUDP       udp;
  uint32_t      seqNum    = 0;
  unsigned long lastHbeat = 0;

  // ── WebSocket drum page ────────────────────────────────────────────────
  AsyncWebSocket _drumWs{"/drum/ws"};

  // ── Stats shown in /json/info ──────────────────────────────────────────
  unsigned long lastHitTs = 0;
  uint8_t       lastVel   = 0;
  uint16_t      peakSeen  = 0;  // all-time peak |ω| in deg/s (for tuning gyroOnsetDps)

  // ── Config (persisted via cfg.json) ───────────────────────────────────
  bool          enabled       = false;
  // threshold: accel peak floor stored as (m/s² × 100) to use a uint16.
  // default 500 → 5.0 m/s².  Migration: values > 10000 (old LSB-scale) reset to 500.
  uint16_t      threshold     = 500;
  uint16_t      cooldown_ms   = 150;
  uint8_t       activeZone    = 0;    // 0=Snare 1=Kick 2=Hi-Hat 3=Tom
  bool          autoZone      = true;
  uint8_t       velocityCurve = 0;    // 0=linear, 1=log, 2=squared
  bool          rawStream     = false;
  uint16_t      gyroOnsetDps  = 75;   // deg/s onset threshold (default 75 ≈ old 5000 LSBs)
  int8_t        learnZone     = -1;
  int16_t       zoneAim[4][3] = {
    {-707,  707,    0},
    {-707, -707,    0},
    { 707,  707,    0},
    { 707, -707,    0}
  };
  char          udpHost[40]   = {0};
  uint16_t      udpPort       = 9000;

  static const char _name[];
  static const char _enabled[];

  // ── Helpers ───────────────────────────────────────────────────────────

  // Effective accel peak threshold in m/s².
  float accelPeakThreshold() const {
    return max(0.5f, (float)threshold / 100.0f);
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
    if (len < 1e-4f) return false;
    x /= len; y /= len; z /= len;
    return true;
  }

  static float dot3(float ax, float ay, float az,
                    float bx, float by, float bz) {
    return ax*bx + ay*by + az*bz;
  }

  // Rotate body-frame vector (lx,ly,lz) into ENU world frame using the
  // BNO055 NDOF absolute orientation quaternion q = (w, x, y, z).
  //
  // Uses the efficient Rodrigues sandwich product:
  //   t  = 2 * (q.xyz × v)
  //   vw = v + q.w*t + q.xyz × t
  //
  // Returns false when calibration is too low (sys < 1) and falls back to
  // passing the body vector unchanged so the UI still shows something.
  //
  // Reference: https://en.wikipedia.org/wiki/Quaternion#Hamilton_product
  // AI: below section was generated by an AI
  bool rotateToWorldByQuat(float lx, float ly, float lz,
                           float& wr, float& wf, float& wu) const {
    wr = lx; wf = ly; wu = lz;
    if (_calSys < 1) return false;

    const float qw = (float)_quat.w();
    const float qx = (float)_quat.x();
    const float qy = (float)_quat.y();
    const float qz = (float)_quat.z();

    // t = 2*(q.xyz × v)
    const float tx = 2.0f * (qy * lz - qz * ly);
    const float ty = 2.0f * (qz * lx - qx * lz);
    const float tz = 2.0f * (qx * ly - qy * lx);

    // vw = v + q.w*t + q.xyz × t
    wr = lx + qw * tx + (qy * tz - qz * ty);
    wf = ly + qw * ty + (qz * tx - qx * tz);
    wu = lz + qw * tz + (qx * ty - qy * tx);
    return true;
  }
  // AI: end

  bool setZoneAim(uint8_t zone, float wr, float wf, float wu) {
    if (zone > 3) return false;
    if (!normalize3(wr, wf, wu)) return false;
    zoneAim[zone][0] = (int16_t)(wr * 1000.0f);
    zoneAim[zone][1] = (int16_t)(wf * 1000.0f);
    zoneAim[zone][2] = (int16_t)(wu * 1000.0f);
    return true;
  }

  // Match the peak linear-accel world vector against stored zone aim vectors.
  // Returns the zone index with the highest cosine similarity.
  uint8_t inferZoneFromAccelPeak(float axF, float ayF, float azF,
                                 float& worldRight, float& worldForward, float& worldUp) const {
    if (!autoZone) return activeZone;
    rotateToWorldByQuat(axF, ayF, azF, worldRight, worldForward, worldUp);
    float qx = worldRight, qy = worldForward, qz = worldUp;
    if (!normalize3(qx, qy, qz)) return activeZone;
    uint8_t bestZone = 0;
    float bestScore = -2.0f;
    for (uint8_t z = 0; z < 4; z++) {
      float zx = zoneAim[z][0] / 1000.0f;
      float zy = zoneAim[z][1] / 1000.0f;
      float zz = zoneAim[z][2] / 1000.0f;
      if (!normalize3(zx, zy, zz)) continue;
      const float score = dot3(qx, qy, qz, zx, zy, zz);
      if (score > bestScore) { bestScore = score; bestZone = z; }
    }
    return bestZone;
  }

  // ── BNO055 interrupt helpers ─────────────────────────────────────────

  // Configure gyro any-motion interrupt via direct I2C writes to register Page 1.
  //
  // BNO055 interrupt architecture (NDOF fusion mode):
  //   - INT_MSK (Page 1, 0x0F): routes interrupt sources to the INT pin
  //   - INT_EN  (Page 1, 0x10): enables interrupt sources
  //   - GYR_INT_SETTING (Page 1, 0x17): filtered-data flag + AM axis enables
  //     bit 7 = AM_FILT (1=use filtered gyro), bits[2:0] = AM_Z/Y/X axis enable
  //   - GYR_AM_THRES (Page 1, 0x1E): any-motion threshold, 1 LSB ≈ 1 dps
  //   - GYR_AM_SET  (Page 1, 0x1F): slope samples [3:2] and awake-dur [1:0]
  //     (00 = 8 samples each, i.e. 80 ms at 100 Hz ODR)
  // Reference: BNO055 datasheet v1.4, section 4.6, Table 4-16
  bool configureBnoAnyMotionIrq() {
    auto bnoWrite8 = [this](uint8_t reg, uint8_t val) -> bool {
      Wire.beginTransmission(bnoI2cAddr);
      Wire.write(reg);
      Wire.write(val);
      return Wire.endTransmission() == 0;
    };
    auto bnoRead8 = [this](uint8_t reg) -> uint8_t {
      Wire.beginTransmission(bnoI2cAddr);
      Wire.write(reg);
      Wire.endTransmission(false);
      Wire.requestFrom((int)bnoI2cAddr, 1, 1);
      return Wire.available() ? Wire.read() : 0xFF;
    };
    // Switch to register page 1
    if (!bnoWrite8(0x07, 0x01)) {
      DEBUG_PRINTLN(F("[Drumstick] BNO055 page1 switch failed"));
      return false;
    }
    // GYR_AM_THRES: ~10 dps — low enough to catch swing onset well before FSM
    bnoWrite8(0x1E, 10);
    // GYR_AM_SET: 8 slope samples, 8 awake-duration samples
    bnoWrite8(0x1F, 0x00);
    // GYR_INT_SETTING: AM_FILT=1 (filtered data), AM on X+Y+Z axes
    bnoWrite8(0x17, 0x87);
    // INT_MSK: route gyro any-motion (bit 2) to INT pin
    bnoWrite8(0x0F, 0x04);
    // INT_EN: enable gyro any-motion interrupt (bit 2)
    bnoWrite8(0x10, 0x04);
    // Return to page 0
    if (!bnoWrite8(0x07, 0x00)) return false;
    // Clear any pre-existing interrupt state
    (void)bnoRead8(0x37);  // INT_STA (page 0) — reading clears
    return true;
  }

  // Read INT_STA (page 0, reg 0x37) to release the BNO055 INT pin after a read.
  void clearBnoInterrupt() {
    if (!sensorOk || !irqEnabled) return;
    Wire.beginTransmission(bnoI2cAddr);
    Wire.write(0x37);
    Wire.endTransmission(false);
    Wire.requestFrom((int)bnoI2cAddr, 1, 1);
    if (Wire.available()) Wire.read();
  }

  // Convert peak angular velocity (deg/s) to MIDI velocity 1–127.
  // Maps gyroOnsetDps → 1, 6× onset → 127.
  uint8_t calcVelocity(float omega) const {
    const float minO = (float)gyroOnsetDps;
    const float maxO = minO * 6.0f;
    if (omega <= minO) return 1;
    float t = (omega - minO) / (maxO - minO);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float v;
    switch (velocityCurve) {
      case 1: v = log10f(1.0f + 9.0f * t); break;
      case 2: v = t * t;                   break;
      default: v = t;                       break;
    }
    return (uint8_t)(1u + (uint8_t)(v * 126.0f));
  }

  // ── Battery ───────────────────────────────────────────────────────────

  void updateBattery() {
    if (!_battPinOk) return;
    // ADC reading → Vin through 1:2 divider.
    const float Vmeas = analogRead(battPin) * DS_ADC_VREF / DS_ADC_COUNTS * DS_BATT_DIV;
    if (!_battInit) {
      _battV = Vmeas;
      _battInit = true;
    } else {
      _battV = _battV * DS_BATT_LPF + (1.0f - DS_BATT_LPF) * Vmeas;
    }
    const float pct = (_battV - DS_BATT_MIN_V) / (DS_BATT_MAX_V - DS_BATT_MIN_V) * 100.0f;
    _battPct = (uint8_t)constrain((int)pct, 0, 100);
  }

  // ── WebSocket helpers ─────────────────────────────────────────────────

  void sendWsInfo(const char* msg) {
    if (_drumWs.count() == 0 || !msg) return;
    char buf[192];
    int len = snprintf(buf, sizeof(buf), "{\"type\":\"info\",\"msg\":\"%s\"}", msg);
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    _drumWs.textAll(buf);
  }

  void sendWsConfig(AsyncWebSocketClient* client = nullptr) {
    char buf[320];
    int len = snprintf(buf, sizeof(buf),
      "{\"type\":\"cfg\","
      "\"th\":%u,\"ath\":%.2f,\"gth\":%u,\"cd\":%u,"
      "\"az\":%u,\"zone\":%u,\"learn\":%d,"
      "\"z0x\":%d,\"z0y\":%d,\"z0z\":%d,"
      "\"z1x\":%d,\"z1y\":%d,\"z1z\":%d,"
      "\"z2x\":%d,\"z2y\":%d,\"z2z\":%d,"
      "\"z3x\":%d,\"z3y\":%d,\"z3z\":%d}",
      (unsigned)threshold,
      accelPeakThreshold(),
      (unsigned)gyroOnsetDps,
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
    char buf[640];
    int len = snprintf(buf, sizeof(buf),
      "{\"type\":\"tele\","
      "\"ok\":%u,\"st\":\"%s\",\"ori\":%u,"
      "\"wr\":%.2f,\"wf\":%.2f,\"wu\":%.2f,"
      "\"pwr\":%.2f,\"pwf\":%.2f,\"pwu\":%.2f,"
      "\"la\":%.3f,\"pla\":%.3f,\"om\":%.1f,\"zone\":%u,"
      "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
      "\"lax\":%.2f,\"lay\":%.2f,\"laz\":%.2f,"
      "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
      "\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,"
      "\"cal\":%u,\"cg\":%u,\"ca\":%u,\"cm\":%u,"
      "\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,"
      "\"eh\":%.1f,\"er\":%.1f,\"ep\":%.1f,"
      "\"temp\":%d,\"batt\":%.2f,\"battp\":%u}",
      (unsigned)(sensorOk ? 1 : 0),
      swingStateName(),
      (unsigned)_calSys,
      currWorldRight, currWorldForward, currWorldUp,
      peakWorldRight, peakWorldForward, peakWorldUp,
      currLinAccel, peakLinAccel, currOmega,
      (unsigned)(autoZone ? detectedZone : activeZone),
      _lastAx, _lastAy, _lastAz,
      _lastLax, _lastLay, _lastLaz,
      _lastGx, _lastGy, _lastGz,
      _lastMx, _lastMy, _lastMz,
      (unsigned)_calSys, (unsigned)_calGyro, (unsigned)_calAccel, (unsigned)_calMag,
      (float)_quat.w(), (float)_quat.x(), (float)_quat.y(), (float)_quat.z(),
      _eulerH, _eulerR, _eulerP,
      (int)_bnoTemp, _battV, (unsigned)_battPct
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
        sendWsInfo("Point at target area and play one stroke to teach that zone.");
      }
    } else if (!strcmp(cmd, "selftest")) {
      // Read BNO055 system status and self-test results directly.
      uint8_t sys_status = 0, self_test = 0, sys_error = 0;
      _bno->getSystemStatus(&sys_status, &self_test, &sys_error);
      // self_test bits: 0=accel, 1=mag, 2=gyro, 3=MCU
      char msg[160];
      snprintf(msg, sizeof(msg),
        "SelfTest: status=0x%02X err=0x%02X | accel:%s mag:%s gyro:%s mcu:%s",
        (unsigned)sys_status, (unsigned)sys_error,
        (self_test & 0x01) ? "PASS" : "FAIL",
        (self_test & 0x02) ? "PASS" : "FAIL",
        (self_test & 0x04) ? "PASS" : "FAIL",
        (self_test & 0x08) ? "PASS" : "FAIL"
      );
      sendWsInfo(msg);
    } else if (!strcmp(cmd, "set")) {
      if (doc.containsKey("threshold")) {
        uint16_t v = (uint16_t)max(50, (int)(doc["threshold"] | (int)threshold));
        if (threshold != v) { threshold = v; changed = true; }
      }
      if (doc.containsKey("gyroOnset")) {
        uint16_t v = (uint16_t)max(10, (int)(doc["gyroOnset"] | (int)gyroOnsetDps));
        if (gyroOnsetDps != v) { gyroOnsetDps = v; changed = true; }
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

  void broadcastWsHit(uint8_t zone, uint8_t velocity) {
    if (_drumWs.count() == 0) return;
    char buf[40];
    snprintf(buf, sizeof(buf), "{\"type\":\"hit\",\"z\":%u,\"vel\":%u}",
             (unsigned)zone, (unsigned)velocity);
    _drumWs.textAll(buf);
  }

  void sendHitPacket(uint8_t zone, uint8_t velocity, uint8_t confidence) {
    if (!WLED_CONNECTED || udpHost[0] == '\0') return;
    char buf[DS_UDP_BUF];
    int len = snprintf(buf, sizeof(buf),
      "{\"v\":1,\"seq\":%lu,\"t\":%lu,\"z\":%u,\"vel\":%u,\"conf\":%u,\"s\":\"BNO\"}",
      (unsigned long)seqNum++, (unsigned long)millis(),
      (unsigned)zone, (unsigned)velocity, (unsigned)confidence
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
      "{\"v\":1,\"seq\":%lu,\"t\":%lu,\"hb\":1,\"imu\":%u,\"cal\":%u,\"sda\":%d,\"scl\":%d}",
      (unsigned long)seqNum++, (unsigned long)millis(),
      (unsigned)(sensorOk ? 1 : 0), (unsigned)_calSys,
      (int)i2c_sda, (int)i2c_scl
    );
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    udp.beginPacket(udpHost, udpPort);
    udp.write((const uint8_t*)buf, (size_t)len);
    udp.endPacket();
  }

  // Raw stream: scaled integers for PC-side processing.
  // accel/linear accel: m/s² × 100 → int; gyro: deg/s × 10 → int; mag: µT × 10 → int
  void sendRawPacket() {
    if (!WLED_CONNECTED || udpHost[0] == '\0') return;
    char buf[DS_RAW_UDP_BUF];
    int len = snprintf(buf, sizeof(buf),
      "{\"v\":2,\"seq\":%lu,\"t\":%lu,"
      "\"ax\":%d,\"ay\":%d,\"az\":%d,"
      "\"gx\":%d,\"gy\":%d,\"gz\":%d,"
      "\"mx\":%d,\"my\":%d,\"mz\":%d}",
      (unsigned long)seqNum++, (unsigned long)millis(),
      (int)(_lastLax * 100.0f), (int)(_lastLay * 100.0f), (int)(_lastLaz * 100.0f),
      (int)(_lastGx  *  10.0f), (int)(_lastGy  *  10.0f), (int)(_lastGz  *  10.0f),
      (int)(_lastMx  *  10.0f), (int)(_lastMy  *  10.0f), (int)(_lastMz  *  10.0f)
    );
    if (len <= 0 || len >= (int)sizeof(buf)) return;
    udp.beginPacket(udpHost, udpPort);
    udp.write((const uint8_t*)buf, (size_t)len);
    udp.endPacket();
  }

public:

  // ── WLED lifecycle hooks ──────────────────────────────────────────────

  void setup() override {
    DEBUG_PRINTF("[Drumstick] I2C SDA=%d SCL=%d\n", (int)i2c_sda, (int)i2c_scl);

    // ── WebSocket + HTTP route ────────────────────────────────────────
    _drumWs.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient* c,
                           AwsEventType t, void* arg, uint8_t* data, size_t len) {
      if (t == WS_EVT_CONNECT) {
        DEBUG_PRINTF("[Drumstick] WS client #%u connected\n", c->id());
        sendWsConfig(c);
      } else if (t == WS_EVT_DISCONNECT) {
        DEBUG_PRINTF("[Drumstick] WS client #%u disconnected\n", c->id());
      } else if (t == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info && info->final && info->index == 0 && info->len == len
            && info->opcode == WS_TEXT) {
          handleWsCommand(c, data, len);
        }
      }
    });
    server.addHandler(&_drumWs);

    server.on("/drum", HTTP_GET, [](AsyncWebServerRequest *req) {
      req->send_P(200, "text/html", DRUM_PAGE);
    });

  

    // ── User button ───────────────────────────────────────────────────
    if (btnPin >= 0) {
      _btnPinOk = PinManager::allocatePin(btnPin, false, PinOwner::UM_Drumstick);
      if (_btnPinOk) {
        pinMode(btnPin, INPUT_PULLDOWN);
        DEBUG_PRINTF("[Drumstick] Button pin %d configured INPUT_PULLDOWN (active-HIGH)\n", (int)btnPin);
      }
    }

    // ── Battery ADC ───────────────────────────────────────────────────
    if (battPin >= 0) {
      _battPinOk = PinManager::allocatePin(battPin, false, PinOwner::UM_Drumstick);
      if (_battPinOk) {
        pinMode(battPin, INPUT);
        DEBUG_PRINTF("[Drumstick] Battery ADC on pin %d\n", (int)battPin);
      }
    }

    // ── BNO055 interrupt pin (allocated; polling mode used) ───────────
    if (irqPin >= 0) {
      irqBound = PinManager::allocatePin(irqPin, false, PinOwner::UM_Drumstick);
      if (irqBound) {
        pinMode(irqPin, INPUT);
        DEBUG_PRINTF("[Drumstick] BNO055 INT pin %d allocated (polling mode)\n", (int)irqPin);
      } else {
        DEBUG_PRINTF("[Drumstick] BNO055 INT pin %d in use\n", (int)irqPin);
      }
    }

    // ── BNO055 initialisation ─────────────────────────────────────────
    // Try configured address first, then auto-probe the other address.
    _bno = new Adafruit_BNO055(55, bnoI2cAddr, &Wire);
    sensorOk = _bno->begin(OPERATION_MODE_NDOF);
    if (!sensorOk) {
      // Auto-probe: try the other address
      const uint8_t otherAddr = (bnoI2cAddr == 0x28) ? 0x29 : 0x28;
      DEBUG_PRINTF("[Drumstick] BNO055 not found at 0x%02X, trying 0x%02X\n",
                   (unsigned)bnoI2cAddr, (unsigned)otherAddr);
      delete _bno;
      _bno = new Adafruit_BNO055(55, otherAddr, &Wire);
      sensorOk = _bno->begin(OPERATION_MODE_NDOF);
      if (sensorOk) {
        bnoI2cAddr = otherAddr; // persist the working address
        configNeedsWrite = true;
      }
    }
    if (sensorOk) {
      _bno->setExtCrystalUse(true);
      DEBUG_PRINTF("[Drumstick] BNO055 OK at 0x%02X, NDOF mode, external crystal\n",
                   (unsigned)bnoI2cAddr);
      // Configure gyro any-motion interrupt and attach ISR if pin is available.
      // BNO055 INT pin goes HIGH when angular rate on any axis exceeds ~10 dps
      // for 8 consecutive ODR samples. This provides low-latency swing onset
      // detection as a complement to the gyro FSM (which checks actual values).
      if (irqBound) {
        if (configureBnoAnyMotionIrq()) {
          const int irqNum = digitalPinToInterrupt(irqPin);
          if (irqNum >= 0) {
            _irqFired = false;
            attachInterrupt(irqNum, onBnoInterrupt, RISING);
            irqEnabled = true;
            DEBUG_PRINTF("[Drumstick] BNO055 any-motion IRQ attached on pin %d\n", (int)irqPin);
          } else {
            DEBUG_PRINTF("[Drumstick] Pin %d has no interrupt, using polling\n", (int)irqPin);
          }
        } else {
          DEBUG_PRINTLN(F("[Drumstick] BNO055 any-motion IRQ config failed, using polling"));
        }
      }
    } else {
      DEBUG_PRINTLN(F("[Drumstick] BNO055 NOT FOUND (check power/wiring/I2C addr)"));
    }

    initDone = true;
  }

  void loop() override {
    if (!enabled || !initDone) return;

    _drumWs.cleanupClients();

    const unsigned long now = millis();

    // ── Heartbeat ─────────────────────────────────────────────────────
    if (WLED_CONNECTED && (now - lastHbeat >= DS_HEARTBEAT_MS)) {
      sendHeartbeat();
      lastHbeat = now;
    }

    //── Button FSM ────────────────────────────────────────────────────
    if (_btnPinOk) {
      const bool pressed = (digitalRead(btnPin) == HIGH);
      if (pressed) {
        if (!_btnWasPressed) {
          _btnPressedTs  = now;
          _btnWasPressed = true;
          _longFired     = false;
        }
      } else {
        if (_btnWasPressed && !_longFired) {
          // Short press: cycle LED effect
          ++effectCurrent %= strip.getModeCount();
          colorUpdated(CALL_MODE_BUTTON);
          DEBUG_PRINTLN(F("[Drumstick] Short press: effect cycled"));
        }
        _btnWasPressed = false;
        _longFired     = false;
      }
    }

    // ── Poll sensor ───────────────────────────────────────────────────
    // IRQ-gated: read immediately on any-motion event, fall back to polling
    // at DS_POLL_MS so telemetry and calibration reads continue even at rest.
    if (!_irqFired && (now - lastPoll < DS_POLL_MS)) return;
    _irqFired = false;
    lastPoll = now;

    // Battery ADC update on every poll cycle (LPF handles noise)
    updateBattery();

    if (!sensorOk) return;

    // Clear BNO055 INT_STA register (page 0, 0x37) so the INT pin de-asserts.
    // The register clears on read, so polling keeps the sensor line settled.
    clearBnoInterrupt();

    // Read linear acceleration (gravity removed by BNO055 fusion), gyro, quat, cal
    const imu::Vector<3> linVec = _bno->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    const imu::Vector<3> gyroVec = _bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    _quat = _bno->getQuat();
    _bno->getCalibration(&_calSys, &_calGyro, &_calAccel, &_calMag);

    _lastLax = (float)linVec.x();
    _lastLay = (float)linVec.y();
    _lastLaz = (float)linVec.z();
    _lastGx  = (float)gyroVec.x();
    _lastGy  = (float)gyroVec.y();
    _lastGz  = (float)gyroVec.z();

    // Slower reads: Euler angles, temperature, raw accel, magnetometer
    if (now - lastSlowRead >= DS_WS_TELEM_MS) {
      lastSlowRead = now;
      const imu::Vector<3> euler = _bno->getVector(Adafruit_BNO055::VECTOR_EULER);
      const imu::Vector<3> accel = _bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      const imu::Vector<3> mag   = _bno->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      _eulerH = (float)euler.x();  // heading (yaw)
      _eulerR = (float)euler.z();  // roll
      _eulerP = (float)euler.y();  // pitch
      _bnoTemp = _bno->getTemp();
      _lastAx = (float)accel.x(); _lastAy = (float)accel.y(); _lastAz = (float)accel.z();
      _lastMx = (float)mag.x();   _lastMy = (float)mag.y();   _lastMz = (float)mag.z();
    }

    if (rawStream) {
      sendRawPacket();
      return;
    }

    // ── Swing FSM ─────────────────────────────────────────────────────
    const float omegaRaw = vecLen3(_lastGx, _lastGy, _lastGz);
    omegaFilt = omegaFilt * DS_OMEGA_LPF + (1.0f - DS_OMEGA_LPF) * omegaRaw;
    const float omega = omegaFilt;

    linFilt[0] = linFilt[0] * DS_LINACC_LPF + (1.0f - DS_LINACC_LPF) * _lastLax;
    linFilt[1] = linFilt[1] * DS_LINACC_LPF + (1.0f - DS_LINACC_LPF) * _lastLay;
    linFilt[2] = linFilt[2] * DS_LINACC_LPF + (1.0f - DS_LINACC_LPF) * _lastLaz;
    const float linAccel = vecLen3(linFilt[0], linFilt[1], linFilt[2]);

    currLinAccel = linAccel;
    currOmega    = omega;
    rotateToWorldByQuat(linFilt[0], linFilt[1], linFilt[2],
                        currWorldRight, currWorldForward, currWorldUp);

    // Running peak for calibration display (all-time, in deg/s)
    const uint16_t omegaU16 = (uint16_t)min((float)65535.0f, omega);
    if (omegaU16 > peakSeen) peakSeen = omegaU16;

    switch (swingState) {
      case SwingState::IDLE:
        if (omega >= (float)gyroOnsetDps) {
          swingState      = SwingState::SWING;
          peakOmega       = omega;
          peakLinAccel    = linAccel;
          peakLinAx = linFilt[0]; peakLinAy = linFilt[1]; peakLinAz = linFilt[2];
          peakWorldRight  = currWorldRight;
          peakWorldForward = currWorldForward;
          peakWorldUp     = currWorldUp;
          swingSamples    = 1;
          lastOnsetTs     = now;
        }
        break;

      case SwingState::SWING:
        swingSamples++;
        if (omega > peakOmega) peakOmega = omega;
        if (linAccel > peakLinAccel) {
          peakLinAccel = linAccel;
          peakLinAx = linFilt[0]; peakLinAy = linFilt[1]; peakLinAz = linFilt[2];
          peakWorldRight   = currWorldRight;
          peakWorldForward = currWorldForward;
          peakWorldUp      = currWorldUp;
        }
        // Wrist-stop: |ω| decayed to DS_GYRO_DECAY of peak; require ≥3 samples to
        // reject single-sample noise spikes (3 × 10 ms = 30 ms min stroke).
        if (swingSamples >= 3 && omega < peakOmega * DS_GYRO_DECAY) {
          swingState = SwingState::DECAY;
          break;
        }
        if (swingSamples > DS_SWING_MAX_SAMPLES) {
          swingState = SwingState::IDLE; // abort runaway stroke
        }
        break;

      case SwingState::DECAY:
        swingState = SwingState::IDLE;
        {
          if (lastHitTs > 0 && (now - lastHitTs) < (unsigned long)cooldown_ms) break;
          if (peakLinAccel < accelPeakThreshold()) break;

          // Zone learning: capture aim vector from this stroke
          if (learnZone >= 0 && learnZone < 4) {
            if (setZoneAim((uint8_t)learnZone,
                           peakWorldRight, peakWorldForward, peakWorldUp)) {
              configNeedsWrite = true;
            }
            detectedZone = (uint8_t)learnZone;
            char msg[64];
            snprintf(msg, sizeof(msg), "Learned zone %u from last stroke.", (unsigned)learnZone);
            sendWsInfo(msg);
            learnZone = -1;
            sendWsConfig();
          }

          detectedZone = inferZoneFromAccelPeak(peakLinAx, peakLinAy, peakLinAz,
                                                peakWorldRight, peakWorldForward, peakWorldUp);
          const uint8_t zone = autoZone ? detectedZone : activeZone;
          const uint8_t vel  = calcVelocity(peakOmega);

          // Confidence: 0 = just above floor, 100 = 4× above floor
          const float accelFloor = accelPeakThreshold();
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

    // ── Telemetry ─────────────────────────────────────────────────────
    if (now - lastWsTelemetry >= DS_WS_TELEM_MS) {
      lastWsTelemetry = now;
      sendWsTelemetry();
    }
  }

  // ── /json/info (read-only runtime data) ──────────────────────────────

  void addToJsonInfo(JsonObject& root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    JsonArray sArr = user.createNestedArray(F("Drumstick sensor"));
    sArr.add(sensorOk ? F("BNO055 connected") : F("BNO055 not found"));

    JsonArray calArr = user.createNestedArray(F("Drumstick calibration"));
    calArr.add(_calSys); calArr.add(F("sys / "));
    calArr.add(_calGyro); calArr.add(F("gyr / "));
    calArr.add(_calAccel); calArr.add(F("acc / "));
    calArr.add(_calMag); calArr.add(F("mag"));

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

    JsonArray wArr = user.createNestedArray(F("World vector (ENU)"));
    wArr.add(currWorldRight); wArr.add(currWorldForward); wArr.add(currWorldUp);

    JsonArray iArr = user.createNestedArray(F("Swing state"));
    iArr.add(swingStateName());
    iArr.add(F(" peak-omega="));
    iArr.add((int)peakSeen);
    iArr.add(F(" deg/s"));

    JsonArray bArr = user.createNestedArray(F("Battery"));
    bArr.add(_battV); bArr.add(F("V  "));
    bArr.add(_battPct); bArr.add(F("%"));

    JsonArray eArr = user.createNestedArray(F("Euler H/R/P"));
    eArr.add(_eulerH); eArr.add(_eulerR); eArr.add(_eulerP);

    JsonArray tArr = user.createNestedArray(F("BNO055 temp"));
    tArr.add((int)_bnoTemp); tArr.add(F(" C"));
  }

  // ── Config persistence ────────────────────────────────────────────────

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = enabled;
    top["threshold"]     = threshold;
    top["cooldown"]      = cooldown_ms;
    top["bnoAddr"]       = bnoI2cAddr;
    top["irqPin"]        = irqPin;
    top["btnPin"]        = btnPin;
    top["battPin"]       = battPin;
    top["zone"]          = activeZone;
    top["autoZone"]      = autoZone;
    top["velCurve"]      = velocityCurve;
    top["rawStream"]     = rawStream;
    top["gyroOnset"]     = gyroOnsetDps;
    top["z0x"]           = zoneAim[0][0]; top["z0y"] = zoneAim[0][1]; top["z0z"] = zoneAim[0][2];
    top["z1x"]           = zoneAim[1][0]; top["z1y"] = zoneAim[1][1]; top["z1z"] = zoneAim[1][2];
    top["z2x"]           = zoneAim[2][0]; top["z2y"] = zoneAim[2][1]; top["z2z"] = zoneAim[2][2];
    top["z3x"]           = zoneAim[3][0]; top["z3y"] = zoneAim[3][1]; top["z3z"] = zoneAim[3][2];
    top["udpHost"]       = udpHost;
    top["udpPort"]       = udpPort;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[FPSTR(_name)];
    bool ok = !top.isNull();
    ok &= getJsonValue(top[FPSTR(_enabled)], enabled,      false);
    ok &= getJsonValue(top["threshold"],     threshold,     (uint16_t)500);
    ok &= getJsonValue(top["cooldown"],      cooldown_ms,   (uint16_t)150);
    ok &= getJsonValue(top["bnoAddr"],       bnoI2cAddr,    (uint8_t)0x28);
    // Sanitise: only accept 0x28 or 0x29
    if (bnoI2cAddr != 0x28 && bnoI2cAddr != 0x29) bnoI2cAddr = 0x28;
    ok &= getJsonValue(top["irqPin"],        irqPin,        (int8_t)3);
    ok &= getJsonValue(top["btnPin"],        btnPin,        (int8_t)0);
    ok &= getJsonValue(top["battPin"],       battPin,       (int8_t)1);
    ok &= getJsonValue(top["zone"],          activeZone,    (uint8_t)0);
    ok &= getJsonValue(top["autoZone"],      autoZone,      true);
    ok &= getJsonValue(top["velCurve"],      velocityCurve, (uint8_t)0);
    ok &= getJsonValue(top["rawStream"],     rawStream,     false);
    ok &= getJsonValue(top["gyroOnset"],     gyroOnsetDps,  (uint16_t)75);
    ok &= getJsonValue(top["z0x"], zoneAim[0][0], (int16_t)-707);
    ok &= getJsonValue(top["z0y"], zoneAim[0][1], (int16_t) 707);
    ok &= getJsonValue(top["z0z"], zoneAim[0][2], (int16_t)   0);
    ok &= getJsonValue(top["z1x"], zoneAim[1][0], (int16_t)-707);
    ok &= getJsonValue(top["z1y"], zoneAim[1][1], (int16_t)-707);
    ok &= getJsonValue(top["z1z"], zoneAim[1][2], (int16_t)   0);
    ok &= getJsonValue(top["z2x"], zoneAim[2][0], (int16_t) 707);
    ok &= getJsonValue(top["z2y"], zoneAim[2][1], (int16_t) 707);
    ok &= getJsonValue(top["z2z"], zoneAim[2][2], (int16_t)   0);
    ok &= getJsonValue(top["z3x"], zoneAim[3][0], (int16_t) 707);
    ok &= getJsonValue(top["z3y"], zoneAim[3][1], (int16_t)-707);
    ok &= getJsonValue(top["z3z"], zoneAim[3][2], (int16_t)   0);
    ok &= getJsonValue(top["udpPort"], udpPort, (uint16_t)9000);
    if (top.containsKey("udpHost") && top["udpHost"].is<const char*>()) {
      strlcpy(udpHost, top["udpHost"] | "", sizeof(udpHost));
    }
    // Migration guard: old threshold values were raw LSBs (>10000); reset to default.
    if (threshold > 10000) threshold = 500;
    return ok;
  }

  // ── Settings UI helpers ───────────────────────────────────────────────

  void appendConfigData() override {
    // Zone mode
    oappend(F("dd=addDropdown('Drumstick','autoZone');"));
    oappend(F("addOption(dd,'Auto zone from world vector',1);"));
    oappend(F("addOption(dd,'Manual fixed zone',0);"));
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
    // Raw stream
    oappend(F("dd=addDropdown('Drumstick','rawStream');"));
    oappend(F("addOption(dd,'Processed hit events (v1 UDP)',0);"));
    oappend(F("addOption(dd,'Raw sensor stream (v2 UDP)',1);"));
    // Info hints
    oappend(F("addInfo('Drumstick:threshold',1,'accel peak floor stored as m/s2 x 100; default 500 = 5.0 m/s2');"));
    oappend(F("addInfo('Drumstick:gyroOnset',1,'angular velocity onset in deg/s; default 75; lower = more sensitive');"));
    oappend(F("addInfo('Drumstick:cooldown',1,'minimum ms between two consecutive hits');"));
    oappend(F("dd=addDropdown('Drumstick','bnoAddr');"));
    oappend(F("addOption(dd,'0x28 (ADR=GND)',40);"));   // 0x28 = 40 decimal
    oappend(F("addOption(dd,'0x29 (ADR=3V3)',41);"));   // 0x29 = 41 decimal
    oappend(F("addInfo('Drumstick:bnoAddr',1,'BNO055 I2C address; 0x28 when ADR pin is GND, 0x29 when ADR pin is 3V3. Wrong address is auto-probed on boot.');"));
    oappend(F("addInfo('Drumstick:irqPin',1,'BNO055 INT pin (GPIO3); gyro any-motion IRQ fires when angular rate exceeds ~10 dps, triggering immediate sensor read; -1 to disable and use polling only');"));
    oappend(F("addInfo('Drumstick:btnPin',1,'user button input pin (XIAO default D0, active-HIGH with pull-down); short=effect cycle');"));
    oappend(F("addInfo('Drumstick:battPin',1,'battery ADC pin; default D1 on XIAO ESP32-C6');"));
    oappend(F("addInfo('Drumstick:udpHost',1,'hit event receiver IP or hostname');"));
    oappend(F("addInfo('Drumstick:udpPort',1,'hit event receiver UDP port; default 9000');"));
    oappend(F("addInfo('Drumstick:autoZone',1,'auto matches peak linear-accel world vector to nearest taught zone aim (BNO055 ENU quaternion required, sys cal >= 1)');"));
    oappend(F("addInfo('Drumstick:zone',1,'manual zone for non-auto mode; use /drum for live BNO055 diagnostics and zone teaching');"));
    oappend(F("addInfo('Drumstick:rawStream',1,'raw mode sends linear-accel+gyro+mag scaled integers at 100 Hz; PC does all detection');"));
  }

  uint16_t getId() override { return USERMOD_ID_DRUMSTICK; }
};

// ── PROGMEM string storage ─────────────────────────────────────────────────

const char DrumstickUsermod::_name[]    PROGMEM = "Drumstick";
const char DrumstickUsermod::_enabled[] PROGMEM = "enabled";
volatile bool DrumstickUsermod::_irqFired = false;

// ── Static instance + auto-registration ───────────────────────────────────

static DrumstickUsermod drumstick_usermod;
REGISTER_USERMOD(drumstick_usermod);
