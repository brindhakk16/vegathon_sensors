'use strict';
// =============================================================
//  VEGA Smart Sensing Platform — Unified Multi-Sensor Logic
//  Page 0: Landing | Page 1: Thermal | Page 2: Air Quality
// =============================================================

// --- Page Navigation ---
let currentPage = 0;

function navigateTo(idx) {
  document.querySelectorAll('.page').forEach(p => { p.classList.add('hidden'); p.classList.remove('active'); });
  const target = document.getElementById('page' + idx);
  if (target) { target.classList.remove('hidden'); target.classList.add('active'); }
  currentPage = idx;

  const nav = document.getElementById('topNav');
  const tabThermal = document.getElementById('tabThermal');
  const tabAir = document.getElementById('tabAir');

  if (idx === 0) {
    nav?.classList.remove('visible');
  } else {
    nav?.classList.add('visible');
    if (tabThermal) tabThermal.classList.toggle('active', idx === 1);
    if (tabAir) tabAir.classList.toggle('active', idx === 2);
  }

  if (idx === 2 && !mq135Running) {
    startMQ135();
  }
}

navigateTo(0);

// =============================================================
//  PARTICLE CANVAS (Landing)
// =============================================================
(function () {
  const cv = document.getElementById('particleCanvas');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const pts = [];
  function resize() { cv.width = innerWidth; cv.height = innerHeight; }
  resize();
  window.addEventListener('resize', resize);
  for (let i = 0; i < 65; i++) {
    pts.push({
      x: Math.random() * innerWidth, y: Math.random() * innerHeight,
      vx: (Math.random() - .5) * .35, vy: -(Math.random() * .5 + .15),
      r: Math.random() * 1.6 + .4,
      c: Math.random() > .5 ? 'rgba(59,130,246,' : 'rgba(6,182,212,',
      a: Math.random() * .45 + .1
    });
  }
  function tick() {
    if (currentPage !== 0) { requestAnimationFrame(tick); return; }
    ctx.clearRect(0, 0, cv.width, cv.height);
    pts.forEach(p => {
      p.x += p.vx; p.y += p.vy;
      if (p.y < -10) { p.y = cv.height + 5; p.x = Math.random() * cv.width; }
      ctx.beginPath(); ctx.arc(p.x, p.y, p.r, 0, Math.PI * 2);
      ctx.fillStyle = p.c + p.a + ')'; ctx.fill();
    });
    requestAnimationFrame(tick);
  }
  tick();
})();

// =============================================================
//  SERIAL CONNECTION & MULTI-SENSOR STATE
// =============================================================
let serialPort = null, serialReader = null, keepReading = true;
let isPaused = false, frameCount = 0, lastFpsTime = Date.now();
let historyData = [], historyChart = null;
let scaleMode = 'auto', manualMin = 20, manualMax = 40;
let selectedPalette = 'thermal';
const SAMPLE_PIXELS = new Float32Array([
  21.3, 20.4, 20.2, 21.1, 21.5, 21.0, 20.8, 20.9,
  20.1, 22.8, 20.3, 21.2, 21.4, 23.5, 21.1, 20.9,
  20.2, 20.4, 21.8, 21.6, 25.4, 31.8, 21.5, 21.1,
  25.1, 21.0, 20.2, 21.5, 26.2, 31.0, 23.4, 21.2,
  23.2, 22.8, 20.1, 21.3, 26.8, 28.5, 23.1, 21.4,
  23.0, 25.5, 25.4, 21.4, 26.5, 26.8, 25.2, 23.0,
  21.2, 23.4, 26.0, 25.8, 26.1, 26.2, 24.8, 22.8,
  21.0, 21.3, 26.2, 26.5, 26.4, 26.2, 25.4, 21.1
]);
let currentPixels = new Float32Array(SAMPLE_PIXELS);
const MAX_HIST = 40;

// DOM Elements
const $ = id => document.getElementById(id);
const btnConnect = $('btnConnect'), btnConnectAir = $('btnConnectAir');
const baudRate = $('baudRate'), baudRateAir = $('baudRateAir');
const statusBadge = $('statusBadge'), statusBadgeAir = $('statusBadgeAir');

const valMax = $('valMax'), valMin = $('valMin'), valAvg = $('valAvg'), valTherm = $('valTherm'), valFPS = $('valFPS');
const btnPause = $('btnPause'), btnExport = $('btnExport');
const paletteSelect = $('paletteSelect'), scaleModeEl = $('scaleMode');
const smoothing = $('smoothingFactor'), manualGroup = $('manualRangeGroup');
const rMin = $('rangeMin'), rMax = $('rangeMax');
const lblMin = $('lblMinTemp'), lblMax = $('lblMaxTemp');
const chkVals = $('chkShowValues');
const rawCv = $('rawCanvas'), rawCtx = rawCv?.getContext('2d'), rawTip = $('rawTooltip');
const intCv = $('interpolatedCanvas'), intCtx = intCv?.getContext('2d'), intTip = $('interpolatedTooltip');

if (!('serial' in navigator)) {
  if (btnConnect) { btnConnect.disabled = true; btnConnect.title = 'Web Serial only works in Chrome/Edge on localhost.'; }
  if (btnConnectAir) { btnConnectAir.disabled = true; btnConnectAir.title = 'Web Serial only works in Chrome/Edge on localhost.'; }
}

// Palettes
const PALETTES = {
  thermal:  [{pos:0,color:[150,90,200]},{pos:.15,color:[90,110,220]},{pos:.3,color:[40,150,220]},{pos:.45,color:[0,200,200]},{pos:.6,color:[50,200,120]},{pos:.75,color:[200,210,100]},{pos:.88,color:[230,100,40]},{pos:1,color:[210,40,20]}],
  rainbow:  [{pos:0,color:[0,0,255]},{pos:.25,color:[0,255,255]},{pos:.5,color:[0,255,0]},{pos:.75,color:[255,255,0]},{pos:1,color:[255,0,0]}],
  ironbow:  [{pos:0,color:[10,0,30]},{pos:.15,color:[25,0,100]},{pos:.45,color:[160,0,110]},{pos:.75,color:[230,60,10]},{pos:.9,color:[250,170,0]},{pos:1,color:[255,255,200]}],
  jet:      [{pos:0,color:[0,0,143]},{pos:.15,color:[0,0,255]},{pos:.4,color:[0,255,255]},{pos:.6,color:[255,255,0]},{pos:.85,color:[255,0,0]},{pos:1,color:[128,0,0]}],
  grayscale:[{pos:0,color:[0,0,0]},{pos:1,color:[255,255,255]}],
  icefire:  [{pos:0,color:[0,0,80]},{pos:.25,color:[0,160,255]},{pos:.5,color:[240,240,255]},{pos:.7,color:[255,180,0]},{pos:.9,color:[255,40,0]},{pos:1,color:[100,0,0]}]
};

function palColor(v, mn, mx, pal) {
  let n = Math.max(0, Math.min(1, (v - mn) / (mx - mn)));
  const st = PALETTES[pal] || PALETTES.rainbow;
  let lo = st[0], hi = st[st.length-1];
  for (let i = 0; i < st.length-1; i++) { if (n >= st[i].pos && n <= st[i+1].pos) { lo = st[i]; hi = st[i+1]; break; } }
  const rng = hi.pos - lo.pos, f = rng === 0 ? 0 : (n - lo.pos) / rng;
  return `rgb(${Math.round(lo.color[0]+f*(hi.color[0]-lo.color[0]))},${Math.round(lo.color[1]+f*(hi.color[1]-lo.color[1]))},${Math.round(lo.color[2]+f*(hi.color[2]-lo.color[2]))})`;
}

function bilinear(src, sw, sh, dw, dh) {
  const dst = new Float32Array(dw*dh), xr = (sw-1)/(dw-1), yr = (sh-1)/(dh-1);
  for (let y = 0; y < dh; y++) {
    const gy = y*yr, y1 = Math.floor(gy), y2 = Math.min(y1+1,sh-1), dy = gy-y1;
    for (let x = 0; x < dw; x++) {
      const gx = x*xr, x1 = Math.floor(gx), x2 = Math.min(x1+1,sw-1), dx = gx-x1;
      dst[y*dw+x] = src[y1*sw+x1]*(1-dx)*(1-dy)+src[y1*sw+x2]*dx*(1-dy)+src[y2*sw+x1]*(1-dx)*dy+src[y2*sw+x2]*dx*dy;
    }
  }
  return dst;
}

function initHistoryChart() {
  const el = $('historyChart'); if (!el) return;
  Chart.defaults.color = '#8899bb'; Chart.defaults.font.family = 'Inter';
  historyChart = new Chart(el.getContext('2d'), {
    type:'line',
    data:{ labels:[], datasets:[
      {label:'Max',data:[],borderColor:'#ef4444',backgroundColor:'rgba(239,68,68,.05)',borderWidth:2,pointRadius:0,tension:.3,fill:true},
      {label:'Avg',data:[],borderColor:'#10b981',borderWidth:2,pointRadius:0,tension:.3},
      {label:'Min',data:[],borderColor:'#06b6d4',borderWidth:2,pointRadius:0,tension:.3}
    ]},
    options:{responsive:true,maintainAspectRatio:false,
      plugins:{legend:{display:true,position:'top',labels:{boxWidth:10,font:{size:10},padding:8}}},
      scales:{x:{display:false},y:{grid:{color:'rgba(255,255,255,.04)'},ticks:{font:{size:10}}}}}
  });
}

function updateHistChart(mn, mx, avg) {
  if (!historyChart) return;
  const {labels, datasets} = historyChart.data;
  labels.push(''); datasets[0].data.push(mx); datasets[1].data.push(avg); datasets[2].data.push(mn);
  if (labels.length > MAX_HIST) { labels.shift(); datasets.forEach(d => d.data.shift()); }
  historyChart.update('none');
}

function drawRaw(px, mn, mx) {
  if (!rawCtx) return;
  const cs = rawCv.width / 8;
  rawCtx.clearRect(0, 0, rawCv.width, rawCv.height);
  for (let r = 0; r < 8; r++) for (let c = 0; c < 8; c++) {
    const t = px[r*8+c];
    rawCtx.fillStyle = palColor(t, mn, mx, selectedPalette);
    rawCtx.fillRect(c*cs, r*cs, cs, cs);
    if (chkVals?.checked) {
      const norm = (t-mn)/(mx-mn);
      rawCtx.fillStyle = norm > .65 ? '#000' : '#fff';
      rawCtx.font = 'bold 9px Inter'; rawCtx.textAlign = 'center'; rawCtx.textBaseline = 'middle';
      rawCtx.fillText(t.toFixed(1), c*cs+cs/2, r*cs+cs/2);
    }
  }
}

function drawInterp(px, mn, mx) {
  if (!intCtx) return;
  const f = parseInt(smoothing?.value || 3), gs = 8*f, cs = intCv.width/gs;
  const ip = bilinear(px, 8, 8, gs, gs);
  intCtx.clearRect(0, 0, intCv.width, intCv.height);
  for (let r = 0; r < gs; r++) for (let c = 0; c < gs; c++) {
    intCtx.fillStyle = palColor(ip[r*gs+c], mn, mx, selectedPalette);
    intCtx.fillRect(c*cs, r*cs, cs, cs);
  }
}

function getRenderRange(d) {
  if (scaleMode === 'manual') return [manualMin, manualMax];
  if (d) return [d.min, d.max];
  return [parseFloat(valMin?.textContent) || 20.1, parseFloat(valMax?.textContent) || 31.8];
}

function processFrame(d) {
  if (isPaused) return;
  frameCount++;
  const now = Date.now();
  if (now - lastFpsTime >= 1000) {
    if (valFPS) valFPS.textContent = Math.round(frameCount*1000/(now-lastFpsTime));
    frameCount = 0; lastFpsTime = now;
  }
  currentPixels = new Float32Array(d.pixels);
  if (valMax) valMax.textContent = d.max.toFixed(1);
  if (valMin) valMin.textContent = d.min.toFixed(1);
  if (valAvg) valAvg.textContent = d.avg.toFixed(1);
  if (valTherm) valTherm.textContent = d.thermistor.toFixed(1);
  const [mn, mx] = getRenderRange(d);
  drawRaw(currentPixels, mn, mx);
  drawInterp(currentPixels, mn, mx);
  updateHistChart(d.min, d.max, d.avg);
  historyData.push({ timestamp: new Date().toISOString(), ...d });
  if (historyData.length > 2000) historyData.shift();
}

function mouseOnCanvas(e, isInterp) {
  const cv = isInterp ? intCv : rawCv, tip = isInterp ? intTip : rawTip;
  if (!cv || !tip) return;
  const rect = cv.getBoundingClientRect();
  const mx = (e.clientX-rect.left)*(cv.width/rect.width);
  const my = (e.clientY-rect.top)*(cv.height/rect.height);
  let html;
  if (!isInterp) {
    const cs = cv.width/8, cx = Math.min(7,Math.max(0,Math.floor(mx/cs))), cy = Math.min(7,Math.max(0,Math.floor(my/cs)));
    html = `Pixel (${cx},${cy})<br><strong>${currentPixels[cy*8+cx].toFixed(1)}°C</strong>`;
  } else {
    const f = parseInt(smoothing?.value||3), gs = 8*f, cs = cv.width/gs;
    const cx = Math.min(gs-1,Math.max(0,Math.floor(mx/cs))), cy = Math.min(gs-1,Math.max(0,Math.floor(my/cs)));
    const ip = bilinear(currentPixels,8,8,gs,gs);
    html = `(~${(cx/f).toFixed(1)},~${(cy/f).toFixed(1)})<br><strong>${ip[cy*gs+cx].toFixed(1)}°C</strong>`;
  }
  tip.innerHTML = html;
  tip.style.left = `${e.clientX-cv.getBoundingClientRect().left}px`;
  tip.style.top = `${e.clientY-cv.getBoundingClientRect().top}px`;
  tip.style.display = 'block';
}

function updateConnectionUI(connected, baud) {
  const buttons = [btnConnect, btnConnectAir];
  const badges = [statusBadge, statusBadgeAir];
  const selects = [baudRate, baudRateAir];

  buttons.forEach(btn => {
    if (!btn) return;
    if (connected) {
      btn.innerHTML = `<svg class="btn-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path stroke-linecap="round" stroke-linejoin="round" d="M18.364 18.364A9 9 0 005.636 5.636m12.728 12.728A9 9 0 015.636 5.636m12.728 12.728L5.636 5.636"/></svg>Disconnect`;
      btn.classList.add('connected');
    } else {
      btn.innerHTML = `<svg class="btn-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path stroke-linecap="round" stroke-linejoin="round" d="M13 10V3L4 14h7v7l9-11h-7z"/></svg>Connect Board`;
      btn.classList.remove('connected');
    }
  });

  badges.forEach(badge => {
    if (!badge) return;
    const txt = badge.querySelector('.status-text');
    if (connected) {
      badge.classList.add('connected');
      if (txt) txt.textContent = `Connected (${baud} Baud) — Live Multi-Sensor Stream Active`;
    } else {
      badge.classList.remove('connected');
      if (txt) txt.textContent = 'Disconnected — Connect VEGA Aries V2 via USB';
    }
  });

  selects.forEach(sel => {
    if (sel) sel.disabled = connected;
  });

  const livePill = $('aqLivePill');
  if (livePill) {
    if (connected) {
      livePill.innerHTML = '<span class="live-dot" style="background:#10b981"></span> Live Hardware Data — VEGA AMG8833, MQ-135 & MQ-2';
    } else {
      livePill.innerHTML = '<span class="live-dot"></span> Waiting for Hardware Connection — Click Connect Board';
    }
  }
}

async function connectSerial() {
  try {
    const chosenBaud = parseInt(baudRate?.value || baudRateAir?.value || 115200);
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: chosenBaud });
    updateConnectionUI(true, chosenBaud);
    keepReading = true;
    readLoop();
  } catch(e) {
    alert('Could not open serial port. Ensure the VEGA board is plugged in via USB and not open in Arduino Serial Monitor.');
  }
}

async function disconnectSerial() {
  keepReading = false;
  try { await serialReader?.cancel(); } catch(e) {}
  try { await serialPort?.close(); } catch(e) {}
  serialPort = null;
  updateConnectionUI(false, 115200);
  if (valFPS) valFPS.textContent = '0';
}

async function readLoop() {
  const dec = new TextDecoder();
  let buf = '';
  
  let frameBuffer = {
    thermistor: 0.0,
    min: 0.0,
    max: 0.0,
    avg: 0.0,
    pixels: [],
    matrixLinesCount: 0,
    inMatrix: false
  };

  while (serialPort?.readable && keepReading) {
    try {
      serialReader = serialPort.readable.getReader();
      while (keepReading) {
        const {value, done} = await serialReader.read();
        if (done) break;
        buf += dec.decode(value, {stream: true});
        const lines = buf.split('\n');
        buf = lines.pop();

        for (const ln of lines) {
          const t = ln.trim();
          if (!t) continue;

          // 0. Hardware Synchronized Page Navigation (from Pin 2 trigger)
          if (t.startsWith('ActivePage:')) {
            const pIdx = parseInt(t.split(':')[1].trim());
            if (!isNaN(pIdx) && pIdx !== currentPage) {
              navigateTo(pIdx);
            }
            continue;
          }

          // 1. Detect Frame Start Header
          if (t.includes('--- AMG8833 8x8 Thermal Data ---')) {
            frameBuffer.thermistor = 0.0;
            frameBuffer.min = 0.0;
            frameBuffer.max = 0.0;
            frameBuffer.avg = 0.0;
            frameBuffer.pixels = [];
            frameBuffer.matrixLinesCount = 0;
            frameBuffer.inMatrix = false;
            continue;
          }

          // 2. Parse Internal Thermistor Temp
          if (t.includes('Internal Thermistor Temp:')) {
            const match = t.match(/Internal Thermistor Temp:\s*([-\d.]+)/);
            if (match) {
              frameBuffer.thermistor = parseFloat(match[1]);
            }
            continue;
          }

          // 3. Parse Min, Max, and Avg Temperatures
          if (t.includes('Min Temp:')) {
            const minMatch = t.match(/Min Temp:\s*([-\d.]+)/);
            const maxMatch = t.match(/Max Temp:\s*([-\d.]+)/);
            const avgMatch = t.match(/Avg Temp:\s*([-\d.]+)/);
            if (minMatch) frameBuffer.min = parseFloat(minMatch[1]);
            if (maxMatch) frameBuffer.max = parseFloat(maxMatch[1]);
            if (avgMatch) frameBuffer.avg = parseFloat(avgMatch[1]);
            continue;
          }

          // 4. Detect Numerical Matrix Start Header
          if (t.includes('Numerical Temperature Matrix')) {
            frameBuffer.inMatrix = true;
            frameBuffer.matrixLinesCount = 0;
            frameBuffer.pixels = [];
            continue;
          }

          // 5. Parse Multi-Gas Telemetry (MQ-135 & MQ-2)
          if (t.includes('CO2:') || t.includes('H2:') || t.includes('ADC:')) {
            const adcMatch = t.match(/ADC:\s*(\d+)/);
            const co2Match = t.match(/CO2:\s*([\d.]+)/);
            const nh3Match = t.match(/NH3:\s*([\d.]+)/);
            const h2Match  = t.match(/H2:\s*([\d.]+)/);
            const ch4Match = t.match(/CH4:\s*([\d.]+)/);
            const statusMatch = t.match(/AirStatus:\s*([^|]+)/);
            const alertMatch = t.match(/AirAlert:\s*(YES|NO)/i);

            const realADC = adcMatch ? parseInt(adcMatch[1]) : 150;
            const realCO2 = co2Match ? parseFloat(co2Match[1]) : 412.0;
            const realNH3 = nh3Match ? parseFloat(nh3Match[1]) : 0.8;
            const realH2  = h2Match ? parseFloat(h2Match[1]) : 0.5;
            const realCH4 = ch4Match ? parseFloat(ch4Match[1]) : 1.2;
            const realStatus = statusMatch ? statusMatch[1].trim() : 'NORMAL';
            const isAlert = alertMatch && alertMatch[1].toUpperCase() === 'YES';

            processMultiGasTelemetry(realCO2, realNH3, realH2, realCH4, realStatus, isAlert, realADC);
            continue;
          }

          // 6. Parse Matrix rows (formatted as "[ 24.0, 24.1, ... ]")
          if (frameBuffer.inMatrix && t.startsWith('[') && t.endsWith(']')) {
            const content = t.substring(1, t.length - 1);
            const parts = content.split(',').map(s => parseFloat(s.trim()));
            if (parts.length === 8) {
              frameBuffer.pixels.push(...parts);
              frameBuffer.matrixLinesCount++;

              if (frameBuffer.matrixLinesCount === 8) {
                frameBuffer.inMatrix = false;
                if (frameBuffer.pixels.length === 64) {
                  processFrame({
                    thermistor: frameBuffer.thermistor,
                    min: frameBuffer.min,
                    max: frameBuffer.max,
                    avg: frameBuffer.avg,
                    pixels: Array.from(frameBuffer.pixels)
                  });
                }
              }
            }
          }
        }
      }
    } catch(e) { console.error(e); break; }
    finally { serialReader?.releaseLock(); serialReader = null; }
  }
  if (keepReading) disconnectSerial();
}

// Event wiring
btnConnect?.addEventListener('click', () => serialPort ? disconnectSerial() : connectSerial());
btnConnectAir?.addEventListener('click', () => serialPort ? disconnectSerial() : connectSerial());

baudRate?.addEventListener('change', e => { if (baudRateAir) baudRateAir.value = e.target.value; });
baudRateAir?.addEventListener('change', e => { if (baudRate) baudRate.value = e.target.value; });

paletteSelect?.addEventListener('change', e => {
  selectedPalette = e.target.value;
  const [mn,mx] = getRenderRange();
  drawRaw(currentPixels,mn,mx); drawInterp(currentPixels,mn,mx);
});

scaleModeEl?.addEventListener('change', e => {
  scaleMode = e.target.value;
  manualGroup?.classList.toggle('hidden', scaleMode !== 'manual');
  const [mn,mx] = getRenderRange();
  drawRaw(currentPixels,mn,mx); drawInterp(currentPixels,mn,mx);
});

function onSlider() {
  let mn = parseFloat(rMin.value), mx = parseFloat(rMax.value);
  if (mn > mx) { if (document.activeElement===rMin) { rMax.value=mn; mx=mn; } else { rMin.value=mx; mn=mx; } }
  manualMin=mn; manualMax=mx;
  if (lblMin) lblMin.textContent=mn; if (lblMax) lblMax.textContent=mx;
  drawRaw(currentPixels,mn,mx); drawInterp(currentPixels,mn,mx);
}
rMin?.addEventListener('input', onSlider); rMax?.addEventListener('input', onSlider);

chkVals?.addEventListener('change', () => { const [mn,mx]=getRenderRange(); drawRaw(currentPixels,mn,mx); });

smoothing?.addEventListener('change', () => {
  const f = parseInt(smoothing.value), s = 8*f;
  const lbl = $('lblInterpolatedTitle'); if (lbl) lbl.textContent = `interpolated: ${s}x${s} color tiles`;
  const [mn,mx] = getRenderRange(); drawInterp(currentPixels,mn,mx);
});

btnPause?.addEventListener('click', () => {
  isPaused = !isPaused;
  btnPause.innerHTML = isPaused
    ? `<svg class="btn-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path stroke-linecap="round" stroke-linejoin="round" d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z"/><path stroke-linecap="round" stroke-linejoin="round" d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z"/></svg>Resume`
    : `<svg class="btn-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path stroke-linecap="round" stroke-linejoin="round" d="M10 9v6m4-6v6m7-3a9 9 0 11-18 0 9 9 0 0118 0z"/></svg>Pause`;
  btnPause.classList.toggle('btn-primary', isPaused);
});

btnExport?.addEventListener('click', () => {
  if (!historyData.length) { alert('No data yet. Connect the sensor first.'); return; }
  let csv = 'Timestamp,Thermistor,Min,Max,Avg,' + Array.from({length:64},(_,i)=>`P${i}`).join(',') + '\n';
  historyData.forEach(r => { csv += `${r.timestamp},${r.thermistor.toFixed(2)},${r.min.toFixed(2)},${r.max.toFixed(2)},${r.avg.toFixed(2)},${r.pixels.map(v=>v.toFixed(2)).join(',')}\n`; });
  const a = document.createElement('a');
  a.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv);
  a.download = `thermal_${Date.now()}.csv`;
  document.body.appendChild(a); a.click(); document.body.removeChild(a);
});

rawCv?.addEventListener('mousemove', e => mouseOnCanvas(e, false));
rawCv?.addEventListener('mouseleave', () => { if (rawTip) rawTip.style.display='none'; });
intCv?.addEventListener('mousemove', e => mouseOnCanvas(e, true));
intCv?.addEventListener('mouseleave', () => { if (intTip) intTip.style.display='none'; });

// =============================================================
//  PAGE 3 — MULTI-GAS & AIR QUALITY SYSTEM
// =============================================================
let mq135Running = false;
let aqHistChart = null;
const sparks = {};

const GAS = {
  CO2:     {v:412,  base:412,  max:2000, th:[800, 1500]},
  CO:      {v:2.1,  base:2.1,  max:50,   th:[10,  35]  },
  NH3:     {v:0.8,  base:0.8,  max:10,   th:[2,   7]   },
  H2:      {v:0.5,  base:0.5,  max:1000, th:[100, 300] },
  CH4:     {v:1.2,  base:1.2,  max:5000, th:[500, 1500]},
  Benzene: {v:.004, base:.004, max:.1,   th:[.03, .07] }
};
const MAX_SP = 40;
const clamp = (v,a,b) => Math.max(a,Math.min(b,v));

function calcAQI(co2,co,nh3,h2,ch4,benz) {
  return Math.round(Math.max(
    clamp((co2/2000)*300,0,300), clamp((co/50)*300,0,300),
    clamp((nh3/10)*300,0,300),  clamp((h2/1000)*300,0,300),
    clamp((ch4/5000)*300,0,300), clamp((benz/.1)*300,0,300)
  ));
}

function aqiInfo(aqi) {
  if (aqi<=50)  return {label:'Good',      bg:'rgba(16,185,129,.1)',  color:'#6ee7b7', border:'rgba(16,185,129,.25)'};
  if (aqi<=100) return {label:'Moderate',  bg:'rgba(245,158,11,.1)',  color:'#fcd34d', border:'rgba(245,158,11,.3)'};
  if (aqi<=150) return {label:'Sensitive', bg:'rgba(249,115,22,.1)',  color:'#fdba74', border:'rgba(249,115,22,.3)'};
  if (aqi<=200) return {label:'Unhealthy', bg:'rgba(239,68,68,.1)',   color:'#fca5a5', border:'rgba(239,68,68,.3)'};
  return              {label:'Very Bad',   bg:'rgba(139,92,246,.1)',  color:'#c4b5fd', border:'rgba(139,92,246,.3)'};
}

function gasStatus(g, v) {
  if (v < g.th[0]) return ['Normal', ''];
  if (v < g.th[1]) return ['Elevated','warn'];
  return ['High!','danger'];
}

function drawGauge(aqi) {
  const cv = $('aqiGaugeCanvas'); if (!cv) return;
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height, cx = W/2, cy = H-10, r = Math.min(W,H)*.78;
  ctx.clearRect(0,0,W,H);
  const MAX_AQI = 300, arc = Math.PI;
  // Zone arcs
  [[0,50,'#10b981'],[50,100,'#f59e0b'],[100,150,'#f97316'],[150,200,'#ef4444'],[200,300,'#8b5cf6']].forEach(([a,b,col]) => {
    ctx.beginPath();
    ctx.arc(cx, cy, r, Math.PI+(a/MAX_AQI)*arc, Math.PI+(b/MAX_AQI)*arc);
    ctx.lineWidth = 20; ctx.strokeStyle = col+'3a'; ctx.stroke();
  });
  // Active arc
  const info = aqiInfo(aqi), clamped = clamp(aqi,0,MAX_AQI);
  const g = ctx.createLinearGradient(cx-r,cy,cx+r,cy);
  g.addColorStop(0,'#3b82f6'); g.addColorStop(1,info.color);
  ctx.beginPath();
  ctx.arc(cx, cy, r, Math.PI, Math.PI+(clamped/MAX_AQI)*arc);
  ctx.lineWidth = 20; ctx.strokeStyle = g; ctx.lineCap = 'round'; ctx.stroke();
  // Needle
  const ang = Math.PI + (clamped/MAX_AQI)*arc;
  ctx.beginPath(); ctx.moveTo(cx,cy);
  ctx.lineTo(cx+(r-12)*Math.cos(ang), cy+(r-12)*Math.sin(ang));
  ctx.lineWidth = 3; ctx.strokeStyle = '#fff'; ctx.lineCap = 'round'; ctx.stroke();
  ctx.beginPath(); ctx.arc(cx,cy,7,0,Math.PI*2); ctx.fillStyle='#fff'; ctx.fill();
}

function initSparks() {
  const cfg = (col) => ({
    type:'line', data:{labels:[],datasets:[{data:[],borderColor:col,borderWidth:1.5,pointRadius:0,tension:.4,fill:true,backgroundColor:col+'20'}]},
    options:{responsive:true,maintainAspectRatio:false,animation:false,plugins:{legend:{display:false}},scales:{x:{display:false},y:{display:false}}}
  });
  [['CO2','#3b82f6'],['CO','#f97316'],['NH3','#10b981'],['H2','#6366f1'],['CH4','#f59e0b'],['Benzene','#8b5cf6']].forEach(([k,c]) => {
    const el = $('spark'+k); if (!el) return;
    sparks[k] = new Chart(el.getContext('2d'), cfg(c));
  });
}

function initAQHistChart() {
  const el = $('aqHistoryChart'); if (!el) return;
  aqHistChart = new Chart(el.getContext('2d'), {
    type:'line',
    data:{labels:[],datasets:[
      {label:'CO2/10',data:[],borderColor:'#3b82f6',borderWidth:1.5,pointRadius:0,tension:.4},
      {label:'H2',    data:[],borderColor:'#6366f1',borderWidth:1.5,pointRadius:0,tension:.4},
      {label:'CH4',   data:[],borderColor:'#f59e0b',borderWidth:1.5,pointRadius:0,tension:.4},
      {label:'NH3',   data:[],borderColor:'#10b981',borderWidth:1.5,pointRadius:0,tension:.4}
    ]},
    options:{responsive:true,maintainAspectRatio:false,
      plugins:{legend:{display:true,position:'top',labels:{boxWidth:9,font:{size:9},padding:6}}},
      scales:{x:{display:false},y:{grid:{color:'rgba(255,255,255,.04)'},ticks:{font:{size:9}}}}}
  });
}

function updateGasCard(key, val, gasObj) {
  const valEl = $('val'+key), barEl = $('bar'+key), stEl = $('st'+key);
  const [stTxt, stCls] = gasStatus(gasObj, val);
  const pct = clamp((val/gasObj.max)*100, 0, 100);
  if (valEl) {
    valEl.textContent = val < 0.1 ? val.toFixed(4) : val < 1 ? val.toFixed(3) : val < 10 ? val.toFixed(2) : val.toFixed(1);
  }
  if (barEl) barEl.style.width = pct + '%';
  if (stEl)  { stEl.textContent = stTxt; stEl.className = 'gas-status ' + stCls; }
  if (sparks[key]) {
    const ch = sparks[key];
    ch.data.labels.push(''); ch.data.datasets[0].data.push(val);
    if (ch.data.labels.length > MAX_SP) { ch.data.labels.shift(); ch.data.datasets[0].data.shift(); }
    ch.update('none');
  }
}

function processMultiGasTelemetry(co2, nh3, h2, ch4, statusText, isAlert, realADC) {
  if (!mq135Running) {
    startMQ135();
  }

  const co = Math.max(0.5, (co2 - 400) * 0.005 + 1.2);
  const benz = Math.max(0.001, nh3 * 0.005);

  updateGasCard('CO2', co2, GAS.CO2);
  updateGasCard('NH3', nh3, GAS.NH3);
  updateGasCard('H2', h2, GAS.H2);
  updateGasCard('CH4', ch4, GAS.CH4);
  updateGasCard('CO', co, GAS.CO);
  updateGasCard('Benzene', benz, GAS.Benzene);

  const aqi = calcAQI(co2, co, nh3, h2, ch4, benz);
  const info = aqiInfo(aqi);
  const aqiEl = $('aqiValue'), lblEl = $('aqiLabel'), badge = $('aqOverallBadge');
  if (aqiEl) aqiEl.textContent = aqi;
  if (lblEl) lblEl.textContent = info.label;
  if (badge) {
    badge.textContent = 'AQI: ' + info.label;
    badge.style.background = info.bg;
    badge.style.color = info.color;
    badge.style.borderColor = info.border;
  }
  drawGauge(aqi);

  const sT = $('infoSensorTemp'), adc = $('infoADC');
  if (sT && valTherm) sT.textContent = valTherm.textContent + '°C';
  if (adc && realADC !== undefined) adc.textContent = realADC;

  const alert = $('alertBanner'), alertTxt = $('alertText');
  if (alert && alertTxt) {
    if (isAlert || aqi > 150) {
      alertTxt.textContent = `Alert: Gas Hazard Detected! (${statusText})`;
      alert.classList.remove('hidden');
    } else {
      alert.classList.add('hidden');
    }
  }

  if (aqHistChart) {
    const { labels, datasets } = aqHistChart.data;
    labels.push('');
    datasets[0].data.push(co2 / 10);
    datasets[1].data.push(h2);
    datasets[2].data.push(ch4);
    datasets[3].data.push(nh3);
    if (labels.length > MAX_SP) {
      labels.shift();
      datasets.forEach(d => d.data.shift());
    }
    aqHistChart.update('none');
  }
}

function startMQ135() {
  mq135Running = true;
  initSparks();
  initAQHistChart();
  drawGauge(42);
  
  // Set initial clean baseline cards
  updateGasCard('CO2', 412, GAS.CO2);
  updateGasCard('NH3', 0.8, GAS.NH3);
  updateGasCard('H2', 0.5, GAS.H2);
  updateGasCard('CH4', 1.2, GAS.CH4);
  updateGasCard('CO', 2.1, GAS.CO);
  updateGasCard('Benzene', 0.004, GAS.Benzene);
}

// =============================================================
//  INIT
// =============================================================
window.addEventListener('load', () => {
  initHistoryChart();
  
  // Set initial UI values to match sample data
  if (valMax) valMax.textContent = "31.8";
  if (valMin) valMin.textContent = "20.1";
  if (valAvg) valAvg.textContent = "23.5";
  if (valTherm) valTherm.textContent = "24.2";
  
  drawRaw(currentPixels, 20.1, 31.8);
  drawInterp(currentPixels, 20.1, 31.8);
});
