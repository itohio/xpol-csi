import { signal } from '@preact/signals';
import { useEffect, useRef } from 'preact/hooks';
import './style.css';
import { decodeMessage, decodeParams, encodeParam, encodeIllum, encodeTx, encodeLeaderIllum, encodeView, cmd } from './wire.js';

// Everything arrives over one WebSocket; the board sends what the shown tab
// needs ('V' view) and the browser keeps every history (the board keeps none).
const status = signal(null);      // 1 Hz, both boards
const scene = signal(null);       // 4 Hz: presence, tracks, links
const spectra = signal(null);     // 4 Hz: spectra + waveform history
const rawView = signal(null);     // raw tab: rows history of the strongest link
const radio = signal(null);       // 1 Hz: radio tab
const connected = signal(false);
const transport = signal('connecting');
const page = signal('overview');

const HIST = 240;                 // per-track rate history: 60 s at 4 Hz
const WAVE = 96;                  // waveform: 24 s at 4 Hz
const trackHist = new Map();      // track id -> { breath: [], heart: [] }
let wave = [];
let rawRows = [], rawKey = '';
const rssiHist = [];
const push = (a, v, cap) => { a.push(v); if (a.length > cap) a.splice(0, a.length - cap); };

function handle(m) {
  connected.value = true;
  switch (m.type) {
    case 'status': status.value = m; break;
    case 'scene': {
      const ids = new Set();
      for (const b of m.breathers) {
        ids.add(b.id);
        let h = trackHist.get(b.id);
        if (!h) trackHist.set(b.id, (h = { breath: [], heart: [] }));
        push(h.breath, b.bpm, HIST); push(h.heart, b.heart_bpm, HIST);
      }
      for (const id of trackHist.keys()) if (!ids.has(id)) trackHist.delete(id);
      scene.value = m;
      break;
    }
    case 'spectra':
      if (m.wave == null) wave = []; else push(wave, m.wave, WAVE);   // no still window: the curve restarts
      spectra.value = { spectrum: m.spectrum, hspectrum: m.hspectrum, wave: wave.slice() };
      break;
    case 'raw': {
      const key = `${m.slot}/${m.sub}`;
      if (key !== rawKey) { rawKey = key; rawRows = []; rssiHist.length = 0; }   // another link
      for (const row of m.rows) push(rawRows, row, Math.round(24 / (m.dt || 0.05)));
      push(rssiHist, m.rssi, 120);
      rawView.value = m.slot < 0 ? null : { rssi: m.rssi, sub: m.sub, dt: m.dt, m: rawRows.slice(), t: rawRows.length };
      break;
    }
    case 'radio': radio.value = { ...m, ...radioRates(m, performance.now()) }; break;
  }
}

let gWs = null;
// Binary frame to the board; false when the socket is not open.
function wsSend(bytes) {
  try { if (gWs && gWs.readyState === 1) { gWs.send(bytes); return true; } } catch {}
  return false;
}
function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  gWs = ws;
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { transport.value = 'live'; wsSend(encodeView(page.value)); };
  ws.onmessage = (e) => { try { const m = decodeMessage(e.data); if (m) handle(m); } catch {} };
  ws.onerror = () => { try { ws.close(); } catch {} };
  ws.onclose = () => {
    connected.value = false;
    transport.value = 'reconnecting';
    setTimeout(connectWS, 1500);
  };
}

// ---- canvas line chart -------------------------------------------------
function Chart({ series, height = 70, ymin, ymax, zero = false, labels }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current; if (!c) return;
    const dpr = devicePixelRatio || 1;
    const w = c.clientWidth, h = height;
    c.width = w * dpr; c.height = h * dpr;
    const g = c.getContext('2d'); g.scale(dpr, dpr);
    g.clearRect(0, 0, w, h);
    const all = series.flatMap((s) => s.data);
    let lo = ymin ?? Math.min(...all, 0), hi = ymax ?? Math.max(...all, 1);
    if (zero) { const m = Math.max(Math.abs(lo), Math.abs(hi), 1e-3); lo = -m; hi = m; }
    if (hi - lo < 1e-6) hi = lo + 1;
    const X = (i, n) => (n <= 1 ? 0 : (i / (n - 1)) * (w - 2) + 1);
    const Y = (v) => h - 2 - ((v - lo) / (hi - lo)) * (h - 4);
    if (zero) { g.strokeStyle = '#30363d'; g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(w, Y(0)); g.stroke(); }
    series.forEach((s) => {
      if (!s.data.length) return;
      g.strokeStyle = s.color; g.lineWidth = 1.5; g.beginPath();
      s.data.forEach((v, i) => { const x = X(i, s.data.length), y = Y(v); i ? g.lineTo(x, y) : g.moveTo(x, y); });
      g.stroke();
    });
  });
  return (
    <div class="chart">
      <canvas ref={ref} style={{ width: '100%', height: height + 'px' }} />
      {labels && <div class="chartlabels">{labels}</div>}
    </div>
  );
}

// ---- spectrum: bars, or (click) a waterfall of the last 2 min ------------
// Waterfall colour = dB above the row's median (the noise floor the firmware
// uses), 0..20 dB, so a real line stands out as a steady vertical trace and
// sidelobes/noise stay dark. Newest row at the bottom.
const WF_ROWS = 480;   // 2 min at 4 Hz
function wfColor(db) {
  const t = Math.max(0, Math.min(1, db / 20));
  return [Math.round(255 * Math.min(1, t * 1.6)), Math.round(255 * Math.max(0, t * 1.6 - 0.6)), Math.round(80 * (1 - t))];
}
function Spectrum({ spec }) {
  const ref = useRef(null);
  const hist = useRef({ last: null, rows: [] });
  const mode = useRef('bars');
  useEffect(() => {
    const c = ref.current; if (!c || !spec || !spec.p || !spec.p.length) return;
    const H = hist.current;
    if (spec !== H.last) {                       // one row per scene update
      H.last = spec;
      const p = spec.p, s = [...p].sort((a, b) => a - b), med = s[s.length >> 1] || 1e-9;
      H.rows.push(Float32Array.from(p, (v) => 10 * Math.log10(v / med)));
      if (H.rows.length > WF_ROWS) H.rows.shift();
    }
    const dpr = devicePixelRatio || 1, w = c.clientWidth, h = mode.current === 'bars' ? 110 : 160;
    c.width = w * dpr; c.height = h * dpr; c.style.height = h + 'px';
    const g = c.getContext('2d'); g.scale(dpr, dpr); g.clearRect(0, 0, w, h);
    const n = spec.p.length, bw = w / n;
    if (mode.current === 'bars') {
      const p = spec.p, mx = Math.max(...p, 1);
      for (let i = 0; i < n; i++) {
        const bh = (p[i] / mx) * (h - 16);
        g.fillStyle = `hsl(${200 - 160 * (p[i] / mx)},70%,55%)`;
        g.fillRect(i * bw, h - 14 - bh, bw - 1, bh);
      }
    } else {
      const rows = H.rows, img = new ImageData(n, rows.length);
      for (let r = 0; r < rows.length; r++)
        for (let i = 0; i < n; i++) {
          const [R, G, B] = wfColor(rows[r][i] || 0), o = 4 * (r * n + i);
          img.data[o] = R; img.data[o + 1] = G; img.data[o + 2] = B; img.data[o + 3] = 255;
        }
      const off = new OffscreenCanvas(n, rows.length); off.getContext('2d').putImageData(img, 0, 0);
      g.imageSmoothingEnabled = false;
      const ph = (h - 14) * (rows.length / WF_ROWS);          // grows downward until full
      g.drawImage(off, 0, h - 14 - ph, w, ph);
    }
    g.fillStyle = '#8b949e'; g.font = '10px system-ui';
    for (let b = 0; b <= n; b += n > 60 ? 15 : 6) g.fillText(Math.round(spec.fmin + b) + '', b * bw, h - 2);
  });
  const toggle = () => { mode.current = mode.current === 'bars' ? 'waterfall' : 'bars'; scene.value = { ...scene.value }; };   // re-render now
  return <canvas ref={ref} onClick={toggle} title="click: bars / waterfall" style={{ width: '100%', height: '110px', cursor: 'pointer' }} />;
}

// ---- synced pulsing dots ----------------------------------------------
// Each track keeps its own phase, advanced every animation frame at its rate and
// gently pulled toward the firmware phase, so scene updates never jump the dots.
const wrap = (x) => x - Math.round(x);          // to [-0.5, 0.5)
function heartPath(g, cx, cy, s) {
  g.beginPath();
  g.moveTo(cx, cy + s * 0.9);
  g.bezierCurveTo(cx - s * 1.6, cy - s * 0.1, cx - s * 0.7, cy - s * 1.3, cx, cy - s * 0.45);
  g.bezierCurveTo(cx + s * 0.7, cy - s * 1.3, cx + s * 1.6, cy - s * 0.1, cx, cy + s * 0.9);
  g.closePath();
}
function PulseDots({ breathers }) {
  const ref = useRef(null);
  const list = useRef([]);
  const state = useRef(new Map());              // id -> { b, h, fw } phases 0..1
  list.current = breathers || [];
  useEffect(() => {
    let raf, last = performance.now();
    const draw = () => {
      raf = requestAnimationFrame(draw);
      const c = ref.current; if (!c) return;
      const now = performance.now(), dt = Math.min((now - last) / 1000, 0.1); last = now;
      const dpr = devicePixelRatio || 1, w = c.clientWidth, h = 130;
      c.width = w * dpr; c.height = h * dpr;
      const g = c.getContext('2d'); g.scale(dpr, dpr); g.clearRect(0, 0, w, h);
      const items = list.current, slot = w / Math.max(items.length, 1);
      items.forEach((b, i) => {
        let st = state.current.get(b.id);
        if (!st) { st = { b: b.phase || 0, h: 0, fw: b.phase }; state.current.set(b.id, st); }
        st.b = (st.b + (b.bpm / 60) * dt + 2) % 1;
        if (b.phase !== st.fw) st.fw = b.phase;                    // new firmware estimate
        st.b = (st.b + 0.05 * wrap((st.fw ?? st.b) - st.b) + 1) % 1; // soft lock, no jumps
        if (b.heart_bpm) st.h = (st.h + (b.heart_bpm / 60) * dt) % 1;
        const pulseB = (1 - Math.cos(2 * Math.PI * st.b)) / 2;
        const pulseH = (1 - Math.cos(2 * Math.PI * st.h)) / 2;
        const cx = slot * (i + 0.5), cyB = 32, cyH = 96;
        // breathing dot (blue), size+alpha pulse, ring = confidence
        g.beginPath(); g.arc(cx, cyB, 8 + 14 * pulseB, 0, 2 * Math.PI);
        g.fillStyle = `rgba(31,111,235,${0.35 + 0.6 * pulseB * (b.confirmed ? 1 : 0.4)})`; g.fill();
        g.beginPath(); g.arc(cx, cyB, 25, -Math.PI / 2, -Math.PI / 2 + 2 * Math.PI * (b.conf ?? 0));
        g.strokeStyle = b.confirmed ? '#3fb950' : '#8b949e'; g.lineWidth = 3; g.stroke();
        g.fillStyle = '#e6edf3'; g.font = '11px system-ui'; g.textAlign = 'center';
        g.fillText(`${b.bpm.toFixed(1)} bpm`, cx, 70);
        // heart (red): solid when confirmed, faint while a hypothesis
        if (b.heart_bpm) {
          heartPath(g, cx - 22, cyH, 5 + 3 * pulseH);
          g.fillStyle = `rgba(248,81,73,${(b.heart_confirmed ? 0.45 : 0.15) + (b.heart_confirmed ? 0.55 : 0.25) * pulseH})`; g.fill();
          g.fillStyle = b.heart_confirmed ? '#e6edf3' : '#8b949e'; g.textAlign = 'left';
          g.fillText(`${b.heart_bpm.toFixed(0)}±${b.heart_ci.toFixed(0)}`, cx - 12, cyH + 4);
        }
      });
      for (const id of state.current.keys()) if (!items.some((b) => b.id === id)) state.current.delete(id);
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, []);
  return <canvas ref={ref} style={{ width: '100%', height: '130px' }} />;
}

// ---- pages -------------------------------------------------------------
function Overview() {
  const s = scene.value;
  const br = s?.breathers || [];
  return (
    <>
      <div class="card">
        <div class={'big presence ' + (s?.presence ? 'on' : 'off')}>
          {s ? (s.presence ? 'PRESENCE' : 'clear') : '—'}
        </div>
        <div class="row"><span>motion index</span><span>{s ? s.motion.toFixed(2) : '–'}</span></div>
        <div class="bar mi"><i class={(s?.motion || 0) >= 1 ? 'hot' : ''} style={{ width: Math.min(100, (s?.motion || 0) * 50) + '%' }} /><b class="thr" /></div>
        <div class="muted">line = motion threshold (1.0)</div>
      </div>

      <div class="card">
        <div class="row">
          <span>breathers</span>
          <span><span class="b">{br.filter((b) => b.confirmed).length}</span> <span class="muted">confirmed · {br.filter((b) => !b.confirmed).length} hypotheses</span></span>
        </div>
        <PulseDots breathers={br} />
        {br.map((b) => (
          <div class="hyp" key={b.id}>
            <div class="brow">
              <span class={'tag ' + (b.confirmed ? 'ok' : 'maybe')}>{b.confirmed ? 'confirmed' : 'forming'}</span>
              <span>{b.bpm.toFixed(1)} ± {b.ci.toFixed(1)} bpm</span>
              <span class="muted">{b.amp.toFixed(1)} dB</span>
              <span class="muted">{(b.conf * 100).toFixed(0)}% · {b.age.toFixed(0)} s</span>
              <div class="bar sm"><i style={{ width: b.conf * 100 + '%' }} /></div>
            </div>
            <div class="brow sub">
              <span class={'tag ' + (b.heart_confirmed ? 'ok' : 'maybe')}>{b.heart_bpm ? (b.heart_confirmed ? '♥ confirmed' : '♥ forming') : '♥ none'}</span>
              {b.heart_bpm > 0 && <span>{b.heart_bpm.toFixed(0)} ± {b.heart_ci.toFixed(1)} bpm</span>}
              {b.heart_bpm > 0 && <span class="muted">{b.heart_snr.toFixed(1)} dB</span>}
            </div>
          </div>
        ))}
        {!br.length && <div class="muted">no hypotheses: vitals are evaluated after being still for a while (tune: "still before vitals")</div>}
      </div>

      <RfPanel rf={s?.rf} />
    </>
  );
}

// RSSI on a fixed -100..0 dBm scale so links are comparable at a glance.
const rssiPct = (dbm) => Math.min(100, Math.max(0, dbm + 100));
const SATURATION_DBM = -20;
function RssiBar({ last, avg, used = false }) {
  const hot = Math.max(last, avg ?? last) > SATURATION_DBM;
  const cls = hot ? ' warn' : used ? ' used' : '';
  return (
    <>
      <div class="bar rs" title={hot ? 'receiver near saturation' : undefined}>
        <i class={hot ? 'hot' : 'ok'} style={{ width: rssiPct(avg ?? last) + '%' }} />
        {avg != null && <b class="mark" style={{ left: rssiPct(last) + '%' }} />}
      </div>
      <span class={'num' + cls} title={hot ? 'receiver near saturation' : used ? 'its CSI feeds the scene' : undefined}>
        {avg != null ? `${last} / ${avg}` : last} dBm
      </span>
    </>
  );
}

function RfPanel({ rf }) {
  if (!rf || !rf.length) return null;
  return (
    <div class="card">
      <div class="row"><span>RF links (RSSI)</span><span class="muted">|H| = AGC-normalised CSI amplitude</span></div>
      {rf.map((r) => (
        <div class="brow" key={r.mac}>
          <span class="muted mono">{r.mac}</span>
          <RssiBar last={r.rssi} />
          <span class="muted">|H| {r.coupling.toFixed(1)}</span>
        </div>
      ))}
    </div>
  );
}

function Graphs() {
  const s = scene.value, sp = spectra.value;
  const br = s?.breathers || [];
  return (
    <>
      <div class="card">
        <div class="row"><span>breathing waveform (strongest link)</span><span class="muted">24 s, principal component, built here</span></div>
        <Chart series={[{ data: sp?.wave || [], color: '#1f6feb' }]} height={80} zero />
      </div>
      <div class="card">
        <div class="row"><span>breathing spectrum (raw filter, bpm)</span><span class="muted">click for waterfall</span></div>
        <Spectrum spec={sp?.spectrum} />
      </div>
      <div class="card">
        <div class="row"><span>heart-band spectrum (same series, 45–150 bpm)</span><span class="muted">experimental</span></div>
        <Spectrum spec={sp?.hspectrum} />
      </div>
      {br.map((b) => {
        const h = trackHist.get(b.id);
        return (
          <div class="card" key={b.id}>
            <div class="row">
              <span>breather {b.id} · {b.bpm.toFixed(1)}±{b.ci.toFixed(1)} bpm</span>
              <span class="muted">{(b.conf * 100).toFixed(0)}% conf</span>
            </div>
            <Chart series={[{ data: h?.breath || [], color: '#1f6feb' }]} height={60}
                   labels={<span class="muted">breathing rate, last 60 s</span>} />
            <Chart series={[{ data: h?.heart || [], color: '#f85149' }]} height={60}
                   labels={<span class="muted">heart rate, last 60 s</span>} />
          </div>
        );
      })}
    </>
  );
}

// Per-subcarrier deviation from its own window mean: the static frequency
// profile is removed so only changes (breathing, motion) remain visible.
function deviations(m) {
  const T = m.length, S = T ? m[0].length : 0;
  const mean = new Array(S).fill(0);
  for (const row of m) for (let s = 0; s < S; s++) mean[s] += row[s] / T;
  return m.map((row) => row.map((v, s) => v - mean[s]));
}

// One lane per subcarrier, each auto-scaled; label = peak-to-peak amplitude.
function Traces({ m, dt }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current; if (!c || !m || !m.length) return;
    const S = m[0].length, lane = 22, h = S * lane;
    const dpr = devicePixelRatio || 1, w = c.clientWidth;
    c.width = w * dpr; c.height = h * dpr; c.style.height = `${h}px`;
    const g = c.getContext('2d'); g.scale(dpr, dpr); g.clearRect(0, 0, w, h);
    const d = deviations(m), T = d.length, x0 = 44;
    g.font = '10px ui-monospace, monospace';
    for (let s = 0; s < S; s++) {
      let lo = Infinity, hi = -Infinity;
      for (let t = 0; t < T; t++) { const v = d[t][s]; if (v < lo) lo = v; if (v > hi) hi = v; }
      const pp = hi - lo, mid = s * lane + lane / 2, k = pp > 1e-6 ? (lane - 4) / pp : 0;
      g.fillStyle = '#8b949e'; g.fillText(pp.toFixed(2), 2, mid + 3);
      g.strokeStyle = `hsl(${(s * 360) / S},70%,60%)`; g.beginPath();
      for (let t = 0; t < T; t++) {
        const x = x0 + (t / Math.max(T - 1, 1)) * (w - x0 - 2), y = mid - (d[t][s] - (lo + hi) / 2) * k;
        t ? g.lineTo(x, y) : g.moveTo(x, y);
      }
      g.stroke();
    }
  });
  return (
    <>
      <canvas ref={ref} style={{ width: '100%' }} />
      <div class="muted">{m && m.length ? `${(m.length * dt).toFixed(0)} s window · label = peak-to-peak amplitude` : ''}</div>
    </>
  );
}

// Deviation waterfall: blue = below the subcarrier's mean, red = above.
function Waterfall({ m }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current; if (!c || !m || !m.length) return;
    const dpr = devicePixelRatio || 1, w = c.clientWidth, h = 160;
    c.width = w * dpr; c.height = h * dpr;
    const g = c.getContext('2d'); g.scale(dpr, dpr); g.clearRect(0, 0, w, h);
    const d = deviations(m), T = d.length, S = d[0].length;
    let mx = 1e-6;
    for (const row of d) for (const v of row) mx = Math.max(mx, Math.abs(v));
    const cw = w / T, ch = h / S;
    for (let t = 0; t < T; t++)
      for (let s = 0; s < S; s++) {
        const v = d[t][s] / mx;
        g.fillStyle = v >= 0 ? `hsl(5,75%,${15 + 50 * v}%)` : `hsl(215,75%,${15 - 50 * v}%)`;
        g.fillRect(t * cw, h - (s + 1) * ch, cw + 0.6, ch + 0.6);
      }
  });
  return <canvas ref={ref} style={{ width: '100%', height: '160px' }} />;
}

function Raw() {
  const j = rawView.value;
  return (
    <>
      <div class="card">
        <div class="row"><span>CSI subcarrier traces</span><span class="muted">deviation from mean, strongest link</span></div>
        {j ? <Traces m={j.m} dt={j.dt} /> : <div class="muted">waiting…</div>}
      </div>
      <div class="card">
        <div class="row"><span>deviation waterfall</span><span class="muted">time → , subcarrier ↑</span></div>
        <Waterfall m={j?.m} />
        <div class="muted">{j ? `${j.t} rows × ${j.sub} subcarriers · ${(j.dt * 1000).toFixed(0)} ms/row` : ''}</div>
      </div>
      <div class="card">
        <div class="row"><span>RSSI (strongest link)</span><span class="b">{j ? j.rssi : '–'} dBm</span></div>
        <Chart series={[{ data: [...rssiHist], color: '#3fb950' }]} height={60} />
      </div>
    </>
  );
}

// ---- tune: live DSP parameters ----------------------------------------
const params = signal([]);
const calibrating = signal(false);
const tuneMsg = signal('');
const paramsErr = signal(false);
let paramTimer = 0;

async function loadParams() {
  clearTimeout(paramTimer);
  try {
    const p = decodeParams(await (await fetch('/api/params', { cache: 'no-store' })).arrayBuffer());
    if (!p) throw 0;
    params.value = p.params; calibrating.value = p.calibrating; paramsErr.value = false;
    for (const st of pending.values()) st.sent = NaN;   // board values are authoritative now
  } catch { paramsErr.value = true; }
  // the board flags calibrating until it finishes: poll so the banner clears itself
  if (calibrating.value && page.value === 'tune') paramTimer = setTimeout(loadParams, 1000);
}

// At most one 'P' frame per slider every 100 ms; the latest value always goes
// out, and an unchanged value (keyboard step fires input + change) only once.
const pending = new Map();                       // id -> { t, timer, sent }
function sendParam(id, value, now) {
  const st = pending.get(id) || { t: 0, timer: 0, sent: NaN };
  pending.set(id, st);
  clearTimeout(st.timer);
  if (value === st.sent) return;
  const fire = () => { st.t = performance.now(); st.sent = value; wsSend(encodeParam(id, value)); };
  const wait = now ? 0 : st.t + 100 - performance.now();
  if (wait <= 0) fire(); else st.timer = setTimeout(fire, wait);
}
function setParam(id, value, now) {
  params.value = params.value.map((p) => (p.id === id ? { ...p, value } : p));
  sendParam(id, value, now);
}

const COMMANDS = { R: 'buffers reset', C: 'calibration started', S: 'saved to flash', D: 'defaults restored (not saved)' };
function command(ch) {
  if (ch === 'C' && !confirm('Empty-room calibration takes ~20 s. Everyone must leave the room. Start?')) return;
  if (!wsSend(cmd(ch))) { tuneMsg.value = 'not connected'; return; }
  tuneMsg.value = COMMANDS[ch];
  if (ch === 'C') calibrating.value = true;
  if (ch !== 'R') setTimeout(loadParams, 300);   // let the board apply it first
}

const decimals = (step) => Math.min(6, Math.max(0, Math.ceil(-Math.log10(step || 1))));

function ParamRow({ p }) {
  const d = decimals(p.step), v = +p.value.toFixed(d);
  const onNumber = (e) => {
    const x = parseFloat(e.currentTarget.value);
    if (Number.isFinite(x)) setParam(p.id, Math.min(p.max, Math.max(p.min, x)), true);
  };
  return (
    <div class="param">
      <span class="pname">{p.name}</span>
      <input type="range" min={p.min} max={p.max} step={p.step} value={v}
             onInput={(e) => setParam(p.id, +e.currentTarget.value)}
             onChange={(e) => setParam(p.id, +e.currentTarget.value, true)} />
      <input type="number" min={p.min} max={p.max} step={p.step} value={v} onChange={onNumber} />
      <span class="muted">{p.unit}</span>
    </div>
  );
}

function Tune() {
  useEffect(() => { loadParams(); return () => clearTimeout(paramTimer); }, []);
  const s = scene.value;
  const mi = s?.motion || 0;
  const groups = {};
  for (const p of params.value) (groups[p.group] ||= []).push(p);
  return (
    <>
      {calibrating.value && <div class="banner">Calibrating empty room — leave the room now (~20 s)</div>}
      <div class="card">
        <div class="row">
          <span class={'presence ' + (s?.presence ? 'on' : 'off')}>{s ? (s.presence ? 'PRESENCE' : 'clear') : '—'}</span>
          <span>motion index <b>{mi.toFixed(2)}</b></span>
        </div>
        <div class="bar mi"><i class={mi >= 1 ? 'hot' : ''} style={{ width: Math.min(100, mi * 50) + '%' }} /><b class="thr" /></div>
        <div class="muted">line = threshold (1.0)</div>
        {(s?.breathers || []).map((b) => (
          <div class="brow" key={b.id}>
            <span class={'tag ' + (b.confirmed ? 'ok' : 'maybe')}>{b.id}</span>
            <span>{b.bpm.toFixed(1)} bpm</span>
            <span class="muted">snr {b.amp.toFixed(1)} dB</span>
            {b.heart_bpm > 0 && <span class="muted">♥ {b.heart_bpm.toFixed(0)} bpm</span>}
          </div>
        ))}
        <Chart series={[{ data: spectra.value?.wave || [], color: '#1f6feb' }]} height={60} zero />
        <Spectrum spec={spectra.value?.spectrum} />
      </div>
      <div class="card btns">
        <button onClick={() => command('R')}>reset buffers</button>
        <button onClick={() => command('C')} disabled={calibrating.value}>calibrate (empty room)</button>
        <button onClick={() => command('S')}>save to flash</button>
        <button onClick={() => command('D')}>defaults</button>
        <span class="muted">{tuneMsg.value}</span>
      </div>
      {Object.entries(groups).map(([g, ps]) => (
        <div class="card" key={g}>
          <div class="row"><b>{g}</b></div>
          {ps.map((p) => <ParamRow p={p} key={p.id} />)}
        </div>
      ))}
      {!params.value.length && <div class="muted">{paramsErr.value ? 'could not load /api/params' : 'loading parameters…'}</div>}
    </>
  );
}

// ---- radio: RF environment, TX power, CPU and CSI ring load -------------
// Cumulative counters -> per-second rates. The board refreshes them every 2 s
// while RADIO arrives every 1 s, so rates are taken between snapshots that changed.
let radioPrev = null, radioRate = { csi_rate: null, drop_rate: null };
function radioRates(r, t) {
  const p = radioPrev;
  if (p && r.csi_written === p.written && r.csi_drops === p.drops) return radioRate;   // same snapshot
  radioPrev = { t, written: r.csi_written, drops: r.csi_drops };
  const dt = p ? (t - p.t) / 1000 : 0;
  radioRate = !(dt > 0) || r.csi_written < p.written || r.csi_drops < p.drops
    ? { csi_rate: null, drop_rate: null }
    : { csi_rate: (r.csi_written - p.written) / dt, drop_rate: (r.csi_drops - p.drops) / dt };
  return radioRate;
}

const kb = (b) => (b / 1024).toFixed(1);
const fmt = (v, d = 1) => (v == null ? '–' : v.toFixed(d));

function sourceLabel(s) {
  if (s.flags & 8) return 'follower board';
  if (s.flags & 6) {
    const name = s.ssid ? `AP ${s.ssid}` : 'AP (hidden)';
    return s.flags & 2 ? `${name} (connected)` : name;
  }
  return '';
}

// used: its CSI fed the scene on the last tick (green RSSI); link: admitted as
// a link on that board but not used (too slow, still filling, or beyond
// "links per board").
function LinkRow({ label, mac, last, avg, rate, used, link }) {
  return (
    <div class="link">
      <div class="row">
        <span>{label && <b>{label} </b>}<span class="muted mono">{mac}</span>
          {used ? <span class="tag ok">used</span> : link && <span class="tag maybe">link</span>}</span>
        <span class="muted">{fmt(rate)} fr/s</span>
      </div>
      <div class="brow"><RssiBar last={last} avg={avg} used={used} /></div>
    </div>
  );
}

// Transmit controls. Each board illuminates (ESP-NOW broadcasts) for the other's
// receiver. Follower: rate (0 = off) and max TX, re-sent by the leader every
// 2 s; "actual" is what it reports back. Leader: its own max TX (all its
// frames) and rate; it shares its TX with the SoftAP/STA, so lower this if
// phones fail to join the SoftAP.
const ILLUM_RATES = [0, 10, 25, 50, 100, 200];
const TX_DBM = [2, 5, 8, 11, 14, 18, 21];
function TxSelect({ value, onPick }) {
  // The driver reports 0.25 dBm steps; show the nearest preset as selected.
  const sel = value == null ? '' : TX_DBM.reduce((a, b) => (Math.abs(b - value) < Math.abs(a - value) ? b : a));
  return (
    <select value={sel} onChange={(e) => onPick(+e.target.value)}>
      {value == null && <option value="">unknown</option>}
      {TX_DBM.map((tx) => <option key={tx} value={tx}>{tx} dBm</option>)}
    </select>
  );
}
function RateButtons({ current, onPick }) {
  return (
    <div class="btns">
      {ILLUM_RATES.map((hz) => (
        <button key={hz} class={current === hz ? 'on' : ''} onClick={() => onPick(hz)}>{hz ? `${hz} Hz` : 'off'}</button>
      ))}
    </div>
  );
}
function TxControl({ r }) {
  const illum = (hz, tx) => wsSend(encodeIllum(hz, tx));
  const actual = r.peer_illum_hz == null ? 'unknown' : r.peer_illum_hz === 0 ? 'off' : `${r.peer_illum_hz} Hz`;
  return (
    <>
      <div class="row"><span>transmit power (max)</span><span class="muted">ch {r.channel}</span></div>
      <div class="row"><span>leader</span><TxSelect value={r.own_tx} onPick={(tx) => wsSend(encodeTx(tx))} /></div>
      <div class="row"><span>follower</span><TxSelect value={r.peer_tx} onPick={(tx) => illum(r.want_illum_hz, tx)} /></div>
      <div class="row"><span>leader illumination</span><span>{r.own_illum_hz ? `${r.own_illum_hz} Hz` : 'off'}</span></div>
      <RateButtons current={r.own_illum_hz} onPick={(hz) => wsSend(encodeLeaderIllum(hz))} />
      <div class="row"><span>follower illumination</span><span>{actual} <span class="muted">(requested {r.want_illum_hz ? `${r.want_illum_hz} Hz` : 'off'})</span></span></div>
      <RateButtons current={r.want_illum_hz} onPick={(hz) => illum(hz, r.want_tx)} />
    </>
  );
}

function Radio() {
  const r = radio.value;
  if (!r) return <div class="muted">waiting for the board…</div>;
  const idle = r.tasks.filter((t) => /^IDLE/.test(t.name)).reduce((a, t) => a + t.permille, 0);
  const tasks = [...r.tasks].sort((a, b) => b.permille - a.permille);
  const peers = r.peers.filter((p) => p.flags & 1);
  const ringPct = (v) => (r.ring_size ? Math.min(100, (v / r.ring_size) * 100) : 0);
  return (
    <>
      <div class="card">
        <TxControl r={r} />
      </div>

      <div class="card">
        <div class="row"><span>leader hears</span><span class="muted">RSSI last / avg</span></div>
        {r.sources.map((s) => (
          <LinkRow key={s.mac} label={sourceLabel(s)} mac={s.mac} last={s.rssi_last} avg={s.rssi_avg}
                   rate={s.rate} used={!!(s.flags & 1)} link={!!(s.flags & 16)} />
        ))}
        {!r.sources.length && <div class="muted">nothing heard</div>}
      </div>

      <div class="card">
        <div class="row"><span>follower hears</span><span class="muted">RSSI avg</span></div>
        {peers.map((p) => {
          const ssid = r.sources.find((s) => s.mac === p.mac)?.ssid;   // SSID as heard by the leader
          // The leader appears twice: its STA (joins the router, sends data) and its SoftAP (MAC + 1, beacons).
          const label = p.flags & 2 ? (p.flags & 8 ? 'leader board · SoftAP' : 'leader board · STA')
                      : p.flags & 8 ? (ssid ? `AP ${ssid}` : 'AP') : '';
          return <LinkRow key={p.mac} label={label} mac={p.mac} last={p.rssi} rate={p.rate} used={!!(p.flags & 4)} link={!!(p.flags & 16)} />;
        })}
        {!peers.length && <div class="muted">no report from the follower</div>}
      </div>

      <div class="card">
        <div class="row"><span>CPU busy</span><span class="b">{fmt(100 - idle / 10)}%</span></div>
        {tasks.map((t) => (
          <div class="task" key={t.name}>
            <span class="mono">{t.name}</span>
            <div class="bar"><i style={{ width: t.permille / 10 + '%' }} /></div>
            <span class="num">{fmt(t.permille / 10)}%</span>
          </div>
        ))}
      </div>

      <div class="card">
        <div class="row"><span>CSI ring</span><span>{kb(r.ring_used)} / {kb(r.ring_size)} KB <span class="muted">peak {kb(r.ring_peak)}</span></span></div>
        <div class="bar rs"><i style={{ width: ringPct(r.ring_used) + '%' }} /><b class="mark" style={{ left: ringPct(r.ring_peak) + '%' }} /></div>
        <div class="row"><span>CSI frames</span><span>{fmt(r.csi_rate)} /s</span></div>
        <div class="row"><span>drops</span><span class={r.drop_rate > 0 ? 'warn' : ''}>{r.csi_drops} total · {fmt(r.drop_rate)} /s</span></div>
        <div class="row"><span>backlog peak</span><span>{r.backlog_peak} frames</span></div>
        <div class="row"><span>heap free / min</span><span>{kb(r.heap_free)} / {kb(r.heap_min)} KB</span></div>
        <div class="row"><span>main loop</span><span>{r.loop_ms} ms <span class="muted">/ 250 ms</span></span></div>
      </div>
    </>
  );
}

const PAGES = { overview: Overview, graphs: Graphs, raw: Raw, tune: Tune, radio: Radio };

// Always visible, one line: each board's CPU, free heap (and its lowest point
// over the last 2 s: the Wi-Fi driver needs headroom), TX power, illumination
// and CSI rate; red when its CSI is frozen.
function BoardStat({ name, b }) {
  if (!b) return <span class="st muted">{name} —</span>;
  return (
    <span class={'st' + (b.frozen ? ' bad' : '')}
          title={`${name}: CPU busy, free heap now (lowest in the last 2 s), max TX power, illumination rate, CSI frames/s`}>
      <b>{name}</b> {b.cpu}% · {kb(b.heap_free)}K<span class={b.heap_min < 12 * 1024 ? ' warn' : ' muted'}>↓{kb(b.heap_min)}</span>
      {' '}· {b.tx == null ? '?' : b.tx} dBm · {b.illum_hz ? `${b.illum_hz} Hz` : 'tx off'} · {b.csi_fps} fr/s
      {b.frozen && <span> · CSI FROZEN</span>}
      {b.csi_restarts > 0 && <span class="muted"> · {b.csi_restarts} restarts</span>}
    </span>
  );
}
function StatusBar() {
  const st = status.value;
  return (
    <div class="statusbar">
      <span class={connected.value ? 'muted' : 'warn'}>{transport.value}</span>
      {st && <span class="muted">ch {st.channel} · {st.links_per_board} link{st.links_per_board > 1 ? 's' : ''}/board</span>}
      <BoardStat name="L" b={st?.leader} />
      <BoardStat name="F" b={st?.follower} />
    </div>
  );
}

export function App() {
  useEffect(() => { connectWS(); }, []);
  useEffect(() => { wsSend(encodeView(page.value)); }, [page.value]);
  const s = scene.value, Page = PAGES[page.value];
  return (
    <div class="wrap">
      <h1>
        xpol-csi
        <span class="muted">{s ? s.mode : ''}</span>
      </h1>
      <StatusBar />
      <div class="tabs">
        {Object.keys(PAGES).map((p) => (
          <button key={p} class={page.value === p ? 'on' : ''} onClick={() => (page.value = p)}>{p}</button>
        ))}
      </div>
      <Page />
    </div>
  );
}
