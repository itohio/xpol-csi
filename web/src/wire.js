// Decoders for the firmware's packed little-endian binary WebSocket messages.
// First byte = type. Must match src/phone_http.cpp (STATUS, RAW, RADIO),
// src/scene_state.cpp (SCENE, SPECTRA) and params_get (GET /api/params).

const reader = (ab) => {
  const dv = new DataView(ab);
  let o = 0;
  return {
    get o() { return o; },
    u8: () => dv.getUint8(o++),
    i8: () => dv.getInt8(o++),
    u16: () => { const v = dv.getUint16(o, true); o += 2; return v; },
    i16: () => { const v = dv.getInt16(o, true); o += 2; return v; },
    u32: () => { const v = dv.getUint32(o, true); o += 4; return v; },
    f32: () => { const v = dv.getFloat32(o, true); o += 4; return v; },
    bytes: (n) => { const b = new Uint8Array(ab, o, n); o += n; return b; },
  };
};

// Any board message -> { type: 'status' | 'scene' | 'spectra' | 'raw' | 'radio', ... }.
export function decodeMessage(ab) {
  if (!(ab instanceof ArrayBuffer) || ab.byteLength < 1) return null;
  switch (new DataView(ab).getUint8(0)) {
    case 0xc0: return decodeStatus(ab);
    case 0xc5: return decodeScene(ab);
    case 0xc9: return decodeSpectra(ab);
    case 0xc6: return decodeRaw(ab);
    case 0xc8: { const r = decodeRadio(ab); return r && { type: 'radio', ...r }; }
    default: return null;
  }
}

// STATUS (1 Hz, every view): both boards' load and radio state.
export function decodeStatus(ab) {
  const r = reader(ab); r.u8();
  const channel = r.u8(), links_per_board = r.u8(); r.u8();
  const board = () => {
    const present = r.u8() !== 0, cpu = r.u8(), heap_free = r.u32(), heap_min = r.u32();
    const tx_q = r.i8(), illum_hz = r.u16(), csi_fps = r.u16(), csi_restarts = r.u16(), frozen = r.u8() !== 0;
    return present ? { cpu, heap_free, heap_min, tx: tx_q === -128 ? null : tx_q / 4, illum_hz, csi_fps, csi_restarts, frozen } : null;
  };
  const leader = board(), follower = board();
  return { type: 'status', channel, links_per_board, leader, follower };
}

// SCENE (4 Hz: overview, graphs, tune): current values only.
export function decodeScene(ab) {
  const r = reader(ab); r.u8();
  const presence = r.u8() !== 0, channel = r.u8();
  const mode = r.u8() === 0 ? 'calibrate' : 'run';
  const active_links = r.u8(), heart_count = r.u8(), nb = r.u8();
  const motion = r.f32();
  const breathers = [];
  for (let i = 0; i < nb; i++) {
    const id = r.u8(), confirmed = r.u8() !== 0;
    const bpm = r.f32(), ci = r.f32(), conf = r.f32(), amp = r.f32(), heart_bpm = r.f32(), phase = r.f32(), age = r.f32();
    const heart_ci = r.f32(), heart_snr = r.f32(), heart_confirmed = r.u8() !== 0;
    breathers.push({ id, confirmed, bpm, ci, conf, amp, heart_bpm, phase, age, heart_ci, heart_snr, heart_confirmed });
  }
  const rf = [];
  for (let n = r.u8(); n > 0; n--) {
    const label = new TextDecoder().decode(r.bytes(r.u8()));
    rf.push({ mac: label, coupling: r.f32(), rssi: r.i16() });
  }
  return { type: 'scene', presence, channel, mode, active_links, heart_count, motion,
           breather_count: breathers.length, breathers, rf };
}

// SPECTRA (4 Hz: graphs, tune): power bins as half-dB over the median bin,
// returned as linear power relative to the median, plus one waveform point.
export function decodeSpectra(ab) {
  const r = reader(ab); r.u8();
  const bins = () => {
    const fmin = r.f32(), n = r.u8(), p = new Array(n);
    for (let i = 0; i < n; i++) p[i] = 10 ** (r.i8() / 20);   // i8 = 2 x dB
    return { fmin, df: 1, p };
  };
  const spectrum = bins(), hspectrum = bins();
  const wave_valid = r.u8() !== 0, wave_point = r.f32();
  return { type: 'spectra', spectrum, hspectrum, wave: wave_valid ? wave_point : null };
}

// RAW (4 Hz: raw): the strongest link's breathing-grid rows since the last
// message. rows[k][sub] in amplitude units; a new slot or sub = a new link.
export function decodeRaw(ab) {
  const r = reader(ab); r.u8();
  const scale = 1 / (1 << r.u8()), rssi = r.i8(), slot = r.i8(), row_ms = r.u16(), sub = r.u8(), n = r.u8();
  const rows = [];
  for (let i = 0; i < n; i++) {
    const row = new Array(sub);
    for (let j = 0; j < sub; j++) row[j] = r.i16() * scale;
    rows.push(row);
  }
  return { type: 'raw', rssi, slot, sub, dt: row_ms / 1000, rows };
}

// 'V' u8 view: which tab is shown, so the board sends only what it needs.
export const VIEWS = ['overview', 'graphs', 'raw', 'radio', 'tune'];
export const encodeView = (page) => Uint8Array.of(0x56, Math.max(0, VIEWS.indexOf(page)));

// GET /api/params: u8 magic 0xC7, u8 count, records
// [u8 id, f32 value, min, max, step, u8 len + name, u8 len + unit, u8 len + group],
// then u8 calibrating.
export function decodeParams(ab) {
  const dv = new DataView(ab);
  if (dv.byteLength < 3 || dv.getUint8(0) !== 0xc7) return null;
  let o = 1;
  const u8 = () => dv.getUint8(o++);
  const f32 = () => { const v = dv.getFloat32(o, true); o += 4; return v; };
  const str = () => { let s = ''; for (let n = u8(); n > 0; n--) s += String.fromCharCode(u8()); return s; };
  const params = [];
  for (let n = u8(); n > 0; n--) {
    const id = u8(), value = f32(), min = f32(), max = f32(), step = f32();
    params.push({ id, value, min, max, step, name: str(), unit: str(), group: str() });
  }
  return { params, calibrating: u8() !== 0 };
}

// RADIO (1 Hz: radio tab): RF environment, TX power and board load (type 0xC8).
// TX powers in 0.25 dBm units (-128 = unknown); RSSI -128 = not connected.
const hex = (b) => Array.from(b, (x) => x.toString(16).padStart(2, '0')).join('');
export function decodeRadio(ab) {
  const dv = new DataView(ab);
  if (dv.byteLength < 1 || dv.getUint8(0) !== 0xc8) return null;
  let o = 1;
  const u8 = () => dv.getUint8(o++);
  const i8 = () => dv.getInt8(o++);
  const u16 = () => { const v = dv.getUint16(o, true); o += 2; return v; };
  const u32 = () => { const v = dv.getUint32(o, true); o += 4; return v; };
  const bytes = (n) => { const b = new Uint8Array(ab, o, n); o += n; return b; };
  const mac = () => hex(bytes(6));
  const str = () => new TextDecoder().decode(bytes(u8()));
  const dbm = (q) => (q === -128 ? null : q / 4);

  const channel = u8();
  const own_tx = dbm(i8()), peer_tx = dbm(i8());
  const peer_illum_hz = u16(), want_illum_hz = u16(), want_tx = i8();
  const own_illum_hz = u16();
  const sta_bssid = mac();
  const sta_rssi = i8();
  const heap_free = u32(), heap_min = u32(), loop_ms = u16();
  const csi_written = u32(), csi_drops = u32();
  const ring_size = u32(), ring_used = u32(), ring_peak = u32();
  const backlog_peak = u16();
  const tasks = [];
  for (let n = u8(); n > 0; n--) tasks.push({ name: str(), permille: u16() });
  const sources = [];
  for (let n = u8(); n > 0; n--) {
    sources.push({ mac: mac(), rssi_last: i8(), rssi_avg: i8(), rate: u16() / 10, flags: u8(), ssid: str() });
  }
  const peers = [];
  for (let n = u8(); n > 0; n--) peers.push({ mac: mac(), rssi: i8(), rate: u16() / 10, flags: u8() });

  return {
    channel, own_tx, peer_tx,
    peer_illum_hz: peer_illum_hz === 0xffff ? null : peer_illum_hz,   // follower's actual rate (0 = off)
    want_illum_hz, want_tx: want_tx === -128 ? null : want_tx,         // what the leader asks for
    own_illum_hz,                                                      // the leader's own ESP-NOW rate (0 = off)
    sta_bssid: /^0+$/.test(sta_bssid) ? null : sta_bssid,
    sta_rssi: sta_rssi === -128 ? null : sta_rssi,
    heap_free, heap_min, loop_ms, csi_written, csi_drops,
    ring_size, ring_used, ring_peak, backlog_peak, tasks, sources, peers,
  };
}

// WebSocket commands to the board: 'P' u8 id f32 value, or a single command byte.
export function encodeParam(id, value) {
  const b = new Uint8Array(6), dv = new DataView(b.buffer);
  b[0] = 0x50; b[1] = id; dv.setFloat32(2, value, true);
  return b;
}
export const cmd = (ch) => Uint8Array.of(ch.charCodeAt(0));
// 'I' u16 rate_hz i8 tx_dbm: follower illumination (0 = off); tx -128 = unchanged.
export function encodeIllum(rate_hz, tx_dbm) {
  const b = new Uint8Array(4), dv = new DataView(b.buffer);
  b[0] = 0x49; dv.setUint16(1, rate_hz, true); dv.setInt8(3, tx_dbm == null ? -128 : tx_dbm);
  return b;
}
// 'T' i8 tx_dbm: the leader's own max TX power.
export const encodeTx = (tx_dbm) => Int8Array.of(0x54, tx_dbm);
// 'L' u16 rate_hz: the leader's own illumination rate (0 = off).
export function encodeLeaderIllum(rate_hz) {
  const b = new Uint8Array(3); new DataView(b.buffer).setUint16(1, rate_hz, true); b[0] = 0x4c;
  return b;
}
