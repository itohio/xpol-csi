# xpol-csi

Cross-polarized bistatic Wi-Fi **CSI** (Channel State Information) presence and
breathing sensor. Two Seeed **XIAO ESP32-C5** boards, mounted back-to-back with
opposite-hand circularly polarized antennas (one LHCP, one RHCP), each sitting in
the other's radiation null. One board illuminates while the other listens; the
cross-polarization and physical separation suppress the direct path so the weak
body reflection is a larger fraction of what the ADC sees.

This is a **prototype** for evaluating the core principle:

> maximize separation between the direct signal and the reflected signal, because
> breathing and motion live almost entirely in the reflection.

It is RF phase sensing, not networking. No traffic is decoded; only the physical
channel estimate of the node's own ESP-NOW frames (and, opportunistically, any
other frames on the same channel) is measured.

---

## Physics reality check (read before building hardware)

These bound what the prototype can and cannot do. See the chat notes for the math.

| Claim | Reality |
| --- | --- |
| Opposite-hand antenna rejection | ~10–20 dB in practice (axial-ratio limited), not 36 dB. It suppresses the **direct path**, which is the point; it does not add SNR to the reflection. |
| 5 GHz gives better spatial resolution | No. Range resolution is `c/2B` = 3.75 m at 40 MHz. 5 GHz helps because a 5 mm chest motion rotates the phase ~2x more than at 2.4 GHz. |
| Antenna nulls isolate the two radios | At a few cm the antennas are in each other's near field; isolation is dominated by shared ground/battery coupling. **Measure it with a VNA.** |
| Two chips share a phase reference | They do not (separate crystals), so raw CSI **phase** across the link is unusable unless you drive both from one clock. Amplitude still works. A shared 40 MHz clock is the highest-value hardware upgrade. |
| Detects a motionless person | CSI reliably detects **motion**; breathing of one still person is achievable; still, non-breathing presence is research-grade. For guaranteed stillness use 60 GHz FMCW radar. |

**Safety:** never present "no detection" as "room is empty". This is a sensing
aid, not a life-safety guarantee.

---

## Hardware

- 2x Seeed Studio XIAO ESP32-C5 (dual-band, 5 GHz capable).
- LHCP + RHCP antennas on u.FL pigtails (5.8 GHz FPV antennas work and overlap
  Wi-Fi channels 149–165).
- UART cross-link between the two boards:

  | board 1 pin | wire | board 2 pin |
  | --- | --- | --- |
  | D6 / GPIO11 (TX) | ---> | D7 / GPIO12 (RX) |
  | D7 / GPIO12 (RX) | <--- | D6 / GPIO11 (TX) |
  | GND | --- | GND |
  | 5V | --- | 5V (optional: power both from one USB-C) |

- USB-C to each board (flashing + console + power). Shared battery in the final
  ball; for the bench, USB is fine.
- Optional: NanoVNA (~$60) to measure real antenna-to-antenna isolation.

The USB-C port is the console + telemetry (USB Serial/JTAG). The inter-board link
is UART1 on D6/D7 at 921600 baud. Power one board over USB-C and bridge the two
**5V** pins (+ shared GND) to run both from one cable; only one USB-C plugged at
a time (see the chat notes on current/caveats).

---

## One firmware, runtime roles

Every board flashes the **same image**. At boot the two boards run a **leader
election** over the UART (identity = factory MAC; higher MAC wins; no peer heard
=> standalone leader). The leader runs the survey, picks the quietest channel,
hosts the SoftAP + dashboard at **`10.0.0.1`**, and fuses the scene; the follower
illuminates + senses and reports features to the leader. The leader repeats its
channel decision every 2 s, so a follower can reboot on its own and rejoin (and
retunes if the leader re-plans a different channel). A follower promotes
itself if the leader's heartbeat stops.

Both boards illuminate and sense on the **same agreed channel** (the faithful
bistatic measurement is same-channel; one radio cannot TX on ch X and receive a
reflection on ch Y at once). Each also picks up CSI from any external AP on that
channel for extra views. **Both boards stream telemetry over their own USB-C**, so
connecting to either works; reconnecting resets that board and re-runs election.

### LED (GPIO27)

| Pattern | Meaning |
| --- | --- |
| fast blink | election / waiting for channel agreement |
| solid | channel agreed |
| slow blink | calibration (learning empty-room baseline) |
| heartbeat | running |

---

## Operating modes (`DEFAULT_MODE` in `config.h`)

- `RUN` — reports occupancy / respiration / heartbeat. The empty-room motion
  calibration (`CALIB_SECONDS`, 20 s) is stored per link (MAC + CSI format) in
  NVS, so it survives reboots and reflashes; a board that has none calibrates
  itself 6 s after sensing starts (links need a few seconds to be admitted and
  rated first; the LED blinks slowly meanwhile). Redo it with the room empty:
  `c` on the USB console, or **calibrate** in the web tune tab.
- `CALIBRATE` — same baseline learning, intended for data gathering.

The heavy per-packet **CSI stream is off by default** so the board never floods
its USB console (esptool always connects for reflashing). Turn it on at runtime
by sending `v` (off with `q`); `monitor.py` does this automatically. The `verbose`
build has it on from boot for bench use.

---

## Build / flash / monitor

Toolchain: PlatformIO with the pioarduino ESP-IDF 5.5 platform (pinned in
`platformio.ini`). Build the web bundle first so it bakes into the SPIFFS image.

```sh
npm --prefix web install                       # once
pio run -e step4_leader -t upload              # firmware -> board on usbmodem101 (SoftAP build)
pio run -e step4_leader -t uploadfs            # web UI (rebuilds web/dist first)
pio run -e step4_follower -t upload            # firmware -> board on usbmodem1101
pio device monitor -e step4_leader             # debug_* instead: leader joins your router (STA), no SoftAP
```

Board presets (`[board_leader]`, `[board_follower]` in `platformio.ini`) pin the
USB ports; `step4_leader` / `step4_follower` are the same without the home-Wi-Fi
join. Use one `-t` target per invocation: `-t upload -t uploadfs` together
writes only the filesystem on this platform.

Phone: join SSID **`xpol-csi`** (password `sensesense`) on the sensing channel,
open `http://10.0.0.1/`. The phone must support the sensing band (5 GHz default;
set `SENSE_BAND_5G 0` in `config.h` to prototype on 2.4 GHz).

---

## Diagnostics and tuning (host tools, uv)

Host tools live in `host/` and are managed with `uv`.

```sh
cd host
uv sync                                          # once

# live monitor: election, CSI waterfall, breathing/heart, scene
uv run python monitor.py /dev/cu.usbmodemAAAA
uv run python monitor.py /dev/cu.usbmodemAAAA --record data/   # save raw stream
uv run python monitor.py --replay data/raw-*.log               # offline review

# offline analyzer: re-run breathing/heart detection on a recording to tune
uv run python analyze.py data/raw-*.log
uv run python analyze.py --selftest             # synthesize + verify, no hardware

uv run python make_fixture.py > sample.log      # synthetic telemetry for testing
```

The monitor sends `s` on connect so the current role + election decision print
immediately, parses the CSV-comment header/election narration, and shows which
role (leader/follower/standalone) is the telemetry source.

### Raw I/Q sessions (offline research)

```sh
uv run python record_session.py empty 120     # both boards: raw I/Q + telemetry -> data/empty-<board>-<time>.log
uv run python align.py                         # live: each board's level from the other (antenna alignment)
uv run --with numpy python analyze_session.py data/present-1101-*.log data/empty-1101-*.log
c++ -std=c++17 -O2 -I../src replay.cpp ../src/dsp.cpp -o /tmp/replay
/tmp/replay data/present-1101-*.log --thr 10bda3cedaac=2.5e-05 [--spectrum] > replay.csv
```

`record_session.py` sends `r` (raw dump on) and `q` (off). `analyze_session.py`
reports, per link ≥ 20 frames/s, RSSI, temporal CV and band powers (breathing,
heart, motion) of the amplitude and of the adjacent-subcarrier phase.
`replay.cpp` runs a recording through the exact firmware DSP (`src/dsp.cpp`)
at the firmware's 4 Hz tick; `--thr` takes the threshold printed by
`# calibrated <mac> motion_threshold=...`.

### Telemetry line protocol (`src/telemetry.h`)

```
# ...                                             CSV comments: header + election POV
EVT,<t_ms>,<text>
ROLE,<t_ms>,<role>,<self_mac>,<peer_mac>,<peer_count>
CH,<t_ms>,<channel>,<second>,<band5g>
CSI,<t_ms>,<src_mac>,<rssi>,<noise>,<nsub>,<a0> a1 ... aN-1>     (verbose only, 'v')
IQ,<t_us>,<src_mac>,<rssi>,<noise>,<gain_q8>,<nsub>,<hex int8 I0 Q0 I1 Q1 ...>   (raw dump, 'r')
FEAT,<t_ms>,<src_mac>,<valid>,<motion>,<is_motion>,<rate>,<nb>,<bpm:snr;..>,<nh>,<bpm:snr;..>
SCENE,<t_ms>,<presence>,<motion>,<links>,<nb>,<bpm:snr;..>,<nh>,<bpm:snr;..>
```

Every 2 s the main loop also prints its health:

```
# health loop=<ms per tick> csi_drops=<ring overflows> csi_stale=<frames flagged without a channel estimate> csi_restarts=<CSI restarts since boot> csi_backlog_max=<frames> ring=<used>/<size> peak=<bytes> heap=.. min=..
# cpu IDLE=90.2% main=5.7% wifi=2.3% csi_consume=0.5% ...    (single core: shares add to ~100%)
```

**Frozen CSI.** The leader's CSI engine can latch: every frame from every
transmitter then carries one byte-identical channel estimate (flagged valid by
the hardware; RSSI, timestamps and frame counts normal). Same symptom as
esp-idf#18493 (ESP32-C5, 5 GHz). **Trigger: the leader joined to the home
router (STA)**, with or without its SoftAP. Power-on bisect on channel 44: with
every leader feature on except the router join (SoftAP, HTTP/WebSocket, fusion
loop, UART reports) the leader ran clean for 8–10 min in three power-ons; with
the router join it latched within 40–120 s in five (SoftAP on or off; one
earlier run stayed clean 45 min); disabling RX aggregation (A-MPDU) did not
help. The follower never joins a network and never latched. So use the SoftAP
build (`step4_*`) for sensing; the router build (`debug_*`) gives LAN access
but a frozen leader receiver (see Network builds). Once latched it survives
CSI restarts, resets, reflashing and deep sleep; unplug both USB cables (the
boards' 5 V rails are joined) to clear it. Leader ESP-NOW latched it in earlier
tests, but all of those had the router joined; both boards illuminate at
100 Hz (verified from power-on in the SoftAP build, see below). ESP-NOW starts
on the first nonzero rate. Ruled out: heap, stored PHY
calibration, promiscuous mode alone, legacy-frame CSI, PHY bandwidth, the web
server and the memory refactor. Detection: repeated frames never reach the
DSP; capture is restarted (3× in 10 s, then every 30 s, no reboot); the status
bar shows `CSI FROZEN`; the boot line `# boot reset_reason=0x..` tells a
power-on (0x01) from a reset (0x15 USB, 0x0c software, ...).

**Network builds** (leader only; the follower never joins a network):

| env | leader network | dashboard |
|---|---|---|
| `step4_leader` / `step4_follower` | SoftAP `xpol-csi` | `http://10.0.0.1/` after joining it |
| `debug_leader` / `debug_follower` | joins `XPOL_WIFI_SSID` (STA), no SoftAP | the leader's LAN address (console: `DEBUG_STA joined ip=...`) |

Flash both boards with the same pair: the leader is elected at boot.

Console commands: `v`/`q` CSI stream on/off, `r` raw I/Q dump on (`q` off),
`c` empty-room calibration, `s` status (role, TX power, illumination rate,
channel), `t<dBm>` max TX power (e.g. `t2`; not persisted). Both boards boot at
`TX_POWER_DBM` (14 dBm, 25 mW; the driver default is 18) and the leader keeps
the follower there (`ILLUM_SET`, every 2 s). The boards are 5–10 cm apart: at
18 dBm the leader heard the follower at −4 dBm, near receiver compression.

### Web protocol (one WebSocket per tab, `ws://<board>/ws`)

The page tells the board which tab it shows (`'V' u8 view`); the board then
pushes only the binary messages that tab needs and the browser builds every
history (rate charts, waveform, waterfalls, raw CSI window). Little-endian,
first byte = type; layouts in `src/phone_http.cpp` / `src/scene_state.cpp`,
decoders in `web/src/wire.js`.

| message | rate | tabs | content |
|---|---|---|---|
| STATUS `0xC0` (40 B) | 1 Hz | all | channel, links per board; per board: CPU %, free heap now and its 2 s minimum, TX power, illumination rate, CSI frames/s, CSI restarts, frozen flag |
| SCENE `0xC5` (~80 B) | 4 Hz | overview, graphs, tune | presence, motion index, tracks (rate ± CI, SNR, confidence, heart), this board's links |
| SPECTRA `0xC9` (~160 B) | 4 Hz | graphs, tune | breathing and heart-band spectra (i8 half-dB over the median bin), one waveform point |
| RAW `0xC6` (≤ 520 B) | 4 Hz | raw | the strongest link's new breathing-grid rows since the last message (16 subcarriers, i16 Q4) |
| RADIO `0xC8` (~350 B) | 1 Hz | radio | TX powers, both boards' RF source tables, per-task CPU, CSI ring |

Commands to the board: `'V' view`, `'P' id f32` (parameter), `R`/`C`/`S`/`D`
(reset, calibrate, save, defaults), `'I' u16 hz i8 dBm` (follower illumination
and TX power, forwarded over the UART as `ILLUM_SET` and repeated every 2 s),
`'T' i8 dBm` / `'L' u16 hz` (leader TX power / illumination). `GET /api/params`
returns the self-describing parameter table. No authentication: anyone on the
network can change parameters.

A one-line footer on every tab shows each board's CPU, free heap (↓ = lowest in
the last 2 s), TX power, illumination and CSI rate, red when its CSI is frozen.

### Radio tab (web)

Every transmitter each board hears on the channel (RSSI last/avg, frames/s, AP
SSID; **used** — RSSI in green — when that transmitter's CSI fed the scene on
the last tick, grey **link** when it is admitted as a link but not used: too
slow, motion window still filling, or beyond `links per board`), from a
promiscuous RX sniffer table (FCS-failed frames ignored); the follower sends its
table and status over the UART every 2 s (`RF_SOURCES`). Per-task CPU share, CSI
ring fill/peak, drops, backlog. RSSI above about −20 dBm means the receiver is
near saturation. The tab sets each board's max TX power and illumination rate
(off, 10–200 Hz; max `ILLUMINATE_MAX_HZ`). Not saved to flash: after a reboot
the follower returns to `ILLUMINATE_RATE_HZ`, both boards to `TX_POWER_DBM`
(14 dBm), and both illumination rates to 100 Hz.

### Live tuning (web "tune" tab)

Every DSP parameter (SNR thresholds, bands, low-cut, motion threshold scale,
hysteresis, votes, Hampel k, minimum link rate, links per board, ...) is a
slider; changes apply on the board immediately. **reset buffers** clears the
signal windows (keeps calibration), **calibrate** runs the empty-room
calibration, **save to flash** persists the parameters (NVS, loaded at boot),
**defaults** restores `config.h` values.

---

## DSP

`src/dsp.{h,cpp}` is pure C++ (no IDF), so it compiles and runs on the host and in
firmware unchanged.

- **Links** — a link is one transmitter in one PHY format (802.11 frame
  type + MCS / legacy rate, from the receive metadata) as heard by one board: an
  AP's HT data and its legacy beacons are two links, and so are a board's MCS0
  ESP-NOW broadcasts and its higher-MCS data frames (mixed in one link they
  made the per-frame spread jump and the motion detector never settled). CSI
  is captured for HT and legacy frames; frames that failed reception (bad FCS)
  or carry no transmitter address are dropped in the Wi-Fi callback.
- **Link quality** — `link_quality_db` = RSSI above −100 dBm (a per-frame EMA,
  capped at −15 dBm: beyond that the receiver compresses and more power gains
  nothing) + 10·log10(frames/s) (averaging gain per slot). E.g. −25 dBm at
  100 fr/s = 95 dB, −55 dBm at 100 fr/s = 65 dB, −25 dBm at 5 fr/s = 82 dB.
- **Admission** — up to `MAX_LINKS` links per board (build flag,
  `sensing_flags` in `platformio.ini`: 3 at the 20 Hz grid, or 6 at 10 Hz for
  the same RAM). Every other transmitter/format is a candidate measured over
  2 s windows (rate, mean RSSI); it needs half of `min link rate` (so the
  router's data qualifies, beacon-only APs do not), takes a free slot, or
  replaces the weakest link (older than 30 s) once its quality beats it by 3 dB
  for 10 s straight. Links silent for 10 s are freed.
- **Which links feed the scene** (`src/link_select.h`, host test
  `host/test_link_select.cpp`) — of the valid links on both boards (frame rate
  ≥ `min link rate`, measured as a 5 s EMA, leaving only below 75 % of it; full
  motion window), the best `links fused` (tune tab, 1…2×`MAX_LINKS`: each
  board holds `MAX_LINKS` links and the leader chooses among both boards') by
  quality smoothed over 5 s. A link must be valid for 10 s straight before it
  can be chosen (a router's bursty data never flickers in); a chosen link
  stays at least 20 s; another replaces it only after being 3 dB better for
  5 s without a break; a link that stops being valid leaves at once. The radio
  tab shows them as **used** (green RSSI).
- **Low-cut** — before the breathing component and both spectra, each
  subcarrier has its centered moving average over `low-cut window` (tune tab,
  default 8 s, 0 = off) subtracted. Slow intensity changes (drift, AGC/gain
  steps, fading as someone shifts nearby) otherwise dominate the variance, so
  the principal component follows them and their 1/f skirt fills the
  breathing band. Response of x − MA₈ₛ: DC removed, 1.2 bpm −28 dB, 3 bpm
  −12 dB, 7 bpm (band edge) −0.7 dB, breathing and heart bands unchanged.
- **Per frame (consumer task, integer only)** — amplitude of the 16 selected
  subcarriers in Q4; every frame of a grid slot is averaged into the 24 s
  breathing window (`DSP_DECIM_HZ`, build flag: 20 Hz, 50 ms slots; the heart
  band up to 2.5 Hz gets a 10 Hz Nyquist).
- **Motion** (after ESPectre) — per-frame spread across subcarriers (coefficient
  of variation, gain-invariant) → Hampel(7, 3 MAD) → moving variance over 100
  frames. Threshold = P95 × 1.1 of that variance during the empty-room
  calibration; hysteresis (stay while ≥ 0.7 T) and 4-of-6 / 5-of-6 votes at
  4 Hz. `motion_index` = variance / threshold (≥ 1 means moving).
- **Breathing** — only after `BREATH_IDLE_SEC` without motion, and only on the
  data recorded since motion ended (the ring may still hold the transient), so
  rates appear 12–24 s after settling. First principal component of the
  detrended subcarriers (sign-aligns them), decimated to 5 Hz, Hann-windowed,
  one bin per bpm over 0.12–0.70 Hz. The 24 s window resolves 2.5 bpm, so the
  bins are oversampled 2.5× and a peak's main lobe is ±5 bins. A peak counts if
  it is an interior local maximum ≥ `BREATH_SNR_DB` above the noise floor =
  median of the bins outside the strongest peak's lobe / ln 2 (the median of a
  noise periodogram is ln 2 × its mean; a second peak barely moves it, unlike
  a mean). Neighbours within two lobes and 25 dB below a stronger peak are its
  sidelobes. 14 dB: white noise p99.9 is 13.5 dB; the empty-room recordings
  exceed it in 0–0.7% of still ticks, which the tracker absorbs. Two people
  closer than ~5 bpm are one peak; expect "1" or "≥2", not exact counts.
- **Hypotheses vs detections** — every spectral peak ≥ the *hypothesis* SNR
  (default 8 dB, tunable) is reported and tracked ("forming" in the UI, with
  rate ± CI, SNR, confidence and age). A track is **confirmed** only after
  `confirm after` seconds with a smoothed SNR ≥ the *detection* SNR. Presence,
  breather count and heart count all come from confirmed tracks: a single
  24 s window is ~7 independent trials a minute per link, and per-tick
  detections alone would give a few false presence events an hour on noise.
  The CI is the spread of the raw per-tick rate estimates.
- **Heartbeat (experimental)** — only while still and a breathing hypothesis
  exists; same test over 0.75–2.5 Hz (45–150 bpm), hypothesis/detection at
  8/12 dB, breathing harmonics 1–8× excluded; tracked per breather with its own
  CI and confirmation. Heartbeat is 20–30 dB below breathing in amplitude CSI,
  the search runs on the *breathing* component, and the harmonic mask blanks
  60/72/75/90 bpm for ordinary breathing rates: treat any heart output as a
  hypothesis to test, not a measurement.
- **Fusion** (`fuseLinks`) — clusters breathing and heart rates across all links
  (within 2.5 bpm of a cluster's seed); each cluster is one person; a cluster's
  SNR is the **max** over its links, not the sum: both boards hear the same
  transmitters with antennas a few cm apart and the leader↔follower link is one
  reciprocal channel, so the links are not independent trials. Motion on any
  link means presence at once; breathing presence comes from the tracker.

### Run the DSP tests (host)

```sh
cd host
c++ -std=c++17 -O2 -I../src test_dsp.cpp ../src/dsp.cpp -o /tmp/td && /tmp/td
```

Covers breathing recovery (including sub-unit modulation and opposite-sign
subcarriers), two-breather fusion, heartbeat alongside breathing, motion enter/
exit with debouncing, a one-frame spike, the integer amplitude, and **no
breathers/heartbeats on noise-only windows**.

---

## Layout

```
platformio.ini        envs: node (default), verbose (CSI stream on)
sdkconfig.defaults    flash size, CSI enable, USB console, 1 kHz tick
partitions.csv        8 MB layout (app + spiffs 'storage')
src/
  config.h            pins, band/channel, DSP + vitals bands, election, telemetry
  main.cpp            calls app_run()
  app.cpp             unified flow: election -> survey -> sense -> fuse -> serve
  election.{h,cpp}    leader/follower/standalone, heartbeat, failover
  telemetry.{h,cpp}   verbose USB line protocol + CSV header + stdin commands
  radio.{h,cpp}       wifi bring-up, CSI capture, ESP-NOW illuminator, survey, AP
  dsp.{h,cpp}         motion + breathing + heartbeat detector (host-testable)
  sensing.{h,cpp}     CSI -> per-link processors, CSI telemetry hook
  link_proto.{h,cpp}  framed UART message format + CRC
  link.{h,cpp}        UART transport
  led.{h,cpp}         status LED
  scene_state.{h,cpp} thread-safe web scene + SCENE / SPECTRA encoders
  phone_http.{h,cpp}  SoftAP 10.0.0.1 + web (SPIFFS, gzip) + WebSocket views (leader)
  node_common.{h,cpp} survey->plan, feature (de)serialization
web/                  Vite + Preact dashboard (gzipped into the SPIFFS image)
host/                 uv project
  monitor.py          live diagnostics (election, CSI, vitals) + record/replay
  analyze.py          offline breathing/heart analyzer for tuning (+ --selftest)
  make_fixture.py     synthetic telemetry generator
  test_dsp.cpp        host unit tests for the DSP
```

---

## Status / next steps

Implemented and host-tested: unified firmware with runtime leader election,
channel survey + agreement, CSI capture, ESP-NOW illumination, the motion +
breathing + heartbeat DSP, multi-link fusion, verbose USB telemetry (with CSV
header + election narration), the Preact dashboard at 10.0.0.1, and the uv host
tools (`monitor.py`, `analyze.py`). Both envs build clean; DSP + analyzer verified
on synthetic data. **Not yet validated on hardware.**

First hardware milestone: flash both boards, confirm election + CSI flow in
`monitor.py` (and dual-antenna links appear), record empty/walking/breathing
scripts, tune bands/thresholds in `config.h` via `analyze.py`. Then measure
antenna isolation with/without cross-pol + separation to confirm the direct-path
suppression the whole idea rests on.

Planned (ARCHITECTURE §11): scene replication leader->follower, Betaflight-style
adaptive filters that track a found rate, and an ESP-NOW mesh of >2 nodes jointly
mapping the space.
