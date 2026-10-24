# Experiment state — 2026-10-21

**Status: inconclusive.** The two-board sensor records clean CSI, but in the
first valid overnight recordings neither sleeper is distinguishable from the
empty room. The likely cause is weak direct-path isolation of the reworked
antenna (20–22 dB vs 31–35 dB for the previous antenna after alignment). More
recordings are needed after the antenna is re-aligned.

## Setup

| item | state |
|---|---|
| boards | 2 × Seeed XIAO ESP32-C5, ESP-IDF 5.5.5 (PlatformIO), 5 V rails joined (either USB cable powers both) |
| boards' MACs | `10bda3cef81c` (usbmodem101, usually leader), `10bda3cedaac` (usbmodem1101, usually follower); the leader is elected at boot |
| antennas | cross-polarized pair, **reworked on the night of Oct 19→20**; same placement since |
| isolation (path loss between boards) | **20–22 dB** with the current antenna (every log since Oct 20 00:50, on ch 44 and 153, at 2 and 14 dBm) |
| firmware | `step4_leader` / `step4_follower` (SoftAP build): leader runs SoftAP `xpol-csi` at 10.0.0.1, never joins a router |
| channel | the leader picks the quietest surveyed channel at boot (by design); 153 in all valid recordings |
| TX power | 14 dBm on both boards at boot (`TX_POWER_DBM`) |
| illumination | both boards, ESP-NOW broadcast, 100 Hz, HT20 MCS0 |
| sensing | recordings below: `MAX_LINKS=3` per board, 20 Hz breathing grid, breath detect 14 dB. Branch `psram-long-history` (not yet on the boards): 16 link slots per board in PSRAM, 3 feed the scene by default, plus a 10 min per-link history with a long-window breathing analysis (README, DSP) |

## What changed in the last two days

- **Frozen CSI trigger found:** a board whose STA is associated to the home
  router gets a frozen CSI buffer (byte-identical payload for every frame)
  within 40–120 s of power-on, with or without its SoftAP. Bisected over
  power-ons on Oct 20; the follower never associates and never froze. Once
  latched only a power cut clears it (unplug both cables). Network is now one or
  the other per build: `step4_*` = SoftAP, `debug_*` = router (gives a frozen
  leader receiver; not for sensing). Details: README, "Frozen CSI".
- Boot TX power restored, 14 dBm default; leader illumination restored to 100 Hz
  (it had been turned off on evidence confounded by the router join).
- Web: one WebSocket per tab with view-specific messages, status footer (CPU,
  heap, TX, illumination, CSI rate, frozen flag) on every tab, browser-built
  histories; links-per-board cap. Protocols documented in README.
- `host/align.py`: live per-board received level and path loss for antenna
  alignment.

## Recordings (`host/data/`)

All valid: 600.0 s each, both boards live (0 % repeated frames), 14 dBm, both
illuminating at 100 Hz, channel 153, boards not rebooted between sessions.

| session | local start | condition |
|---|---|---|
| `kitchen-walk-14dbm-ap` | 10-21 00:15 | user at the table, left for the kitchen after a few minutes. **Leader illumination was off** in this one: the follower heard only the leader's beacons (10/s) |
| `sleep-two-1-14dbm-ap` | 10-21 00:37 | two people asleep (user + wife) |
| `sleep-two-2-14dbm-ap` | 10-21 01:02 | same |
| `sleep-two-3-14dbm-ap` | 10-21 01:27 | same |
| `empty-room-14dbm-ap` | 10-21 09:52 | room empty |

Files: `<session>-{leader,follower}-HHMMSS.log`, one `IQ,t_us,mac,rssi,noise,gain_q8,nsub,fmt,hex`
line per frame, host time prefix, `ROLE` and `# radio` provenance lines at the
start. Links present: leader ← follower and follower ← leader ESP-NOW at
100/s; follower ← leader SoftAP beacons at 10/s; nothing else useful on ch 153.

Older recordings are in `host/void-recordings/` and are not usable for
sensing: leader CSI frozen (Oct 19 23:07 onward), the previous antenna, or the
old log format.

Not recorded: the sleepers' positions and distance from the antennas, exact
times of the kitchen walk, any ground truth (breathing belt, video).

## Findings

Received level: leader −8 dBm, follower −6 dBm, constant to the dB all night;
receiver gain at its lowest step; mean CSI amplitude ≈ 27.5 (8-bit). Frame-to-
frame fluctuation ≈ 0.7 amplitude units (quantization + thermal noise).

| test (replay of firmware DSP, or offline analysis) | empty | sleep 1 / 2 / 3 | kitchen (user present) |
|---|---|---|---|
| firmware motion variance, P99.9 (leader link) | 3.95e-5 | 3.92 / 4.05 / 4.17e-5 | 4.28e-5 |
| firmware breath peak ≥ 14 dB, share of ticks (leader / follower) | 0.0 / 0.0 % | ≤ 0.3 % | 0.9 % |
| PC1 breathing peak, 60 s windows, median dB | 7.6 | 7.1–8.1 | 9.1 (up to 17.3) |
| 10-min averaged spectrum vs empty, < 20 bpm | — | +0 / +0.5–1 / +0.5–1 dB | +0.9 to +1.5 dB |
| same, 48–120 bpm (outside breathing) | — | ±0.2 / +0.5–0.7 / +0.6–0.7 dB | +0.2 dB |
| long window (firmware DSP replay, `--long`), strongest peak, max dB: 120 s (leader / follower) | 10.7 / 9.5 | 10.3–10.9 / 9.0–11.7 | 11.4 (leader) |
| same, 300 s windows | 8.9 / 9.5 | 9.7–10.9 / 9.3–11.3 | 11.6 |

- **Sleepers: not detected.** No breathing line in any sleep session. Sleep 2
  and 3 sit slightly above the empty room across all bands (broadband, not
  breathing: likely noise or environment).
- **A person moving near the boards (kitchen session): detected**, as
  low-frequency energy and as motion around minute 5.
- One probable turn-over in sleep 3 (both receivers flag motion around minutes
  5–7).
- **Long window (replayed Oct 22):** no peak reaches the 14 dB default in any
  session, or 12 dB; the empty room peaks at 10.7 dB, the sleep sessions at
  11.7 dB (follower, sleep 1, 12.3–12.5 bpm). It does not separate the sleepers
  either: the evidence still points at isolation, not window length. The 120 s
  windows are mostly reached; 300 s ones only occasionally (motion restarts the
  window) and 600 s never in 10 min sessions. One empty-room session is thin
  evidence for the long-window threshold.

### Isolation history

| antenna | period | path loss |
|---|---|---|
| previous, before alignment | Oct 19 19:22–19:55 | ~20 dB |
| previous, after alignment by flexing the box | Oct 19 22:37–23:55 | 31–35 dB (user could see himself moving in CSI traces) |
| reworked (current) | Oct 20 00:50 → now | 20–22 dB |

## Calibration candidates (provisional, from one empty-room session)

Motion threshold sweep (replay with votes and hysteresis, % of ticks in motion):

| link | threshold | empty | sleep 1 / 2 / 3 | kitchen |
|---|---|---|---|---|
| leader ← follower | 3.45e-5 | 1.6 % | 0.3 / 3.0 / 3.1 % | 19.7 % |
| leader ← follower | **3.60e-5** | 0.0 % | 0.0 / 1.5 / 1.8 % | 9.8 % |
| leader ← follower | 3.75e-5 | 0.0 % | 0.0 / 0.0 / 0.8 % | 1.5 % |
| follower ← leader | 3.60e-5 | 1.5 % | 0.6 / 0.7 / 3.0 % | — |
| follower ← leader | **3.75e-5** | 0.0 % | 0.3 / 0.7 / 3.0 % | — |

- The firmware's rule (P95 × 1.1 over 20 s) yields 3.1–3.9e-5 depending on the
  20 s it sees; the low end gives up to ~10 % false motion in the empty room.
  The P99 over the full 10 min (leader 3.55e-5, follower 3.75e-5) lands on the
  good values: a ~60 s calibration with P99 is the candidate rule. Not applied.
- Breath detect: keep 14 dB (0 empty-room ticks above it; 12 dB gives ~0.7 %).
- These thresholds belong to this antenna, placement, channel and TX power;
  any change invalidates them. The channel can change at every boot (quietest
  channel), and the stored NVS calibration is per link, not per channel.
- The stored NVS calibration on the leader is stale (made on ch 44 at a
  different TX power); it made the live scene report constant motion during the
  kitchen session. The raw recordings are unaffected.

## Known deviations and open items

- **A/B channel design not implemented.** Specified on Oct 19: leader on channel
  A (AP/phone) listening for CSI on B, follower illuminating B and listening on
  A. It was dropped on Oct 19 without agreement; both boards share one channel.
  One radio per board means it needs off-channel visits: `esp_wifi_remain_on_channel`
  (leader visits B ~20–30 ms per 100 ms, AP stays on A) and
  `esp_now_switch_channel_tx` (follower bursts on B, UART-synced). Unproven:
  CSI delivery during remain-on-channel on the C5, and whether that path
  triggers the freeze.
- Console `t` on the follower does not stick: the leader re-applies the
  follower's TX every 2 s (set it from the web radio tab).
- The firmware's motion statistic (1 s variance of the cross-subcarrier
  spread) barely separates anything at this isolation; a slower statistic
  (10 s std of the 5 Hz series, drift > 30 s removed) separated the kitchen
  presence better but also responds to broadband changes.

## Next steps

1. Re-align the antenna with `uv run python host/align.py 0 2` until the path
   loss is ≥ 30 dB on both boards.
2. Power-cycle both boards, check the status footer (no `CSI FROZEN`).
3. Record an empty room (10 min), then occupied sessions with ground truth:
   note positions and distances, and the times of any movement; a short
   at-the-table control with deliberate breathing at a known rate.
4. Re-run the analysis below; set motion thresholds and the calibration rule
   (≈60 s, P99) from the new empty-room data.
5. If sleepers are still invisible: test board separation (one board on each
   side of the bed), then decide on the A/B remain-on-channel prototype.

## Reproducing the analysis

```sh
cd host
c++ -std=c++17 -O2 -I../src replay.cpp ../src/dsp.cpp -o /tmp/replay
/tmp/replay data/empty-room-14dbm-ap-leader-095206.log                        # per-tick motion / breaths
/tmp/replay data/kitchen-walk-14dbm-ap-leader-001527.log --thr 10bda3cedaac=3.6e-5   # threshold per transmitter MAC
```

Per-link liveness: every `IQ` payload of a link must differ from the previous
one; repeated payloads mean a frozen CSI engine.
