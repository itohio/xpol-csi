#!/usr/bin/env python3
"""Live diagnostic monitor for the xpol-csi sensor.

Connects over USB-C, parses the telemetry stream, shows leader election, CSI,
and detected breathing/heartbeat, and records everything for later tuning.

Run (uv):
    uv run python monitor.py /dev/cu.usbmodemXXXX
    uv run python monitor.py /dev/cu.usbmodemXXXX --record data/
    uv run python monitor.py --replay data/raw-*.log            # offline review
    uv run python monitor.py --replay data/raw-*.log --no-plot  # headless summary

Telemetry line protocol (see src/telemetry.h):
    EVT,<t_ms>,<text>
    ROLE,<t_ms>,<role>,<self_mac>,<peer_mac>,<peer_count>
    CH,<t_ms>,<channel>,<second>,<band5g>
    CSI,<t_ms>,<src_mac>,<rssi>,<noise>,<nsub>,<a0> a1 ... aN-1>
    FEAT,<t_ms>,<src_mac>,<valid>,<motion>,<is_motion>,<rate>,<nb>,<b:snr;..>,<nh>,<h:snr;..>
    SCENE,<t_ms>,<presence>,<motion>,<links>,<nb>,<b:snr;..>,<nh>,<h:snr;..>
"""
import argparse
import collections
import sys
import threading
import time

CSI_MAXLEN = 400  # waterfall history per link


def fmt_mac(h):
    return ":".join(h[i:i + 2] for i in range(0, 12, 2)) if len(h) == 12 else h


def parse_peaks(field):
    """'15.0:40.2;22.1:9.3' -> [(15.0,40.2),(22.1,9.3)]"""
    out = []
    field = field.strip()
    if not field:
        return out
    for part in field.split(";"):
        if ":" in part:
            a, b = part.split(":")
            try:
                out.append((float(a), float(b)))
            except ValueError:
                pass
    return out


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.role = "?"
        self.self_mac = ""
        self.peer_mac = ""
        self.peers = 0
        self.channel = None
        self.second = None
        self.band5g = None
        self.source = "?"
        self.header = collections.deque(maxlen=40)
        self.events = collections.deque(maxlen=40)
        self.csi = collections.defaultdict(lambda: collections.deque(maxlen=CSI_MAXLEN))
        self.rf = {}   # mac -> {amp(ema), min, rssi(ema), hist(deque)} for antenna alignment
        self.feats = {}
        self.scene = {}
        self.scene_hist = collections.deque(maxlen=600)   # (t, motion)
        self.breath_hist = collections.deque(maxlen=600)  # (t, [bpm])
        self.heart_hist = collections.deque(maxlen=600)   # (t, [bpm])
        self.t0 = time.time()


def election_reason(st):
    r = st.role
    if r == "STANDALONE":
        return "no peer heard within the election window -> standalone leader"
    if r == "LEADER":
        return f"won election: self MAC {fmt_mac(st.self_mac)} > peer {fmt_mac(st.peer_mac)}"
    if r == "FOLLOWER":
        return f"peer is leader: peer MAC {fmt_mac(st.peer_mac)} > self {fmt_mac(st.self_mac)}"
    return "electing..."


class Parser:
    def __init__(self, st):
        self.st = st

    def _comment(self, text):
        st = self.st
        with st.lock:
            st.header.append(text)
            st.events.append(("#", text))
            if text.startswith("telemetry_source="):
                st.source = text.split("=", 1)[1].split()[0]
            elif text.startswith("promoted"):
                st.source = "LEADER"
    def feed(self, line):
        line = line.rstrip("\r\n")
        if not line:
            return
        if line.startswith("#"):
            self._comment(line[1:].strip())
            return
        if "," not in line:
            return
        tag, _, rest = line.partition(",")
        f = rest.split(",")
        st = self.st
        try:
            with st.lock:
                if tag == "EVT":
                    st.events.append((f[0], ",".join(f[1:])))
                elif tag == "ROLE":
                    st.role, st.self_mac, st.peer_mac = f[1], f[2], f[3]
                    st.peers = int(f[4])
                    st.events.append((f[0], f"role={st.role} ({election_reason(st)})"))
                elif tag == "CH":
                    st.channel, st.second, st.band5g = int(f[1]), int(f[2]), int(f[3])
                elif tag == "CSI":
                    t = int(f[0]); mac = f[1]
                    rssi = int(f[2]) if len(f) > 2 else 0
                    amps = [int(x) for x in f[5].split()] if len(f) > 5 and f[5] else []
                    if amps:
                        st.csi[mac].append((t, amps))
                        mean = sum(amps) / len(amps)   # direct-path coupling proxy
                        r = st.rf.get(mac)
                        if r is None:
                            r = {"amp": mean, "min": mean, "rssi": float(rssi),
                                 "hist": collections.deque(maxlen=400)}
                            st.rf[mac] = r
                        else:
                            r["amp"] = 0.9 * r["amp"] + 0.1 * mean   # heavy EMA (hunt the null)
                            r["min"] = min(r["min"], r["amp"])
                            r["rssi"] = 0.9 * r["rssi"] + 0.1 * rssi
                        r["hist"].append(r["amp"])
                elif tag == "FEAT":
                    mac = f[1]
                    st.feats[mac] = {
                        "valid": int(f[2]), "motion": float(f[3]), "is_motion": int(f[4]),
                        "rate": float(f[5]),
                        "breaths": parse_peaks(f[7]) if len(f) > 7 else [],
                        "hearts": parse_peaks(f[9]) if len(f) > 9 else [],
                    }
                elif tag == "SCENE":
                    t = int(f[0])
                    sc = {
                        "presence": int(f[1]), "motion": float(f[2]), "links": int(f[3]),
                        "nb": int(f[4]), "breaths": parse_peaks(f[5]) if len(f) > 5 else [],
                        "nh": int(f[6]) if len(f) > 6 else 0,
                        "hearts": parse_peaks(f[7]) if len(f) > 7 else [],
                    }
                    st.scene = sc
                    rel = t / 1000.0
                    st.scene_hist.append((rel, sc["motion"]))
                    st.breath_hist.append((rel, [b for b, _ in sc["breaths"]]))
                    st.heart_hist.append((rel, [h for h, _ in sc["hearts"]]))
        except (IndexError, ValueError):
            pass


def reader_serial(ser, parser, record_fp, stop):
    buf = b""
    while not stop.is_set():
        try:
            data = ser.read(4096)
        except Exception:
            break
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", "ignore")
            if record_fp:
                record_fp.write(line + "\n")
            parser.feed(line)


def reader_file(path, parser, realtime, stop):
    with open(path) as fp:
        for line in fp:
            if stop.is_set():
                break
            parser.feed(line)
            if realtime:
                time.sleep(0.002)


def print_summary(st):
    with st.lock:
        print(f"\n== role: {st.role}  (telemetry source: {st.source}) ==  {election_reason(st)}")
        print(f"   self={fmt_mac(st.self_mac)} peer={fmt_mac(st.peer_mac)} peers={st.peers}")
        print(f"   channel={st.channel} second={st.second} band5g={st.band5g}")
        print(f"   links seen (CSI): {len(st.csi)}  " +
              ", ".join(f"{fmt_mac(m)}:{len(d)}" for m, d in st.csi.items()))
        if st.scene:
            sc = st.scene
            print(f"   scene: presence={sc['presence']} motion={sc['motion']:.3f} "
                  f"links={sc['links']} breathers={sc['nb']} hearts={sc.get('nh',0)}")
            if sc["breaths"]:
                print("   breathing: " + ", ".join(f"{b:.1f}bpm(snr{s:.1f})" for b, s in sc["breaths"]))
            if sc["hearts"]:
                print("   heartbeat: " + ", ".join(f"{h:.1f}bpm(snr{s:.1f})" for h, s in sc["hearts"]))
        if st.rf:
            print("   RF alignment (minimize coupling -> antennas orthogonal):")
            for m, r in sorted(st.rf.items(), key=lambda kv: -kv[1]["amp"]):
                iso = 20 * (r["amp"] / r["min"] - 1) if r["min"] > 0 else 0
                bar = "#" * int(min(40, r["amp"] / 2))
                print(f"     {fmt_mac(m)}: coupling={r['amp']:6.1f}  min={r['min']:6.1f}  "
                      f"rssi={r['rssi']:5.0f}dBm  {bar}")
        print("   recent events:")
        for tms, txt in list(st.events)[-6:]:
            print(f"     [{tms}] {txt}")


def run_dashboard(st, stop):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(12, 9))
    fig.canvas.manager.set_window_title("xpol-csi monitor")
    gs = GridSpec(4, 2, height_ratios=[0.8, 1.5, 1.6, 1.2], hspace=0.5, wspace=0.25)
    ax_txt = fig.add_subplot(gs[0, :]); ax_txt.axis("off")
    ax_wf = fig.add_subplot(gs[1, :])
    ax_rf = fig.add_subplot(gs[2, :])          # RF alignment (the big one)
    ax_mo = fig.add_subplot(gs[3, 0])
    ax_vi = fig.add_subplot(gs[3, 1])
    banner = ax_txt.text(0.01, 0.5, "", va="center", family="monospace", fontsize=10)

    def update(_):
        with st.lock:
            role, reason = st.role, election_reason(st)
            ch, sc = st.channel, dict(st.scene)
            links = list(st.csi.items())
            mo = list(st.scene_hist)
            bh = list(st.breath_hist)
            hh = list(st.heart_hist)
            rf = {m: (list(r["hist"]), r["amp"], r["min"], r["rssi"]) for m, r in st.rf.items()}
        txt = [f"role: {role}   {reason}",
               f"self {fmt_mac(st.self_mac)}   peer {fmt_mac(st.peer_mac)}   channel {ch}"]
        if sc:
            txt.append(f"presence={sc.get('presence')}  motion={sc.get('motion',0):.3f}  "
                       f"links={sc.get('links',0)}  breathers={sc.get('nb',0)}  hearts={sc.get('nh',0)}")
        banner.set_text("\n".join(txt))

        ax_wf.clear(); ax_wf.set_title("CSI amplitude (strongest link)")
        if links:
            mac, dq = max(links, key=lambda kv: len(kv[1]))
            rows = [a for _, a in dq]
            if rows:
                w = min(len(r) for r in rows)
                mat = np.array([r[:w] for r in rows], dtype=float).T
                ax_wf.imshow(mat, aspect="auto", origin="lower", cmap="viridis", interpolation="nearest")
                ax_wf.set_xlabel(f"time (samples)   link {fmt_mac(mac)}"); ax_wf.set_ylabel("subcarrier")

        # --- RF alignment panel ---
        ax_rf.clear()
        ax_rf.set_title("RF ALIGNMENT  —  rotate antenna to MINIMIZE coupling (cross-pol null)", fontweight="bold")
        colors = ["tab:red", "tab:blue", "tab:green", "tab:orange"]
        for i, (mac, (hist, amp, mn, rssi)) in enumerate(sorted(rf.items(), key=lambda kv: -kv[1][1])):
            c = colors[i % len(colors)]
            if hist:
                ax_rf.plot(range(len(hist)), hist, color=c, lw=1.5, label=fmt_mac(mac))
                ax_rf.axhline(mn, color=c, ls="--", lw=0.8, alpha=0.6)
            ax_rf.text(0.01, 0.92 - i * 0.14,
                       f"{fmt_mac(mac)}  coupling {amp:5.1f}   min {mn:5.1f}   rssi {rssi:5.0f} dBm",
                       transform=ax_rf.transAxes, color=c, family="monospace", fontsize=12, fontweight="bold")
        ax_rf.set_ylabel("coupling\n(lower=better)"); ax_rf.set_xlabel("recent samples")
        if rf: ax_rf.legend(loc="upper right", fontsize=8)

        ax_mo.clear(); ax_mo.set_title("motion index")
        if mo:
            ax_mo.plot([x for x, _ in mo], [y for _, y in mo], lw=1); ax_mo.set_xlabel("t (s)")
        ax_vi.clear(); ax_vi.set_title("vitals (bpm)")
        for hist, color, label in ((bh, "tab:blue", "breath"), (hh, "tab:red", "heart")):
            xs, ys = [], []
            for t, rates in hist:
                for r in rates: xs.append(t); ys.append(r)
            if xs: ax_vi.scatter(xs, ys, s=8, c=color, label=label)
        ax_vi.set_ylim(0, 200); ax_vi.set_xlabel("t (s)"); ax_vi.legend(loc="upper right", fontsize=8)
        return []

    print("DASHBOARD OPEN", flush=True)
    _anim = FuncAnimation(fig, update, interval=300, cache_frame_data=False)
    plt.show()
    stop.set()


def main():
    ap = argparse.ArgumentParser(description="xpol-csi live monitor")
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/cu.usbmodemXXXX")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay", help="read a recorded raw log instead of serial")
    ap.add_argument("--record", help="directory to save the raw stream")
    ap.add_argument("--no-plot", action="store_true", help="headless: print a summary instead of plotting")
    ap.add_argument("--realtime", action="store_true", help="replay at ~stream rate")
    args = ap.parse_args()

    st = State()
    parser = Parser(st)
    stop = threading.Event()
    record_fp = None
    ser = None

    if args.replay:
        reader = threading.Thread(target=reader_file, args=(args.replay, parser, args.realtime, stop), daemon=True)
    else:
        if not args.port:
            ap.error("give a serial port or --replay FILE")
        import serial
        # The ESP32-C5 USB-Serial/JTAG console only delivers output when the host
        # asserts DTR (terminal-present). DTR does NOT reset the board; only
        # esptool's DTR+RTS toggle sequence does. RTS stays deasserted.
        ser = serial.Serial()
        ser.port = args.port; ser.baudrate = args.baud; ser.timeout = 0.2
        ser.dtr = True; ser.rts = False
        ser.open()
        if args.record:
            import os, datetime
            os.makedirs(args.record, exist_ok=True)
            name = datetime.datetime.now().strftime("raw-%Y%m%d-%H%M%S.log")
            record_fp = open(os.path.join(args.record, name), "w")
            print(f"recording to {record_fp.name}")
        try:
            ser.write(b"s\nv\n")  # status (election) + enable the CSI stream (board is quiet by default)
        except Exception:
            pass
        reader = threading.Thread(target=reader_serial, args=(ser, parser, record_fp, stop), daemon=True)

    reader.start()

    if args.no_plot:
        try:
            if args.replay:
                reader.join()
            else:
                while True:
                    time.sleep(2)
                    print_summary(st)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
        print_summary(st)
    else:
        try:
            run_dashboard(st, stop)
        except KeyboardInterrupt:
            stop.set()

    if not args.replay and ser is not None:
        try: ser.write(b"q\n"); ser.flush()  # stop the CSI flood -> board quiet for easy reflash
        except Exception: pass
        try: ser.close()
        except Exception: pass
    if record_fp:
        record_fp.close()


if __name__ == "__main__":
    main()
