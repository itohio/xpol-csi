"""Raw-I/Q statistics of a recorded session (record_session.py), per link.

    uv run --with numpy python analyze_session.py data/<label>-1101-*.log [more.log ...]

For every link with >= 20 frames/s it reports RSSI, amplitude statistics and the
fraction of power in physiologically meaningful bands, for two observables:
  amp  - |H_k| per subcarrier (what the firmware uses)
  dphi - angle(H_k * conj(H_k+1)): phase between adjacent subcarriers. The
         per-frame common phase (CFO/SFO between two free-running boards)
         cancels, so this is a usable phase observable for bistatic CSI.
Series are resampled to 20 Hz on the radio clock, linearly detrended, Hann-
windowed in 24 s segments (50% overlap), and the band power is the mean over
subcarriers. Compare sessions (e.g. present vs empty) side by side.
"""
import collections
import sys

import numpy as np

FS = 20.0
SEG = int(24 * FS)
BANDS = {"breath 0.1-0.6Hz": (0.1, 0.6), "heart 0.8-2.5Hz": (0.8, 2.5),
         "motion 2.5-10Hz": (2.5, 10.0)}


def load(path):
    frames = collections.defaultdict(list)
    for line in open(path, errors="replace"):
        i = line.find(" IQ,")
        if i < 0:
            continue
        f = line[i + 4:].strip().split(",")
        if len(f) != 7:
            continue
        t_us, mac, rssi, _noise, gain, nsub, hx = f
        raw = np.frombuffer(bytes.fromhex(hx), dtype=np.int8).astype(np.float32)
        if raw.size != 2 * int(nsub):
            continue
        frames[mac].append((int(t_us), int(rssi), int(gain), raw))
    return frames


def detrend(seg):
    """Remove each column's least-squares line."""
    t = np.arange(seg.shape[0], dtype=np.float64)
    t -= t.mean()
    slope = (seg * t[:, None]).sum(0) / (t ** 2).sum()
    return seg - seg.mean(0) - np.outer(t, slope)


def band_power(x):
    """x: [time, k] at FS. Per band, averaged over k and 24 s segments:
    fraction of the 0.05-10 Hz power, and absolute variance in the band
    (Parseval on the Hann-windowed one-sided spectrum)."""
    frac = {b: [] for b in BANDS}
    var = {b: [] for b in BANDS}
    w = np.hanning(SEG)
    for s in range(0, x.shape[0] - SEG + 1, SEG // 2):
        seg = detrend(x[s:s + SEG])
        p = np.abs(np.fft.rfft(seg * w[:, None], axis=0)) ** 2
        f = np.fft.rfftfreq(SEG, 1 / FS)
        tot = p[(f >= 0.05) & (f <= 10)].sum(0) + 1e-12
        for b, (lo, hi) in BANDS.items():
            pb = p[(f >= lo) & (f < hi)].sum(0)
            frac[b].append(pb / tot)
            var[b].append(2 * pb / (SEG * (w ** 2).sum()))
    mean = lambda d: {b: float(np.mean(v)) if v else float("nan") for b, v in d.items()}
    return mean(frac), mean(var)


def breath_peak(x):
    """Strongest breathing-band line over all segments: (bpm, dB over band median)."""
    best = (0.0, 0.0)
    for s in range(0, x.shape[0] - SEG + 1, SEG // 2):
        seg = detrend(x[s:s + SEG])
        p = (np.abs(np.fft.rfft(seg * np.hanning(SEG)[:, None], n=4 * SEG, axis=0)) ** 2).mean(1)
        f = np.fft.rfftfreq(4 * SEG, 1 / FS)
        m = (f >= 0.1) & (f <= 0.6)
        k = np.argmax(p[m])
        snr = 10 * np.log10(p[m][k] / np.median(p[m]))
        if snr > best[1]:
            best = (float(f[m][k] * 60), float(snr))
    return best


def analyze(path):
    print(f"== {path}")
    for mac, fr in sorted(load(path).items(), key=lambda kv: -len(kv[1])):
        t = np.array([x[0] for x in fr], dtype=np.int64)
        t = np.where(np.diff(t, prepend=t[0]) < -2**31, t + 2**32, t)   # radio clock wrap
        dur = (t[-1] - t[0]) / 1e6
        if len(fr) < 50 or dur <= 0 or len(fr) / dur < 20:
            continue
        iq = np.stack([x[3] for x in fr])
        h = iq[:, 0::2] + 1j * iq[:, 1::2]
        amp = np.abs(h)
        used = amp.mean(0) > 0.2 * amp.mean(0).max()           # skip guard/null subcarriers
        dphi = np.angle(h[:, 1:] * np.conj(h[:, :-1]))[:, used[1:] & used[:-1]]
        amp = amp[:, used]
        # resample to a uniform 20 Hz grid (radio clock)
        g = np.arange(t[0], t[-1], 1e6 / FS)
        a20 = np.stack([np.interp(g, t, amp[:, k]) for k in range(amp.shape[1])], 1)
        p20 = np.stack([np.interp(g, t, np.unwrap(dphi[:, k])) for k in range(dphi.shape[1])], 1)
        rssi = np.array([x[1] for x in fr])
        cv_t = np.median(amp.std(0) / amp.mean(0))
        print(f"  link {mac}: {len(fr)} frames, {len(fr)/dur:.0f}/s, {used.sum()} subcarriers, "
              f"RSSI {np.median(rssi):.0f} dBm (p5 {np.percentile(rssi, 5):.0f}, p95 {np.percentile(rssi, 95):.0f}), "
              f"mean |H| {amp.mean():.1f}, temporal CV {100*cv_t:.2f}%")
        for name, x, ref, unit in (("amp ", a20, amp.mean() ** 2, "dB re |H|^2"), ("dphi", p20, 1.0, "dB re rad^2")):
            frac, var = band_power(x)
            bpm, snr = breath_peak(x)
            print(f"    {name}: " + ", ".join(f"{b} {100*frac[b]:.1f}% ({10*np.log10(var[b]/ref):.1f})" for b in BANDS)
                  + f"  [{unit}] | strongest breathing line {bpm:.1f} bpm at {snr:.1f} dB over band median")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
