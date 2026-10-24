#!/usr/bin/env python3
"""Offline CSI analyzer for tuning the breathing/heartbeat algorithms.

Reads a recorded raw telemetry log (the CSI lines), reconstructs per-link
amplitude matrices, and runs the same breathing + heartbeat detection the
firmware does, so you can tune thresholds without reflashing.

Run (uv):
    uv run python analyze.py data/raw-*.log
    uv run python analyze.py data/raw-*.log --no-plot
    uv run python analyze.py --selftest          # synthesize + verify, no hardware

Mirrors src/dsp.cpp: decimate to 20 Hz, DC-remove, Goertzel/FFT power over the
breathing band (7-42 bpm) and heart band (45-132 bpm, breathing harmonics
excluded), peak SNR = peak/median, confirmed on >=2 subcarriers.
"""
import argparse
import sys
import numpy as np

DECIM_HZ = 20.0
BREATH = (0.12, 0.70, 4.0)   # fmin Hz, fmax Hz, snr   (configurable)
HEART = (0.67, 3.34, 3.5)    # ~40-200 bpm


def load_log(path):
    rows = {}
    with open(path) as fp:
        for line in fp:
            if not line.startswith("CSI,"):
                continue
            f = line.strip().split(",")
            if len(f) < 6:
                continue
            t = int(f[1]) / 1000.0
            mac = f[2]
            try:
                amps = [float(x) for x in f[6].split()] if len(f) > 6 else []
            except ValueError:
                continue
            if amps:
                rows.setdefault(mac, []).append((t, amps))
    out = {}
    for mac, lst in rows.items():
        w = min(len(a) for _, a in lst)
        t = np.array([x for x, _ in lst])
        X = np.array([a[:w] for _, a in lst])
        out[mac] = (t, X)
    return out


def resample_uniform(t, X, fs=DECIM_HZ):
    if len(t) < 4:
        return None, None
    t = t - t[0]
    n = int(t[-1] * fs)
    if n < 8:
        return None, None
    grid = np.arange(n) / fs
    Y = np.empty((n, X.shape[1]))
    for k in range(X.shape[1]):
        Y[:, k] = np.interp(grid, t, X[:, k])
    return grid, Y


def band_peaks(Y, fs, band, exclude=None):
    """Return clustered [(bpm, snr)] confirmed on >=2 subcarriers."""
    fmin, fmax, snr_thresh = band
    n = Y.shape[0]
    if n < fs * 12:  # need >=12 s
        return []
    win = np.hanning(n)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    sel = (freqs >= fmin) & (freqs <= fmax)
    if not sel.any():
        return []
    fb = freqs[sel]
    cand = []
    # stable subcarriers only (mean amplitude above 15% of the strongest)
    means = Y.mean(axis=0)
    keep = means > 0.15 * means.max()
    for k in np.where(keep)[0]:
        col = Y[:, k] - Y[:, k].mean()
        P = np.abs(np.fft.rfft(col * win)) ** 2
        Pb = P[sel].copy()
        if exclude:
            for ef_bpm, _ in exclude:
                ef = ef_bpm / 60.0
                for h in (1, 2, 3):
                    Pb[np.abs(fb - h * ef) < 3.0 / 60.0] = 0.0
        if Pb.max() <= 0:
            continue
        i = int(np.argmax(Pb))
        med = np.median(Pb[Pb > 0]) if (Pb > 0).any() else 0
        snr = Pb[i] / med if med > 0 else 0
        if snr >= snr_thresh:
            cand.append((fb[i] * 60.0, float(snr)))
    if not cand:
        return []
    cand.sort()
    clusters, counts = [], []
    for bpm, snr in cand:
        if clusters and abs(bpm - clusters[-1][0]) <= 2.0:
            w0, w1 = clusters[-1][1], snr
            clusters[-1] = ((clusters[-1][0] * w0 + bpm * w1) / (w0 + w1), max(w0, w1))
            counts[-1] += 1
        else:
            clusters.append((bpm, snr)); counts.append(1)
    return [c for c, n in zip(clusters, counts) if n >= 2]


def detect(t, X):
    grid, Y = resample_uniform(t, X)
    if Y is None:
        return [], []
    breaths = band_peaks(Y, DECIM_HZ, BREATH)
    hearts = band_peaks(Y, DECIM_HZ, HEART, exclude=breaths)
    return breaths, hearts


def synth(bpm_breath=15.0, bpm_heart=60.0, secs=30.0, nsub=56, seed=1):
    rng = np.random.default_rng(seed)
    fs = 100.0
    n = int(secs * fs)
    t = np.arange(n) / fs
    base = 20 + 15 * ((np.arange(nsub) % 7) / 6.0)
    X = np.empty((n, nsub))
    fb, fh = bpm_breath / 60.0, bpm_heart / 60.0
    for k in range(nsub):
        ph = 0.3 * k
        X[:, k] = (base[k]
                   + base[k] * 0.08 * np.sin(2 * np.pi * fb * t + ph)
                   + base[k] * 0.02 * np.sin(2 * np.pi * fh * t + ph)
                   + rng.normal(0, 0.5, n))
    return t, np.clip(X, 0, None)


def selftest():
    ok = True
    t, X = synth(15.0, 60.0)
    breaths, hearts = detect(t, X)
    print("breaths:", [f"{b:.1f}bpm/snr{s:.1f}" for b, s in breaths])
    print("hearts :", [f"{h:.1f}bpm/snr{s:.1f}" for h, s in hearts])
    if not any(abs(b - 15.0) <= 2.0 for b, _ in breaths):
        print("FAIL: breathing 15 bpm not recovered"); ok = False
    if not any(abs(h - 60.0) <= 3.0 for h, _ in hearts):
        print("FAIL: heart 60 bpm not recovered"); ok = False
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if not args.log:
        ap.error("give a raw log or --selftest")

    links = load_log(args.log)
    if not links:
        print("no CSI rows found in", args.log); sys.exit(1)
    print(f"{len(links)} link(s)")
    for mac, (t, X) in links.items():
        breaths, hearts = detect(t, X)
        dur = t[-1] - t[0] if len(t) else 0
        print(f"\nlink {mac}: {X.shape[0]} samples, {X.shape[1]} subcarriers, {dur:.1f}s")
        print("  breathing:", [f"{b:.1f}bpm(snr{s:.1f})" for b, s in breaths] or "none")
        print("  heartbeat:", [f"{h:.1f}bpm(snr{s:.1f})" for h, s in hearts] or "none")

    if args.no_plot:
        return
    import matplotlib.pyplot as plt
    mac, (t, X) = max(links.items(), key=lambda kv: kv[1][1].shape[0])
    grid, Y = resample_uniform(t, X)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 7))
    a1.imshow(X.T, aspect="auto", origin="lower", cmap="viridis")
    a1.set_title(f"CSI amplitude {mac}"); a1.set_xlabel("sample"); a1.set_ylabel("subcarrier")
    if Y is not None:
        win = np.hanning(Y.shape[0])
        freqs = np.fft.rfftfreq(Y.shape[0], 1.0 / DECIM_HZ) * 60.0
        spec = np.zeros_like(freqs)
        for k in range(Y.shape[1]):
            spec += np.abs(np.fft.rfft((Y[:, k] - Y[:, k].mean()) * win)) ** 2
        m = (freqs >= 5) & (freqs <= 140)
        a2.plot(freqs[m], spec[m])
        a2.set_title("summed spectrum"); a2.set_xlabel("bpm"); a2.set_ylabel("power")
    plt.tight_layout(); plt.show()


if __name__ == "__main__":
    main()
