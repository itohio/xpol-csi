#!/usr/bin/env python3
"""Generate a synthetic telemetry log for testing monitor.py / analyze.py
without hardware. Mirrors the firmware line protocol (src/telemetry.h).

    uv run python make_fixture.py > sample.log
"""
import sys
import numpy as np
from analyze import synth

SELF = "aabbccddee01"
PEER = "aabbccddee02"


def main():
    out = sys.stdout
    # header + election narration (this board is the leader: self > peer)
    out.write("# xpol-csi telemetry v1\n")
    out.write(f"# boot self={SELF} band=5G rate_hz=100\n")
    out.write("# schema CSI,t_ms,src_mac,rssi,noise,nsub,amp0 amp1 ...\n")
    out.write(f"# election start self={SELF} window_ms=2000\n")
    out.write(f"# election heard_peer=1 peer={PEER}\n")
    out.write(f"# election decision: self_mac({SELF}) > peer_mac({PEER}) -> this=LEADER peer=FOLLOWER\n")
    out.write(f"# telemetry_source=LEADER self={SELF} peer={PEER}\n")
    out.write(f"ROLE,0,LEADER,{SELF},{PEER},1\n")
    out.write("CH,0,149,2,1\n")

    # one link carrying 13 bpm breathing + 72 bpm heartbeat
    t, X = synth(bpm_breath=13.0, bpm_heart=72.0, secs=30.0, nsub=56, seed=3)
    step = 5  # emit at ~20 Hz (every 5th 100 Hz sample) to keep the file small
    for i in range(0, len(t), step):
        tms = int(t[i] * 1000)
        amps = " ".join(str(int(v + 0.5)) for v in X[i])
        out.write(f"CSI,{tms},{SELF},-55,-92,56,{amps}\n")

    out.write("FEAT,30000,%s,1,0.0120,0,98.0,1,13.0:22.0,1,72.0:15.0\n" % SELF)
    out.write("SCENE,30000,1,0.0120,1,1,13.0:22.0,1,72.0:15.0\n")


if __name__ == "__main__":
    main()
