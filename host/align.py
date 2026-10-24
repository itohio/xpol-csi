"""Live antenna alignment: each board's received level from the other board.

    uv run python align.py [seconds] [leader_tx_dbm]   (default: until Ctrl-C, TX unchanged)

Turns the raw I/Q dump on and prints one line per second: for each receiving
board, the median RSSI of the other board's illumination frames (ESP-NOW, HT
MCS0) and the path loss = the other board's TX power - RSSI. The direct path is
what the cross-polarized antennas must suppress, so higher path loss is better:
the old aligned antenna reached 31-35 dB; below ~25 dB the body reflection sinks
into the 8-bit CSI. Near 0 dBm RSSI the receiver compresses, so align at a low
TX power. leader_tx_dbm goes to the leader only: the leader sets the follower's
TX itself (web radio tab) and re-applies it every 2 s. Not persisted: a reboot
restores TX_POWER_DBM.
"""
import re
import statistics
import sys
import time

import serial

PORTS = {"101": "/dev/cu.usbmodem101", "1101": "/dev/cu.usbmodem1101"}
ILLUM_FMT = "32"   # (RX_BB_FORMAT_HT << 4) | MCS0: the boards' ESP-NOW frames


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else float("inf")
    leader_tx = f"t{int(sys.argv[2])}\n".encode() if len(sys.argv) > 2 else b""
    sers, bufs, rssi, self_mac, role, tx_dbm = {}, {}, {}, {}, {}, {}
    for name, dev in PORTS.items():
        s = serial.Serial(dev, 115200, timeout=0)
        s.dtr, s.rts = True, False
        s.write(b"s\nr\n")   # status (role, own MAC, TX power) + raw dump on
        sers[name], bufs[name], rssi[name] = s, b"", []
    tx_sent = not leader_tx
    t0 = next_print = time.time() + 1
    next_status = t0 + 1
    try:
        while time.time() - t0 < seconds:
            for name, s in sers.items():
                bufs[name] += s.read(65536)
                *lines, bufs[name] = bufs[name].split(b"\n")
                for raw in lines:
                    line = raw.decode(errors="replace")
                    if line.startswith("ROLE,"):
                        p = line.split(",")
                        role[name], self_mac[name] = p[2], p[3]
                        if not tx_sent and p[2] == "LEADER":
                            s.write(leader_tx + b"s\n")
                            tx_sent = True
                    elif "tx_power_dbm=" in line:
                        tx_dbm[name] = float(re.search(r"tx_power_dbm=([\d.]+)", line).group(1))
                    elif line.startswith("IQ,"):
                        p = line.split(",")
                        if len(p) == 9 and p[7] == ILLUM_FMT and p[2] != self_mac.get(name):
                            rssi[name].append(int(p[3]))
            now = time.time()
            if now >= next_status:   # the leader re-applies the follower's TX every 2 s: keep TX readings current
                next_status += 2
                for s in sers.values():
                    s.write(b"s\n")
            if now >= next_print:
                next_print += 1
                cols = []
                for name in PORTS:
                    peer = next(n for n in PORTS if n != name)
                    r = rssi[name]
                    if not r:
                        cols.append(f"{name:>4} {role.get(name, '?')[:1]}: no frames")
                        continue
                    med = statistics.median(r)
                    loss = f"{tx_dbm[peer] - med:5.1f} dB loss (peer TX {tx_dbm[peer]:4.1f})" if peer in tx_dbm else "    ? dB loss"
                    bar = "#" * max(0, min(40, int((med + 60) * 2 / 3)))   # -60..0 dBm; longer = stronger direct path
                    cols.append(f"{name:>4} {role.get(name, '?')[:1]}: {med:6.1f} dBm {loss} {len(r):3d} fr {bar:<40}")
                    rssi[name] = []
                print(time.strftime("%H:%M:%S"), " | ".join(cols), flush=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        for s in sers.values():
            s.write(b"q\n")
            s.close()


if __name__ == "__main__":
    main()
