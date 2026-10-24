"""Record a labelled session from both boards: raw I/Q dump + all telemetry.

    uv run python record_session.py <label> <seconds> [--port NAME=/dev/...]...

Writes data/<label>-<role>-<HHMMSS>.log per board (role = leader/follower as the
board reports it; USB port names can swap between re-enumerations), every line
prefixed with the host receive time in seconds. Sends 'r' (raw I/Q on) at start
and 'q' at the end.
Line formats: see the '# schema' header the firmware prints and src/telemetry.cpp.
"""
import argparse
import pathlib
import time

import serial

DEFAULT_PORTS = {"1101": "/dev/cu.usbmodem1101", "101": "/dev/cu.usbmodem101"}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("seconds", type=float)
    ap.add_argument("--port", action="append", default=[], help="NAME=/dev/cu.usbmodemX")
    args = ap.parse_args()
    ports = dict(p.split("=", 1) for p in args.port) or DEFAULT_PORTS

    out_dir = pathlib.Path(__file__).parent / "data"
    out_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%H%M%S")
    sers, files, bufs, counts = {}, {}, {}, {}
    for name, dev in ports.items():
        s = serial.Serial(dev, 115200, timeout=0)
        s.dtr = True
        s.rts = False
        sers[name] = s
        files[name] = open(out_dir / f"{args.label}-{name}-{stamp}.log", "w")
        bufs[name], counts[name] = b"", 0
        s.write(b"s\nr\n")   # status (role) + raw dump on

    roles = {}
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            for name, s in sers.items():
                bufs[name] += s.read(65536)
                *lines, bufs[name] = bufs[name].split(b"\n")
                now = time.time() - t0
                for line in lines:
                    files[name].write(f"{now:.3f} {line.decode(errors='replace').rstrip()}\n")
                    counts[name] += line.startswith(b"IQ,")
                    if name not in roles and line.startswith(b"ROLE,"):
                        roles[name] = line.split(b",")[2].decode().lower()
            time.sleep(0.01)
    finally:
        for name, s in sers.items():
            s.write(b"q\n")
            s.close()
            files[name].close()
    for name in ports:
        path = pathlib.Path(files[name].name)
        if name in roles:
            path = path.rename(path.with_name(f"{args.label}-{roles[name]}-{stamp}.log"))
        print(f"{name} ({roles.get(name, 'role unknown')}): {counts[name]} IQ frames -> {path}")


if __name__ == "__main__":
    main()
