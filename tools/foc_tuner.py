#!/usr/bin/env python3
"""FOC Tuner: live scope for COBS+CRC32 telemetry from sil_main (or UART).

Wire format per packet_protocol.h / telemetry.h:
    frame = COBS(payload32 + CRC32_LE) + 0x00
    payload32 = <I timestamp_ms, 7x float32 LE: id,iq,id_ref,iq_ref,theta_e,omega_m,vbus>

Usage:
    ./build/sil_main 2>/dev/null | python3 tools/foc_tuner.py --stdin
    python3 tools/foc_tuner.py --stdin --fft
    python3 tools/foc_tuner.py --port /dev/tty.usbserial-0001 --baud 115200

Zero dropped packets is verified by monotonic timestamp_ms @ 1 kHz.
"""
from __future__ import annotations

import argparse
import struct
import sys


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            raise ValueError("zero byte inside COBS stream")
        for _ in range(1, code):
            if i >= n:
                raise ValueError("COBS overrun")
            out.append(data[i])
            i += 1
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def crc32_ieee(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


PAYLOAD = struct.Struct("<Ifffffff")
N_FIELDS = ("ts_ms", "id", "iq", "id_ref", "iq_ref", "theta_e", "omega_m", "vbus")


def decode_frame(cobs: bytes):
    raw = cobs_decode(cobs)
    if len(raw) != 36:
        raise ValueError(f"bad length {len(raw)}")
    payload, crc_le = raw[:32], raw[32:]
    (crc,) = struct.unpack("<I", crc_le)
    if crc32_ieee(payload) != crc:
        raise ValueError("CRC mismatch")
    return PAYLOAD.unpack(payload)


def stream_stdin(args) -> int:
    buf = bytearray()
    n_ok = n_bad = 0
    last_ts = None
    drops = 0
    series: dict[str, list] = {k: [] for k in N_FIELDS}
    data = sys.stdin.buffer.read() if args.file is None else open(args.file, "rb").read()
    for byte in data:
        if byte == 0x00:
            if buf:
                try:
                    rec = decode_frame(bytes(buf))
                    n_ok += 1
                    for k, v in zip(N_FIELDS, rec):
                        series[k].append(v)
                    if last_ts is not None and rec[0] != last_ts + 1:
                        drops += max(0, rec[0] - last_ts - 1)
                    last_ts = rec[0]
                except ValueError as e:
                    n_bad += 1
                    print(f"[warn] {e}", file=sys.stderr)
                buf.clear()
        else:
            buf.append(byte)
    print(f"frames ok={n_ok} bad={n_bad} timestamp_drops={drops}")
    if n_ok == 0:
        return 1
    # Text scope: last-sample summary + ASCII sparkline of iq.
    iq = series["iq"]
    print(f"iq: n={len(iq)} last={iq[-1]:.3f}A min={min(iq):.3f} max={max(iq):.3f}")
    idq = series["id"]
    print(f"id: last={idq[-1]:.3f}A min={min(idq):.3f} max={max(idq):.3f}")
    w = series["omega_m"]
    print(f"omega_m: last={w[-1]:.2f} rad/s")
    if args.ascii:
        width = 60
        lo, hi = min(iq), max(iq)
        span = (hi - lo) or 1.0
        for v in iq[:: max(1, len(iq) // 24)]:
            col = int((v - lo) / span * (width - 1))
            print(" " * col + "*")
    if args.fft:
        try:
            import numpy as np

            x = np.asarray(iq, dtype=float)
            x = x - x.mean()
            spec = np.abs(np.fft.rfft(x))
            freqs = np.fft.rfftfreq(len(x), d=1.0 / 1000.0)
            peak = freqs[int(np.argmax(spec[1:])) + 1] if len(spec) > 2 else 0.0
            print(f"FFT peak (excl. DC): {peak:.1f} Hz, bins={len(spec)}")
        except ImportError:
            print("[warn] numpy not installed; skipping FFT", file=sys.stderr)
    if args.plot:
        try:
            import matplotlib.pyplot as plt

            t = [v / 1000.0 for v in series["ts_ms"]]
            _, ax = plt.subplots(2, 1, sharex=True)
            ax[0].plot(t, series["id"], label="id")
            ax[0].plot(t, series["iq"], label="iq")
            ax[0].plot(t, series["iq_ref"], "--", label="iq_ref")
            ax[0].legend()
            ax[0].set_ylabel("A")
            ax[1].plot(t, series["omega_m"], label="omega_m")
            ax[1].legend()
            ax[1].set_ylabel("rad/s")
            plt.xlabel("s")
            plt.show()
        except ImportError:
            print("[warn] matplotlib not installed", file=sys.stderr)
    return 0 if n_bad == 0 and drops == 0 else 2


def main() -> int:
    ap = argparse.ArgumentParser(description="FOC live tuner / telemetry scope")
    ap.add_argument("--stdin", action="store_true", help="read COBS stream from stdin/file")
    ap.add_argument("--file", default=None, help="read stream from file instead of stdin")
    ap.add_argument("--port", default=None, help="serial port (requires pyserial)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--fft", action="store_true")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--ascii", action="store_true", default=True)
    args = ap.parse_args()
    if args.port:
        try:
            import serial  # type: ignore
        except ImportError:
            print("pyserial required: pip install -r tools/requirements.txt", file=sys.stderr)
            return 1
        import tempfile, os

        with serial.Serial(args.port, args.baud, timeout=5) as s, tempfile.NamedTemporaryFile(
            delete=False
        ) as f:
            f.write(s.read(1_000_000))
            args.file = f.name
        rc = stream_stdin(args)
        os.unlink(args.file)
        return rc
    return stream_stdin(args)


if __name__ == "__main__":
    raise SystemExit(main())
