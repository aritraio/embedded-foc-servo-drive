#!/usr/bin/env python3
"""Automated electrical zero-offset calibration.

Procedure (matches firmware CALIB state):
  1. Command a stationary D-axis voltage vector (Vd=Vcal, Vq=0) so the rotor
     locks to the stator d-axis.
  2. Wait for settle, then average N encoder angle samples.
  3. theta_offset = P * theta_mech averaged (wrapped to [0, 2pi)).
  4. Emit a C header fragment for config_params / foc_core_set_theta_offset().

Offline mode (no hardware): synthesize a lock angle to validate the math:
    python3 tools/calibrate_offset.py --simulate --lock-deg 37.5 --pole-pairs 7
"""
from __future__ import annotations

import argparse
import math
import random


def wrap_2pi(a: float) -> float:
    return a % (2.0 * math.pi)


def estimate_offset(samples_rad: list[float], pole_pairs: int) -> float:
    # Circular mean of mechanical angle, then scale by P.
    sx = sum(math.cos(s) for s in samples_rad) / len(samples_rad)
    sy = sum(math.sin(s) for s in samples_rad) / len(samples_rad)
    mech = math.atan2(sy, sx)
    if mech < 0:
        mech += 2.0 * math.pi
    return wrap_2pi(pole_pairs * mech)


def main() -> int:
    ap = argparse.ArgumentParser(description="FOC zero-angle calibration")
    ap.add_argument("--pole-pairs", type=int, default=7)
    ap.add_argument("--vcal", type=float, default=3.0, help="D-axis cal voltage [V]")
    ap.add_argument("--samples", type=int, default=500)
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--lock-deg", type=float, default=30.0)
    ap.add_argument("--noise-lsb", type=float, default=1.0, help="encoder noise [LSB @14bit]")
    args = ap.parse_args()

    if args.simulate:
        lock = math.radians(args.lock_deg)
        lsb = 2.0 * math.pi / 16384.0
        samples = [lock + random.gauss(0.0, args.noise_lsb * lsb) for _ in range(args.samples)]
    else:
        # Hardware path: user pipes encoder samples (rad, one per line).
        import sys

        samples = [float(line.strip()) for line in sys.stdin if line.strip()]
        if not samples:
            print("no samples on stdin", file=sys.stderr)
            return 1
    offset = estimate_offset(samples, args.pole_pairs)
    print(f"# Electrical zero offset: {offset:.6f} rad ({math.degrees(offset):.3f} deg)")
    print(f"# Apply with: foc_core_set_theta_offset(&foc, {offset:.6f}f);")
    print(f"#define CALIBRATED_THETA_OFFSET ({offset:.6f}f)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
