#!/usr/bin/env python3
"""
Synthetic flight telemetry generator for Elevon FBW test harness.
Generates structured flight mission profiles covering departure, climb, cruise,
and approach phases.
"""

import math
import json
import os

def generate_profile(duration_seconds=60, sample_rate_hz=100):
    total_samples = int(duration_seconds * sample_rate_hz)
    dt = 1.0 / sample_rate_hz
    profile = []

    for i in range(total_samples):
        t = i * dt
        # Smooth flight phases
        if t < 15.0:
            phase = "TAKEOFF_CLIMB"
            cas_kt = 120.0 + (t / 15.0) * 130.0
            alt_m = 100.0 + (t / 15.0) * 1500.0
            pitch_deg = 8.0
            roll_deg = 0.0
            nz = 1.15
        elif t < 40.0:
            phase = "ENROUTE_CLIMB"
            progress = (t - 15.0) / 25.0
            cas_kt = 250.0 + progress * 40.0
            alt_m = 1600.0 + progress * 8400.0
            pitch_deg = 4.5
            roll_deg = 0.0
            nz = 1.0
        elif t < 55.0:
            phase = "CRUISE_COORDINATED_TURN"
            cas_kt = 290.0
            alt_m = 10000.0
            pitch_deg = 1.8
            roll_deg = 25.0 * math.sin((t - 40.0) / 15.0 * math.pi)
            nz = 1.0 / max(0.5, math.cos(math.radians(roll_deg)))
        else:
            phase = "DESCENT_APPROACH"
            progress = (t - 55.0) / 5.0
            cas_kt = 290.0 - progress * 80.0
            alt_m = 10000.0 - progress * 2000.0
            pitch_deg = -1.5
            roll_deg = 0.0
            nz = 0.95

        profile.append({
            "timestamp_us": int(t * 1000000),
            "phase": phase,
            "cas_knots": round(cas_kt, 2),
            "altitude_m": round(alt_m, 2),
            "pitch_deg": round(pitch_deg, 2),
            "roll_deg": round(roll_deg, 2),
            "nz_g": round(nz, 3)
        })

    return profile

def main():
    profile = generate_profile(60, 100)
    out_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "build"))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "synthetic_flight_profile.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(profile[:500], f, indent=2) # Save sample preview
    print(f"Generated {len(profile)} flight profile samples. Preview saved to {out_path}")

if __name__ == "__main__":
    main()
