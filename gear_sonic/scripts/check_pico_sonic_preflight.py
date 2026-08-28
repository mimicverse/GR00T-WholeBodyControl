#!/usr/bin/env python3
"""Read-only preflight gate for an already-running PICO/SONIC/MuJoCo chain."""

from __future__ import annotations

import argparse
import json
import queue
import time
from collections import Counter
from pathlib import Path

import numpy as np
import zmq

from record_pico_sonic_trace import OdomSubscriber, unpack_pose_message

MODE_NAMES = {
    0: "OFF",
    1: "POSE",
    2: "PLANNER",
    3: "PLANNER_FROZEN_UPPER_BODY",
    4: "POSE_PAUSE",
    5: "PLANNER_VR_3PT",
}
MODE_VALUES = {name: value for value, name in MODE_NAMES.items()}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-mode", choices=sorted(MODE_VALUES), required=True)
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--pose-host", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=5556)
    parser.add_argument("--dds-interface", default="lo")
    parser.add_argument("--z-min", type=float, default=0.60)
    parser.add_argument("--z-max", type=float, default=1.00)
    parser.add_argument("--fall-z", type=float, default=0.50)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    expected = MODE_VALUES[args.expected_mode]
    ctx = zmq.Context()
    state_sock = ctx.socket(zmq.SUB)
    state_sock.setsockopt(zmq.LINGER, 0)
    state_sock.setsockopt_string(zmq.SUBSCRIBE, "manager_state")
    state_sock.connect(f"tcp://{args.pose_host}:{args.pose_port}")

    odom_queue: queue.SimpleQueue = queue.SimpleQueue()
    odom = OdomSubscriber(args.dds_interface, odom_queue)
    modes: list[int] = []
    z_values: list[float] = []
    speeds: list[float] = []
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            if state_sock.poll(timeout=20, flags=zmq.POLLIN):
                raw = state_sock.recv()
                try:
                    payload = unpack_pose_message(raw, "manager_state")
                    value = np.asarray(payload.get("stream_mode", [])).reshape(-1)
                    if value.size:
                        modes.append(int(value[-1]))
                except Exception:
                    pass
            while True:
                try:
                    _mono, _wall, payload = odom_queue.get_nowait()
                except queue.Empty:
                    break
                position = np.asarray(payload["position"], dtype=float).reshape(-1)
                velocity = np.asarray(payload["linear_velocity"], dtype=float).reshape(-1)
                if position.size >= 3:
                    z_values.append(float(position[2]))
                if velocity.size >= 3:
                    speeds.append(float(np.linalg.norm(velocity[:3])))
    finally:
        state_sock.close()
        ctx.term()

    counts = Counter(modes)
    expected_ratio = counts[expected] / len(modes) if modes else 0.0
    z_median = float(np.median(z_values)) if z_values else None
    z_min_seen = float(np.min(z_values)) if z_values else None
    speed_p95 = float(np.percentile(speeds, 95)) if speeds else None
    failures: list[str] = []
    if odom.error:
        failures.append(f"DDS initialization failed: {odom.error}")
    if len(modes) < 10:
        failures.append(f"manager_state samples too few: {len(modes)}")
    elif expected_ratio < 0.95:
        observed = {MODE_NAMES.get(key, str(key)): value for key, value in counts.items()}
        failures.append(
            f"expected {args.expected_mode} >=95%, observed {observed} "
            f"(ratio={expected_ratio:.3f})"
        )
    if len(z_values) < 10:
        failures.append(f"odostate samples too few: {len(z_values)}")
    else:
        if not args.z_min <= z_median <= args.z_max:
            failures.append(
                f"base z median {z_median:.4f} outside [{args.z_min}, {args.z_max}]"
            )
        if z_min_seen < args.fall_z:
            failures.append(
                f"base z minimum {z_min_seen:.4f} below fall threshold {args.fall_z}"
            )

    result = {
        "pass": not failures,
        "expected_mode": args.expected_mode,
        "duration_sec": time.monotonic() - started,
        "manager_state_samples": len(modes),
        "mode_counts": {MODE_NAMES.get(key, str(key)): value for key, value in counts.items()},
        "expected_mode_ratio": expected_ratio,
        "odostate_samples": len(z_values),
        "base_z_median_m": z_median,
        "base_z_min_m": z_min_seen,
        "base_speed_p95_mps": speed_p95,
        "dds_error": odom.error,
        "failures": failures,
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
