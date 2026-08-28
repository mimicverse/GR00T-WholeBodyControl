#!/usr/bin/env python3
"""Estimate wall-clock offset and RTT between the local and GPU hosts."""

from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import numpy as np
import zmq


def run_server(bind: str, port: int, duration: float) -> int:
    ctx = zmq.Context()
    socket = ctx.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(f"tcp://{bind}:{port}")
    stop = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    start = time.monotonic()
    print(f"[clock-probe] server tcp://{bind}:{port}", flush=True)
    try:
        while not stop and (duration <= 0 or time.monotonic() - start < duration):
            if not socket.poll(100):
                continue
            request = socket.recv_json()
            server_rx_wall_ns = time.time_ns()
            server_rx_mono_ns = time.monotonic_ns()
            server_tx_wall_ns = time.time_ns()
            server_tx_mono_ns = time.monotonic_ns()
            socket.send_json(
                {
                    "sequence": int(request.get("sequence", -1)),
                    "server_rx_wall_ns": server_rx_wall_ns,
                    "server_tx_wall_ns": server_tx_wall_ns,
                    "server_rx_monotonic_ns": server_rx_mono_ns,
                    "server_tx_monotonic_ns": server_tx_mono_ns,
                }
            )
    finally:
        socket.close()
        ctx.term()
    return 0


def run_client(host: str, port: int, count: int, interval: float, output: Path) -> int:
    ctx = zmq.Context()
    socket = ctx.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVTIMEO, 2000)
    socket.setsockopt(zmq.SNDTIMEO, 2000)
    socket.connect(f"tcp://{host}:{port}")
    samples = []
    try:
        for sequence in range(count):
            local_tx_wall_ns = time.time_ns()
            local_tx_mono_ns = time.monotonic_ns()
            socket.send_json(
                {
                    "sequence": sequence,
                    "local_tx_wall_ns": local_tx_wall_ns,
                    "local_tx_monotonic_ns": local_tx_mono_ns,
                }
            )
            reply = socket.recv_json()
            local_rx_mono_ns = time.monotonic_ns()
            local_rx_wall_ns = time.time_ns()
            t1 = local_tx_wall_ns
            t2 = int(reply["server_rx_wall_ns"])
            t3 = int(reply["server_tx_wall_ns"])
            t4 = local_rx_wall_ns
            rtt_ns = (t4 - t1) - (t3 - t2)
            offset_ns = ((t2 - t1) + (t3 - t4)) / 2.0
            samples.append(
                {
                    "sequence": sequence,
                    "local_tx_wall_ns": t1,
                    "server_rx_wall_ns": t2,
                    "server_tx_wall_ns": t3,
                    "local_rx_wall_ns": t4,
                    "local_elapsed_ns": local_rx_mono_ns - local_tx_mono_ns,
                    "rtt_ns": rtt_ns,
                    "offset_remote_minus_local_ns": offset_ns,
                }
            )
            time.sleep(interval)
    except zmq.ZMQError as exc:
        print(f"[clock-probe] failed: {exc}")
        return 2
    finally:
        socket.close()
        ctx.term()

    rtt = np.asarray([sample["rtt_ns"] for sample in samples], dtype=float)
    best_index = int(np.argmin(rtt))
    best = samples[best_index]
    result = {
        "method": "NTP four-timestamp estimator over ZMQ; best/min-RTT sample",
        "host": host,
        "port": port,
        "count": len(samples),
        "offset_remote_minus_local_ns": best["offset_remote_minus_local_ns"],
        "uncertainty_bound_ns": best["rtt_ns"] / 2.0,
        "min_rtt_ms": float(np.min(rtt) * 1e-6),
        "median_rtt_ms": float(np.median(rtt) * 1e-6),
        "p95_rtt_ms": float(np.percentile(rtt, 95) * 1e-6),
        "samples": samples,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(
        "[clock-probe] offset(remote-local)="
        f"{result['offset_remote_minus_local_ns'] * 1e-6:.3f}ms, "
        f"uncertainty<= {result['uncertainty_bound_ns'] * 1e-6:.3f}ms, "
        f"min RTT={result['min_rtt_ms']:.3f}ms"
    )
    print(f"[clock-probe] wrote {output}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--server", action="store_true")
    mode.add_argument("--client", action="store_true")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5570)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--interval", type=float, default=0.02)
    parser.add_argument("--output", type=Path, default=Path("clock_probe.json"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.server:
        return run_server(args.bind, args.port, args.duration)
    return run_client(args.host, args.port, args.count, args.interval, args.output)


if __name__ == "__main__":
    raise SystemExit(main())
