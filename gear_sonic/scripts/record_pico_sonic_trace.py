#!/usr/bin/env python3
"""Passive trace recorder for the PICO -> SONIC -> MuJoCo chain.

The recorder never opens XRoboToolkit.  It only subscribes to the manager ZMQ
topics, the remote ``g1_debug`` feedback topic, and local MuJoCo odometry DDS.
Samples are appended to SQLite so a long run does not accumulate in memory.
"""

from __future__ import annotations

import argparse
import json
import queue
import signal
import socket
import sqlite3
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq

mnp.patch()

HEADER_SIZE = 1280


def unpack_pose_message(packed_data: bytes, topic: str) -> dict[str, Any]:
    """Decode the manager's topic + JSON-header + binary-array format."""
    topic_bytes = topic.encode("utf-8")
    if not packed_data.startswith(topic_bytes):
        raise ValueError(f"message does not start with {topic!r}")
    offset = len(topic_bytes)
    header_bytes = packed_data[offset : offset + HEADER_SIZE]
    header_bytes = header_bytes.split(b"\x00", 1)[0]
    header = json.loads(header_bytes.decode("utf-8"))
    dtype_map = {
        "f32": np.float32,
        "f64": np.float64,
        "i32": np.int32,
        "i64": np.int64,
        "bool": bool,
        "u8": np.uint8,
    }
    result: dict[str, Any] = {
        "version": header.get("v", 0),
        "endian": header.get("endian", "le"),
    }
    current = offset + HEADER_SIZE
    for field in header.get("fields", []):
        dtype = np.dtype(dtype_map.get(field["dtype"], np.float32))
        shape = tuple(field["shape"])
        size = int(np.prod(shape)) * dtype.itemsize
        result[field["name"]] = (
            np.frombuffer(packed_data[current : current + size], dtype=dtype)
            .reshape(shape)
            .copy()
        )
        current += size
    return result


def _scalar_int(value: Any, default: int = -1, *, last: bool = False) -> int:
    if value is None:
        return default
    arr = np.asarray(value).reshape(-1)
    if arr.size == 0:
        return default
    return int(arr[-1] if last else arr[0])


def _scalar_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    arr = np.asarray(value).reshape(-1)
    return float(arr[0]) if arr.size else default


class TraceDatabase:
    def __init__(self, path: Path, metadata: dict[str, Any]):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=NORMAL")
        self.db.execute(
            """
            CREATE TABLE IF NOT EXISTS samples (
              id INTEGER PRIMARY KEY AUTOINCREMENT,
              stream TEXT NOT NULL,
              rx_monotonic_ns INTEGER NOT NULL,
              rx_realtime_ns INTEGER NOT NULL,
              source_index INTEGER,
              source_time_ns INTEGER,
              payload BLOB NOT NULL
            )
            """
        )
        self.db.execute(
            "CREATE INDEX IF NOT EXISTS samples_stream_time "
            "ON samples(stream, rx_monotonic_ns)"
        )
        self.db.execute(
            "CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)"
        )
        for key, value in metadata.items():
            self.db.execute(
                "INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)",
                (key, json.dumps(value, ensure_ascii=False)),
            )
        self.db.commit()
        self.pending = 0
        self.last_commit = time.monotonic()

    def append(
        self,
        stream: str,
        payload: dict[str, Any],
        source_index: int = -1,
        source_time_ns: int = 0,
        rx_monotonic_ns: int | None = None,
        rx_realtime_ns: int | None = None,
    ) -> None:
        mono_ns = time.monotonic_ns() if rx_monotonic_ns is None else rx_monotonic_ns
        wall_ns = time.time_ns() if rx_realtime_ns is None else rx_realtime_ns
        blob = msgpack.packb(payload, use_bin_type=True)
        self.db.execute(
            """
            INSERT INTO samples(
              stream, rx_monotonic_ns, rx_realtime_ns,
              source_index, source_time_ns, payload
            ) VALUES (?, ?, ?, ?, ?, ?)
            """,
            (stream, mono_ns, wall_ns, source_index, source_time_ns, blob),
        )
        self.pending += 1
        now = time.monotonic()
        if self.pending >= 500 or now - self.last_commit >= 1.0:
            self.db.commit()
            self.pending = 0
            self.last_commit = now

    def close(self, metadata: dict[str, Any] | None = None) -> None:
        if metadata:
            for key, value in metadata.items():
                self.db.execute(
                    "INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)",
                    (key, json.dumps(value, ensure_ascii=False)),
                )
        self.db.commit()
        self.db.close()


class OdomSubscriber:
    def __init__(self, interface: str, output_queue: queue.SimpleQueue):
        self.error: str | None = None
        self.subscriber = None
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import OdoState_

            ChannelFactoryInitialize(0, interface)
            self.subscriber = ChannelSubscriber("rt/odostate", OdoState_)

            def callback(msg) -> None:
                output_queue.put(
                    (
                        time.monotonic_ns(),
                        time.time_ns(),
                        {
                            "tick": int(msg.tick),
                            "position": np.asarray(msg.position, dtype=np.float64),
                            "linear_velocity": np.asarray(
                                msg.linear_velocity, dtype=np.float64
                            ),
                            "orientation": np.asarray(msg.orientation, dtype=np.float64),
                            "angular_velocity": np.asarray(
                                msg.angular_velocity, dtype=np.float64
                            ),
                        },
                    )
                )

            self.subscriber.Init(callback, 10)
        except Exception as exc:  # DDS must not prevent ZMQ recording.
            self.error = f"{type(exc).__name__}: {exc}"


def _make_subscriber(ctx: zmq.Context, endpoint: str, topics: list[str]) -> zmq.Socket:
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 1000)
    sock.setsockopt(zmq.LINGER, 0)
    for topic in topics:
        sock.setsockopt_string(zmq.SUBSCRIBE, topic)
    sock.connect(endpoint)
    return sock


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Output SQLite trace")
    parser.add_argument("--label", default="manual", help="Experiment label")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds; 0 runs until signal")
    parser.add_argument("--pose-host", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=5556)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=5557)
    parser.add_argument("--dds-interface", default="lo")
    parser.add_argument("--marker-port", type=int, default=5572)
    parser.add_argument("--metadata-json", default="{}")
    parser.add_argument(
        "--ready-file",
        type=Path,
        help="Write this file after all recording sockets and DDS are initialized",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        extra_metadata = json.loads(args.metadata_json)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Invalid --metadata-json: {exc}") from exc

    started_wall_ns = time.time_ns()
    metadata = {
        "schema_version": 1,
        "label": args.label,
        "started_realtime_ns": started_wall_ns,
        "started_monotonic_ns": time.monotonic_ns(),
        "pose_endpoint": f"tcp://{args.pose_host}:{args.pose_port}",
        "state_endpoint": f"tcp://{args.state_host}:{args.state_port}",
        "dds_interface": args.dds_interface,
        "argv": sys.argv,
        **extra_metadata,
    }
    trace = TraceDatabase(args.output, metadata)

    ctx = zmq.Context()
    pose_sock = _make_subscriber(
        ctx,
        f"tcp://{args.pose_host}:{args.pose_port}",
        ["pose", "planner", "manager_state", "command"],
    )
    state_sock = _make_subscriber(
        ctx,
        f"tcp://{args.state_host}:{args.state_port}",
        ["g1_debug"],
    )
    poller = zmq.Poller()
    poller.register(pose_sock, zmq.POLLIN)
    poller.register(state_sock, zmq.POLLIN)

    marker_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    marker_sock.setblocking(False)
    marker_sock.bind(("127.0.0.1", args.marker_port))

    odom_queue: queue.SimpleQueue = queue.SimpleQueue()
    odom = OdomSubscriber(args.dds_interface, odom_queue)
    if odom.error:
        trace.db.execute(
            "INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)",
            ("dds_error", json.dumps(odom.error)),
        )
        trace.db.commit()

    stop = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    counts: Counter[str] = Counter()
    start = time.monotonic()
    last_report = start
    if args.ready_file is not None:
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.write_text(
            json.dumps(
                {
                    "ready_realtime_ns": time.time_ns(),
                    "ready_monotonic_ns": time.monotonic_ns(),
                    "dds_error": odom.error,
                },
                ensure_ascii=False,
            )
            + "\n",
            encoding="utf-8",
        )
    print(f"[trace] recording to {args.output}")
    print(
        f"[trace] pose=tcp://{args.pose_host}:{args.pose_port}, "
        f"state=tcp://{args.state_host}:{args.state_port}, DDS={args.dds_interface}"
    )

    try:
        while not stop:
            if args.duration > 0 and time.monotonic() - start >= args.duration:
                break

            events = dict(poller.poll(timeout=20))
            if pose_sock in events:
                raw = pose_sock.recv()
                rx_mono = time.monotonic_ns()
                rx_wall = time.time_ns()
                topic = next(
                    (
                        name
                        for name in ("manager_state", "command", "planner", "pose")
                        if raw.startswith(name.encode())
                    ),
                    None,
                )
                if topic is not None:
                    try:
                        data = unpack_pose_message(raw, topic=topic)
                    except Exception as exc:
                        data = {"decode_error": f"{type(exc).__name__}: {exc}"}
                    source_index = _scalar_int(
                        data.get("frame_index"), default=-1, last=True
                    )
                    source_time_ns = _scalar_int(
                        data.get("pico_timestamp_ns"), default=0
                    )
                    trace.append(
                        topic,
                        data,
                        source_index,
                        source_time_ns,
                        rx_mono,
                        rx_wall,
                    )
                    counts[topic] += 1

            if state_sock in events:
                raw = state_sock.recv()
                rx_mono = time.monotonic_ns()
                rx_wall = time.time_ns()
                try:
                    data = msgpack.unpackb(raw[len(b"g1_debug") :], raw=False)
                except Exception as exc:
                    data = {"decode_error": f"{type(exc).__name__}: {exc}"}
                source_index = int(data.get("index", -1))
                ros_time = float(data.get("ros_timestamp", 0.0) or 0.0)
                trace.append(
                    "g1_debug",
                    data,
                    source_index,
                    int(ros_time * 1e9) if ros_time > 0 else 0,
                    rx_mono,
                    rx_wall,
                )
                counts["g1_debug"] += 1

            while True:
                try:
                    rx_mono, rx_wall, data = odom_queue.get_nowait()
                except queue.Empty:
                    break
                trace.append(
                    "odostate",
                    data,
                    int(data["tick"]),
                    0,
                    rx_mono,
                    rx_wall,
                )
                counts["odostate"] += 1

            while True:
                try:
                    marker, _addr = marker_sock.recvfrom(8192)
                except BlockingIOError:
                    break
                label = marker.decode("utf-8", errors="replace")
                trace.append("marker", {"label": label})
                counts["marker"] += 1
                print(f"[trace] marker: {label}")

            now = time.monotonic()
            if now - last_report >= 5.0:
                elapsed = max(now - start, 1e-9)
                rates = ", ".join(
                    f"{key}={value / elapsed:.1f}Hz" for key, value in sorted(counts.items())
                )
                print(f"[trace] {elapsed:.1f}s {rates or 'waiting for streams'}")
                last_report = now
    finally:
        ended_wall_ns = time.time_ns()
        pose_sock.close()
        state_sock.close()
        marker_sock.close()
        ctx.term()
        trace.close(
            {
                "ended_realtime_ns": ended_wall_ns,
                "duration_sec": time.monotonic() - start,
                "counts": dict(counts),
            }
        )

    print(f"[trace] complete: {dict(counts)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
