"""Relay the Unitree simulation DDS topics over a point-to-point ZMQ link.

The official simulator and deployment assume that both processes share a local
DDS domain.  This bridge preserves that assumption on each host while moving
the serialized DDS samples between hosts over TCP.  No robot fields are
translated: CycloneDDS CDR payloads are forwarded verbatim.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import signal
import struct
import threading
import time
from typing import Callable, Sequence

import zmq

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__HandState_,
    unitree_hg_msg_dds__IMUState_,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    IMUState_,
    LowCmd_,
    LowState_,
)


STATE_TOPICS = (
    ("rt/lowstate", LowState_),
    ("rt/secondary_imu", IMUState_),
    ("rt/dex3/left/state", HandState_),
    ("rt/dex3/right/state", HandState_),
)
COMMAND_TOPICS = (
    ("rt/lowcmd", LowCmd_),
    ("rt/dex3/left/cmd", HandCmd_),
    ("rt/dex3/right/cmd", HandCmd_),
)
DEFAULT_FACTORIES = {
    LowState_: unitree_hg_msg_dds__LowState_,
    IMUState_: unitree_hg_msg_dds__IMUState_,
    HandState_: unitree_hg_msg_dds__HandState_,
    LowCmd_: unitree_hg_msg_dds__LowCmd_,
    HandCmd_: unitree_hg_msg_dds__HandCmd_,
}

_MAGIC = b"GWB1"
_HEADER = struct.Struct("!4sQQB")
_LENGTH = struct.Struct("!I")


class LatestBundle:
    """Thread-safe latest-value cache populated by DDS callback threads."""

    def __init__(self, topic_types: Sequence[type]):
        self._lock = threading.Lock()
        self._parts = [DEFAULT_FACTORIES[typ]().serialize() for typ in topic_types]
        self._sequence = 0

    def callback(self, index: int) -> Callable[[object], None]:
        def update(sample: object) -> None:
            payload = sample.serialize()
            with self._lock:
                self._parts[index] = payload
                if index == 0:
                    self._sequence += 1

        return update

    def snapshot(self) -> tuple[int, list[bytes]]:
        with self._lock:
            return self._sequence, self._parts.copy()


def encode_bundle(sequence: int, parts: Sequence[bytes]) -> bytes:
    chunks = [_HEADER.pack(_MAGIC, sequence, time.time_ns(), len(parts))]
    for part in parts:
        chunks.extend((_LENGTH.pack(len(part)), part))
    return b"".join(chunks)


def decode_bundle(frame: bytes, expected_parts: int) -> tuple[int, int, list[bytes]]:
    if len(frame) < _HEADER.size:
        raise ValueError("short bridge frame")
    magic, sequence, sent_ns, part_count = _HEADER.unpack_from(frame)
    if magic != _MAGIC or part_count != expected_parts:
        raise ValueError("invalid bridge frame header")
    offset = _HEADER.size
    parts: list[bytes] = []
    for _ in range(part_count):
        if offset + _LENGTH.size > len(frame):
            raise ValueError("truncated bridge length")
        (length,) = _LENGTH.unpack_from(frame, offset)
        offset += _LENGTH.size
        end = offset + length
        if end > len(frame):
            raise ValueError("truncated bridge payload")
        parts.append(frame[offset:end])
        offset = end
    if offset != len(frame):
        raise ValueError("unexpected trailing bridge data")
    return sequence, sent_ns, parts


def configure_pub(socket: zmq.Socket) -> None:
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.setsockopt(zmq.LINGER, 0)


def configure_sub(socket: zmq.Socket) -> None:
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt(zmq.LINGER, 0)


def create_dds_publishers(topics: Sequence[tuple[str, type]]) -> list[ChannelPublisher]:
    publishers = []
    for topic, message_type in topics:
        publisher = ChannelPublisher(topic, message_type)
        publisher.Init()
        publishers.append(publisher)
    return publishers


def create_dds_subscribers(
    topics: Sequence[tuple[str, type]], cache: LatestBundle
) -> list[ChannelSubscriber]:
    subscribers = []
    for index, (topic, message_type) in enumerate(topics):
        subscriber = ChannelSubscriber(topic, message_type)
        subscriber.Init(cache.callback(index), 1)
        subscribers.append(subscriber)
    return subscribers


def damping_command() -> LowCmd_:
    command = unitree_hg_msg_dds__LowCmd_()
    for motor in command.motor_cmd:
        motor.q = 0.0
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = 0.0
        motor.kd = 8.0
    return command


@dataclass
class Stats:
    frames: int = 0
    dropped: int = 0
    last_sequence: int = 0
    started_at: float = field(default_factory=time.monotonic)

    def observe(self, sequence: int) -> None:
        if self.last_sequence and sequence > self.last_sequence + 1:
            self.dropped += sequence - self.last_sequence - 1
        self.last_sequence = sequence
        self.frames += 1

    def report(self, direction: str) -> None:
        now = time.monotonic()
        elapsed = max(now - self.started_at, 1e-6)
        print(
            f"[dds-zmq] {direction}: {self.frames / elapsed:.1f} Hz, "
            f"sequence={self.last_sequence}, skipped={self.dropped}",
            flush=True,
        )
        self.frames = 0
        self.dropped = 0
        self.started_at = now


def run_sim_side(args: argparse.Namespace, stop: threading.Event) -> None:
    state_cache = LatestBundle([typ for _, typ in STATE_TOPICS])
    state_subscribers = create_dds_subscribers(STATE_TOPICS, state_cache)
    command_publishers = create_dds_publishers(COMMAND_TOPICS)

    context = zmq.Context()
    state_socket = context.socket(zmq.PUB)
    command_socket = context.socket(zmq.SUB)
    configure_pub(state_socket)
    configure_sub(command_socket)
    state_socket.bind(f"tcp://{args.bind_host}:{args.state_port}")
    command_socket.bind(f"tcp://{args.bind_host}:{args.command_port}")
    print(
        f"[dds-zmq] sim side listening on state={args.state_port}, "
        f"command={args.command_port}, DDS={args.dds_interface}",
        flush=True,
    )

    poller = zmq.Poller()
    poller.register(command_socket, zmq.POLLIN)
    last_state_sequence = 0
    last_command_at: float | None = None
    damping_sent = False
    state_stats = Stats()
    command_stats = Stats()
    next_report = time.monotonic() + args.report_interval

    try:
        while not stop.is_set():
            state_sequence, state_parts = state_cache.snapshot()
            if state_sequence != last_state_sequence:
                state_socket.send(encode_bundle(state_sequence, state_parts), zmq.NOBLOCK)
                last_state_sequence = state_sequence
                state_stats.observe(state_sequence)

            events = dict(poller.poll(timeout=1))
            if command_socket in events:
                try:
                    sequence, _, parts = decode_bundle(
                        command_socket.recv(), len(COMMAND_TOPICS)
                    )
                    for publisher, (_, message_type), payload in zip(
                        command_publishers, COMMAND_TOPICS, parts
                    ):
                        publisher.Write(message_type.deserialize(payload))
                    last_command_at = time.monotonic()
                    damping_sent = False
                    command_stats.observe(sequence)
                except ValueError as error:
                    print(f"[dds-zmq] rejected command frame: {error}", flush=True)

            now = time.monotonic()
            if (
                last_command_at is not None
                and now - last_command_at > args.command_timeout
                and not damping_sent
            ):
                command_publishers[0].Write(damping_command())
                damping_sent = True
                print(
                    f"[dds-zmq] command timeout ({args.command_timeout:.3f}s): "
                    "published damping command",
                    flush=True,
                )
            if now >= next_report:
                state_stats.report("state tx")
                command_stats.report("command rx")
                next_report = now + args.report_interval
    finally:
        state_socket.close()
        command_socket.close()
        context.term()
        # Keep DDS entities alive for the full loop lifetime.
        del state_subscribers, command_publishers


def run_wbc_side(args: argparse.Namespace, stop: threading.Event) -> None:
    command_cache = LatestBundle([typ for _, typ in COMMAND_TOPICS])
    command_subscribers = create_dds_subscribers(COMMAND_TOPICS, command_cache)
    state_publishers = create_dds_publishers(STATE_TOPICS)

    context = zmq.Context()
    state_socket = context.socket(zmq.SUB)
    command_socket = context.socket(zmq.PUB)
    configure_sub(state_socket)
    configure_pub(command_socket)
    state_socket.connect(f"tcp://{args.peer_host}:{args.state_port}")
    command_socket.connect(f"tcp://{args.peer_host}:{args.command_port}")
    print(
        f"[dds-zmq] WBC side connected to {args.peer_host}:"
        f"{args.state_port}/{args.command_port}, DDS={args.dds_interface}",
        flush=True,
    )

    poller = zmq.Poller()
    poller.register(state_socket, zmq.POLLIN)
    last_command_sequence = 0
    state_stats = Stats()
    command_stats = Stats()
    next_report = time.monotonic() + args.report_interval

    try:
        while not stop.is_set():
            events = dict(poller.poll(timeout=1))
            if state_socket in events:
                try:
                    sequence, _, parts = decode_bundle(state_socket.recv(), len(STATE_TOPICS))
                    for publisher, (_, message_type), payload in zip(
                        state_publishers, STATE_TOPICS, parts
                    ):
                        publisher.Write(message_type.deserialize(payload))
                    state_stats.observe(sequence)
                except ValueError as error:
                    print(f"[dds-zmq] rejected state frame: {error}", flush=True)

            command_sequence, command_parts = command_cache.snapshot()
            if command_sequence != last_command_sequence:
                command_socket.send(
                    encode_bundle(command_sequence, command_parts), zmq.NOBLOCK
                )
                last_command_sequence = command_sequence
                command_stats.observe(command_sequence)

            now = time.monotonic()
            if now >= next_report:
                state_stats.report("state rx")
                command_stats.report("command tx")
                next_report = now + args.report_interval
    finally:
        state_socket.close()
        command_socket.close()
        context.term()
        del command_subscribers, state_publishers


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("sim", "wbc"), required=True)
    parser.add_argument("--peer-host", help="Simulator host IP (required for WBC role)")
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--state-port", type=int, default=5560)
    parser.add_argument("--command-port", type=int, default=5561)
    parser.add_argument("--dds-interface", default="lo")
    parser.add_argument("--command-timeout", type=float, default=0.25)
    parser.add_argument("--report-interval", type=float, default=5.0)
    args = parser.parse_args()
    if args.role == "wbc" and not args.peer_host:
        parser.error("--peer-host is required for --role wbc")
    return args


def main() -> None:
    args = parse_args()
    stop = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    ChannelFactoryInitialize(0, args.dds_interface)
    if args.role == "sim":
        run_sim_side(args, stop)
    else:
        run_wbc_side(args, stop)


if __name__ == "__main__":
    main()
