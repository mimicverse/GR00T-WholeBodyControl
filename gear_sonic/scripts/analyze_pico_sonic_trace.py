#!/usr/bin/env python3
"""Analyze a SQLite trace recorded by ``record_pico_sonic_trace.py``."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sqlite3
from pathlib import Path
from typing import Any, Iterable

import msgpack
import msgpack_numpy as mnp
import numpy as np

mnp.patch()

BODY_JOINT_NAMES = [
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
]

BODY_XML_JOINT_NAMES = [f"{name}_joint" for name in BODY_JOINT_NAMES]


def percentile(values: np.ndarray, q: float) -> float | None:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, q)) if values.size else None


def scalar(value: Any, default: float = math.nan, *, last: bool = False) -> float:
    if value is None:
        return default
    arr = np.asarray(value).reshape(-1)
    if not arr.size:
        return default
    return float(arr[-1] if last else arr[0])


def array(value: Any, size: int | None = None) -> np.ndarray | None:
    if value is None:
        return None
    out = np.asarray(value, dtype=float).reshape(-1)
    if size is not None and out.size != size:
        return None
    return out


def finite_or_none(value: Any) -> Any:
    """Recursively replace non-finite NumPy/Python numbers for strict JSON."""
    if isinstance(value, dict):
        return {key: finite_or_none(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_or_none(item) for item in value]
    if isinstance(value, np.ndarray):
        return finite_or_none(value.tolist())
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def fmt(value: Any, digits: int = 2, suffix: str = "") -> str:
    if value is None:
        return "N/A"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "N/A"
    return f"{number:.{digits}f}{suffix}"


def quat_geodesic_deg(q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    q0 = q0 / np.maximum(np.linalg.norm(q0, axis=-1, keepdims=True), 1e-12)
    q1 = q1 / np.maximum(np.linalg.norm(q1, axis=-1, keepdims=True), 1e-12)
    dot = np.clip(np.abs(np.sum(q0 * q1, axis=-1)), 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(dot))


def load_trace(path: Path) -> tuple[dict[str, Any], dict[str, list[dict[str, Any]]]]:
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    metadata = {
        key: json.loads(value) for key, value in db.execute("SELECT key, value FROM metadata")
    }
    streams: dict[str, list[dict[str, Any]]] = {}
    rows = db.execute(
        """
        SELECT stream, rx_monotonic_ns, rx_realtime_ns,
               source_index, source_time_ns, payload
        FROM samples ORDER BY rx_monotonic_ns
        """
    )
    for stream, rx_mono, rx_wall, source_index, source_time, payload in rows:
        try:
            decoded = msgpack.unpackb(payload, raw=False)
        except Exception as exc:
            decoded = {"decode_error": f"{type(exc).__name__}: {exc}"}
        streams.setdefault(stream, []).append(
            {
                "rx_monotonic_ns": int(rx_mono),
                "rx_realtime_ns": int(rx_wall),
                "source_index": int(source_index if source_index is not None else -1),
                "source_time_ns": int(source_time if source_time is not None else 0),
                "data": decoded,
            }
        )
    db.close()
    return metadata, streams


def stream_stats(rows: list[dict[str, Any]], expected_index_step: int = 1) -> dict[str, Any]:
    result: dict[str, Any] = {"samples": len(rows)}
    if len(rows) < 2:
        return result
    times = np.asarray([row["rx_monotonic_ns"] for row in rows], dtype=np.float64) * 1e-9
    intervals_ms = np.diff(times) * 1000.0
    duration = float(times[-1] - times[0])
    result.update(
        {
            "duration_sec": duration,
            "rate_hz": (len(rows) - 1) / duration if duration > 0 else None,
            "interval_ms_p50": percentile(intervals_ms, 50),
            "interval_ms_p95": percentile(intervals_ms, 95),
            "interval_ms_p99": percentile(intervals_ms, 99),
            "interval_ms_max": percentile(intervals_ms, 100),
            "interval_jitter_ms_std": float(np.std(intervals_ms)),
        }
    )
    indices = np.asarray([row["source_index"] for row in rows], dtype=np.int64)
    indices = indices[indices >= 0]
    if indices.size >= 2:
        deltas = np.diff(indices)
        positive = deltas[deltas > 0]
        result["source_index_first"] = int(indices[0])
        result["source_index_last"] = int(indices[-1])
        result["source_index_nonmonotonic"] = int(np.sum(deltas <= 0))
        if positive.size:
            result["source_index_gap_count"] = int(np.sum(positive > expected_index_step))
            result["source_index_missing_estimate"] = int(
                np.sum(np.maximum(positive // expected_index_step - 1, 0))
            )
    return result


def estimate_lag_regular(
    target: np.ndarray,
    measured: np.ndarray,
    sample_period_sec: float,
    max_lag_sec: float = 0.5,
) -> tuple[float | None, float | None]:
    """Return non-negative lag and correlation for measured(t)=target(t-lag)."""
    target = np.asarray(target, dtype=float)
    measured = np.asarray(measured, dtype=float)
    valid = np.isfinite(target) & np.isfinite(measured)
    target = target[valid]
    measured = measured[valid]
    if target.size < 20 or np.std(target) < 1e-5 or np.std(measured) < 1e-5:
        return None, None
    max_lag = min(int(max_lag_sec / sample_period_sec), target.size // 3)
    best_lag = 0
    best_corr = -2.0
    for lag in range(max_lag + 1):
        x = target[: target.size - lag or None]
        y = measured[lag:]
        if x.size < 10:
            continue
        corr = float(np.corrcoef(x, y)[0, 1])
        if np.isfinite(corr) and corr > best_corr:
            best_corr = corr
            best_lag = lag
    return best_lag * sample_period_sec, best_corr


def estimate_lag_irregular(
    target_time: np.ndarray,
    target: np.ndarray,
    measured_time: np.ndarray,
    measured: np.ndarray,
    max_lag_sec: float = 0.4,
    grid_hz: float = 50.0,
) -> tuple[float | None, float | None]:
    target_time = np.asarray(target_time, dtype=float)
    measured_time = np.asarray(measured_time, dtype=float)
    target = np.asarray(target, dtype=float)
    measured = np.asarray(measured, dtype=float)
    if min(target_time.size, measured_time.size) < 20:
        return None, None
    start = max(target_time[0], measured_time[0]) + max_lag_sec
    end = min(target_time[-1], measured_time[-1])
    if end - start < 1.0:
        return None, None
    grid = np.arange(start, end, 1.0 / grid_hz)
    measured_grid = np.interp(grid, measured_time, measured)
    best_lag = 0.0
    best_corr = -2.0
    for lag in np.arange(0.0, max_lag_sec + 0.5 / grid_hz, 1.0 / grid_hz):
        target_grid = np.interp(grid - lag, target_time, target)
        if np.std(target_grid) < 1e-5 or np.std(measured_grid) < 1e-5:
            continue
        corr = float(np.corrcoef(target_grid, measured_grid)[0, 1])
        if np.isfinite(corr) and corr > best_corr:
            best_corr = corr
            best_lag = float(lag)
    return (best_lag, best_corr) if best_corr > -1.5 else (None, None)


def manager_sample_age(rows: list[dict[str, Any]]) -> dict[str, Any]:
    ages_ms = []
    for row in rows:
        source_mono = scalar(row["data"].get("timestamp_monotonic"))
        if np.isfinite(source_mono) and source_mono > 0:
            ages_ms.append(row["rx_monotonic_ns"] * 1e-6 - source_mono * 1000.0)
    arr = np.asarray(ages_ms, dtype=float)
    return {
        "description": "manager sample timestamp -> passive local subscriber; not full PICO E2E",
        "samples": int(arr.size),
        "p50_ms": percentile(arr, 50),
        "p95_ms": percentile(arr, 95),
        "p99_ms": percentile(arr, 99),
        "max_ms": percentile(arr, 100),
    }


def device_timing(rows: list[dict[str, Any]]) -> dict[str, Any]:
    stamps = np.asarray(
        [int(row["source_time_ns"]) for row in rows if int(row["source_time_ns"]) > 0],
        dtype=np.int64,
    )
    if stamps.size < 2:
        return {"samples": int(stamps.size), "note": "pico_timestamp_ns absent in this trace"}
    delta_ms = np.diff(stamps).astype(float) * 1e-6
    delta_ms = delta_ms[delta_ms > 0]
    return {
        "samples": int(stamps.size),
        "device_rate_hz": 1000.0 / float(np.median(delta_ms)) if delta_ms.size else None,
        "interval_ms_p50": percentile(delta_ms, 50),
        "interval_ms_p95": percentile(delta_ms, 95),
        "interval_ms_p99": percentile(delta_ms, 99),
        "interval_jitter_ms_std": float(np.std(delta_ms)) if delta_ms.size else None,
    }


def stack_field(rows: list[dict[str, Any]], key: str, size: int) -> np.ndarray:
    values = []
    for row in rows:
        value = array(row["data"].get(key), size)
        if value is not None and np.isfinite(value).all():
            values.append(value)
    return np.asarray(values, dtype=float)


def state_accuracy(rows: list[dict[str, Any]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    targets = []
    measured = []
    velocities = []
    valid_times = []
    base_target = []
    base_measured = []
    hand_errors: dict[str, list[np.ndarray]] = {"left": [], "right": []}
    for row in rows:
        target = array(row["data"].get("body_q_target"), 29)
        actual = array(row["data"].get("body_q_measured"), 29)
        velocity = array(row["data"].get("body_dq"), 29)
        if target is not None and actual is not None:
            targets.append(target)
            measured.append(actual)
            velocities.append(velocity if velocity is not None else np.full(29, np.nan))
            valid_times.append(row["rx_monotonic_ns"] * 1e-9)
        qt = array(row["data"].get("base_quat_target"), 4)
        qm = array(row["data"].get("base_quat_measured"), 4)
        if qt is not None and qm is not None:
            base_target.append(qt)
            base_measured.append(qm)
        for side in ("left", "right"):
            action = array(row["data"].get(f"last_{side}_hand_action"), 7)
            hand = array(row["data"].get(f"{side}_hand_q_measured"), 7)
            if action is not None and hand is not None:
                hand_errors[side].append(action - hand)

    summary: dict[str, Any] = {"samples": len(targets)}
    joint_rows: list[dict[str, Any]] = []
    if targets:
        target_arr = np.asarray(targets)
        measured_arr = np.asarray(measured)
        error = target_arr - measured_arr
        dt = np.diff(np.asarray(valid_times))
        sample_period = float(np.median(dt)) if dt.size else 0.02
        for idx, name in enumerate(BODY_JOINT_NAMES):
            lag, corr = estimate_lag_regular(
                target_arr[:, idx], measured_arr[:, idx], sample_period
            )
            joint_rows.append(
                {
                    "index": idx,
                    "joint": name,
                    "target_range_deg": float(np.degrees(np.ptp(target_arr[:, idx]))),
                    "mae_deg": float(np.degrees(np.mean(np.abs(error[:, idx])))),
                    "rmse_deg": float(np.degrees(np.sqrt(np.mean(error[:, idx] ** 2)))),
                    "p95_deg": float(np.degrees(np.percentile(np.abs(error[:, idx]), 95))),
                    "max_deg": float(np.degrees(np.max(np.abs(error[:, idx])))),
                    "lag_ms": lag * 1000.0 if lag is not None else None,
                    "lag_correlation": corr,
                }
            )
        active = [row for row in joint_rows if row["target_range_deg"] >= 3.0]
        summary.update(
            {
                "all_joint_rmse_deg": float(np.degrees(np.sqrt(np.mean(error**2)))),
                "all_joint_abs_error_p95_deg": float(
                    np.degrees(np.percentile(np.abs(error), 95))
                ),
                "active_joint_count": len(active),
                "active_joint_lag_ms_median": percentile(
                    np.asarray([row["lag_ms"] for row in active if row["lag_ms"] is not None]),
                    50,
                ),
                "body_dq_abs_p95_rad_s": percentile(np.abs(np.asarray(velocities)), 95),
                "body_dq_abs_max_rad_s": percentile(np.abs(np.asarray(velocities)), 100),
            }
        )
    if base_target:
        orientation_error = quat_geodesic_deg(
            np.asarray(base_target), np.asarray(base_measured)
        )
        summary["base_orientation_error_deg_p50"] = percentile(orientation_error, 50)
        summary["base_orientation_error_deg_p95"] = percentile(orientation_error, 95)
    for side, values in hand_errors.items():
        if values:
            err = np.asarray(values)
            summary[f"{side}_hand_rmse_deg"] = float(
                np.degrees(np.sqrt(np.mean(err**2)))
            )
            summary[f"{side}_hand_abs_error_p95_deg"] = float(
                np.degrees(np.percentile(np.abs(err), 95))
            )
    return summary, joint_rows


def odom_accuracy(rows: list[dict[str, Any]]) -> dict[str, Any]:
    pos = stack_field(rows, "position", 3)
    vel = stack_field(rows, "linear_velocity", 3)
    quat = stack_field(rows, "orientation", 4)
    result: dict[str, Any] = {"samples": int(pos.shape[0])}
    if pos.size:
        result.update(
            {
                "base_z_min_m": float(np.min(pos[:, 2])),
                "base_z_median_m": float(np.median(pos[:, 2])),
                "base_z_max_m": float(np.max(pos[:, 2])),
                "xyz_range_m": np.ptp(pos, axis=0).tolist(),
                "fallen_samples_z_lt_0_5m": int(np.sum(pos[:, 2] < 0.5)),
            }
        )
    if vel.size:
        speed = np.linalg.norm(vel, axis=1)
        result["speed_p95_m_s"] = percentile(speed, 95)
        result["speed_max_m_s"] = percentile(speed, 100)
    if quat.size:
        w, x, y, z = quat.T
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        result["roll_abs_p95_deg"] = percentile(np.degrees(np.abs(roll)), 95)
        result["pitch_abs_p95_deg"] = percentile(np.degrees(np.abs(pitch)), 95)
    return result


def pose_to_feedback_lag(
    pose_rows: list[dict[str, Any]], state_rows: list[dict[str, Any]]
) -> dict[str, Any]:
    pose_samples = []
    for row in pose_rows:
        value = array(row["data"].get("vr_position"), 9)
        if value is not None:
            pose_samples.append((row["rx_monotonic_ns"] * 1e-9, value))
    state_samples = []
    for row in state_rows:
        value = array(row["data"].get("vr_3point_position"), 9)
        if value is not None:
            state_samples.append((row["rx_monotonic_ns"] * 1e-9, value))
    if len(pose_samples) < 20 or len(state_samples) < 20:
        return {"note": "insufficient pose/feedback VR samples"}
    pt = np.asarray([sample[0] for sample in pose_samples])
    pv = np.asarray([sample[1] for sample in pose_samples])
    st = np.asarray([sample[0] for sample in state_samples])
    sv = np.asarray([sample[1] for sample in state_samples])
    axes = []
    for idx in range(9):
        if np.ptp(pv[:, idx]) < 0.02:
            continue
        lag, corr = estimate_lag_irregular(pt, pv[:, idx], st, sv[:, idx])
        if lag is not None and corr is not None and corr >= 0.5:
            axes.append({"axis": idx, "lag_ms": lag * 1000.0, "correlation": corr})
    return {
        "description": "local pose publish observation -> returned g1_debug VR target; includes two Wi-Fi directions",
        "active_axes": axes,
        "lag_ms_median": percentile(np.asarray([item["lag_ms"] for item in axes]), 50),
        "lag_ms_p95": percentile(np.asarray([item["lag_ms"] for item in axes]), 95),
    }


def rows_between(
    rows: list[dict[str, Any]], start_ns: int, end_ns: int
) -> list[dict[str, Any]]:
    return [
        row
        for row in rows
        if start_ns <= row["rx_monotonic_ns"] < end_ns
    ]


def segment_metrics(
    streams: dict[str, list[dict[str, Any]]]
) -> list[dict[str, Any]]:
    markers = sorted(streams.get("marker", []), key=lambda row: row["rx_monotonic_ns"])
    if not markers:
        return []
    all_times = [
        row["rx_monotonic_ns"]
        for rows in streams.values()
        for row in rows
    ]
    if not all_times:
        return []
    trace_end = max(all_times) + 1
    output = []
    for index, marker in enumerate(markers):
        start_ns = marker["rx_monotonic_ns"]
        end_ns = (
            markers[index + 1]["rx_monotonic_ns"]
            if index + 1 < len(markers)
            else trace_end
        )
        if end_ns <= start_ns:
            continue
        pose_rows = rows_between(streams.get("pose", []), start_ns, end_ns)
        state_rows = rows_between(streams.get("g1_debug", []), start_ns, end_ns)
        odom_rows = rows_between(streams.get("odostate", []), start_ns, end_ns)
        state, _ = state_accuracy(state_rows)
        odom = odom_accuracy(odom_rows)
        pose_stats = stream_stats(pose_rows)
        state_stats = stream_stats(state_rows)
        lag = pose_to_feedback_lag(pose_rows, state_rows)
        output.append(
            {
                "segment": marker["data"].get("label", f"marker-{index}"),
                "start_monotonic_ns": start_ns,
                "duration_sec": (end_ns - start_ns) * 1e-9,
                "pose_samples": len(pose_rows),
                "pose_rate_hz": pose_stats.get("rate_hz"),
                "state_samples": len(state_rows),
                "state_rate_hz": state_stats.get("rate_hz"),
                "odom_samples": len(odom_rows),
                "joint_rmse_deg": state.get("all_joint_rmse_deg"),
                "active_joint_lag_ms": state.get("active_joint_lag_ms_median"),
                "pose_feedback_lag_ms": lag.get("lag_ms_median"),
                "base_z_min_m": odom.get("base_z_min_m"),
                "roll_abs_p95_deg": odom.get("roll_abs_p95_deg"),
                "pitch_abs_p95_deg": odom.get("pitch_abs_p95_deg"),
                "fallen_samples": odom.get("fallen_samples_z_lt_0_5m"),
            }
        )
    return output


def safety_timing(streams: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    stop_rows = [
        row
        for row in streams.get("command", [])
        if _scalar_bool(row["data"].get("stop"))
    ]
    result: dict[str, Any] = {"stop_command_count": len(stop_rows)}
    if not stop_rows:
        result["note"] = "no STOP command observed"
        return result
    stop_ns = stop_rows[0]["rx_monotonic_ns"]
    result["first_stop_rx_monotonic_ns"] = stop_ns

    safety_markers = [
        row
        for row in streams.get("marker", [])
        if "stop" in str(row["data"].get("label", "")).lower()
    ]
    if safety_markers:
        marker_ns = safety_markers[0]["rx_monotonic_ns"]
        result["stop_marker_to_command_ms"] = (stop_ns - marker_ns) * 1e-6

    pose_before = [
        row["rx_monotonic_ns"]
        for row in streams.get("pose", [])
        if row["rx_monotonic_ns"] <= stop_ns
    ]
    if pose_before:
        result["last_pose_to_stop_command_ms"] = (stop_ns - max(pose_before)) * 1e-6

    for name in ("g1_debug", "odostate"):
        after = [
            row["rx_monotonic_ns"]
            for row in streams.get(name, [])
            if row["rx_monotonic_ns"] >= stop_ns
        ]
        if after:
            result[f"stop_to_last_{name}_ms"] = (max(after) - stop_ns) * 1e-6
    return result


def _scalar_bool(value: Any) -> bool:
    if value is None:
        return False
    values = np.asarray(value).reshape(-1)
    return bool(values[0]) if values.size else False


def fk_task_error(
    state_rows: list[dict[str, Any]], model_xml: Path, max_samples: int = 1000
) -> dict[str, Any]:
    try:
        import mujoco
    except ImportError:
        return {"note": "mujoco unavailable; FK task-space analysis skipped"}
    if not model_xml.is_file():
        return {"note": f"model XML missing: {model_xml}"}

    valid = []
    for row in state_rows:
        q = array(row["data"].get("body_q_measured"), 29)
        target_pos = array(row["data"].get("vr_3point_position"), 9)
        target_orn = array(row["data"].get("vr_3point_orientation"), 12)
        measured_base_quat = array(row["data"].get("base_quat_measured"), 4)
        if q is not None and target_pos is not None and np.linalg.norm(target_pos) > 1e-6:
            if measured_base_quat is None:
                measured_base_quat = np.asarray([1.0, 0.0, 0.0, 0.0])
            valid.append(
                (q, target_pos.reshape(3, 3), target_orn, measured_base_quat)
            )
    if not valid:
        return {"samples": 0, "note": "no valid VR target samples"}
    if len(valid) > max_samples:
        pick = np.linspace(0, len(valid) - 1, max_samples).astype(int)
        valid = [valid[i] for i in pick]

    model = mujoco.MjModel.from_xml_path(str(model_xml))
    data = mujoco.MjData(model)
    joint_addresses = [
        int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
        for name in BODY_XML_JOINT_NAMES
    ]
    body_names = ["left_wrist_yaw_link", "right_wrist_yaw_link", "torso_link"]
    body_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in body_names
    ]
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    offsets = np.asarray([[0.18, -0.025, 0.0], [0.18, 0.025, 0.0], [0.0, 0.0, 0.35]])
    pos_errors = []
    orn_errors = []
    for q, target_pos, target_orn, measured_base_quat in valid:
        data.qpos[:] = 0.0
        measured_base_quat = measured_base_quat / max(
            np.linalg.norm(measured_base_quat), 1e-12
        )
        data.qpos[3:7] = measured_base_quat
        for address, value in zip(joint_addresses, q, strict=True):
            data.qpos[address] = value
        mujoco.mj_forward(model, data)
        pelvis_pos = data.xpos[pelvis_id].copy()
        pelvis_rot = data.xmat[pelvis_id].reshape(3, 3).copy()
        points = []
        quats = []
        for body_id, offset in zip(body_ids, offsets, strict=True):
            body_rot = data.xmat[body_id].reshape(3, 3)
            world_point = data.xpos[body_id] + body_rot @ offset
            # g1_debug publishes the target as a heading/world-aligned vector
            # from the base.  Compare it with the FK vector in the same axes.
            points.append(world_point - pelvis_pos)
            body_quat = data.xquat[body_id].copy()
            quats.append(body_quat)
        pos_errors.append(np.linalg.norm(np.asarray(points) - target_pos, axis=1))
        if target_orn is not None:
            orn_errors.append(
                quat_geodesic_deg(np.asarray(quats), target_orn.reshape(3, 4))
            )
    pe = np.asarray(pos_errors)
    result: dict[str, Any] = {
        "description": "VR task targets vs FK of measured 29-DoF state; not external PICO ground truth",
        "samples": int(pe.shape[0]),
    }
    point_labels = ["left_wrist", "right_wrist", "head_torso"]
    for idx, label in enumerate(point_labels):
        result[f"{label}_position_rmse_m"] = float(np.sqrt(np.mean(pe[:, idx] ** 2)))
        result[f"{label}_position_p95_m"] = percentile(pe[:, idx], 95)
    if orn_errors:
        oe = np.asarray(orn_errors)
        for idx, label in enumerate(point_labels):
            result[f"{label}_orientation_rmse_deg"] = float(
                np.sqrt(np.mean(oe[:, idx] ** 2))
            )
            result[f"{label}_orientation_p95_deg"] = percentile(oe[:, idx], 95)
    return result


def write_joint_csv(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    rows = list(rows)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_markers(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["rx_monotonic_ns", "rx_realtime_ns", "label"])
        for row in rows:
            writer.writerow(
                [
                    row["rx_monotonic_ns"],
                    row["rx_realtime_ns"],
                    row["data"].get("label", ""),
                ]
            )


def write_segment_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def markdown(summary: dict[str, Any]) -> str:
    streams = summary["streams"]
    state = summary.get("state_accuracy", {})
    odom = summary.get("odometry", {})
    lines = [
        "# PICO + SONIC + MuJoCo trace analysis",
        "",
        f"- Trace: `{summary['trace']}`",
        f"- Label: `{summary.get('metadata', {}).get('label', '')}`",
        "",
        "## Stream quality",
        "",
        "| Stream | Samples | Rate Hz | P95 interval ms | Missing estimate |",
        "|---|---:|---:|---:|---:|",
    ]
    for name, data in streams.items():
        lines.append(
            f"| {name} | {data.get('samples', 0)} | "
            f"{fmt(data.get('rate_hz'))} | "
            f"{fmt(data.get('interval_ms_p95'))} | "
            f"{data.get('source_index_missing_estimate', '')} |"
        )
    lines.extend(
        [
            "",
            "## Latency",
            "",
            f"- Manager sample -> local trace P50/P95: "
            f"{fmt(summary.get('manager_sample_age', {}).get('p50_ms'))} / "
            f"{fmt(summary.get('manager_sample_age', {}).get('p95_ms'))} ms.",
            f"- Pose -> returned WBC target lag median/P95: "
            f"{fmt(summary.get('pose_to_feedback_lag', {}).get('lag_ms_median'))} / "
            f"{fmt(summary.get('pose_to_feedback_lag', {}).get('lag_ms_p95'))} ms.",
            "- These software values do not include display scan-out. Motion-to-photon requires a high-speed camera.",
            "",
            "## Tracking and stability",
            "",
            f"- 29-DoF target/measured RMSE: {fmt(state.get('all_joint_rmse_deg'))} deg.",
            f"- Active-joint lag median: {fmt(state.get('active_joint_lag_ms_median'))} ms.",
            f"- Base orientation target error P95: {fmt(state.get('base_orientation_error_deg_p95'))} deg.",
            f"- Base z min/median/max: {fmt(odom.get('base_z_min_m'), 3)} / "
            f"{fmt(odom.get('base_z_median_m'), 3)} / {fmt(odom.get('base_z_max_m'), 3)} m.",
            f"- Fallen samples (z < 0.5 m): {odom.get('fallen_samples_z_lt_0_5m', 'N/A')}.",
            "",
            "## Interpretation constraints",
            "",
            "- `Streaming data mean delay` and local receive age are not PICO-to-display E2E latency.",
            "- FK task error compares official VR task targets with robot state; it is not external PICO absolute accuracy.",
            "- Absolute PICO accuracy requires an AprilTag/optical-motion-capture ground truth rig.",
        ]
    )
    segments = summary.get("segments", [])
    if segments:
        lines.extend(
            [
                "",
                "## Marker segments",
                "",
                "| Segment | Duration s | Pose Hz | State Hz | Joint RMSE deg | Joint lag ms | z min m | Fallen |",
                "|---|---:|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for row in segments:
            lines.append(
                f"| {row['segment']} | {fmt(row.get('duration_sec'), 1)} | "
                f"{fmt(row.get('pose_rate_hz'))} | {fmt(row.get('state_rate_hz'))} | "
                f"{fmt(row.get('joint_rmse_deg'))} | {fmt(row.get('active_joint_lag_ms'))} | "
                f"{fmt(row.get('base_z_min_m'), 3)} | {row.get('fallen_samples', 'N/A')} |"
            )
    safety = summary.get("safety_timing", {})
    if safety.get("stop_command_count", 0):
        lines.extend(
            [
                "",
                "## Safety timing",
                "",
                f"- STOP commands observed: {safety.get('stop_command_count')}.",
                f"- Stop marker -> STOP command: {fmt(safety.get('stop_marker_to_command_ms'))} ms.",
                f"- Last PICO pose -> STOP command: {fmt(safety.get('last_pose_to_stop_command_ms'))} ms.",
                f"- STOP command -> last WBC feedback: {fmt(safety.get('stop_to_last_g1_debug_ms'))} ms.",
            ]
        )
    return "\n".join(lines) + "\n"


def self_test() -> int:
    dt = 0.02
    time_axis = np.arange(0.0, 20.0, dt)
    target = np.sin(2 * np.pi * 0.5 * time_axis) + 0.2 * np.sin(2 * np.pi * time_axis)
    expected = 0.08
    measured = np.interp(time_axis - expected, time_axis, target, left=target[0])
    lag, corr = estimate_lag_regular(target, measured, dt)
    print(f"self-test expected={expected * 1000:.1f}ms measured={lag * 1000:.1f}ms corr={corr:.4f}")
    if lag is None or abs(lag - expected) > dt:
        return 1
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, nargs="?")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--model-xml",
        type=Path,
        default=Path("gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"),
    )
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return self_test()
    if args.trace is None:
        raise SystemExit("trace is required unless --self-test is used")
    output_dir = args.output_dir or args.trace.with_suffix("")
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata, streams = load_trace(args.trace)
    expected_steps = {"odostate": 5}
    stream_summary = {
        name: stream_stats(rows, expected_steps.get(name, 1))
        for name, rows in streams.items()
    }
    state_summary, joint_rows = state_accuracy(streams.get("g1_debug", []))
    summary = {
        "trace": str(args.trace.resolve()),
        "metadata": metadata,
        "streams": stream_summary,
        "pico_device_timing": device_timing(streams.get("pose", [])),
        "manager_sample_age": manager_sample_age(streams.get("pose", [])),
        "pose_to_feedback_lag": pose_to_feedback_lag(
            streams.get("pose", []), streams.get("g1_debug", [])
        ),
        "state_accuracy": state_summary,
        "odometry": odom_accuracy(streams.get("odostate", [])),
        "fk_task_accuracy": fk_task_error(
            streams.get("g1_debug", []), args.model_xml
        ),
        "segments": segment_metrics(streams),
        "safety_timing": safety_timing(streams),
    }
    summary = finite_or_none(summary)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    (output_dir / "summary.md").write_text(markdown(summary), encoding="utf-8")
    write_joint_csv(output_dir / "joint_error.csv", joint_rows)
    write_markers(output_dir / "markers.csv", streams.get("marker", []))
    write_segment_csv(output_dir / "segments.csv", summary.get("segments", []))
    print(f"[analysis] wrote {output_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
