#!/usr/bin/env python3
"""SITL-only IMU accelerometer and gyroscope bias consumer smoke test.

Tests the EKF2 feedback during ACTIVE and after CLEAR, without ARM/takeoff.
Requires live EKF2 sensor inputs. Never run against physical PX4.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "fault_injection"
from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import fault_injection as dialect  # noqa: E402

PROTOCOL_VERSION = 1
SET, CLEAR, CLEAR_ALL, QUERY = 1, 2, 3, 4
ACTIVE, CLEARED = 2, 3
TARGET_IMU = 1
AXES_XYZ = 7
CASES = (
    ("IMU_ACCEL_BIAS", 1, (0.08, -0.04, 0.02), 731, "fault_imu_accel_bias_applied"),
    ("IMU_GYRO_BIAS", 3, (0.010, -0.020, 0.005), 732, "fault_imu_gyro_bias_applied"),
)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--px4-host", required=True)
    p.add_argument("--px4-port", type=int, default=14680)
    p.add_argument("--bind-port", type=int, default=14541)
    p.add_argument("--source-system", type=int, default=245)
    p.add_argument("--source-component", type=int, default=190)
    p.add_argument("--target-system", type=int, default=1)
    p.add_argument("--target-component", type=int, default=1)
    p.add_argument("--timeout", type=float, default=6.0)
    p.add_argument("--sample-seconds", type=float, default=1.5)
    p.add_argument("--output", type=Path, required=True, help="JSON report path outside Git tree")
    p.add_argument("--confirm-isolated-sitl", action="store_true", required=True,
                   help="Confirm the endpoint is the independent SITL PX4, NOT hardware")
    return p.parse_args()


def send(tx, a, scenario, sequence, fault_id, action, mode, vector):
    target = 0 if action == CLEAR_ALL else TARGET_IMU
    effective_mode = 0 if action == CLEAR_ALL else mode
    tx.mav.fault_command_send(
        scenario, sequence, 0, 0, 0, 0, 0, list(vector), 0.0,
        fault_id, a.target_system, a.target_component, PROTOCOL_VERSION,
        action, target, effective_mode, AXES_XYZ if action == SET else 0,
    )
    print(f"[SEND] action={action} seq={sequence} fault_id={fault_id} mode={mode}", flush=True)


def exchange(rx, tx, a, events, scenario, sequence, fault_id, action, mode, vector, expected_state):
    send(tx, a, scenario, sequence, fault_id, action, mode, vector)
    ack = None
    status = None
    limit = time.monotonic() + a.timeout
    while time.monotonic() < limit and (ack is None or status is None):
        m = rx.recv_match(type=["FAULT_ACK", "FAULT_STATUS"], blocking=True,
                          timeout=max(0.0, limit - time.monotonic()))
        if m is None:
            continue
        if (int(m.scenario_id), int(m.sequence), int(m.fault_id)) != (scenario, sequence, fault_id):
            continue
        if (int(m.target_system), int(m.target_component)) != (a.source_system, a.source_component):
            continue
        events.append({"action": action, "sequence": sequence, "type": m.get_type(), "data": m.to_dict()})
        if m.get_type() == "FAULT_ACK":
            ack = m
        else:
            status = m
    if ack is None or status is None:
        raise AssertionError(f"seq={sequence}: ACK or STATUS missing; check MAVLink streams and endpoint")
    if (int(ack.result), int(ack.reason), int(ack.state)) != (0, 0, expected_state):
        raise AssertionError(f"seq={sequence}: unexpected ACK result/reason/state "
                             f"{ack.result}/{ack.reason}/{ack.state}")
    if (int(status.state), int(status.target), int(status.mode), int(status.protocol_version)) != (
            expected_state, TARGET_IMU, mode, PROTOCOL_VERSION):
        raise AssertionError(f"seq={sequence}: incorrect status state/target/mode/version")
    if action == SET:
        if int(status.axis_mask) != AXES_XYZ or any(abs(float(x)-y)>1e-6 for x,y in zip(status.vector, vector)):
            raise AssertionError(f"seq={sequence}: configured mask/vector not preserved in STATUS")
    count = int(status.applied_count)
    print(f"[ACK/STATUS] seq={sequence} state={status.state} applied_count={count}", flush=True)
    return count


def main():
    a = parse_args()
    if a.px4_host not in ("127.0.0.1", "localhost", "::1"):
        raise SystemExit("[SAFETY] Localhost-only isolated PX4 SITL is supported")
    if a.px4_port != 14680 or a.bind_port != 14541:
        raise SystemExit("[SAFETY] Expected isolated SITL UDP ports 14680 / 14541")
    if not (0.5 <= a.sample_seconds <= 5.0):
        raise SystemExit("--sample-seconds must be in [0.5, 5.0]")
    for n, msgid, crc in (("FAULT_COMMAND", 12956, 28), ("FAULT_ACK", 12957, 233), ("FAULT_STATUS", 12958, 105)):
        if getattr(dialect, "MAVLINK_MSG_ID_"+n) != msgid or dialect.mavlink_map[msgid].crc_extra != crc:
            raise SystemExit(f"Wrong dialect ID for {n}")
    scenario = (int(time.time()*1000) ^ os.getpid()) & 0xFFFFFFFF
    events = []
    summary = []
    seq = 0
    current_fault = None
    tx = None
    passed = False
    try:
        rx = mavutil.mavlink_connection(f"udpin:127.0.0.1:{a.bind_port}",
                source_system=a.source_system, source_component=a.source_component, dialect="fault_injection")
        tx = mavutil.mavlink_connection(f"udpout:{a.px4_host}:{a.px4_port}",
                source_system=a.source_system, source_component=a.source_component, dialect="fault_injection")
        print(f"[SAFETY] Isolated SITL only; NO arm/takeoff/motor commands. scenario={scenario}")
        print(f"[LINK] -> {a.px4_host}:{a.px4_port}; <- WSL UDP {a.bind_port}")
        for name, mode, vector, fault_id, applied_topic in CASES:
            current_fault = (fault_id, mode)
            print(f"\n[CASE] {name} (unit={'m/s^2' if mode==1 else 'rad/s'}; {vector})")
            seq += 1
            exchange(rx, tx, a, events, scenario, seq, fault_id, SET, mode, vector, ACTIVE)
            time.sleep(a.sample_seconds)
            seq += 1
            first = exchange(rx, tx, a, events, scenario, seq, fault_id, QUERY, mode, (0., 0., 0.), ACTIVE)
            time.sleep(a.sample_seconds)
            seq += 1
            second = exchange(rx, tx, a, events, scenario, seq, fault_id, QUERY, mode, (0., 0., 0.), ACTIVE)
            if first <= 0 or second <= first:
                raise AssertionError(f"{name}: application not confirmed (counts {first} -> {second}). "
                                     f"This can reflect missing live EKF2 input in 'none'/SIH mode; "
                                     f"inspect 'ekf2 status', 'listener {applied_topic} 1' and "
                                     "'fault_manager status'. Do not call this a pass.")
            seq += 1
            exchange(rx, tx, a, events, scenario, seq, fault_id, CLEAR, mode, (0., 0., 0.), CLEARED)
            time.sleep(1.2)
            seq += 1
            frozen1 = exchange(rx, tx, a, events, scenario, seq, fault_id, QUERY, mode, (0., 0., 0.), CLEARED)
            time.sleep(1.2)
            seq += 1
            frozen2 = exchange(rx, tx, a, events, scenario, seq, fault_id, QUERY, mode, (0., 0., 0.), CLEARED)
            if frozen1 < second or frozen1 != frozen2:
                raise AssertionError(f"{name}: post-clear count mismatch; "
                                     f"active={second}, final={frozen1}->{frozen2}")
            summary.append({"case": name, "active_counts": [first, second],
                            "frozen_counts": [frozen1, frozen2], "passed": True})
            print(f"[PASS] {name}: actual applications {first}->{second}; CLEAR frozen at {frozen2}")
            current_fault = None
        passed = True
        print("\n[PASS] Both IMU modes: ACTIVE application counts increase and freeze after CLEAR")
        print("[NOTE] Per-estimator final_report evidence should also be checked through uORB/ULog.")
    except Exception as exc:
        print(f"\n[NOT VERIFIED] {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
    finally:
        if tx is not None:
            try:
                seq += 1
                send(tx, a, scenario, seq, current_fault[0] if current_fault else 731,
                     CLEAR_ALL, current_fault[1] if current_fault else 1, (0., 0., 0.))
                print("[CLEANUP] CLEAR_ALL sent; verify manager shows no active definitions", flush=True)
            except Exception as exc:
                print(f"[CLEANUP WARNING] {exc}", file=sys.stderr)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(json.dumps({"test": "SITL IMU bias actual-application audit",
                        "passed": passed, "scenario_id": scenario, "cases": summary,
                        "observations": events}, indent=2), encoding="utf-8")
        print(f"[REPORT] {a.output}", flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
