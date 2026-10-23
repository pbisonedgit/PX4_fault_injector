#!/usr/bin/env python3
"""SITL-only motor maximum-output cap interface smoke test.

This does NOT arm the aircraft, command motor outputs, or claim motor-cap applications.
Only checks the motor fault definition, MAVLink ACK/STATUS, lifecycle and audit stability.
A separate driven-motor SITL test is required to prove applied_count > 0.
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

SET, CLEAR, CLEAR_ALL, QUERY = 1, 2, 3, 4
ACTIVE, CLEARED = 2, 3
TARGET_MOTOR, MODE_MAX_OUTPUT = 6, 18
MOTOR_MASK, CAP = 0x0F, 0.70


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
    p.add_argument("--output", type=Path, required=True, help="JSON report path outside Git tree")
    p.add_argument("--confirm-isolated-sitl", action="store_true", required=True,
                   help="Confirm the remote endpoint is the independent SITL, NEVER a real aircraft")
    return p.parse_args()


def exchange(tx, rx, a, events, scenario, seq, fault_id, action, expected_state):
    target = 0 if action == CLEAR_ALL else TARGET_MOTOR
    mode = 0 if action == CLEAR_ALL else MODE_MAX_OUTPUT
    axis_mask = MOTOR_MASK if action == SET else 0
    scalar = CAP if action == SET else 0.0
    tx.mav.fault_command_send(scenario, seq, 0, 0, 0, 0, 0,
                                   [0.0, 0.0, 0.0], scalar, fault_id,
                                   a.target_system, a.target_component, 1,
                                   action, target, mode, axis_mask)
    ack = status = None
    end = time.monotonic() + a.timeout
    while time.monotonic() < end and (ack is None or status is None):
        msg = rx.recv_match(type=["FAULT_ACK", "FAULT_STATUS"], blocking=True,
                            timeout=max(0.0, end - time.monotonic()))
        if msg is None:
            continue
        if (int(msg.scenario_id), int(msg.sequence), int(msg.fault_id)) != (scenario, seq, fault_id):
            continue
        if (int(msg.target_system), int(msg.target_component)) != (a.source_system, a.source_component):
            continue
        events.append({"action": action, "sequence": seq, "type": msg.get_type(), "data": msg.to_dict()})
        if msg.get_type() == "FAULT_ACK":
            ack = msg
        else:
            status = msg
    if ack is None or status is None:
        raise AssertionError(f"seq={seq}: missing ACK or STATUS")
    if (int(ack.result), int(ack.reason), int(ack.state)) != (0, 0, expected_state):
        raise AssertionError(f"seq={seq}: bad ACK result/reason/state {ack.result}/{ack.reason}/{ack.state}")
    if (int(status.target), int(status.mode), int(status.state), int(status.protocol_version)) != (
            TARGET_MOTOR, MODE_MAX_OUTPUT, expected_state, 1):
        raise AssertionError(f"seq={seq}: wrong target/mode/state/protocol")
    if action == SET and (int(status.axis_mask) != MOTOR_MASK or
                          abs(float(status.scalar) - CAP) > 1e-6):
        raise AssertionError("Motor selection or normalized cap was not preserved")
    count = int(status.applied_count)
    print(f"[ACK/STATUS] action={action} seq={seq} state={int(status.state)} applied_count={count}", flush=True)
    return count


def main():
    a = parse_args()
    if a.px4_host not in ("127.0.0.1", "localhost", "::1"):
        raise SystemExit("[SAFETY] Localhost-only isolated PX4 SITL is supported")
    if a.px4_port != 14680 or a.bind_port != 14541:
        raise SystemExit("[SAFETY] Expected isolated SITL UDP ports 14680 / 14541")
    for n, msgid, crc in (("FAULT_COMMAND", 12956, 28),
                          ("FAULT_ACK", 12957, 233),
                          ("FAULT_STATUS", 12958, 105)):
        if getattr(dialect, "MAVLINK_MSG_ID_" + n) != msgid or dialect.mavlink_map[msgid].crc_extra != crc:
            raise SystemExit(f"Renamed dialect mismatch: {n}")
    scenario = (int(time.time() * 1000) ^ os.getpid()) & 0xffffffff
    fault_id = 741
    seq = 0
    events = []
    passed = False
    tx = None
    counts = {}
    print(f"[SAFETY] SITL only; NO ARM/TAKEOFF/MOTOR COMMANDS; scenario={scenario}")
    print(f"[LINK] -> {a.px4_host}:{a.px4_port}, <- UDP {a.bind_port}")
    try:
        rx = mavutil.mavlink_connection(f"udpin:127.0.0.1:{a.bind_port}", source_system=a.source_system,
                                        source_component=a.source_component, dialect="fault_injection")
        tx = mavutil.mavlink_connection(f"udpout:{a.px4_host}:{a.px4_port}", source_system=a.source_system,
                                        source_component=a.source_component, dialect="fault_injection")
        seq += 1
        counts["set"] = exchange(tx, rx, a, events, scenario, seq, fault_id, SET, ACTIVE)
        time.sleep(1.2)
        seq += 1
        counts["active"] = exchange(tx, rx, a, events, scenario, seq, fault_id, QUERY, ACTIVE)
        seq += 1
        counts["clear"] = exchange(tx, rx, a, events, scenario, seq, fault_id, CLEAR, CLEARED)
        time.sleep(1.2)
        seq += 1
        counts["post_clear_1"] = exchange(tx, rx, a, events, scenario, seq, fault_id, QUERY, CLEARED)
        time.sleep(1.2)
        seq += 1
        counts["post_clear_2"] = exchange(tx, rx, a, events, scenario, seq, fault_id, QUERY, CLEARED)
        if counts["post_clear_2"] != counts["post_clear_1"] or counts["post_clear_1"] < counts["active"]:
            raise AssertionError(f"Post-CLEAR count is not frozen: {counts}")
        passed = True
        print("[PASS] Motor cap definition/ACK/STATUS/SET/QUERY/CLEAR and audit freeze")
        if counts["active"] == 0:
            print("[NOT TESTED] Actual motor cap application: no evidence of cap-hit in this disarmed test")
        else:
            print("[OBSERVED] Nonzero application; inspect actuator_motors and fault_motor_output_measurement")
            print("[NOT VERIFIED] Actual signal values were not independently inspected")
    except Exception as exc:
        print(f"[FAIL] {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
    finally:
        if tx is not None:
            try:
                seq += 1
                tx.mav.fault_command_send(scenario, seq, 0, 0, 0, 0, 0,
                                               [0.0, 0.0, 0.0], 0.0, fault_id,
                                               a.target_system, a.target_component, 1,
                                               CLEAR_ALL, 0, 0, 0)
                print("[CLEANUP] CLEAR_ALL sent (not ACK-verified)")
            except Exception as exc:
                print(f"[CLEANUP WARNING] {exc}", file=sys.stderr)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(json.dumps({"test": "SITL motor max-output interface",
                                        "passed": passed, "motor_application_verified": False,
                                        "scenario_id": scenario, "cap": CAP, "motor_mask": MOTOR_MASK,
                                        "counts": counts, "observations": events}, indent=2), encoding="utf-8")
        print(f"[REPORT] {a.output}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
