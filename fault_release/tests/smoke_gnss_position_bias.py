#!/usr/bin/env python3
"""Isolated PX4 SITL-only GNSS POSITION BIAS audit (WSL/Linux).

This test never arms or commands propulsion. It verifies MAVLink SET/QUERY/CLEAR,
configured NED bias preservation, actual EKF2 GNSS sample application via
manager audit counts, and post-CLEAR freeze. It does NOT independently quantify
lat/lon/alt shifts or validate in-flight navigation consequences.

If no GNSS samples reach EKF2, interface_pass may be true but
application_verified is false; script exits nonzero (NOT VERIFIED).
"""
from __future__ import annotations

import argparse
import json
import math
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
TARGET_GNSS, MODE_GNSS_POSITION_BIAS = 3, 7
AXES_XYZ = 7
PROTOCOL_VERSION = 1


def args_parse():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--px4-host", default="127.0.0.1")
    p.add_argument("--px4-port", type=int, default=14680)
    p.add_argument("--bind-port", type=int, default=14541)
    p.add_argument("--source-system", type=int, default=245)
    p.add_argument("--source-component", type=int, default=190)
    p.add_argument("--target-system", type=int, default=1)
    p.add_argument("--target-component", type=int, default=1)
    p.add_argument("--timeout", type=float, default=6.0)
    p.add_argument("--sample-seconds", type=float, default=3.0)
    p.add_argument("--bias-ned-m", type=float, nargs=3, metavar=("NORTH", "EAST", "DOWN"), default=[1.0, -0.5, 0.2])
    p.add_argument("--output", type=Path, required=True, help="JSON report path outside Git tree")
    p.add_argument("--confirm-isolated-sitl", action="store_true", required=True,
                   help="Mandatory confirmation that localhost:14680 is PX4 SITL, never hardware")
    return p.parse_args()


def send(tx, a, scenario, sequence, fault_id, action, vector):
    target = 0 if action == CLEAR_ALL else TARGET_GNSS
    mode = 0 if action == CLEAR_ALL else MODE_GNSS_POSITION_BIAS
    mask = AXES_XYZ if action == SET else 0
    tx.mav.fault_command_send(scenario, sequence, 0, 0, 0, 0, 0,
                              [float(v) for v in vector], 0.0, fault_id,
                              a.target_system, a.target_component,
                              PROTOCOL_VERSION, action, target, mode, mask)
    print(f"[SEND] seq={sequence} action={action} target={target} mode={mode}", flush=True)


def exchange(rx, tx, a, observations, scenario, sequence, fault_id, action, vector, expected_state):
    send(tx, a, scenario, sequence, fault_id, action, vector)
    ack = status = None
    deadline = time.monotonic() + a.timeout
    while time.monotonic() < deadline and (ack is None or status is None):
        msg = rx.recv_match(type=["FAULT_ACK", "FAULT_STATUS"], blocking=True,
                            timeout=max(0.0, deadline - time.monotonic()))
        if msg is None:
            continue
        if (int(msg.scenario_id), int(msg.sequence), int(msg.fault_id)) != (scenario, sequence, fault_id):
            continue
        if (int(msg.target_system), int(msg.target_component)) != (a.source_system, a.source_component):
            continue
        observations.append({"action": action, "sequence": sequence, "type": msg.get_type(), "data": msg.to_dict()})
        if msg.get_type() == "FAULT_ACK":
            ack = msg
        else:
            status = msg
    if ack is None or status is None:
        raise AssertionError(f"seq={sequence}: ACK/STATUS missing; verify PX4 streams on UDP 14680")
    if (int(ack.result), int(ack.reason), int(ack.state), int(ack.protocol_version)) != (0, 0, expected_state, 1):
        raise AssertionError(f"seq={sequence}: unexpected ACK result={ack.result} reason={ack.reason} state={ack.state}")
    if (int(status.state), int(status.target), int(status.mode), int(status.protocol_version)) != (
            expected_state, TARGET_GNSS, MODE_GNSS_POSITION_BIAS, 1):
        raise AssertionError(f"seq={sequence}: unexpected STATUS state/target/mode/protocol")
    if action == SET:
        if int(status.axis_mask) != AXES_XYZ or any(abs(float(x)-float(y)) > 1e-5
                                                   for x, y in zip(status.vector, vector)):
            raise AssertionError(f"seq={sequence}: configured GNSS NED vector/mask not preserved")
    count = int(status.applied_count)
    print(f"[ACK/STATUS] seq={sequence} state={status.state} applied_count={count}", flush=True)
    return count


def main():
    a = args_parse()
    # The test is intentionally restricted to the local, explicitly confirmed SITL instance.
    if a.px4_host not in ("127.0.0.1", "localhost", "::1"):
        raise SystemExit("[SAFETY] Only loopback-hosted SITL is supported")
    if a.px4_port != 14680 or a.bind_port != 14541:
        raise SystemExit("[SAFETY] Expected isolated SITL UDP ports 14680 and 14541")
    if a.sample_seconds < 1.0 or a.sample_seconds > 15.0:
        raise SystemExit("--sample-seconds must be in [1,15]")
    if not all(math.isfinite(v) for v in a.bias_ned_m) or not any(v != 0.0 for v in a.bias_ned_m):
        raise SystemExit("--bias-ned-m requires finite, nonzero NED displacement")
    for name, mid, crc in (("FAULT_COMMAND", 12956, 28), ("FAULT_ACK", 12957, 233),
                           ("FAULT_STATUS", 12958, 105), ("FAULTED_DOWNWARD_DISTANCE", 12959, 219)):
        if getattr(dialect, "MAVLINK_MSG_ID_" + name) != mid or dialect.mavlink_map[mid].crc_extra != crc:
            raise SystemExit(f"[FAIL] Unexpected MAVLink dialect ID/CRC: {name}")
    scenario = (int(time.time() * 1000) ^ os.getpid()) & 0xffffffff
    fault_id, seq = 751, 0
    observations, counts = [], {}
    interface_pass = application_verified = clear_frozen = False
    error = None
    tx = None
    try:
        rx = mavutil.mavlink_connection(f"udpin:127.0.0.1:{a.bind_port}",
                source_system=a.source_system, source_component=a.source_component, dialect="fault_injection")
        tx = mavutil.mavlink_connection(f"udpout:127.0.0.1:{a.px4_port}",
                source_system=a.source_system, source_component=a.source_component, dialect="fault_injection")
        print(f"[SAFETY] Isolated localhost SITL only. NO arm/takeoff/motor commands. scenario={scenario}")
        print(f"[CASE] GNSS_POSITION_BIAS target=3 mode=7 NED metres={a.bias_ned_m}")
        seq += 1
        counts["set"] = exchange(rx, tx, a, observations, scenario, seq, fault_id, SET, a.bias_ned_m, ACTIVE)
        time.sleep(a.sample_seconds)
        seq += 1
        counts["active_1"] = exchange(rx, tx, a, observations, scenario, seq, fault_id, QUERY, [0,0,0], ACTIVE)
        time.sleep(a.sample_seconds)
        seq += 1
        counts["active_2"] = exchange(rx, tx, a, observations, scenario, seq, fault_id, QUERY, [0,0,0], ACTIVE)
        interface_pass = True
        if counts["active_1"] <= 0 or counts["active_2"] <= counts["active_1"]:
            print("[NOT VERIFIED] GNSS consumer application did not increase."
                  " Check 'listener vehicle_gps_position 1', 'ekf2 status',"
                  " 'listener fault_gnss_position_bias_applied 1'.")
        else:
            application_verified = True
            print(f"[PASS] EKF2 GNSS samples modified (reported): {counts['active_1']} -> {counts['active_2']}")
        seq += 1
        counts["clear"] = exchange(rx, tx, a, observations, scenario, seq, fault_id, CLEAR, [0,0,0], CLEARED)
        time.sleep(1.5)
        seq += 1
        counts["post_clear_1"] = exchange(rx, tx, a, observations, scenario, seq, fault_id, QUERY, [0,0,0], CLEARED)
        time.sleep(1.5)
        seq += 1
        counts["post_clear_2"] = exchange(rx, tx, a, observations, scenario, seq, fault_id, QUERY, [0,0,0], CLEARED)
        clear_frozen = (counts["post_clear_1"] >= counts["active_2"] and
                        counts["post_clear_1"] == counts["post_clear_2"])
        if not clear_frozen:
            raise AssertionError(f"GNSS application count not frozen after CLEAR: {counts}")
        print(f"[PASS] GNSS post-CLEAR count frozen at {counts['post_clear_2']}")
        if interface_pass and application_verified and clear_frozen:
            print("[PASS] GNSS_POSITION_BIAS: interface + EKF2 consumer application + CLEAR freeze")
        else:
            print("[NOT VERIFIED] GNSS application requires a live EKF2 GNSS sample stream; interface tested")
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(f"[FAIL] {error}", file=sys.stderr, flush=True)
    finally:
        if tx is not None:
            try:
                seq += 1
                send(tx, a, scenario, seq, fault_id, CLEAR_ALL, [0,0,0])
                print("[CLEANUP] CLEAR_ALL sent (not ACK-verified)")
            except Exception as exc:
                print(f"[CLEANUP WARNING] {exc}", file=sys.stderr)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(json.dumps({
            "test": "WSL SITL GNSS Position Bias application audit",
            "passed": bool(interface_pass and application_verified and clear_frozen and error is None),
            "interface_pass": interface_pass,
            "application_verified": application_verified,
            "clear_frozen": clear_frozen,
            "numeric_position_offset_verified": False,
            "scenario_id": scenario, "fault_id": fault_id,
            "target": TARGET_GNSS, "mode": MODE_GNSS_POSITION_BIAS,
            "bias_ned_m": list(a.bias_ned_m),
            "counts": counts, "observations": observations,
            "error": error
        }, indent=2), encoding="utf-8")
        print(f"[REPORT] {a.output}")
    return 0 if interface_pass and application_verified and clear_frozen and error is None else 1


if __name__ == "__main__":
    sys.exit(main())
