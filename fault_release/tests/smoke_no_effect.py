#!/usr/bin/env python3
"""SITL-only NO_EFFECT lifecycle and audit smoke test.

This test sends no sensor/actuator transformations; never target real hardware.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "fault_injection"
from pymavlink import mavutil
from pymavlink.dialects.v20 import fault_injection as dialect

PROTOCOL_VERSION = 1
SET, CLEAR, CLEAR_ALL, QUERY = 1, 2, 3, 4
NONE, NO_EFFECT = 0, 0
PENDING, ACTIVE, CLEARED, EXPIRED = 1, 2, 3, 4


def args_parse():
    p = argparse.ArgumentParser(description="Isolated PX4 SITL NO_EFFECT fault lifecycle smoke test")
    p.add_argument("--px4-host", required=True, help="Loopback address of isolated PX4 SITL (127.0.0.1)")
    p.add_argument("--confirm-isolated-sitl", action="store_true", required=True,
                   help="Required acknowledgment: PX4 endpoint is an isolated SITL instance, never a real aircraft")
    p.add_argument("--px4-port", type=int, default=14680)
    p.add_argument("--bind-port", type=int, default=14541)
    p.add_argument("--source-system", type=int, default=245)
    p.add_argument("--source-component", type=int, default=190)
    p.add_argument("--target-system", type=int, default=1)
    p.add_argument("--target-component", type=int, default=1)
    p.add_argument("--timeout", type=float, default=6.0)
    p.add_argument("--output", type=Path, required=True, help="JSON report path outside Git tree")
    return p.parse_args()


def send(tx, a, scenario, seq, fault, action, start_delay_ms=0, duration_ms=0):
    tx.mav.fault_command_send(
        scenario, seq, start_delay_ms, duration_ms, 0, 0, 0,
        [0.0, 0.0, 0.0], 0.0, fault, a.target_system, a.target_component,
        PROTOCOL_VERSION, action, NONE, NO_EFFECT, 0,
    )
    print(f"[SEND] action={action} scenario={scenario} seq={seq} fault={fault} "
          f"onset_ms={start_delay_ms} duration_ms={duration_ms}", flush=True)


def exchange(tx, rx, a, report, scenario, seq, fault, action, expected_state,
             start_delay_ms=0, duration_ms=0, expected_count=0):
    # Start a fresh request. Never retransmit the same sequence automatically:
    # doing so would test duplicate-sequence handling instead of this operation.
    send(tx, a, scenario, seq, fault, action, start_delay_ms, duration_ms)
    ack, status = None, None
    deadline = time.monotonic() + a.timeout
    while time.monotonic() < deadline and (ack is None or status is None):
        message = rx.recv_match(type=["FAULT_ACK", "FAULT_STATUS"], blocking=True,
                                timeout=max(0.0, deadline - time.monotonic()))
        if message is None:
            continue
        name = message.get_type()
        if (int(message.scenario_id), int(message.sequence), int(message.fault_id)) != (scenario, seq, fault):
            continue
        if int(message.target_system) != a.source_system or int(message.target_component) != a.source_component:
            continue
        entry = message.to_dict()
        report.append({"type": name, "data": entry})
        if name == "FAULT_ACK":
            ack = message
            print(f"[ACK] result={ack.result} reason={ack.reason} state={ack.state}", flush=True)
        elif name == "FAULT_STATUS":
            status = message
            print(f"[STATUS] state={status.state} applied_count={status.applied_count} "
                  f"target={status.target} mode={status.mode}", flush=True)
    if ack is None:
        raise AssertionError(f"ACK missing for action={action} seq={seq}. Check link/ACK stream/PX4 manager.")
    if status is None:
        raise AssertionError(f"STATUS missing for action={action} seq={seq}. Check STATUS stream.")
    if (int(ack.result), int(ack.reason), int(ack.state)) != (0, 0, expected_state):
        raise AssertionError(f"Unexpected ACK at seq={seq}: result={ack.result} reason={ack.reason} "
                             f"state={ack.state}; expected (0,0,{expected_state})")
    if (int(status.state), int(status.applied_count), int(status.target), int(status.mode)) != (expected_state, expected_count, NONE, NO_EFFECT):
        raise AssertionError(f"Unexpected status at seq={seq}: state={status.state} "
                             f"applied_count={status.applied_count} target={status.target} mode={status.mode}")
    if int(ack.protocol_version) != PROTOCOL_VERSION or int(status.protocol_version) != PROTOCOL_VERSION:
        raise AssertionError("Protocol version mismatch")
    print(f"[PASS] seq={seq} state={expected_state}", flush=True)


def main():
    a = args_parse()
    if a.px4_host not in ("127.0.0.1", "localhost", "::1"):
        raise SystemExit("[SAFETY] Localhost-only isolated PX4 SITL is supported")
    if a.px4_port != 14680 or a.bind_port != 14541:
        raise SystemExit("[SAFETY] Expected isolated SITL UDP ports 14680 / 14541")
    output = a.output
    report = []
    scenario = int(time.time()) & 0xFFFFFFFF
    fault = 621
    succeeded = False
    tx = None
    try:
        ids = {"FAULT_COMMAND": (12956, 28), "FAULT_ACK": (12957, 233), "FAULT_STATUS": (12958, 105)}
        for name, (expected_id, crc) in ids.items():
            if getattr(dialect, "MAVLINK_MSG_ID_" + name) != expected_id or dialect.mavlink_map[expected_id].crc_extra != crc:
                raise AssertionError("Unexpected dialect ID for " + name)
        print(f"[DIALECT PASS] fault_injection; scenario={scenario}", flush=True)
        rx = mavutil.mavlink_connection(f"udpin:127.0.0.1:{a.bind_port}",
              source_system=a.source_system, source_component=a.source_component, dialect="fault_injection")
        tx = mavutil.mavlink_connection(f"udpout:{a.px4_host}:{a.px4_port}",
              source_system=a.source_system, source_component=a.source_component, dialect="fault_injection")
        print(f"[LINK] WSL listens UDP {a.bind_port}; PX4 receives {a.px4_host}:{a.px4_port}", flush=True)
        print("[SAFETY] NO_EFFECT only. PX4 must be isolated SITL, not physical hardware.", flush=True)
        # Initial active definition (zero duration means active until clear).
        exchange(tx, rx, a, report, scenario, 1, fault, SET, ACTIVE)
        exchange(tx, rx, a, report, scenario, 2, fault, QUERY, ACTIVE)
        exchange(tx, rx, a, report, scenario, 3, fault, CLEAR, CLEARED)
        exchange(tx, rx, a, report, scenario, 4, fault, QUERY, CLEARED)
        # Delayed activation and finite lifetime (no effect on aircraft output).
        exchange(tx, rx, a, report, scenario, 5, fault, SET, PENDING,
                 start_delay_ms=3500, duration_ms=3000)
        exchange(tx, rx, a, report, scenario, 6, fault, QUERY, PENDING)
        print("[WAIT] Pending -> active; no fault effects will be applied", flush=True)
        time.sleep(3.8)
        exchange(tx, rx, a, report, scenario, 7, fault, QUERY, ACTIVE)
        print("[WAIT] Active -> expired", flush=True)
        time.sleep(3.1)
        exchange(tx, rx, a, report, scenario, 8, fault, QUERY, EXPIRED)
        # Re-create then verify CLEAR_ALL without affecting any real sensor/motor.
        exchange(tx, rx, a, report, scenario, 9, fault, SET, ACTIVE)
        exchange(tx, rx, a, report, scenario, 10, fault, CLEAR_ALL, CLEARED)
        exchange(tx, rx, a, report, scenario, 11, fault, QUERY, CLEARED)
        succeeded = True
        print("\n[PASS] SITL NO_EFFECT SET / QUERY / CLEAR / PENDING / ACTIVE / EXPIRED / CLEAR_ALL", flush=True)
        print("[PASS] applied_count remains 0 for all stages", flush=True)
    except Exception as exc:
        print(f"\n[FAIL] {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        # Best effort fail-safe: CLEAR_ALL is limited to the isolated SITL instance.
        # This step is intentionally not run on any physical board.
        if tx is not None:
            try:
                send(tx, a, scenario, 99, fault, CLEAR_ALL)
                print("[CLEANUP] CLEAR_ALL sent; ACK not verified", flush=True)
            except Exception:
                pass
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({"test": "SITL NO_EFFECT lifecycle", "passed": succeeded,
                                      "scenario_id": scenario, "observations": report}, indent=2), encoding="utf-8")
        print(f"[REPORT] {output}", flush=True)
    return 0 if succeeded else 1


if __name__ == "__main__":
    sys.exit(main())
