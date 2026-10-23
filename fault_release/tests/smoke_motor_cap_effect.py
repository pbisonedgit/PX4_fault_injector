#!/usr/bin/env python3
"""Motor-cap effect counter probe for localhost-only isolated PX4 SITL.

No ARM, takeoff, throttle, or actuator command is sent. The caller separately
provides a controlled *software-only* situation that produces finite allocated
motor outputs. Cap application is not considered verified by this probe alone:
inspect fault_motor_output_measurement in ULog independently.
"""
from __future__ import annotations
import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'fault_injection'
from pymavlink import mavutil

SET, CLEAR, CLEAR_ALL, QUERY = 1, 2, 3, 4
ACTIVE, CLEARED = 2, 3


def exchange(tx, rx, a, scenario, seq, fault_id, action, expected_state, events):
    target = 0 if action == CLEAR_ALL else 6
    mode = 0 if action == CLEAR_ALL else 18
    motor_mask = a.mask if action == SET else 0
    cap = a.cap if action == SET else 0.0
    tx.mav.fault_command_send(scenario, seq, 0, 0, 0, 0, 0,
                              [0.0, 0.0, 0.0], cap, fault_id,
                              1, 1, 1, action, target, mode, motor_mask)
    ack = status = None
    end = time.monotonic() + a.timeout
    while time.monotonic() < end and (ack is None or status is None):
        msg = rx.recv_match(type=['FAULT_ACK', 'FAULT_STATUS'], blocking=True,
                            timeout=max(0.0, end - time.monotonic()))
        if msg is None or (int(msg.scenario_id), int(msg.sequence), int(msg.fault_id)) != (scenario, seq, fault_id):
            continue
        if (int(msg.target_system), int(msg.target_component)) != (245, 190):
            continue
        events.append({'sequence': seq, 'type': msg.get_type(), 'data': msg.to_dict()})
        if msg.get_type() == 'FAULT_ACK':
            ack = msg
        else:
            status = msg
    if ack is None or status is None:
        raise AssertionError(f'seq={seq}: missing ACK/STATUS on UDP 14680/14541')
    if (int(ack.result), int(ack.reason), int(ack.state)) != (0, 0, expected_state):
        raise AssertionError(f'seq={seq}: ACK result/reason/state {ack.result}/{ack.reason}/{ack.state}')
    if (int(status.target), int(status.mode), int(status.state), int(status.protocol_version)) != (6, 18, expected_state, 1):
        raise AssertionError(f'seq={seq}: wrong target/mode/state/protocol')
    if action == SET and (int(status.axis_mask) != a.mask or abs(float(status.scalar) - a.cap) > 1e-6):
        raise AssertionError('Motor selection/cap value mismatch')
    count = int(status.applied_count)
    print(f'[ACK/STATUS] action={action} seq={seq} state={status.state} applied_count={count}', flush=True)
    return count


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4-host', default='127.0.0.1')
    p.add_argument('--cap', type=float, default=0.70)
    p.add_argument('--mask', type=lambda x: int(x, 0), default=0x0f)
    p.add_argument('--hold-s', type=float, default=12.0)
    p.add_argument('--timeout', type=float, default=6.0)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--confirm-isolated-sitl', action='store_true', required=True)
    a = p.parse_args()
    if a.px4_host != '127.0.0.1' or not (0.0 <= a.cap < 1.0) or not (0 < a.mask <= 255) or not (2 <= a.hold_s <= 60):
        p.error('localhost 127.0.0.1, cap in [0,1), 8-bit nonzero mask, hold-s in [2,60] required')
    from pymavlink.dialects.v20 import fault_injection as d
    for name, ident, crc in [('FAULT_COMMAND', 12956, 28), ('FAULT_ACK', 12957, 233), ('FAULT_STATUS', 12958, 105)]:
        if getattr(d, 'MAVLINK_MSG_ID_' + name) != ident or d.mavlink_map[ident].crc_extra != crc:
            p.error('MAVLink dialect mismatch: ' + name)
    scenario = (int(time.time() * 1000) ^ os.getpid()) & 0xffffffff
    fault_id, seq = 742, 0
    events, counts = [], {}
    ok, tx, note = False, None, ''
    print(f'[SAFETY] LOCALHOST-ONLY SITL; NO arming/flight/motor commands. scenario={scenario}', flush=True)
    print(f'[PARAM] cap={a.cap:.4f} motor_mask={a.mask:#04x} hold={a.hold_s}s', flush=True)
    try:
        rx = mavutil.mavlink_connection('udpin:127.0.0.1:14541', source_system=245, source_component=190, dialect='fault_injection')
        tx = mavutil.mavlink_connection('udpout:127.0.0.1:14680', source_system=245, source_component=190, dialect='fault_injection')
        seq += 1
        counts['set'] = exchange(tx, rx, a, scenario, seq, fault_id, SET, ACTIVE, events)
        time.sleep(0.5)
        start = time.monotonic()
        while time.monotonic() - start < a.hold_s:
            time.sleep(max(0.0, min(1.5, a.hold_s - (time.monotonic() - start))))
            seq += 1
            counts[f'active_{seq}'] = exchange(tx, rx, a, scenario, seq, fault_id, QUERY, ACTIVE, events)
        seq += 1
        counts['clear'] = exchange(tx, rx, a, scenario, seq, fault_id, CLEAR, CLEARED, events)
        time.sleep(1.0)
        seq += 1
        counts['post_clear_1'] = exchange(tx, rx, a, scenario, seq, fault_id, QUERY, CLEARED, events)
        time.sleep(1.0)
        seq += 1
        counts['post_clear_2'] = exchange(tx, rx, a, scenario, seq, fault_id, QUERY, CLEARED, events)
        active_counts = [v for k, v in counts.items() if k.startswith('active_')]
        if not active_counts or counts['post_clear_1'] != counts['post_clear_2']:
            raise AssertionError('Post-CLEAR count did not freeze')
        if active_counts[-1] == 0:
            note = 'NOT_VERIFIED: no cap-hit reported; generate finite selected commands above cap in isolated SITL'
            print('[NOT VERIFIED]', note, flush=True)
        else:
            ok = True
            note = 'CAP_HIT_REPORTED: independently verify exact raw/faulted motor values using ULog'
            print('[PASS] positive motor-cap application reported; check ULog for numerical clipping', flush=True)
    except Exception as exc:
        note = f'{type(exc).__name__}: {exc}'
        print('[FAIL]', note, file=sys.stderr, flush=True)
    finally:
        if tx is not None:
            try:
                seq += 1
                tx.mav.fault_command_send(scenario, seq, 0, 0, 0, 0, 0, [0.0, 0.0, 0.0],
                                          0.0, fault_id, 1, 1, 1, CLEAR_ALL, 0, 0, 0)
                print('[CLEANUP] CLEAR_ALL submitted; not ACK-verified', flush=True)
            except Exception as exc:
                print(f'[CLEANUP WARNING] {exc}', file=sys.stderr)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(json.dumps({'test': 'motor_cap_effect_probe', 'scenario_id': scenario,
                        'fault_id': fault_id, 'cap': a.cap, 'mask': a.mask,
                        'application_reported': ok, 'note': note,
                        'counts': counts, 'events': events}, indent=2), encoding='utf-8')
        print('[REPORT]', a.output, flush=True)
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
