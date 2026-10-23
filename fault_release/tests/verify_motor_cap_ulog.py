#!/usr/bin/env python3
"""Read-only ULog verification of PX4 motor max-output clipping in SITL.

Requires pyulog. Checks recorded pre/post allocator commands, masks and
consumer final feedback. Does NOT send MAVLink or flight commands.
"""
from __future__ import annotations
import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

MEASUREMENT = 'fault_motor_output_measurement'
FEEDBACK = 'fault_motor_max_output_degradation_applied'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog', type=Path, required=True)
    parser.add_argument('--probe-json', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    a = parser.parse_args()
    from pyulog import ULog
    probe = json.loads(a.probe_json.read_text(encoding='utf-8'))
    scenario, fid = int(probe['scenario_id']), int(probe['fault_id'])
    cap, mask = float(probe['cap']), int(probe['mask'])
    ulog = ULog(str(a.ulog), message_name_filter_list=[MEASUREMENT, FEEDBACK])
    datasets = [d for d in ulog.data_list if d.name == MEASUREMENT]
    feedbacks = [d for d in ulog.data_list if d.name == FEEDBACK]
    if not datasets:
        raise AssertionError(f'Missing {MEASUREMENT}: configure PX4 logger to record this uORB topic at full rate')
    if not feedbacks:
        raise AssertionError(f'Missing {FEEDBACK}: configure PX4 logger to record feedback at full rate')
    checked, hits, unexplained, no_other_faults, checks_mask = 0, 0, [], 0, 0
    examples = []
    eps = 1e-4
    for dataset in datasets:
        d = dataset.data
        required = ['max_output_active', 'max_output_scenario_id', 'max_output_fault_id',
                    'max_output_motor_mask', 'max_output_norm', 'max_output_applied_mask',
                    'bias_active', 'delay_active', 'motor_count', 'reversible_flags']
        missing = [key for key in required if key not in d]
        if missing:
            raise AssertionError(f'Missing fields in measurement: {missing}; available: {sorted(d)}')
        for row in range(len(d['timestamp'])):
            if not bool(d['max_output_active'][row]) or int(d['max_output_scenario_id'][row]) != scenario or int(d['max_output_fault_id'][row]) != fid:
                continue
            if abs(float(d['max_output_norm'][row]) - cap) > eps or int(d['max_output_motor_mask'][row]) != mask:
                unexplained.append(f'row {row}: reported cap/mask differs from probe')
                continue
            checked += 1
            if bool(d['bias_active'][row]) or bool(d['delay_active'][row]):
                # A valid combined fault requires using delayed/bias-modified pre-cap
                # input, so a simple raw-to-clipped comparison is not sufficient.
                continue
            no_other_faults += 1
            motor_count = min(int(d['motor_count'][row]), 8)
            reversible_flags = int(d['reversible_flags'][row])
            expected_mask = 0
            for j in range(motor_count):
                name_raw = f'raw_control[{j}]'
                name_faulted = f'faulted_control[{j}]'
                if name_raw not in d or name_faulted not in d:
                    raise AssertionError(f'ULog arrays missing {name_raw}/{name_faulted}')
                raw, out = float(d[name_raw][row]), float(d[name_faulted][row])
                if not math.isfinite(raw):
                    if math.isfinite(out):
                        unexplained.append(f'row {row}, motor {j}: nonfinite stop unexpectedly revived')
                    continue
                if (mask & (1 << j)):
                    expected = max(-cap, min(cap, raw)) if (reversible_flags & (1 << j)) else max(0.0, min(cap, raw))
                else:
                    expected = raw
                if not math.isfinite(out) or abs(out - expected) > eps:
                    unexplained.append(f'row {row}, motor {j}: raw={raw:.6f}, out={out:.6f}, expected={expected:.6f}')
                if abs(raw - expected) > 1e-6 and (mask & (1 << j)):
                    expected_mask |= 1 << j
                    if len(examples) < 8:
                        examples.append({'motor': j + 1, 'raw': raw, 'faulted': out, 'expected': expected})
            actual_mask = int(d['max_output_applied_mask'][row])
            if actual_mask != expected_mask:
                unexplained.append(f'row {row}: applied mask {actual_mask:#04x}, expected {expected_mask:#04x}')
            checks_mask += 1
            if expected_mask:
                hits += 1
    final_reports = []
    for dataset in feedbacks:
        f = dataset.data
        keys = {'scenario_id', 'fault_id', 'final_report', 'cumulative_applied_count'}
        if not keys.issubset(f):
            raise AssertionError(f'Feedback missing required fields: {sorted(keys.difference(f))}')
        for row in range(len(f['timestamp'])):
            if int(f['scenario_id'][row]) == scenario and int(f['fault_id'][row]) == fid and bool(f['final_report'][row]):
                final_reports.append(int(f['cumulative_applied_count'][row]))
    final_count = max(final_reports) if final_reports else 0
    passed = (checked > 0 and no_other_faults > 0 and checks_mask > 0 and hits > 0
              and not unexplained and final_count > 0)
    outcome = 'PASS' if passed else 'NOT_VERIFIED' if not unexplained else 'FAIL'
    result = {'test': 'motor_cap_numerical_ulog', 'result': outcome, 'scenario_id': scenario,
              'fault_id': fid, 'cap': cap, 'motor_mask': mask,
              'matching_active_measurement_rows': checked,
              'checked_rows_without_other_faults': no_other_faults,
              'logged_frames_with_positive_clipping': hits,
              'feedback_final_cumulative_count': final_count,
              'max_output_examples': examples,
              'violations': unexplained[:30],
              'limitations': 'ULog rows may be sampled; feedback cumulative count need not equal logged clip-row count.'}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('max_output_examples', 'violations')}, indent=2))
    for ex in examples[:3]:
        print(f"[CLIP] motor={ex['motor']} raw={ex['raw']:.6f} out={ex['faulted']:.6f} expected={ex['expected']:.6f}")
    for violation in unexplained[:4]:
        print('[FAIL]', violation)
    print('[REPORT]', a.output)
    return 0 if passed else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(f'[ERROR] {type(exc).__name__}: {exc}', file=sys.stderr)
        sys.exit(2)
