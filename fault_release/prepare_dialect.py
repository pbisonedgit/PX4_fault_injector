#!/usr/bin/env python3
"""Install Git-tracked fault_injection.xml into the official MAVLink submodule.

Run after git submodule update --init --recursive and before each PX4 build.
The XML stays tracked in this main repository, not in the upstream submodule.
"""
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "fault_release/dialects/fault_injection.xml"
TARGET = ROOT / "src/modules/mavlink/mavlink/message_definitions/v1.0/fault_injection.xml"
EXPECTED = "acdc97969120478c136e07f0d7c7f38ec35a626b6cc90c0adf737509a1a1aaf5"
if not (ROOT / "src/modules/mavlink/mavlink/.git").exists():
    raise SystemExit("[FAIL] Official MAVLink submodule not initialized")
if hashlib.sha256(SOURCE.read_bytes()).hexdigest() != EXPECTED:
    raise SystemExit("[FAIL] Tracked MAVLink dialect changed")
if TARGET.exists():
    if TARGET.read_bytes() != SOURCE.read_bytes():
        raise SystemExit("[FAIL] Different dialect already exists; will not overwrite")
    print("[PASS] Fault injection dialect already installed")
else:
    TARGET.write_bytes(SOURCE.read_bytes())
    print("[PASS] Official MAVLink submodule receives fault_injection.xml (untracked submodule workfile)")
