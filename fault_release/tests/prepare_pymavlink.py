#!/usr/bin/env python3
"""Generate and optionally install the fault-only Python MAVLink2 dialect into a venv.

Run with the Python interpreter of a dedicated virtual environment. The upstream
PX4 MAVLink submodule must be initialized. Use --install for a venv-local copy.
No legacy dialects are required, and no PX4 source is modified here.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
from pathlib import Path
import shutil
import sys
import tempfile
import py_compile

XML_SHA256 = "acdc97969120478c136e07f0d7c7f38ec35a626b6cc90c0adf737509a1a1aaf5"
EXPECTED = {"FAULT_COMMAND": (12956, 28), "FAULT_ACK": (12957, 233),
            "FAULT_STATUS": (12958, 105), "FAULTED_DOWNWARD_DISTANCE": (12959, 219)}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_dialect(path):
    spec = importlib.util.spec_from_file_location("_verified_fault_injection_dialect", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for name, (mid, crc) in EXPECTED.items():
        assert getattr(mod, "MAVLINK_MSG_ID_" + name) == mid, name + " ID mismatch"
        assert mod.mavlink_map[mid].crc_extra == crc, name + " CRC mismatch"
    print("[PASS] Four custom MAVLink2 message IDs and CRC_EXTRA values")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--install", action="store_true", help="Install into this interpreter's virtual environment")
    a = p.parse_args()
    repo = Path(__file__).resolve().parents[2]
    master = repo / "fault_release/dialects/fault_injection.xml"
    submodule = repo / "src/modules/mavlink/mavlink/message_definitions/v1.0/fault_injection.xml"
    if not master.is_file() or sha(master) != XML_SHA256:
        raise SystemExit("[FAIL] Published fault_injection.xml missing or unexpected SHA256")
    if not submodule.is_file() or submodule.read_bytes() != master.read_bytes():
        raise SystemExit("[FAIL] Run python3 fault_release/prepare_dialect.py first")
    if a.install and sys.prefix == sys.base_prefix:
        raise SystemExit("[FAIL] --install requires a dedicated Python venv; refusing system installation")
    import pymavlink.dialects.v20 as v20
    from pymavlink.generator import mavgen
    v20dir = Path(v20.__file__).resolve().parent
    target = v20dir / "fault_injection.py"
    with tempfile.TemporaryDirectory(prefix="px4_fault_dialect_") as tmp:
        generated = Path(tmp) / "fault_injection.py"
        opt = mavgen.Opts(output=str(generated), wire_protocol="2.0", language="Python3")
        if not mavgen.mavgen(opt, [str(submodule)]) or not generated.is_file():
            raise RuntimeError("pymavlink generation failed")
        py_compile.compile(str(generated), doraise=True)
        verify_dialect(generated)
        if target.is_file() and sha(target) != sha(generated):
            raise SystemExit("[FAIL] Different fault_injection.py already installed; refusing overwrite")
        if a.install:
            if not target.exists():
                shutil.copy2(generated, target)
            verify_dialect(target)
            print("[PASS] Dialect installed into isolated venv:", target)
        else:
            print("[PREVIEW] Generated dialect validated; use --install to install into this venv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
