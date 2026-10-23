# PX4 v1.15.2 Fault-Injection Integration — SIM

This repository is based exclusively on the public PX4-Autopilot v1.15.2 release
(commit `4817c0618a1286846116e90c6eb8919efaa013cf`). Existing PX4 Git submodule pointers are preserved.

The fault-injection integration adds 18 parameterized fault modes, fault manager,
uORB contracts, MAVLink command/status messages, and consumer-side auditing.
No flight or physical hardware operation is authorized by these instructions.

## Build preparation (required after submodule initialization)

```sh
git submodule update --init --recursive
python3 fault_release/prepare_dialect.py
make px4_sitl_default
```

The custom MAVLink XML is tracked at `fault_release/dialects/fault_injection.xml`.
The preparation script copies it to the standard MAVLink submodule definition
folder. The upstream MAVLink submodule commit is not modified.

The `SIM` target has platform-specific integrations; do not substitute
files from the other target. The REAL target uses a board-specific FMUv6C
flash-budget profile with 16 optional PX4 features disabled.

## Verification scope

SITL lifecycle and IMU bias application regressions were performed on prior
isolated code-identical builds. After public upstream reconstruction, rebuild
and rerun tests before a release. Motor max-output interface tests do not prove
actual actuator clipping. REAL build success does not mean on-device validation.

Retain original PX4 and bundled third-party copyright and license files.
