# PX4 v1.15.2 Fault-Injection Integration — SITL

This `sim` branch extends the **public** PX4-Autopilot v1.15.2 release
(commit `4817c0618a1286846116e90c6eb8919efaa013cf`) with a parametrized
fault-injection manager, consumer-side injectors, uORB status/audit messages,
and a custom MAVLink 2 dialect. It implements 18 non-neutral fault modes plus
a NO_EFFECT mode. Original PX4 and third-party license notices are retained.

This project contains fault-injection infrastructure, **not** the private
flight-planning, experiment orchestration, or safety-evaluation pipeline. The
REAL/FMUv6C variant is maintained separately on the `real` branch. Do not
interchange platform-specific implementation files between branches.

## Build from a clean clone

Use a Linux/WSL host with the required PX4 v1.15.2 SITL development toolchain.

```bash
git clone --branch sim https://github.com/pbisonedgit/PX4_fault_injector.git PX4_fault_SIM
cd PX4_fault_SIM
git submodule update --init --recursive
python3 fault_release/prepare_dialect.py
make px4_sitl_default
```

The **tracked authoritative XML** is `fault_release/dialects/fault_injection.xml`.
`prepare_dialect.py` installs the same XML into the checked-out upstream MAVLink
submodule as a local, untracked build input; that copy is **not** a separate
Git-tracked change. Re-run the installer after a fresh clone or submodule reset.

For external Python controllers, the XML alone does not install Python import
support. The tested, isolated Linux/WSL `pymavlink` setup is described in
[`tests/README.md`](tests/README.md). Windows is not required for this workflow.

## SITL tests

The [`fault_release/tests/`](tests/) directory provides four non-flying
Linux/WSL smoke tests covering lifecycle handling, EKF2 IMU bias, EKF2 GNSS
position bias, and motor-cap interface operations. It also contains a separate
motor-cap effect probe and offline ULog numerical verifier for controlled,
software-only SITL experiments.

For detailed per-test assertions, setup, commands and evidence, see
[`tests/README.md`](tests/README.md). This top-level overview intentionally
keeps the validation matrix and test-specific boundaries in the tests folder.

This integration is an experimental fault-injection research tool. Never connect
the localhost smoke-test endpoints to physical flight hardware.

## Reproducibility

The tests depend only on the repository's tracked XML, upstream PX4 submodules,
and an isolated Python virtual environment; results and ULogs should be saved
**outside** the Git working tree. For exact prerequisites, loopback UDP endpoints,
SITL preparation, limitations and commands, see [`tests/README.md`](tests/README.md).
