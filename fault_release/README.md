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

## SITL verification and scope

SITL build of this branch was independently completed using the clean official
PX4 source tree. The four reviewed, named WSL tests under `tests/` have been
run against this branch, with the following observations:

| Test | Verified | Not established |
| --- | --- | --- |
| `smoke_no_effect.py` | SET, QUERY, CLEAR, pending/active/expired transitions, CLEAR_ALL; zero applications | Physical platform behavior |
| `smoke_imu_bias.py` | EKF2 accelerometer and gyro bias application counters grow while active and freeze after CLEAR | Independently measured numerical bias magnitude |
| `smoke_gnss_position_bias.py` | With `make px4_sitl sihsim_quadx`, GNSS consumer applications and post-CLEAR freeze; example run: 48 applications, final-report feedback | Independent numerical position offset or navigational consequence |
| `smoke_motor_cap_interface.py` | Motor-cap definition, ACK/STATUS, QUERY/CLEAR and count stability while disarmed | Actual motor-output clipping/cap-hit |

An application counter reflects the injector's **consumer-side application
feedback**, not an independently measured actuator or sensor error. The smoke
tests do not arm, take off, or command motors. Never run them against physical
flight hardware, and do not use this integration as a flight-safety guarantee.

A numeric motor-cap validation requires a **separate, deliberately controlled,
software-only SITL** scenario with finite actuator commands above the configured
cap, inspecting both `fault_motor_output_measurement` (raw and faulted values)
and `fault_motor_max_output_degradation_applied`. Do not describe the current
interface-only motor test as proof of clipping.

The `real` branch has passed an FMUv6C firmware build using a documented lean
board configuration; it has **not** been flashed or validated on real hardware
in this release workflow.

## Reproducibility

The tests depend only on the repository's tracked XML, upstream PX4 submodules,
and an isolated Python virtual environment; results and ULogs should be saved
**outside** the Git working tree. For exact prerequisites, loopback UDP endpoints,
SITL preparation, limitations and commands, see [`tests/README.md`](tests/README.md).
