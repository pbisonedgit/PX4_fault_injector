# PX4 Fault Injector

**Parameterized and auditable fault injection for PX4-based UAV experiments in simulation and on physical flight controllers.**

This project extends **PX4-Autopilot v1.15.2** with a fault-management module, PX4-internal sensor and actuator injection hooks, custom MAVLink commands, and consumer-side application auditing. The same fault definitions and transformation semantics are implemented for software-in-the-loop (SITL) and Pixhawk 6C (FMUv6C) builds.

Rather than treating a fault's **ACTIVE** state as proof that it affected the vehicle, the framework records whether the corresponding PX4 consumer actually applied the degradation. It is designed for controlled UAV reliability, fault-response, and safety experiments.

> **Research code based on PX4, not an official PX4 release.** The repository contains the fault-injection infrastructure, not the complete flight-planning, experiment orchestration, datasets, or outcome-analysis pipeline used in the accompanying research.

## What this project provides

- **Parameterized fault scenarios:** select the target, affected axes or motors, magnitude, optional stochastic seed, onset delay, and duration; control definitions using `SET`, `QUERY`, `CLEAR`, and `CLEAR_ALL`.
- **Matched internal injection points:** sensor degradations are introduced immediately before EKF2 measurement consumption; actuator degradations are applied after control allocation and before motor-output publication.
- **Application-level auditing:** target consumers report cumulative actual applications through uORB; `fault_manager` validates and aggregates the feedback into `FAULT_STATUS` over MAVLink.
- **Portable PX4 builds:** a SITL implementation and a hardware-targeted FMUv6C implementation based on the same official PX4 v1.15.2 revision.
- **Reproducible Linux/WSL checks:** isolated SITL smoke tests, a Python MAVLink-dialect installer, and a numerical motor-cap ULog verifier.

### Implemented fault modes

| PX4 target | Configurable modes |
| --- | --- |
| IMU accelerometer | Bias; Gaussian noise |
| IMU gyroscope | Bias; Gaussian noise |
| Magnetometer | Bias; Gaussian noise |
| GNSS position | Bias; Gaussian noise (local NED) |
| GNSS velocity | Bias; Gaussian noise (NED) |
| Downward range | Bias; noise; effective-range degradation |
| Barometric altitude | Bias; noise |
| Motor output | Bias; delay; maximum-output limit |

There are **18 effective modes**, plus `NO_EFFECT` for lifecycle and communication testing. Vector modes support axis selection; motor modes support a motor-selection mask. See the accompanying manuscript for parameter units, transformations, and execution order.

## Repository branches

| Branch | Intended platform | Build target |
| --- | --- | --- |
| [`sim`](https://github.com/pbisonedgit/PX4_fault_injector/tree/sim) | PX4 SITL, including SIH-based smoke tests | `px4_sitl_default` |
| [`real`](https://github.com/pbisonedgit/PX4_fault_injector/tree/real) | PX4 firmware for Pixhawk 6C / FMUv6C | `px4_fmu-v6c_default` |

Both branches descend from the official PX4 v1.15.2 base, commit [`4817c061`](https://github.com/PX4/PX4-Autopilot/commit/4817c0618a1286846116e90c6eb8919efaa013cf). They share the fault definitions and their intended semantics, but contain platform-specific integration differences: **do not copy firmware-specific implementation files from one branch into the other**.

## Quick start: SIM on Linux / WSL

Install the [PX4 v1.15 development toolchain](https://docs.px4.io/v1.15/en/dev_setup/dev_env.html) before building. Commands below run on Linux or WSL; Windows-specific MAVLink generation is not required.

```bash
git clone --branch sim https://github.com/pbisonedgit/PX4_fault_injector.git PX4_fault_SIM
cd PX4_fault_SIM
git submodule update --init --recursive
python3 fault_release/prepare_dialect.py
make px4_sitl_default
```

The project tracks the authoritative custom MAVLink XML at `fault_release/dialects/fault_injection.xml`. The preparation script copies it into the **official pinned MAVLink submodule** for the PX4 build. The copied XML is deliberately not committed inside the upstream submodule. Run the preparation step again after reinitializing/resetting submodules.

### Start the local SIH test vehicle

```bash
make px4_sitl sihsim_quadx
```

At the resulting `pxh>` prompt:

```text
fault_manager start
mavlink start -x -u 14680 -r 4000000 -f -m custom -o 14541 -t 127.0.0.1
mavlink stream -u 14680 -s FAULT_ACK -r 20
mavlink stream -u 14680 -s FAULT_STATUS -r 20
mavlink status
```

The dedicated test link uses **PX4 UDP 14680 → Python UDP 14541**, on `127.0.0.1`. Verify this exact instance in `mavlink status`. An unrelated SIH custom link may appear on `19450/19410`. Run only one intended PX4 SITL instance at a time.

### Prepare Python and run smoke tests

In a second terminal, from the SIM repository root:

```bash
LAB="$HOME/px4_fault_test_lab"
python3 -m venv "$LAB/venv"
PY="$LAB/venv/bin/python"
"$PY" -m pip install 'pymavlink==2.4.50' 'fastcrc==0.5.0' 'lxml==6.1.3'
"$PY" fault_release/tests/prepare_pymavlink.py --install
mkdir -p "$LAB/results"

"$PY" fault_release/tests/smoke_no_effect.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/no_effect.json"
"$PY" fault_release/tests/smoke_imu_bias.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/imu_bias.json"
"$PY" fault_release/tests/smoke_gnss_position_bias.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/gnss_position_bias.json"
"$PY" fault_release/tests/smoke_motor_cap_interface.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/motor_cap_interface.json"
```

These clients **do not arm, take off, or send motor commands**. The GNSS test requires an initialized GNSS-capable SITL estimator; `sihsim_quadx` was used for the tested workflow. Execute the tests sequentially, since they share the test UDP ports.

A separate **motor-cap effect test and ULog numerical verifier** are documented in the SIM branch's [`fault_release/tests/README.md`](https://github.com/pbisonedgit/PX4_fault_injector/blob/sim/fault_release/tests/README.md). That check uses a *manually controlled, isolated software simulation* to produce actual motor commands; the fault command client itself does not initiate flight.

## Build for Pixhawk 6C / FMUv6C

Use a Linux environment with the PX4 v1.15 embedded toolchain:

```bash
git clone --branch real https://github.com/pbisonedgit/PX4_fault_injector.git PX4_fault_REAL
cd PX4_fault_REAL
git submodule update --init --recursive
python3 fault_release/prepare_dialect.py
make px4_fmu-v6c_default
```

The `real` branch includes the reviewed FMUv6C-specific firmware build configuration. **Building firmware is not authorization to flash or fly it.** Integration with an actual vehicle requires separate hardware, failsafe, regulatory, and flight-test review. The supplied localhost SITL smoke-test clients are for the `sim` branch only and must never be redirected to physical hardware.

## Interface and source layout

| Path | Purpose |
| --- | --- |
| `src/modules/fault_manager/` | Fault definitions, registry, lifecycle, runtime effects, feedback aggregation |
| `src/modules/ekf2/` | Sensor-side fault injectors at EKF2 consumption points |
| `src/modules/control_allocator/` | Motor bias/delay/maximum-output injector and signal-level measurements |
| `src/lib/fault_injection/` | Shared fault identity, validation, deterministic noise, and audit helpers |
| `msg/Fault*.msg` | uORB definitions for commands, effects, feedback, and measurements |
| `fault_release/dialects/fault_injection.xml` | Git-tracked MAVLink 2 dialect source of truth |
| `fault_release/prepare_dialect.py` | Installs the dialect into the initialized build submodule |
| `fault_release/tests/` | SIM-side Python setup, smoke tests, and ULog verification |

Custom MAVLink messages are `FAULT_COMMAND` (12956), `FAULT_ACK` (12957), `FAULT_STATUS` (12958), and `FAULTED_DOWNWARD_DISTANCE` (12959). The corresponding Python dialect is **not** installed merely by cloning; use `fault_release/tests/prepare_pymavlink.py` in an isolated Python virtual environment for the Python test client.

Fault status reports both lifecycle state and `applied_count`. Consumer-level audit and selected raw-versus-faulted signal measurements are separate from flight-outcome or simulator-fidelity assessment.

## Research context

The associated manuscript is **“A Parameterized and Auditable PX4 Fault-Injection Framework for Simulation and Physical UAV Flight”**, by Zhen Peng, Yuxiang Feng, and Arnab Majumdar. It describes the framework architecture, implemented modes, consumer-side auditing, and a matched AirSim–PX4 / physical-flight case study. This repository releases the **PX4 fault-injection component**, not a turnkey reproduction of all experiments in the manuscript. Bibliographic details can be added here after publication.

## Safety, provenance, and licences

This is experimental research software. Only use the supplied automated smoke tests with an **isolated, localhost-only SITL** process. The `--confirm-isolated-sitl` flag is an operator declaration, not a hardware detector. Do not connect the test ports to physical autopilots or forwarding routers. Save JSON reports, ULogs, Python virtual environments, and local test configurations outside the Git source tree.

This work builds upon [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) v1.15.2. The original PX4 source, submodules, copyright notices, and upstream/third-party licences remain applicable; see [`LICENSE`](LICENSE) and the notices in the relevant source files. The project is an independent research modification and is **not affiliated with or endorsed by the PX4 development team**.
