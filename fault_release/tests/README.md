# Fault-injection SITL tests (Linux / WSL)

These tests use a custom MAVLink 2 `fault_injection` dialect and an **isolated, localhost-only PX4 SITL** process. The MAVLink test clients send fault definitions and queries; they do **not** arm, take off, or send motor commands. Never connect the test endpoints to an actual autopilot or a MAVLink router that forwards to hardware. `--confirm-isolated-sitl` is a user declaration, not automatic hardware detection.

## 1. Install from the `sim` branch

Install the PX4 v1.15 SITL toolchain, then run from a checked-out `sim` branch:

```bash
git submodule update --init --recursive
python3 fault_release/prepare_dialect.py
make px4_sitl_default

LAB="$HOME/px4_fault_test_lab"
python3 -m venv "$LAB/venv"
PY="$LAB/venv/bin/python"
"$PY" -m pip install 'pymavlink==2.4.50' 'fastcrc==0.5.0' 'lxml==6.1.3'
"$PY" fault_release/tests/prepare_pymavlink.py --install
mkdir -p "$LAB/results"
```

The isolated-venv installer generates the Python dialect from the tracked `fault_release/dialects/fault_injection.xml`; it checks its SHA-256 and the four authored MAVLink messages' IDs/CRC extras. Keep the Python environment, reports and flight logs outside the Git tree. The tested environment was Ubuntu/WSL Python 3.10.

## 2. Start and configure an isolated SITL instance

In terminal A:

```bash
make px4_sitl sihsim_quadx
```

At `pxh>`:

```text
listener vehicle_gps_position 1
ekf2 status
fault_manager start
mavlink start -x -u 14680 -r 4000000 -f -m custom -o 14541 -t 127.0.0.1
mavlink stream -u 14680 -s FAULT_ACK -r 20
mavlink stream -u 14680 -s FAULT_STATUS -r 20
mavlink status
```

Ensure EKF2 has initialized for the GNSS test. The `none` target can run the simpler interface/IMU tests but may not supply usable GNSS samples. The **test** MAVLink link must read **local UDP 14680, remote UDP 14541, partner `127.0.0.1`**; SIH may also create a separate, unrelated `19450/19410` link. Stop any previous SITL processes before starting the new instance.

## 3. Run the four non-flying smoke tests

From terminal B in the `sim` checkout:

```bash
LAB="$HOME/px4_fault_test_lab"
PY="$LAB/venv/bin/python"
"$PY" fault_release/tests/smoke_no_effect.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/no_effect.json"
"$PY" fault_release/tests/smoke_imu_bias.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/imu_bias.json"
"$PY" fault_release/tests/smoke_gnss_position_bias.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/gnss_position_bias.json"
"$PY" fault_release/tests/smoke_motor_cap_interface.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/motor_cap_interface.json"
```

Run these sequentially because they share UDP ports. Their specific scope is:

| Script | Tested behaviour |
| --- | --- |
| `smoke_no_effect.py` | SET/QUERY/CLEAR/CLEAR_ALL; PENDING/ACTIVE/EXPIRED transitions; zero applications |
| `smoke_imu_bias.py` | EKF2 accelerometer and gyroscope bias consumer application counts increase during ACTIVE and freeze following CLEAR |
| `smoke_gnss_position_bias.py` | EKF2 GNSS position bias consumer feedback increases with valid GNSS data and freezes after CLEAR |
| `smoke_motor_cap_interface.py` | Motor cap SET/QUERY/CLEAR, acknowledged definitions, and status behaviour while disarmed |

`smoke_gnss_position_bias.py` uses a nonzero exit status when the command interface works but application cannot be demonstrated. Examine `vehicle_gps_position`, `ekf2 status`, `fault_manager status`, and `listener fault_gnss_position_bias_applied 1`. Counts depend on simulator scheduling and are not fixed expected constants. Some test cleanup CLEAR_ALL requests are transmitted but not separately ACK-verified.

## 4. Motor cap: consumer application and numeric verification

Two additional programs complement the non-flying motor interface test:

- `smoke_motor_cap_effect.py` sends cap SET/QUERY/CLEAR/CLEAR_ALL and checks for positive consumer application counts and post-CLEAR count stability. It **never** arms or commands flight. A positive count alone is *not* numerical verification.
- `verify_motor_cap_ulog.py` reads saved ULogs only. It independently validates per-motor raw-vs-faulted values, the applied mask, the configured cap and the final consumer feedback. No simulator connection is needed.

### Configure logging before the run

By default, your ULog might not contain the custom uORB audit topics. With **all SITL instances stopped**, create a local configuration under the SITL build directory:

```bash
STORAGE="$(pwd)/build/px4_sitl_default/rootfs"
TOPICS="$STORAGE/etc/logging/logger_topics.txt"
if [ -e "$TOPICS" ]; then
    echo "[STOP] Existing topic configuration; review it before replacement: $TOPICS"
else
    mkdir -p "$(dirname "$TOPICS")"
    cat > "$TOPICS" <<'TOPICS_EOF'
fault_motor_output_measurement 0
fault_motor_max_output_degradation_applied 0
actuator_motors 0
vehicle_status 0
TOPICS_EOF
fi
```

This is a **dedicated audit configuration**: in this PX4 version, a populated `logger_topics.txt` replaces rather than supplements the default set. Verify the logger's startup/status output and, after the run, verify that the ULog actually contains both `fault_motor_*` topics. Do not commit the build-directory configuration. Remove it after this dedicated test to restore normal logging selection.

### Produce and capture positive clipping (software-only)

Restart `make px4_sitl sihsim_quadx`, set up `fault_manager` and the localhost MAVLink test link as above, and confirm `logger status` and `listener fault_motor_output_measurement 1`.

After EKF2 and preflight checks are ready, an operator may **manually** use `commander takeoff` in this isolated software simulator to produce motor outputs. Do not bypass failed preflight checks, and never perform this procedure on hardware. In a second terminal, while finite motor outputs are being produced:

```bash
LAB="$HOME/px4_fault_test_lab"
PY="$LAB/venv/bin/python"
"$PY" fault_release/tests/smoke_motor_cap_effect.py \
  --px4-host 127.0.0.1 --confirm-isolated-sitl \
  --cap 0.20 --mask 0x0f --hold-s 12 \
  --output "$LAB/results/motor_cap_effect_020.json"
```

The **0.20** cap provides positive clipping at ordinary SIH motor command levels; it is a functional-check setting, not the platform-calibrated degradation used in the accompanying cross-domain paper. An ordinary hover may never exercise a 0.70 cap. Following the test, manually land the simulated vehicle (`commander land`) and close the SITL logger cleanly.

### Independently validate saved motor-output values

```bash
"$PY" -m pip install 'pyulog==1.3.0' 'numpy==2.2.6'

# Replace this with the exact absolute path reported by the current SITL logger:
ULOG="/absolute/path/to/current/log.ulg"
"$PY" fault_release/tests/verify_motor_cap_ulog.py \
  --ulog "$ULOG" \
  --probe-json "$LAB/results/motor_cap_effect_020.json" \
  --output "$LAB/results/motor_cap_numeric_020.json"
```

The validator requires `fault_motor_output_measurement` and `fault_motor_max_output_degradation_applied` in the ULog. For the selected motors, it checks numerical clipping against the configured cap and compares the per-frame applied mask; it also requires a positive final consumer report. The validated run used `cap=0.20`, mask `0x0f`, with **3,046 numerically checked clipping frames** and a final application count of **3,049**. These are example evidence counts, not hard-coded pass thresholds: logged frames can be sampled at a different rate from the consumer's cumulative counter.

The numerical checker isolates records without simultaneous motor bias/delay. A combined-fault scenario requires different expectations (delay, then bias, then cap), as defined in the accompanying paper.

## Reproducibility and cleanup

Retain test JSON and ULogs outside the Git tree, with hashes when archiving results. Do not stage or commit virtual environments, ULogs, local `logger_topics.txt`, or simulator runtime files. After motor auditing, remove only the **dedicated** `build/px4_sitl_default/rootfs/etc/logging/logger_topics.txt` if you created it for this test; inspect any pre-existing custom configuration before changing it.

This suite checks fault infrastructure, not the full mission, flight-performance, or physical-aircraft experimental workflow described in the research paper.
