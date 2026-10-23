# Fault-injection SITL smoke tests (Linux / WSL)

The four scripts in this directory are standalone, **localhost-only** test
clients. They use a custom MAVLink2 `fault_injection` dialect and target an
isolated software PX4 SITL instance. They neither arm/take off nor send motor
commands. Never connect the test port to real flight hardware or a MAVLink
router forwarding to real hardware. The required `--confirm-isolated-sitl`
flag is a user safety declaration, **not** an automatic hardware detector.

## 1. Prepare a clean SIM checkout and virtual environment

Use a Linux/WSL host with PX4 SITL build dependencies. From this branch's root:

```bash
git submodule update --init --recursive
python3 fault_release/prepare_dialect.py
make px4_sitl_default
LAB="$HOME/px4_fault_test_lab"
python3 -m venv "$LAB/venv"
"$LAB/venv/bin/python" -m pip install 'pymavlink==2.4.50' 'fastcrc==0.5.0' 'lxml==6.1.3'
"$LAB/venv/bin/python" fault_release/tests/prepare_pymavlink.py --install
mkdir -p "$LAB/results"
```

Keep the virtual environment, reports and ULogs outside the Git working tree.
The tested Python environment was CPython 3.10 on Ubuntu/WSL. The installer
validates the authoritative XML's SHA256 and all four custom MAVLink message
IDs and CRC_EXTRA values. It refuses system-wide installation or overwriting a
different existing dialect. No Windows tools or private historical files are
required.

## 2. Start a dedicated, fresh SITL process (terminal A)

```bash
cd /path/to/PX4_fault_SIM
make px4_sitl sihsim_quadx
```

Use `sihsim_quadx` for the full suite: the `none` target may not deliver usable
GNSS samples to EKF2. First confirm that `vehicle_gps_position` is updating,
and allow EKF2 to initialize local/global position.

At `pxh>` run:

```text
listener vehicle_gps_position 1
ekf2 status
fault_manager start
mavlink start -x -u 14680 -r 4000000 -f -m custom -o 14541 -t 127.0.0.1
mavlink stream -u 14680 -s FAULT_ACK -r 20
mavlink stream -u 14680 -s FAULT_STATUS -r 20
mavlink status
```

Check `mavlink status` explicitly reports a **Custom** UDP instance with local
port `14680`, remote port `14541`, and partner IP `127.0.0.1`. PX4 might also
start a different Custom instance automatically (for example `19450/19410`);
that is **not** the test link. Only one PX4 SITL instance should be active.
Manually starting a new SITL instance does not carry over the previous custom
UDP configuration.

## 3. Run the tests (terminal B)

From the same SIM checkout's root:

```bash
LAB="$HOME/px4_fault_test_lab"
PY="$LAB/venv/bin/python"
"$PY" fault_release/tests/smoke_no_effect.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/no_effect.json"
"$PY" fault_release/tests/smoke_imu_bias.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/imu_bias.json"
"$PY" fault_release/tests/smoke_gnss_position_bias.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/gnss_position_bias.json"
"$PY" fault_release/tests/smoke_motor_cap_interface.py --px4-host 127.0.0.1 --confirm-isolated-sitl --output "$LAB/results/motor_cap_interface.json"
```

Run these tests sequentially; they reuse the same localhost UDP ports. Each
returns a nonzero exit status on failure. The GNSS test specifically returns
nonzero with `application_verified=false` if interface ACKs pass but the EKF2
consumer application counter does not increase. Such a result is **NOT
VERIFIED**, not a passing result. Check the GPS stream, `ekf2 status`, and
`listener fault_gnss_position_bias_applied 1` before any conclusions.

## What each test does and does not prove

- **NO_EFFECT**: manager lifecycle and zero-application audit.
- **IMU bias**: EKF2 application counts increase and stop following CLEAR;
  does not separately quantify numerical perturbations.
- **GNSS position bias**: with an initialized GNSS-capable SITL, demonstrates
  that the consumer-side count increases during the configured NED bias and
  freezes following CLEAR. To inspect final feedback, run
  `listener fault_gnss_position_bias_applied 1` and `fault_manager status`.
  The test does **not** independently establish the precise latitude/longitude/
  altitude shift or an actual trajectory consequence.
- **Motor cap interface**: tests SET/QUERY/CLEAR and status reporting for
  normalized cap `0.70`, motor mask `0x0f`, while **disarmed**. It does **not**
  generate commands above the cap or prove physical/numerical clipping.

All cleanup CLEAR_ALL requests in some test scripts are currently **sent but
not individually ACK-verified**. The test suites' PASS statuses do not imply
verified motor-stop authority or hardware safety. Inspect `fault_manager status`
when troubleshooting and retain JSON/ULog evidence outside this repository.

## Verification record

On a clean-upstream-based local `sim` branch, the WSL virtual environment and
MAVLink dialect generation/import were verified. The four named scripts were
run successfully in isolated SITL configurations. A dedicated GNSS-capable SIH
run produced consumer counts `0 -> 24 -> 47 -> 48`, with a post-CLEAR freeze
at `48` and `final_report=True` on the uORB consumer feedback. These are
example test results, **not** reproducible fixed output counts: rates depend
on timing and simulation configuration.

Motor-cap effect validation remains outstanding and should be added as a
separate test only after positive finite motor-command/cap-hit evidence exists.
