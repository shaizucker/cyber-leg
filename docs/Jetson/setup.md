# Jetson Orin Nano bridge and simulator

Prepared on feature/hip, 2026-10-25. The target reports Jetson Linux 36.5.0,
kernel 5.15.185-tegra and approximately 8 GB RAM. USB PCAN is currently can1;
the ankle SLCAN CAN-FD adapter is ttyACM0. Native mttcan can0 is not used.

## Verified on the target by the operator

- PEAK USB 0c72:000c detected. The kernel has CONFIG_CAN_PEAK_USB unset.
- PEAK out-of-tree pcan driver built and loaded in netdev/SocketCAN mode.
- can1 is UP, ERROR-ACTIVE at 1000000 bit/s, with zero reported error counters.
- Ankle USB 16d0:117e detected with cdc_acm as /dev/ttyACM0.
- Account halalit belongs to dialout and has read/write access to ttyACM0.

These checks establish host/adapter setup, not successful communication with
powered motors. Interface names and settings must be rechecked after reconnects.

## Transfer and install

The prepared archive contains the simulator, bridge runtime, hardware-free tests,
dependencies, launcher and these instructions. It excludes Windows executables,
CAD source, Git metadata and runtime logs.

From Windows PowerShell, substitute the Jetson's actual hostname or IP:

```powershell
scp C:\cyber-leg\tmp\cyber-leg-jetson.tar.gz halalit@JETSON_IP:~/
```

Over SSH:

```bash
tar -xzf ~/cyber-leg-jetson.tar.gz -C ~
cd ~/cyber-leg-jetson
sudo apt update
sudo apt install python3-venv
python3 -m venv .venv
.venv/bin/python -m pip install -r drivers/requirements-bridge.txt
```

If the adapter was unplugged or the Jetson rebooted, identify the USB pcan
interface again with `ip -details link show type can`. With that adapter named
can1 and currently DOWN:

```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

No motor movement is requested by installation or CAN interface setup.
The out-of-tree driver must match the running kernel; after a kernel upgrade,
rebuild it against that kernel's headers. Loading pcan and bringing can1 up
were tested manually; automatic startup persistence has not been configured.

## Hip coordinate update

The current package requires hip motor ID 2 to use CAN COM Theta MIN/MAX
**-0.523599 / +0.523599 rad**, with mechanical zero at neutral hip (0 degrees).
This supersedes the earlier hip 0..1.83 rad setting and -20-degree zero.
Save/read back the parameters while disarmed and establish the neutral
reference before arming this version. Hip defaults use P/D 20/4 for responsive tracking;
these gains do not impose a motor torque cap. Keep the separately configured
[21] motor-side limit appropriate for a supported commissioning setup. See docs/Hip/local_bridge.md
for motor-side protection and the unresolved Torque Limit units question.

## Launch

```bash
cd ~/cyber-leg-jetson
bash drivers/start_jetson_bridge.sh --only hip
```

Use `--only knee`, `--only ankle` or `--only all` as needed. Without --only,
all outputs are selected. Hardware connections are opened only by Arm or a
zero operation; startup and viewing the page do not arm motors.

The equivalent explicit Python invocation is:

```bash
.venv/bin/python drivers/knee_bridge.py \
  --can-interface socketcan --can-channel can1 \
  --ankle-port /dev/ttyACM0 --only hip
```

For persistent serial naming, pass the full path shown by
`ls -l /dev/serial/by-id/` to `--ankle-port`. The shell launcher also accepts
ANKLE_PORT, CAN_CHANNEL and BRIDGE_PYTHON environment variables; command-line
options override its connection defaults.

Linux defaults to socketcan/can1 and /dev/ttyACM0. Windows retains
pcan/PCAN_USBBUS1 and COM7. The bridge does not run privileged interface setup
commands; SocketCAN bitrate is configured with ip before starting it.
Both rotary joints remain on classic CAN at 1 Mbps. The ankle driver continues
using its serial SLCAN CAN-FD protocol and existing 1M/5M setup.

## View from the Windows computer

The bridge remains bound to 127.0.0.1, with its existing Host/Origin/token checks.
Use a second Windows terminal for the SSH tunnel:

```powershell
ssh -N -L 8000:127.0.0.1:8000 halalit@JETSON_IP
```

Open http://127.0.0.1:8000/ in the Windows browser. Stop any Windows bridge
already using port 8000. The tunnel's local port must match the bridge port
because of the Host and Origin checks. If another port is needed, start the
bridge with `--port 8001` and tunnel `8001:127.0.0.1:8001` instead.

With this arrangement, the browser computes and renders the simulation on the
Windows computer while the Jetson runs the motor bridge. To move both execution
and rendering to the Jetson, open the same localhost URL in a browser running
on the Jetson's graphical desktop. Remote browser operation continues to depend
on timely browser updates; stale updates disarm the selected hardware.

## Validation

Platform tests verify Linux and Windows defaults, explicit backend/channel
selection, persistent serial paths and accurate startup messages. The existing
knee/hip/ankle and output-selection tests use fake hardware. Physical motor
communication and Jetson-local graphics performance remain to be verified.

Run only the hardware-free suites below; other test scripts in the full
repository include physical motor bring-up routines.

```bash
.venv/bin/python -m unittest discover -s drivers -p test_jetson_bridge.py
.venv/bin/python -m unittest discover -s drivers -p test_output_selection.py
.venv/bin/python -m unittest discover -s drivers -p test_knee_bridge.py
.venv/bin/python -m unittest discover -s drivers -p test_hip_bridge.py
.venv/bin/python -m unittest discover -s drivers -p test_ankle_bridge.py
.venv/bin/python -m unittest discover -s drivers -p test_knee_safety.py
```

## Online motor panel

The current bundle adds the Motors - configure & monitor panel, independent
rotary tuning, grouped ankle PID and saved signed zero references. See
../Motors/online_configuration.md for operation, interlocks, persistence,
capabilities and the external power-sensor API. Include motor_console.py and
motor-console.js when updating an existing installation. Preserve your local
bridge_settings.json; it is deliberately excluded from the transfer archive.
