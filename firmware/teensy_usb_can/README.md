# Teensy 4.1 USB classic-CAN adapter

Replaces Microchip/PCAN for knee node 1 and hip node 2 at 1 Mbps. The RH02
remains a separate USB adapter for the three ankle CAN-FD actuators.
This project implements a USB CDC SLCAN transport using FlexCAN_T4 CAN3.
It does not generate motor commands, zero motors, or change motor parameters.

## User's breakout: Adafruit CAN Pal layout

The supplied board photos match the Adafruit CAN Pal layout: three screw
terminals, a yellow termination switch, and Vcc/GND/RX/TX/SLNT/CANL/CANH pads.
This breakout includes a 5 V charge pump. Power its Vcc pad from Teensy **3.3 V**,
which also sets the logic levels to 3.3 V. Do not apply the bare-chip 5 V VCC
instruction below to this breakout's Vcc pad when interfacing with Teensy.

| Teensy / bus connection | Breakout pad |
|---|---|
| 3.3 V | Vcc |
| GND | GND |
| Pin 31 / CAN3 TX | TX |
| Pin 30 / CAN3 RX | RX |
| GND / normal operation | SLNT |
| Motor bus CAN-H | CANH / screw terminal H |
| Motor bus CAN-L | CANL / screw terminal L |
| Bus signal ground | Middle screw terminal |

Set termination ON only when this board provides one of the two endpoint
terminators. The switch adds 120 ohms (two 60-ohm resistors in series).
Source: https://learn.adafruit.com/adafruit-can-pal/pinouts

## Bare-chip wiring: TJA1051T/3 (not the TJA1051 without /3)

Wire with power disconnected. IC pin numbers below refer to the NXP SO8 part;
use signal labels on a breakout and check its schematic if it has onboard wiring.

| Teensy / bus connection | TJA1051T/3 signal | IC pin |
|---|---|---|
| Teensy pin 31, CAN3 TX | TXD | 1 |
| Teensy GND and bus signal ground | GND | 2 |
| USB-derived 5 V / Teensy VIN when USB-powered | VCC | 3 |
| Teensy pin 30, CAN3 RX | RXD | 4 |
| Teensy 3.3 V | VIO | 5 |
| Knee/hip CAN-L | CANL | 6 |
| Knee/hip CAN-H | CANH | 7 |
| GND, normal transmit mode | S | 8 |

VCC needs 4.5..5.5 V; VIO is 3.3 V for Teensy logic. Do not power VCC from
3.3 V or connect 5 V to a Teensy GPIO. Provide local 100 nF supply decoupling
at VCC and VIO for a bare-chip board. Use 120-ohm termination at the two bus
ends only; the firmware cannot add physical termination. Do not join this bus
to the ankle CAN-FD bus. First commission with a second analyzer, not motors.

Source: https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf

## Build and flash on Windows

```powershell
cd C:\cyber-leg\firmware\teensy_usb_can
pio run -e teensy41
pio run -e teensy41 -t upload
```

Press the physical Teensy Program button if the uploader cannot find the board.
An uploader exit code of zero with "No Teensy boards were found" is not proof
of successful flashing. Verify that a Teensy USB serial port appears and replies
to `V\r` with `V0101\r`. USB command checks can use `C`, `S8`, `V`, `N`, `F`;
none of these commands transmits a CAN frame. Keep CAN closed for these checks.

## Jetson: direct SLCAN, no additional kernel driver

List `/dev/serial/by-id/` after moving the Teensy to the Jetson. Use the entry
identifying Teensyduino as the rotary port and the Jhoinrch entry for RH02.
Never select an adapter from the ttyACM number alone: the numbers can change.
The account needs dialout access to both serial devices.

With TEENSY_PORT and ANKLE_PORT set to the actual persistent paths:

```bash
cd ~/cyber-leg
.venv/bin/python -u drivers/knee_bridge.py --bind 10.0.0.22 \
  --can-interface slcan --can-channel "$TEENSY_PORT" \
  --ankle-port "$ANKLE_PORT"
```

The bridge requires an explicit SLCAN channel. It opens one shared USB serial
connection for both rotary nodes, sets 1 Mbps, and starts disarmed. This direct
python-can backend does not require slcand, can1 or CONFIG_CAN_SLCAN.
The Teensy CDC baud setting is 115200; CAN bitrate is independently 1000000.

## Protocol and validation

Supported commands: C (close), S0..S8 (closed-channel bitrates), O (open), L
(listen-only), V, N, F, Z0 and classic frames t/T/r/R. Unsupported operations,
FD frames, invalid IDs, DLC > 8, malformed lengths, TX while closed/listening,
and a busy TX mailbox are rejected with BEL. ACK means accepted by the adapter,
not proof of reception by another CAN node. USB DTR disconnect closes CAN.
The adapter boots closed, sends no banner, and flushes each received frame
promptly over USB. It does not automatically retry application commands after
a communication fault. Motor timeout protection remains necessary: USB close
cannot retract a frame that already reached a motor.

Firmware V0101 corrects the FlexCAN listen-only enum and reads live error
registers for F status. The read-only D command reports CAN3 register values
for bench diagnosis without transmitting a CAN frame.

The frame parser/formatter can be tested without Teensy hardware:

```bash
g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Iinclude test/protocol_test.cpp -o /tmp/teensy_protocol_test
/tmp/teensy_protocol_test
```

Firmware compilation and parser tests do not establish real CAN latency,
hardware wiring or motor tracking. Measure request/reply latency and frame
loss at the expected traffic rate before commissioning motor movement.
