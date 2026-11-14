# Project Spec: Cyber-Leg Prosthetic Control System

Updated November 8, 2026. See [project.md](project.md) for implementation status
and [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md) for durable configuration.

## Hardware architecture

- Host: NVIDIA Jetson Orin Nano Super developer kit.
- Knee/hip: rotary IDs 1/2, shared classic CAN at 1 Mbps through Teensy 4.1
  project SLCAN firmware and a TJA1051T/3 CAN Pal-style transceiver.
- Ankle: Seenpin linear IDs 1/2/3 on a separate RH02 Plus USB serial CAN-FD
  interface, 1 Mbps arbitration / 5 Mbps data with BRS.
- Windows LAN browsers access the Jetson-hosted application.
- PCAN and Microchip are not the current deployed rotary control interface.

## Current implementation

- CAD simulation with manual joint controls and automatic gait.
- Python bridge with selectable physical outputs and P-T-M rotary control.
- Online PID, limits, cap settings, telemetry and signed reference capture.
- Parameter readback and motor fault checks before enabling outputs.
- Bounded reply deadlines, simulator watchdogs and coordinated stopping.
- Knee motor-angle ratio 0.5; hip ratio 1.0; separate software/CAN ranges.
- Machine-local motor profiles excluded from Git.

## Current objectives

- Validate sustained coordinated motion after the Jetson scheduling fix,
  preserving feedback and freshness stops.
- Commission references, protections, torque/current caps and tracking under load.
- Characterize latency and loss at intended gait speeds.
- Maintain repeatable launch, USB identification and troubleshooting procedures.

## Future work

- External DC sensors for measured electrical consumption.
- Optional dedicated real-time control layer. The current Teensy is a USB-CAN
  adapter; it does not plan or autonomously maintain leg motion.
- Optional ROS 2 / MuJoCo integration and third-party APIs.
- Automatic service startup only when explicitly requested and commissioned.
