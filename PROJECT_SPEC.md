\# Project Spec: Cyber-Leg Prosthetic Control System



\## Hardware Architecture

\- \*\*Master Controllers:\*\* Raspberry Pi 5 / NVIDIA Jetson Orin Nano

\- \*\*Knee Joint:\*\* 1x Large rotating motor (Standard CAN interface)

\- \*\*Ankle Joint:\*\* 3x Linear actuators (CAN-FD interface)

\- \*\*Communication:\*\* CAN / CAN-FD bus network



\## Phase 1 Objectives (Current Step)

\- Basic angle and motor command control only (open-loop/simple position).

\- No torque, load sensing, or hard safety limits yet.

\- Establish baseline CAN communication scripts.

* create basic GUI interface 



\## Future Phases



\- Phase 2: Closed-loop control (Torque, limits, feedback loops).

\-Phase 2: Basic control for Linear actuators

\- Phase 3: 3rd-Party API \& User UI.

\- Phase 4: Integration with Simulation Environments (MuJoCo / ROS 2).

