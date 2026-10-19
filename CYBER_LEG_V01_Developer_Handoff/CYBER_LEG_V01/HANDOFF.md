# CYBER LEG V01 — developer handoff

Exported 2026-10-18 from the current CYBER LEG CAD Simulator Site source.
Source commit: 7a14475e61854bb33d98ac6e6992dd531f2c7477
Reference Site: https://cyber-leg-cad-lab.lmaryasin.chatgpt.site

## Continue using your own account
Upload this ZIP to your own ChatGPT/Codex project. Ask it to read HANDOFF.md and README.md, inspect the existing code, and continue editing this simulator. This is an independent copy; changes do not update Leon's original Site. If publishing with Sites, register a NEW Site in your own account.

Suggested first prompt:
“Extract this project and read HANDOFF.md and README.md. Preserve the existing CAD geometry and all controls. Run it locally, check loading and controls, then wait for my requested changes. Do not rebuild it from scratch. Distinguish kinematic visualization from force/contact simulation.”

## Run locally
Install Python 3 if not already installed. From the extracted project folder run:

    python -m http.server 8000 --bind 127.0.0.1 --directory dist

On Windows, `py` can replace `python`. Open http://localhost:8000 in a browser with WebGL 2/hardware acceleration. Keep the terminal open. Do not double-click index.html; ES modules and asset fetches require HTTP. No npm build is required. Runtime Three.js and meshes are included.

## Source map
- dist/app.js: rendering, CAD rig, controls, stepping, telemetry and CSV.
- dist/motion.js: gait, terrain/stones, support approximation, actuator limits.
- dist/dummy.js: translucent amputee dummy and illustrative socket.
- dist/index.html and style.css: interface.
- dist/cad.json and cad.bin: original pre-tessellated CAD, 114 solids.
- cad/CYBER_LEG_CAD.STEP: original editable CAD source.
- scripts/convert_cad.py: mesh conversion (requires CadQuery/OCP and NumPy).
- dist/LICENSE: bundled Three.js license.

## Current features and engineering limits
Manual/automatic gait, spring-return ankle joystick, knee and height controls, three actuators, treadmill, random stones, adaptive pitch/roll approximation, walking dummy, part selection and telemetry are retained.
The simulation prescribes motion; it does not solve contact forces, torque, inertia, friction, balance or motor dynamics. Joint axes/part grouping are inferred and need engineering verification. Actuator reference extension 20 mm and travel 0–50 mm are assumptions. See README.md for detailed mechanics and earlier validation claims.

## Verification of this export
Mesh buffer bounds and every triangle index were checked again while exporting. ZIP integrity and exact copies are verified. This handoff has not been browser/WebGL tested. README validation statements describe earlier development, not a new browser test.

## Scope and ownership
Original source files are copied unchanged. The original README's private-Site wording is historical; the reference Site is currently public. Git metadata, credentials and original Site deployment identity are excluded. The complete V01 chat transcript is not included; README and these notes carry the implementation context.
