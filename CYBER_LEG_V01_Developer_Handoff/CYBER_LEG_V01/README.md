# CYBER LEG CAD motion lab

Interactive kinematic simulation derived from CYBER_LEG_CAD.STEP (SolidWorks AP203 export, 56,253,361 bytes). All 114 CAD solids are retained. Pre-tessellation at 0.28 mm / 0.28 rad yields 461,852 triangles, 10,249,560 bytes of mesh data. Display materials are assigned for legibility; analytic STEP geometry is unchanged in the source file.

## Run

Use the deployed private Site. All runtime dependencies and CAD mesh data are local Site assets. No CDN, browser-side OpenCascade, STEP selection, Python installation or Windows launcher is required for the hosted application.

For development, serve `dist` with any static HTTP server. JavaScript modules require HTTP; opening index.html via file:// is unsupported.

## Controls

- Choose Manual or Auto gait, then Start. Stop freezes the current pose; Reset clears the pose, clock and recorded telemetry. Switching mode resets the sequence.
- Manual: knee slider, spring-return ankle joystick (drag or arrow keys), and ankle height offset.
- Automatic: prescribed knee/upper-mount gait, moving treadmill, configurable speed, deterministic uneven surface, random small stones, and optional footprint-support ankle correction.
- Cover visibility, wireframe, joint axes, front/side/3D views, orbit, zoom, right-drag pan, and click to identify a CAD part.
- Live actuator extensions, pitch/roll correction graph and CSV export. CSV retains the latest 72,000 samples; graph displays ten simulated seconds. Hidden tabs pause the simulation.

## Rig and assumptions

Source Y remains vertical. Display X is source Z; display Z is negative source X. Units are mm.

The knee rotation axis is display Z through (-7.4, 387.77, 0.831), inferred from the rotor's circular bore. ROTOR_LEVERAGE is the upper moving part; MAIN_BASE_HOLDER and the remaining upper hardware move with the shank. The ankle origin is the lower mount's center (0.435, -16.962, 0.831). The calibration controls adjust the two vertical pivot coordinates.

Three actuator instances are retained by assembly ordering. Their upper and lower attachment centers are inferred from CAD clevis/bearing bounds. Body and piston groups rotate along each calculated attachment vector; the sliding eye/shaft solids from each 500N Linear Actuator are grouped with its PISTON. Extension is 20 mm plus the change in attachment distance. The 20 mm reference and 0–50 mm travel are assumptions, not specifications extracted from STEP. Commands exceeding travel are reduced towards the reference pose by bisection.

Treadmill geometry and feedback use the same deterministic height function. Automatic stance positions move with belt speed; vertical clearance is calculated from a distributed sample of the actual sole vertices. The mounting is translated vertically to maintain geometric nonpenetration and swing clearance. This does not simulate passive balance or a physically constrained hip. The gait is prescribed, and no claim of measured human gait or validated prosthetic performance is made.

No mass/inertia, torque, force, collision/contact dynamics, friction, motor dynamics, real sensor stream, or hardware control is included. The source STEP does not supply motion mates. Foot internals move as a rigid assembly. Joint axes and grouping require engineering confirmation before interpreting motion as mechanically validated.

## Validation

- All 114 mesh buffer ranges and triangle indices verified.
- 2,000 combined pitch/roll/height poses checked against assumed actuator travel.
- Rotation math checked against Three.js quaternion transforms (maximum error below 8e-15).
- Reference lengths, gait cycle continuity and flat terrain verified.
- JavaScript syntax, all referenced HTML IDs and local module/asset paths checked.
- No browser UI/WebGL execution test was performed in this build.

## Rebuild CAD mesh

`python scripts/convert_cad.py /path/to/CYBER_LEG_CAD.STEP dist`

Requires CadQuery/OCP and NumPy. Original STEP remains in the user's existing project files. Three.js 0.180.0 is vendored under its MIT license in dist/LICENSE.

## Small-stone trials

The Small stones switch enables 44 procedurally generated faceted stones on a 2,800 mm repeating belt. Eight randomized placements target alternating sole edges; the remainder are scattered across the belt. Maximum height is adjustable from 2 to 15 mm (default 8). Randomize stones generates a fresh layout and resets the run; ordinary Reset preserves the layout for comparison. Stones can be tested on a flat belt by disabling Uneven treadmill.

The visible octagonal profile and the clearance height query use the same geometry. During stance, a grid over the foot footprint samples maximum support on each side and in front/back regions. Differences generate bounded pitch and roll targets with a 100 ms response filter. Swing feedback decays to neutral; disabling Adaptive ankle removes feedback while retaining stones and geometric clearance. Left means positive display Z. CSV includes supportDelta (left minus right height, mm). This is an estimated rigid-foot support response, not a contact-force solver.

Validation for the stone change: isolated mirrored 8 mm stones produced opposite roll commands of approximately ±9.63°; an empty flat belt produced zero roll. Conveyor transport and periodic wrap, deterministic/randomized layouts, and mesh/profile coordinate agreement were checked. An eight-step seeded run exercised both roll directions. Browser rendering was not tested.

## Translucent walking dummy

A simplified adult above-knee amputee is attached to the upper knee rotor through an illustrative translucent socket. The residual thigh is shown inside that socket, ending above the prosthetic knee. The socket base uses the upper rotor attachment height in CAD coordinates. The body follows the affected hip attachment while remaining upright. The contralateral leg uses a two-link geometric solve and an opposite-phase foot trajectory; arms swing in opposition. The belt is widened for the second foot.

Walking dummy visibility and opacity (10–65%) can be adjusted independently of the original CAD. Hiding the dummy restores close framing of the cyber leg. Reset and Stop also reset or freeze the dummy. The body and socket are approximate visual aids, not patient-specific anatomy, socket fitting, balance, load or gait validation. Unreachable contralateral foot targets are clamped to the leg's geometric reach.

Validation: 120 synchronized poses checked for finite transforms and exact hip attachment; two-link lengths, opacity, visibility, cleanup, module syntax and asset references verified. No browser rendering test was performed.
