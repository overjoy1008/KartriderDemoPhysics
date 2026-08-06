# KartRider Demo physics recovery

This workspace contains a clean C reconstruction of the kart dynamics code in
the 2004 `KartRider.exe` demo. It is an evidence-driven behavioral recovery,
not the original source code.

## Current recovered scope

- the complete `Dynamics` parameter list and original fallback values;
- speed-attenuated steering angle;
- front/rear lateral slip and tire force calculation;
- grip, drift, and drift-trigger force modes;
- local lateral force plus roll/yaw torque outputs.
- four-contact suspension and gravity response;
- drift input/trigger state and original timing;
- linear/angular air drag and grounded quadratic drag;
- fixed-step linear velocity integration.
- the original `speed <= 5` non-drift lateral branch and `CornerDrawFactor`
  forward draw force;
- the runtime grounded-drag field, including the original `x4` enter and
  `x0.25` leave transitions;
- triangle-contact linear collision response with the original restitution and
  tangential-loss constants.
- forward, reverse, drift-escape, boost, grip-brake, and slip-brake forces.
- inverse-inertia construction and angular velocity integration.
- four-wheel ray contact generation and compression history;
- quaternion pose integration and the original anti-flip retry guard;
- hard/late body-collision angular response;
- an engine-independent world-query interface and complete fixed 5 ms
  simulation pipeline.
- the original HUD `|velocity| * 3.6` km/h conversion;
- all 26 playable demo kart parameter/model-AABB presets and all 15 demo track
  mesh-AABB sizes.

The source addresses are recorded next to recovered formulas. Raw Ghidra output
and ranking reports are kept under `analysis/reports`; reusable headless scripts
are under `scripts/ghidra`.

## Binary differential verification

`kart_binary_oracle.exe` starts the installed 32-bit demo, allocates an isolated
synthetic kart object, and calls the original fixed-address physics functions.
It does not modify the live game kart or save data. Two captured references are
kept under `analysis/reports`:

- 140 lateral-force/state frames covering grip, trigger, held drift, release,
  linger, and automatic slip;
- 240 chained frames covering force accumulation, drag, velocity integration,
  pose integration, and drift entry/maintenance/exit.

The current recovered implementation scores `1.000000` average similarity in
both suites. The trajectory suite's worst normalized scalar similarity is
`0.999989`, with zero drift-flag mismatches. These numbers describe the tested
synthetic flat/planar scenarios, not the unrecovered proprietary track query.

## Build

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

`kart_simulation.h` exposes callbacks for ground rays and body contacts. This
keeps the recovered dynamics independent of the demo's proprietary scene and
triangle-query implementation while preserving the original query geometry
and update order.

## Top-down demo

On Windows the build also produces `kart_topdown.exe`. It runs the full 3D
physics state on a flat world and renders only the X-Y projection using the
Win32 API, so no graphics package is required. It starts with the demo-selected
`burst3` and the original Forest Log (`forest_I01`) mesh-AABB footprint
`896.391 x 807.183`; the camera follows the kart over a 10-unit reference grid.
The top-right bounds panel shows the full track, the selected kart's exact
width and length, and its current position as an oriented triangle.

- Up: accelerate
- Down: reverse or brake
- Left, Right: steer
- Shift or `W`: drift
- Ctrl or `D`: start the original 3-second item boost
- `K`: open the 26-entry kart selection list
- `T`: open the 15-entry track selection list
- `G`: emulate the original ground-drag trigger enter/leave (`x4` / `x0.25`)
- `R`: reset

Both simulators show a large lower-right km/h gauge. The `K` and `T` popup
lists support mouse selection or the arrow keys and Enter, and check the
currently active preset.

The recovered original instant boost is automatic: finish a forward drift,
release Up, then press Up again while the HUD shows `INSTANT READY` (a 0.5
second window). It applies the original 1.5x forward-force multiplier for 0.5
seconds. Ctrl/`D` remains the separate external boost control.
The item boost starts only on a new key press while accelerating, lasts 3000
milliseconds, and ignores further booster presses while item or instant boost
is already active. Release and press Ctrl/`D` again after expiry to reuse it.

During any drift state, the renderer leaves fading twin rear-wheel skid marks.
Boost is shown by a cyan kart, a rear flame, and a `BOOST ON` HUD indicator.

## 3D demo

The Windows build also produces `kart_3d.exe`. It uses the same recovered
physics and world callbacks, with a software-rendered perspective chase camera,
3D ground grid, the same default Forest Log footprint and track walls, and a
kart body sized from the selected model AABB. Its input signs, frame stepping,
runtime drag trigger, item/instant boost handling, and
kart/track selection match the top-down demo. Fading rear-wheel skid marks,
boost flame and body color, heading/velocity vectors, slip telemetry, and the
full boost/drift HUD are projected into the chase-camera view. Controls match
the top-down demo. The ground grid follows the camera over the large original
track footprint, boundary lines are clipped at the camera near plane instead
of disappearing, and the top-right bounds map keeps the cyan track perimeter
visible even when the physical wall is hundreds of metres away. Both bounds
maps use an oriented kart triangle without a separate direction-vector line.
Escape
additionally closes the window.

## Confidence boundary

Parameter names/defaults and the core formulas are directly supported by the
listed executable addresses in `analysis/RECOVERY_NOTES.md`. Kart dimensions
and track bounds now come from the installed demo's model assets. The standalone
world is still a flat rectangle using each track's exact full-scene AABB; it
does not claim to reconstruct that track's road mesh or proprietary triangle
query.
