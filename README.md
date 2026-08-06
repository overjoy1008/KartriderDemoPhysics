# KartRider Demo physics recovery

This workspace contains a clean C reconstruction of the kart dynamics code in
the 2004 `KartRider.exe` demo. It is an evidence-driven behavioral recovery,
not the original source code.

## Simulator preview

### Top-down

![Top-down simulator drifting with skid marks, item boost indicator, speedometer, and track bounds](docs/assets/simulator-topdown.png)

### 3D

![3D simulator using the fixed chase camera with boost flame, speedometer, ground grid, and track bounds](docs/assets/simulator-3d.png)

Both programs run the same 3D physics state and 5 ms substep pipeline. The
top-down program projects that state onto X-Y; the 3D program renders it with
the existing fixed chase camera. These screenshots were captured from the
current Windows build while drifting and using the item boost.

## Run the included simulators

No build is required to try the checked-in applications.

On **Windows**, open one of these files:

- `build-win/kart_topdown.exe` — top-down view
- `build-win/kart_3d.exe` — 3D chase-camera view

Each Windows executable is self-contained and can be copied by itself to
another 64-bit Windows 10/11 computer. No asset directory or separate MinGW
runtime DLL is required.

On **macOS**, open one of these application bundles:

- `build-macos/Kart Physics Top Down.app` — top-down view
- `build-macos/Kart Physics 3D.app` — 3D chase-camera view

On macOS, if Gatekeeper blocks the first launch, Control-click the application,
choose **Open**, and confirm once. The ZIP files in `build-macos` are optional
distribution copies; the `.app` bundles are the files to run directly.

## Read the recovery

- **[Physics engine textbook](docs/PHYSICS_ENGINE_TEXTBOOK.md)** builds the
  current implementation bottom-up from vectors and coordinate axes through
  suspension, tire forces, drift states, boosts, drag, collision, and
  quaternion integration. Its equations and update order follow the C code.
- [Recovery notes](analysis/RECOVERY_NOTES.md) connect recovered formulas to
  executable addresses and supporting reports.
- [Differential-oracle results](analysis/RECOVERY_NOTES.md#differential-oracle-results)
  record the original-EXE calls and frame-by-frame comparison results; the
  captured CSV references are in [`analysis/reports`](analysis/reports/).

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

## Build from source

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

`kart_simulation.h` exposes callbacks for ground rays and body contacts. This
keeps the recovered dynamics independent of the demo's proprietary scene and
triangle-query implementation while preserving the original query geometry
and update order.

After a Windows build, launch `build/kart_topdown.exe` or
`build/kart_3d.exe`. Both simulators use the following controls:

| Action | Windows | macOS |
|---|---|---|
| Accelerate / brake or reverse | Up / Down | Up / Down |
| Steer | Left / Right | Left / Right |
| Drift | Shift or `W` | Shift or `W` |
| 3-second item boost | Ctrl or `D` | Command or `D` |
| Choose kart / track | `K` / `T` | `K` / `T` |
| Toggle grounded-drag trigger | `G` | `G` |
| Reset | `R` | `R` |

Press `K` or `T` to open a selectable list. Use the mouse, or the arrow keys
and Enter, to apply one of the 26 kart presets or 15 track bounds immediately.
There is no mode-switch key: top-down and 3D are separate executables that
share the same physics implementation.

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
For the original new-cut input, keep the first steering direction held and
press the opposite direction during the same drift. The most recently pressed
direction owns steering instead of the two keys cancelling to zero, so the
existing opposite yaw torque can straighten the kart without a guessed extra
force.

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

## macOS demos

The same top-down and 3D simulations can be built as native Cocoa application
bundles on macOS. The macOS renderer uses only AppKit/Core Graphics supplied by
the operating system; no third-party graphics package is required. Physics,
kart/track presets, flat-world collision callbacks, skid marks, bounds map,
speedometer, and the `K`/`T` selection menus are shared in behavior with the
Windows demos. The existing fixed 3D chase view is preserved.

On a Mac with CMake and either Xcode or the standalone Xcode Command Line
Tools installed:

```sh
bash scripts/build_macos.sh
```

This builds and tests both applications, then creates directly executable app
bundles plus ZIP copies for distribution:

- `build-macos/Kart Physics Top Down.app`
- `build-macos/Kart Physics 3D.app`
- `build-macos/kart_topdown-macos.zip`
- `build-macos/kart_3d-macos.zip`

CMake's intermediate files are kept separately in the hidden
`.build-macos` directory; only `build-macos` contains files intended for use or
distribution.

macOS uses native `.app` bundles rather than Windows `.exe` files. Extract a
zip and open the contained application. The script applies a local ad-hoc
signature; because it is not an Apple Developer ID signature, the application
may require Control-click, then **Open**, the first time it is launched.

Controls match Windows except that the item-boost modifier is **Command**:

- Arrow keys: drive and steer
- Shift or `W`: drift
- Command or `D`: boost
- `K`: kart list
- `T`: track list
- `G`: grounded-drag trigger
- `R`: reset

## Confidence boundary

Parameter names/defaults and the core formulas are directly supported by the
listed executable addresses in `analysis/RECOVERY_NOTES.md`. Kart dimensions
and track bounds now come from the installed demo's model assets. The standalone
world is still a flat rectangle using each track's exact full-scene AABB; it
does not claim to reconstruct that track's road mesh or proprietary triangle
query.
