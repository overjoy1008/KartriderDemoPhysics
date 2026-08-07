# KartRider demo chase-camera recovery

## Scope and provenance

This note records code recovered from the original unpacked KartRider demo
executable.

Implementation status: the whole mode-0 update below is carried into the
simulator as `src/kart_camera.c` / `include/kart_camera.h`, covered by
`tests/test_camera_follow.c` — orientation follow, asymmetric speed filter,
chase pitch/distance/height, booster FOV, and the Z-only position smoothing.

Two things are present but unexercised. Mode 1 (`Front Chase Cameraman`, 100 ms)
is exposed as a constant but the demo only drives mode 0. The extra-pitch field
at `+0x3C` is an input that stays zero, because which original code writes it
has not been traced.

One deliberate numerical departure: the original normalizes quaternions with a
fast reciprocal-square-root approximation over runtime-initialized tables
(`0x00482490`); the port uses an exact square root, a difference of about 1e-7.

- Input: `input/KartRider.exe`
- Size: 1,769,472 bytes
- SHA-256: `812FB0FFD032A05C1F89478EA809B781E22EBEF12320D0A2BCE3745CFBFFB58C`
- Image base: `0x00400000`
- Relevant RTTI names preserved by the executable: `Cameraman`, `ChaseCameraman`, `KartReCameraman`, `SurroundCameraman`

The C below is normalized pseudocode reconstructed from the x86 program. Class and display names are direct RTTI/string evidence. Descriptive field and local names are assigned for readability because the original local-variable symbols are absent.

## Located class and update slot

`ChaseCameraman::RTTI_Type_Descriptor` is at `0x005998A8`. Its recovered vftable starts at `0x00572650`.

| vftable slot | address | recovered role |
|---:|---:|---|
| 7 | `0x00444C00` | reset/select chase mode |
| 8 | `0x00444C30` | update camera position and orientation |
| 9 | `0x00445320` | mode display name |

Slot 9 returns `Overhead Chase Cameraman` for mode 0 and `Front Chase Cameraman` for mode 1. The constructor at `0x00444B80` installs this vftable and selects mode 0.

## Recovered delayed-orientation path

The central result is in `ChaseCameraman` slot 8 at `0x00444C30`:

```c
// Source-equivalent normalized pseudocode, not newly invented simulator code.
void ChaseCameraman_Update(
    ChaseCameraman *camera,
    uint32_t now_ms,
    Vec3 *out_position,
    Mat3 *out_orientation,
    float *out_fov)
{
    Mat3 *kart_rotation = Kart_GetRotationMatrix(camera->kart); // 0x00412300
    Quat target = Quaternion_FromMatrix(kart_rotation);         // 0x00482180

    if (camera->reset_orientation) {
        camera->orientation = target;
        camera->reset_orientation = 0;
    } else {
        // q and -q encode the same orientation. Pick the same hemisphere so
        // interpolation follows the shorter rotational path.
        if (Quaternion_Dot(camera->orientation, target) < 0.0f) // 0x0042B800
            camera->orientation = -camera->orientation;         // 0x0042B770

        float follow_ms = camera->mode == 0 ? 400.0f : 100.0f;
        float alpha = ClampMax(
            (uint32_t)(now_ms - camera->previous_ms) / follow_ms,
            1.0f);                                               // 0x00447510

        camera->orientation = Quaternion_InterpolateNormalized(
            camera->orientation, target, alpha);                 // 0x0042B840
    }

    camera->previous_ms = now_ms;
    *out_orientation = Matrix_FromQuaternion(camera->orientation); // 0x0042B640

    // The remainder of 0x00444C30 applies the chase-camera offset and its
    // speed-dependent distance to produce out_position and out_fov.
}
```

The two follow times are immediate operands at their call sites, not loads from
`.rdata`; the `400.0f` in the constant table above is a different use of the
same value.

```text
00444DAF  PUSH 0x43C80000      400.0f   Overhead Chase Cameraman
00444DC2  CALL 0x00447510
00444DD4  CALL 0x0042B840

00445244  PUSH 0x42C80000      100.0f   Front Chase Cameraman
00445257  CALL 0x00447510
00445269  CALL 0x0042B840
```

Function `0x00447510` computes:

```c
alpha = min((uint32_t)(now_ms - previous_ms) / denominator_ms, 1.0f);
```

`0x0042B840` enters the interpolation implementation at `0x00481CD0`. That implementation obtains the quaternion dot product, computes an approximate spherical-interpolation weight, blends the two quaternions, and normalizes the result. It is therefore orientation interpolation, not a delayed Euler-yaw variable.

### The interpolation weight

`0x00481CD0` evaluates the weight on whichever half of the interval keeps the
parameter small, then blends componentwise and renormalizes:

```c
float cosine = Dot(a, b);
float w = t > 0.5f ? 1.0f - Weight(1.0f - t, cosine)   // 0x005710B8 = 0.5f
                   : Weight(t, cosine);
result = a + (b - a) * w;
Normalize(result);                                      // 0x00482490
```

`Weight` is `0x00482590`, a polynomial rather than a real slerp:

```c
float base = 1.0f - 0.8227968811988831f * cosine;   // 0x005758F0
float k    = 0.5854921936988831f * base * base;     // 0x005758F4
return ((2.0f * t - 3.0f) * k * t + 1.0f + k) * t;  // 0x00571948, 0x005722C0
```

`t = 0.5` is a fixed point for every angle. Below it the weight runs ahead of
`t`, and the wider the angle the further ahead, which is what lets a normalized
linear blend approximate constant angular speed. `0x00482490` normalizes with a
fast reciprocal-square-root approximation over runtime-initialized tables at
`0x005B1954`/`0x005B1958`.

## What creates the visible drift angle

The update does **not** read the kart's angular-velocity field. It samples the kart's current 3x3 orientation matrix each update and lets a persistent camera quaternion approach it over time. Thus the apparent angular-velocity lag is indirect:

1. the drifting kart rotates rapidly;
2. the target quaternion immediately contains that new orientation;
3. the camera quaternion only moves partway toward it using the 400 ms mode-0 coefficient;
4. the kart therefore appears turned sideways relative to the view until the camera catches up.

At approximately 60 updates per second, `alpha` is about `16 / 400 = 0.04` per update in overhead chase mode. This is the direct original mechanism that permits a large temporary kart/view angle during a deep drift.

## The claimed 90-degree maximum

No explicit 90-degree clamp was found in the recovered `ChaseCameraman` update or in its quaternion conversion/interpolation call chain.

- The executable's `90.0f` constant at `0x00576334` is referenced by `0x004AC3E0`, outside the cameraman path.
- The `+pi/2` and `-pi/2` constants found in the image likewise have no references from `ChaseCameraman` or its interpolation helpers.
- The `dot < 0` branch is quaternion hemisphere correction. It chooses a shortest equivalent quaternion arc; it is not a 90-degree display-angle limit.

Consequently, the original code directly proves delayed camera rotation, but it does not prove a hard 90-degree cap. A roughly 90-degree visible offset can be a dynamic result of kart rotation rate versus 400 ms camera following. Calling 90 degrees an exact source-code limit would be unsupported by this executable.

## Other original constants in the same update

The chase-position portion of `0x00444C30` contains the following direct constants:

| address | value | observed use |
|---:|---:|---|
| `0x00572678` | `60.0f` | speed-scaled offset divisor |
| `0x0057267C` | `0.03f` | speed-dependent distance term |
| `0x00572680` | `400.0f` | chase-pitch divisor (`00444FDA FDIV [0x00572680]`) |
| `0x00572684` | `5.5f` | base/minimum chase distance |
| `0x00572688` | `0.015f` | second speed-dependent distance term |
| `0x005722C0` | `3.0f` | camera-axis offset term |

## Speed-dependent chase geometry

`ChaseCameraman::Update` obtains the magnitude of the kart velocity through `0x00412360` followed by `0x004136F0`. The magnitude is not used raw for the chase geometry. A persistent filtered value at `this+0x24` is updated first:

```c
float response_ms = filtered_speed <= current_speed ? 10000.0f : 100.0f;
float beta = min((uint32_t)(now_ms - previous_ms) / response_ms, 1.0f);
filtered_speed = current_speed * beta + filtered_speed * (1.0f - beta);
```

This asymmetric original filter makes the camera geometry respond slowly while speed rises and quickly while speed falls. The filtered value then drives all of the following mode-0 equations:

```c
float chase_pitch = max(0.0f, filtered_speed) / 400.0f
                  + 0.25f
                  + camera->extra_pitch;

float rear_distance = max(
    5.5f,
    5.5f + filtered_speed * 0.015f + filtered_speed * 0.03f);

float upper_offset = filtered_speed / 60.0f + 3.0f;

out_orientation = delayed_kart_orientation * RotationX(chase_pitch);
out_position = kart_position
             - delayed_axis_1 * rear_distance
             + delayed_axis_2 * upper_offset;
```

`RotationX` is directly identified from `0x0047EE40`: it writes the matrix rows/columns containing `cos(angle)`, `-sin(angle)`, `sin(angle)`, and `cos(angle)` with the X diagonal fixed to 1. The final camera-position Z component is additionally smoothed with a 100 ms coefficient at the end of `0x00444C30`; the other two components are copied directly.

Thus ordinary speed changes three view properties even without a booster: pitch, rear distance, and upper offset. The units are the engine's native world units. No conversion to metres is performed in this path.

## Booster-linked FOV

The fourth output of `0x00444C30` is an FOV value stored persistently at `this+0x34`. It is initialized to `75.0f`. Each subsequent update selects a target and time constant from the kart's vtable slot `+0x68`:

```c
bool wide_view = kart->vftable[26](kart); // byte offset +0x68

float target_fov = wide_view ? 110.0f : 75.0f;
float fov_response_ms = wide_view ? 1000.0f : 1500.0f;
float gamma = min((uint32_t)(now_ms - previous_ms) / fov_response_ms, 1.0f);

camera->fov = target_fov * gamma
            + camera->fov * (1.0f - gamma);
```

RTTI/vftable recovery resolves that virtual slot as follows:

- `the::GoKart`, vftable `0x00571824`, slot 26 -> `0x0042A1C0`, returns byte `this+0xE5`.
- `the::GoPlayKart`, vftable `0x00571A7C`, slot 26 -> `0x00431B00`, returns `(this+0xE5) || (this+0x2D4)`.

The playable-kart event paths at `0x004529D0` and `0x00457AC0` set these fields through `0x00431960`/`0x00431AB0`; directly observed duration arguments include `1000` and `3000` ms in the booster-related event cases. The same state is carried in recorded kart data at record-frame offset `+0x81`.

Therefore the 75-to-110 widening is not a continuous function of ordinary speed. It is a smoothed state change tied to the kart's booster/boost-like runtime flag. Ordinary speed independently changes camera placement and pitch through the equations above.

The projection path confirms that these numbers are degrees:

- scene camera FOV is stored at camera field `+0x200` by `0x0045CE90`;
- `0x004C1450` passes near plane, far plane, FOV, and aspect ratio to `0x004A98A0`;
- at `0x004A98B1`, the projection builder multiplies FOV by `0.00872664f` (`pi / 360`) and calls tangent, producing `tan(FOV / 2)` for the perspective matrix.

This FOV expansion itself produces strong perspective stretching near the sides of the image and a visual impression of acceleration.

## Blur, distortion, wave and lens-flare audit

The executable contains direct visual-resource evidence for:

- UTF-16 resource names `booster`, `boosterwave`, and `shockwave`;
- loader/constructor path `0x00411180`, which retrieves all three from the `effect` resource category and creates effect instances;
- RTTI class `the::ReLensFlare` at `0x0059ABE4` and scene loader reference `0x0045A590` to the `lensflare` resource name.

These findings prove that booster/wave/shock visual nodes and an environment lens-flare renderer exist. They do not, by themselves, prove a full-screen blur or image-space distortion algorithm. The exact booster effect material can be supplied by the external effect resource rather than being encoded as a named algorithm in this EXE.

An exhaustive case-insensitive scan of the executable's ASCII and UTF-16 strings, followed by symbol/RTTI reference scanning, found no `blur`, `motion blur`, `distort`, `bloom`, `speedline`, or `postprocess` implementation name. No reference from `ReLensFlare` to kart velocity or the `ChaseCameraman` speed field was found either. The lens flare is therefore evidenced as a scene/environment effect, not a speed-dependent camera effect.

Current evidence supports this classification:

| effect | status in original EXE |
|---|---|
| speed-dependent distance/height/pitch | directly recovered with equations |
| booster FOV widening, 75 -> 110 degrees | directly recovered and projection path verified |
| delayed kart/camera orientation | directly recovered, 400 ms mode-0 follow |
| booster / boosterwave / shockwave resources | directly recovered as effect-resource construction |
| environment lens flare | directly recovered; no speed link found |
| dedicated full-screen motion blur | not found in this executable |
| dedicated speed-dependent image distortion | not found as executable code; external effect material remains possible |

## Raw evidence files

- `analysis/reports/camera-rtti.txt`: RTTI, complete-object-locator and vftable recovery
- `analysis/reports/camera-class-decompilations.c`: complete Ghidra decompilation of the cameraman class functions
- `analysis/reports/camera-helper-decompilations.c`: factor, quaternion conversion and interpolation wrappers
- `analysis/reports/camera-slerp-decompilation.c`: lower-level quaternion interpolation implementation
- `analysis/reports/camera-constants.txt`: direct constants and references, including the negative 90-degree result
- `analysis/reports/camera-position-math-helpers.c`: rotation-matrix and vector helpers used by the speed geometry
- `analysis/reports/camera-fov-state-functions.c`: GoKart/GoPlayKart vtable slot 26 implementations
- `analysis/reports/fov-state-writers.c`: runtime-state writers used by the FOV condition
- `analysis/reports/fov-state-callers.c`: event paths supplying 1000/3000 ms boost durations
- `analysis/reports/projection-build.c`: camera projection rebuild and FOV handoff
- `analysis/reports/camera-projection-disassembly.txt`: exact FOV half-angle and chase-camera instructions
- `analysis/reports/visual-effect-symbols.txt`: effect/lens-flare symbols and references
- `analysis/reports/visual-effect-decompilations.c`: booster, boosterwave, shockwave and lens-flare resource creation paths
