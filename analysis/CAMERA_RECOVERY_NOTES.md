# KartRider demo chase-camera recovery

## Scope and provenance

This note records code recovered from the original unpacked KartRider demo executable. It does not describe simulator code and no simulator behavior was changed during this investigation.

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

The mode-0 assembly directly pushes IEEE-754 `0x43C80000` (`400.0f`) before calling `0x00447510`. Mode 1 uses `0x42C80000` (`100.0f`). Function `0x00447510` computes:

```c
alpha = min((uint32_t)(now_ms - previous_ms) / denominator_ms, 1.0f);
```

`0x0042B840` enters the interpolation implementation at `0x00481CD0`. That implementation obtains the quaternion dot product, computes an approximate spherical-interpolation weight, blends the two quaternions, and normalizes the result. It is therefore orientation interpolation, not a delayed Euler-yaw variable.

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
| `0x00572680` | `400.0f` | overhead orientation follow time |
| `0x00572684` | `5.5f` | base/minimum chase distance |
| `0x00572688` | `0.015f` | second speed-dependent distance term |
| `0x005722C0` | `3.0f` | camera-axis offset term |

## Raw evidence files

- `analysis/reports/camera-rtti.txt`: RTTI, complete-object-locator and vftable recovery
- `analysis/reports/camera-class-decompilations.c`: complete Ghidra decompilation of the cameraman class functions
- `analysis/reports/camera-helper-decompilations.c`: factor, quaternion conversion and interpolation wrappers
- `analysis/reports/camera-slerp-decompilation.c`: lower-level quaternion interpolation implementation
- `analysis/reports/camera-constants.txt`: direct constants and references, including the negative 90-degree result

