# New-cut input-path recovery

This note investigates the original demo behavior commonly called
"new-cutting": while a drift direction remains held, the opposite direction
is pressed briefly to arrest yaw inside the same drift session. The evidence
was collected before implementation; the final section records the subsequent
minimal simulator change.

## Result

The missing behavior is primarily in the **input-event arbitration layer**, not
a newly discovered tire-force formula.

Before the fix, the demos polled both direction keys and added them:

```c
steering_input = (left_down ? -1.0f : 0.0f)
               + (right_down ? 1.0f : 0.0f);
```

Therefore both held means zero. The original demo instead dispatches each key
transition to an event handler. The most recently pressed direction overwrites
the kart's steering field at `kart + 0x2ec`, so pressing the opposite direction
while the first remains held changes the value directly from one sign to the
other.

## Original executable evidence

### Steering setter and getter

`0x00431940` is a direct setter:

```asm
0043194d  mov [eax+0x2ec],ecx
```

`0x00454010` is a direct getter:

```asm
0045401a  fld dword ptr [eax+0x2ec]
```

There is no left-plus-right summation in either function.

### Input event handlers

Two gameplay handlers contain the same steering switch:

- `0x004529d0`
- `0x00457ac0`

For action `0`, a press calls `0x00431940(..., +1.0f)`. For action `1`, a
press calls it with `-1.0f`. Thus the later press owns `+0x2ec`.

Release is sign-conditional:

- releasing action `0` writes zero only if the current steering value is
  positive;
- releasing action `1` writes zero only if the current value is negative.

Consequently, releasing the older of two held directions does not cancel the
newer direction. Releasing the currently owning direction writes zero; it does
not automatically restore a still-held older direction until another input
event for that direction occurs.

### One drift session, not a second trigger

After either steering event, the handler reaches a shared drift block. On a
press, if the drift modifier is active and steering is nonzero, it calls
`0x00431a30(..., true)`. That function starts a trigger only when the linger
timer at `+0x2d0` is non-positive.

When the opposite key is pressed within the lockout window, the steering field
still flips immediately, but the second trigger is rejected. The existing
manual drift flag remains active. This is the executable-level reason the yaw
correction occurs inside one drift session.

## How the existing tire model produces the correction

Once the trigger phase has ended, held manual drift uses the unattenuated
maximum steer in the front-slip equation:

$$
S_f=\delta_{manual}-\frac{v_s}{D}-\frac{0.5\omega_z}{D}
$$

$$
S_r=-\frac{v_s}{D}+\frac{0.5\omega_z}{D}
$$

$$
F_f=0.2\,S_f(9.8mG_f),\qquad
F_r=0.2\,S_r(9.8mG_r)
$$

$$
F_y=F_f+F_r,\qquad
\tau_z=0.5(F_f-F_r)
$$

Changing steering directly from one sign to the other therefore reverses the
front-axle contribution without ending drift. The opposite yaw torque reduces
the existing angular velocity and pulls the body direction toward the velocity
direction.

The steering-angle history filter does not delay this sign reversal. Its clamp
applies only when the old and new filtered angles have the same sign.

## Numerical snapshot

The input state is taken from original `oracle-trajectory.csv` immediately
after frame 60, when the initial trigger has ended but manual drift remains:

```text
v_f = 19.2309513
v_s = -0.326185763
omega_z = 0.264597356
```

The already binary-validated lateral equations give:

| Effective steer | Lateral force | Yaw torque |
|---:|---:|---:|
| Opposite (`-1`) | -137.802521 | -92.262054 |
| Both summed to zero (`0`) | 33.239758 | -6.740907 |
| Original direction (`+1`) | 204.282043 | 78.780228 |

The current simultaneous-key mapping therefore removes most of the correcting
yaw torque: `-92.26` becomes only `-6.74` in this snapshot. Full intermediate
slip and axle-force values are recorded in
[`reports/new-cut-snapshot.csv`](reports/new-cut-snapshot.csv). The calculation
is reproducible with
[`tools/analyze_new_cut_snapshot.c`](tools/analyze_new_cut_snapshot.c).

## A second input-path mismatch

The original `0x00431a30` setter is called by key-transition events. The current
`kart_simulation.c` calls `kart_drift_set_input(..., controls->drift_input, ...)`
on every 5 ms substep. If Shift remains held until linger reaches zero, the
level-driven call can start another trigger without a new key transition.

This discrepancy is separate from the left/right cancellation, but it affects
the same maneuver and should be corrected at the input boundary rather than by
inventing another physics force.

## Implemented minimal correction

The Windows Top-down and 3D demos now share `KartSteeringInputState` from
`include/kart_input.h` and `src/kart_input.c`:

- a genuine key-down transition assigns that direction as owner;
- pressing the opposite direction overwrites the owner even while both are
  physically held;
- releasing the older direction leaves the newer owner unchanged;
- releasing the owner clears steering;
- auto-repeat key-down messages are ignored as non-transitions.

`KartSimulationState.previous_drift_input` now gates the recovered
`kart_drift_set_input` call. A held Shift/W level therefore generates one
setter call on press and one on release instead of one call per 5 ms substep.
No tire, force, torque, timer, drag, or camera formula was changed.

Regression coverage in `tests/test_kart_dynamics.c` checks both last-event
steering ownership and that a held drift key does not restart TRIGGER after
linger expires.

## Confidence boundary

Direct executable evidence establishes the last-event steering ownership,
sign-conditional release, shared drift call, and linger lockout. The numerical
snapshot begins with an original-oracle state and evaluates lateral formulas
that already match the original binary differential suite. A fresh extended
remote-oracle run was attempted, but Windows denied the cross-process remote
thread call with error 5 in the current environment; no result from that failed
run is treated as evidence.

The evidence supports fixing event semantics first. It does not support adding
a special named "new-cut force", a dedicated new-cut state, or guessed yaw
damping.
