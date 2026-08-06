# Recovery notes

## Binary identity

- Input: `KartRider.exe` (`Kart Rider Client` 1.19.0.1)
- SHA-256: `812fb0ffd032a05c1f89478ea809b781e22ebef12320d0a2bce3745cfbffb58c`
- Format: PE32 i386, image base `0x00400000`
- Linker: MSVC 7.1-era linker
- PE timestamp: 2004-04-12 11:53:34
- Embedded PDB path: `d:\Project Files\Winter\Game\Client\Run\KartRider.pdb`

The binary is not packed. Relocations, COFF symbols, and line information are
stripped, but MSVC RTTI and many parameter/resource strings remain.

## High-confidence functions

| Address | Provisional name | Evidence |
|---|---|---|
| `0x0042e190` | `GoKart_LoadDynamics` | Loads the `Dynamics` section and every named physics parameter. |
| `0x0042f460` | `GoKart_AccumulateSuspensionForce` | Four-contact loop, weight terms, force and torque accumulation. |
| `0x0042f6c0` | `GoKart_AccumulateLongitudinalForce` | Uses forward/reverse acceleration, grip/slip brake, mass, and drift escape force. |
| `0x0042fc40` | `GoKart_AccumulateLateralForce` | Speed steering attenuation, axle slip, tire forces, drift state/timers, roll/yaw torque. |
| `0x0042e750` | `GoKart_UpdatePhysics` | Fixed-step frame driver and sole caller of the three force routines. |
| `0x0042ef90` | `GoKart_QueryWheelContacts` | Four downward contact queries, compression history, and average ground normal. |
| `0x00430640` | `GoKart_AccumulateDrag` | Linear/angular air friction and grounded quadratic drag. |
| `0x00430740` | `GoKart_IntegrateVelocities` | Semi-implicit linear and angular velocity update. |
| `0x00430ed0` | `GoKart_IntegratePose` | Position and quaternion/orientation integration with up-axis correction. |

## Confirmed dynamics layout

These offsets are relative to the object passed to `0x0042e190`.

| Offset | Recovered field | Default |
|---:|---|---:|
| `0x128` | mass | `100.0` |
| `0x15c` | air friction | `3.0` |
| `0x160` | drag factor | `0.5` |
| `0x164` | forward acceleration force | `3000.0` |
| `0x168` | backward acceleration force | `2000.0` |
| `0x16c` | grip brake force | `2000.0` |
| `0x170` | slip brake force | `1500.0` |
| `0x174` | maximum steer angle | `10.0` |
| `0x178` | steer constraint | `30.0` |
| `0x17c` | front grip factor | `5.0` |
| `0x180` | rear grip factor | `5.0` |
| `0x184` | drift trigger factor | `0.05` |
| `0x188` | drift trigger time | `0.1` |
| `0x18c` | drift slip factor | `0.2` |
| `0x190` | drift escape force | `5000.0` |
| `0x194` | corner draw factor | `0.0` |
| `0x198` | drift lean factor | `0.07` |
| `0x19c` | steer lean factor | `0.01` |

## Confirmed lateral model

The steering angle at `0x0042fcde` is:

```text
steer = radians(maxSteerAngle)
      * steeringDirection
      * steeringInput
      * exp(-abs(forwardVelocity / steerConstraint))
```

At speed above the low-speed threshold, the axle slip terms are:

```text
frontSlip = signedSteer - lateralVelocity / speed
                         - 0.5 * yawLeverVelocity / speed
rearSlip  =              - lateralVelocity / speed
                         + 0.5 * yawLeverVelocity / speed
```

Normal tire force is `slip * 9.8 * mass * gripFactor`. Drift multiplies both
axle forces by `driftSlipFactor`. The original then transforms the summed local
lateral force by the kart orientation and accumulates yaw and lean torques.

Binary differential testing exposed two additional steering rules. During held
drift input and the trigger phase, tire force uses the unattenuated maximum
steer angle; grip, automatic slip drift, and linger use the speed-attenuated
angle. Also, while forward input remains active and steering keeps the same
sign, `0x2c0/0x2c4` prevent the attenuated angle from growing again as speed
falls: the prior smaller magnitude is retained. Adding this hysteresis changed
the 240-frame trajectory similarity from `0.995114` to `1.000000`.

Drift lean's half-strength threshold compares total planar speed to `10`, not
lateral velocity alone.

## Remaining uncertainty

- State bytes at `0x2c8`, `0x2c9`, and `0x2ca` form the drift transition state
  machine, but their exact source-level names remain provisional.
- Several object fields hold already-combined geometric values, so
  `yaw_lever_velocity` is intentionally conservative naming.
- Collision geometry helper names remain provisional, although both linear
  response branches and their angular corrections are now recovered.

## Fixed-step pipeline

`0x0042e750` converts integer milliseconds to seconds with `0.001` and limits
each internal step to 5 ms. Each substep performs:

1. clear accumulators and derive body axes/velocity components (`0x0042eda0`);
2. cast four wheel contacts (`0x0042ef90`);
3. accumulate suspension, longitudinal, and lateral forces;
4. accumulate air friction and grounded drag (`0x00430640`);
5. integrate velocities (`0x00430740`);
6. integrate position/orientation (`0x00430ed0`);
7. resolve body collision (`0x00430830`).

## Suspension response

The original gravity vector is `(0, 0, -58.8)`. Per-wheel static force is
`abs(gravity) * mass * 0.5`; rebound damping is 20% of that value and
compression damping is zero. Four flat contacts at compression `0.5` exactly
balance gravity. Contact torque uses the wheel lever arm crossed with the local
up force, scaled by `0.1`.

Raw evidence is in `reports/core-physics-decompilations.c`,
`reports/physics-field-uses.txt`, and `reports/physics-triage.txt`.

## Drift state offsets

| Offset | Meaning | Evidence |
|---:|---|---|
| `0x2c8` | drift input active | Set/cleared directly by `0x00431a30`. |
| `0x2c9` | slip-detected drift | Set by the high-speed slip test in `0x0042fc40`. |
| `0x2ca` | drift trigger phase | Set with input, cleared when the trigger timer expires. |
| `0x2cc` | trigger timer | Initialized to `DriftTriggerTime` (`0.1`). |
| `0x2d0` | drift linger/input lockout | Initialized to twice the trigger time (`0.2`). |
| `0x2dc` | entry direction flag | Captures whether forward velocity was positive. |
| `0x2d4` | instant boost active | Set when acceleration is newly pressed during the opportunity window. |
| `0x2d8` | instant boost timer | Initialized to `0.5` seconds. |
| `0x2e0` | instant boost opportunity | Opened for `0.5` seconds when a forward drift finishes. |

New drift input is accepted only while the linger timer is non-positive.
With no input or trigger phase active, automatic slip drift is detected when
`abs(lateralVelocity) > 1.2 * abs(forwardVelocity)`. During the active drift
branch the linger timer decreases as `max(timer - dt, 0)`.

## Collision contact and linear response

The contact record returned by `0x00433310` is eight 32-bit values: contact
point `float[3]`, triangle normal `float[3]`, sweep fraction, and a surface
identifier. In `0x00430830`, an incoming contact has `dot(normal, velocity) < 0`.

For a sweep fraction at or below `0.65`, the horizontal correction is:

```text
normalComponent = normal * dot(normal, velocity)
tangent          = velocity - normalComponent
tangentLoss      = min(1.5 * length(normalComponent),
                       0.6 * length(tangent))
correction       = -1.5 * normalComponent - normalize(tangent) * tangentLoss
correction.z     = 0
```

This leaves a normal restitution of `0.5`. Later contacts use
`tangent - 0.2 * normalComponent`, giving restitution `0.2`.

For those later contacts, the original also adjusts local angular velocity:

```text
wallTurn = normal x bodyUp
angularVelocity.x -= dot(wallTurn, bodyRight)   * 0.1
angularVelocity.y += dot(wallTurn, bodyForward) * 0.1
```

The sign of the second line follows from the executable storing the forward
axis as the negated second orientation column.

The early/hard branch computes a deterministic local yaw kick. The apparently
parameterless helper at `0x00431b70` is `fabsf`; correcting its prototype shows
that the code selects the dominant body-axis component of the wall normal:

```text
speed = clamp(abs(dot(normal, velocity)), 1, 30)
f = dot(normal, bodyForward)
r = dot(normal, bodyRight)

if abs(f) <= abs(r):
    yawKick = f * (r <= 0 ? 1 : -1) * speed
else:
    yawKick = r * (f <= 0 ? -1 : 1) * speed
```

The kick is added to local angular velocity Z only when
`yawKick * angularVelocity.z <= 1`.

## Instant boost state machine

The tail of `0x0042fc40` detects a completed forward drift. If the previous
lateral frame was drifting, neither manual nor automatic slip remains, and no
opportunity is already open, it clears the entry-direction flag and writes
`0.5` to `+0x2e0`.

`GoKart::SetAccel` at `0x00431960` consumes that opportunity only when forward
acceleration is newly enabled. It clears `+0x2e0`, writes `0.5` to `+0x2d8`,
and sets `+0x2d4`. Thus holding acceleration through the drift does not trigger
the boost; the player must release and press it again within the opportunity
window. `0x0042eda0` decrements both timers and clears `+0x2d4` at expiry.

While either instant boost `+0x2d4` or external timed boost `+0x0e5` is active,
`0x0042f6c0` multiplies forward force by `1.5`. The dedicated binary oracle
captures 107 state/force rows: active boost produces `4500` from the default
`3000` force through tick 99 and returns to `3000` on tick 100.

## External timed booster

`GoKart::StartTimedBoost` at `0x00431ab0` accepts an integer millisecond
duration. It starts only while forward acceleration `+0x2e4` is nonzero, stores
the duration at `+0x158`, and sets `+0x0e5`. At the tail of `0x0042e750`, the
frame driver subtracts `min(remaining, elapsedMilliseconds)` and clears the flag
when the counter reaches zero.

The item-input caller at `0x00457ac0` checks the virtual `IsBoosting` result
before consuming an item, then calls `StartTimedBoost(..., 3000)`. `IsBoosting`
is the logic recovered at `0x00431b00` and returns true for either external
timed boost `+0x0e5` or instant boost `+0x2d4`; consequently another item boost
cannot start while either kind is active. A separate start-timing reward caller
uses the same function with `1000` milliseconds.

## Input and longitudinal force fields

| Offset | Meaning |
|---:|---|
| `0x2e4` | forward input (`0` or `1`) |
| `0x2e8` | reverse/brake input (`0` or `1`) |
| `0x2ec` | analog steering input |
| `0x2f0` | reverse-steering direction flag |
| `0x0e5` | external timed acceleration boost active |

The normal forward force is `3000`; slip-detected drift substitutes the escape
force `5000`. Timed boost multiplies either by `1.5`. When braking, normalized
velocity dotted with the forward axis above `0.8` selects the grip brake force
`2000`; lower alignment selects the slip brake force `1500`.

`0x0e6` is not reverse gear: race setup and finish/retire callers set it while
active racing clears it. It is a drive-disabled/frozen state that forces the
deceleration path.

## Angular integration

The inverse inertia matrix at `0x134` is initialized as a diagonal matrix with
each diagonal equal to `12 / mass`. The angular half of `0x00430740` computes:

```text
inertiaVelocity    = inverseInertia * angularVelocity
gyroscopic         = angularVelocity x inertiaVelocity
effectiveTorque    = accumulatedTorque - gyroscopic
angularAcceleration = inverseInertia * effectiveTorque
angularVelocity    += angularAcceleration * dt
```

## Longitudinal input state machine

The field at `0x2f4` is a reverse-transition timer. Reverse input is braking
while forward speed is above `0.5`. In the low-speed band, the timer advances;
the original snaps velocity to zero when lateral speed is at most `0.2`, then
permits reverse acceleration once the timer exceeds `0.2` seconds. Reverse
travel at or below `-0.5` forces the timer to `1.0`, keeping reverse available.

With no reverse input, time spent in the `[-0.5, 0.5]` band also advances the
timer. Forward input resets it to zero. When forward input is applied while the
kart still travels backward, an additional recovery force of
`min(speed, 5) * mass * 9.8` is added; active drift uses the full speed instead.

The frozen/drive-disabled flag follows the same transition path but never
applies reverse drive: it either applies the grip brake or directly zeroes the
linear velocity at low speed.

## Pose integration and anti-flip guard

`0x00430ed0` first advances position by `linearVelocity * dt`. Orientation uses
a local-angular-velocity quaternion derivative:

```text
q = normalize(q + (q * quaternion(0, angularVelocity)) * (dt * 0.5))
```

If the resulting body-up Z component is below `0.5`, the original restores the
old orientation, multiplies angular X and Y by `0.1`, and retries up to three
times. If the kart still fails the threshold, it sets angular X/Y to zero and
integrates yaw alone. Position is not rolled back during these retries.

## Wheel queries and standalone simulation

`0x0042ef90` casts four wheel rays. The right signs are `+ - + -`; the forward
signs are `+ + - -`. Each ray starts at:

```text
position
  + bodyRight   * halfWidth  * rightSign   * 0.8
  + bodyForward * halfLength * forwardSign * 0.8
  + bodyUp      * suspensionRange
```

Its displacement is `bodyUp * (-2 * suspensionRange)`. For a hit, compression
is recovered as:

```text
bottomHeight = dot(position, bodyUp) - suspensionRange
compression  = clamp(dot(hitPoint, bodyUp) - bottomHeight,
                     0, 2 * suspensionRange)
delta        = compression - previousCompression
```

The contact normal stored for each wheel feeds suspension force, and the mean
of active normals becomes the aggregate ground normal. A transition from no
contacts to any contact produces the one-step landing flag.

The standalone `kart_simulation` module preserves `0x0042e750`'s integer-time
split: every call is processed in chunks of at most 5 ms and each chunk uses
`milliseconds * 0.001` seconds. The substep order is wheel query, grounded or
airborne force accumulation, drag, linear/angular integration, pose
integration, then body collision. The airborne branch clears drift, applies
`(0, 0, -58.8) * mass`, and adds `-angularVelocity * 30` torque.

Scene lookup remains a caller callback because the executable's track spatial
index and triangle storage are outside kart physics. The callback boundary
returns precisely the point/normal/fraction data consumed by the recovered
code.

## Differential oracle results

`tools/oracle/kart_binary_oracle.c` invokes original functions in a separate
synthetic object allocated inside the demo process. Captured outputs are:

- `reports/oracle-lateral.csv`: 140 rows of force, torque, flags, timers, and
  steering angle;
- `reports/oracle-trajectory.csv`: 240 rows of position, linear velocity,
  angular velocity, quaternion, flags, and timers.

CTest replays both files through the recovered C implementation. Current
results are:

```text
lateral:   average 1.000000, worst scalar 0.999999, flag mismatches 0
trajectory: average 1.000000, worst scalar 0.999989, flag mismatches 0
```

The trajectory deliberately excludes proprietary scene queries and suspension
contacts so that it measures the kart force/state/integration chain without
track geometry noise. Suspension, wheel rays, and collision response remain
covered by formula-level unit tests rather than live-track differential data.
