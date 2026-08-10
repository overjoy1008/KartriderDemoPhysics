# KartRider demo skid-mark recovery

All statements below come from the 2004 demo `KartRider.exe` and its
`Data/effect.rho`; no replacement behavior or guessed constants are included.

## Asset

- Runtime resource key: `skidmark` (`FUN_00470f50`, `0x00471106` onward).
- Archive entry: `effect.rho/skidmark/skidmark.tga`.
- Extracted file: `Assets/Models/Karts/demo_effect/skidmark/skidmark.tga`.
- TGA header: 64 x 32, uncompressed true-color (type 2), 32 bpp, 8 alpha bits.
- SHA-256: `AEB659F86F660D95787A8760846004E6B7C9E54E69A651505F22C3514F73E89A`.

## Pool and geometry

- `the::SkidMark` RTTI: `0x0059a184`; vftables `0x005748c8` and `0x005748e0`.
- `the::SkidMarkManager` RTTI: `0x0059a1a0`; vftable `0x0057491c`.
- Manager constructor `FUN_00470860` allocates 50 left and 50 right
  `SkidMark` objects.
- Each `SkidMark` owns a vertex buffer created with stride `0x14` (20 bytes)
  and capacity `0x66` (102), plus an index buffer with capacity `0x6a` (106).
- Vertex layout written by `FUN_00470110` / `FUN_00470470` is five floats:
  position xyz and texture uv. Each cross-section adds two vertices.
- Constructor constants at object offsets `+0x1a8` and `+0x1ac` are exactly
  `0.28f` and `0.02f`. The code multiplies the lateral unit vector by 0.28 and
  then by 0.5, producing half-width 0.14 and total strip width 0.28. The z
  component receives the exact `+0.02f` bias.
- A strip starts with two vertices. Appends add two vertices and the strip is
  disabled when the vertex-count guard `count + 14 > 100` fires.
- UV V is accumulated from `elapsed_ms * scale * 0.001f`, where the exact
  scalar at `0x00571a2c` is `0.00100000005f` (the stored float for 0.001f).

## Track-surface application

- Kart-side setup `FUN_00411180` sets the two wheel-local offsets to
  `(-0.61, 0.5, 0)` and `(0.61, 0.5, 0)`.
- Per-frame caller `FUN_00410850` passes the kart transform/position vectors,
  wheel/contact state, and a normalized direction from the kart collision state
  into `SkidMarkManager::update` (`FUN_00470b80`).
- `FUN_00470b80` forwards both wheel offsets to `FUN_00470110` and
  `FUN_00470770`. Those functions transform the offsets through the supplied
  contact/kart frame (`FUN_00449b00`, `FUN_004138d0`) and build the ribbon at
  that resulting 3-D position, with the 0.02 bias. There is no write of a fixed
  world `z = 0` plane in this path.
- Emission is enabled only when both contact predicates requested as indices 2
  and 3 by `FUN_004123e0` are true. Breaking contact ends the current pair of
  strips.

Raw Ghidra evidence is in `analysis/reports/skidmark-*.{c,txt}`.
