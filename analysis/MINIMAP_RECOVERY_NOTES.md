# Original minimap recovery notes

This document records only values and behavior recovered from the 2004 demo
executable and its shipped RHO data. Function names without symbols remain
their Ghidra addresses.

## Verified input and resources

- `input/KartRider.exe` is byte-identical to the installed demo executable.
  SHA-256: `812FB0FFD032A05C1F89478EA809B781E22EBEF12320D0A2BCE3745CFBFFB58C`.
- RTTI identifies `the::Minimap`, `the::ToMinimap`, and
  `the::ViewportPanel`.
- The concrete minimap scene is `stage_drivinggame/minimap.1s`. It is loaded
  by the literal resource name `minimap` in `FUN_00467050`.
- Every track supplies a 256x256 `xt_minimap.png` and one serialized
  `ToMinimap` object in `track.1s`.

## Panel and camera

`stage_drivinggame/stage.xml` contains this concrete panel payload:

```xml
<RenderPanel>
<Name>minimap</Name>
<ClientRect>576 209 768 401</ClientRect>
<UV>576 209 768 401</UV>
<Plane>1.0 800.0 90.0</Plane>
<Enable>True</Enable>
<Visible>True</Visible>
<NotTexture/>
</RenderPanel>
```

Thus the render rectangle is 192x192 and the camera values are near 1.0,
far 800.0, and FOV 90.0 degrees. The constructor `FUN_00466ec0`
independently installs exactly those three defaults at offsets 0x1f8,
0x1fc, and 0x200.

`FUN_004c1450` passes near, far, FOV, and `panel_height/panel_width` to
`FUN_004a98a0`. The latter constructs a perspective projection (not an
orthographic projection). For this square panel the aspect argument is 1.0.

The generic panel loader `FUN_004c17d0` recognizes `camera`, `scene`,
`cameraName`, `clearZBefore`, `clearZAfter`, `nearPlane`, `farPlane`, and
`fov`. That is not the concrete Minimap construction path. The demo Minimap
allocates its camera internally and directly loads `minimap`; no serialized
`cameraName` is consumed by this path and `minimap.1s` contains no camera
node. Assigning it a camera name would therefore invent a value absent from
the original payload.

The initial camera/scene transform installed by `FUN_00467050` has columns
`(-1,0,0)`, `(0,0,-1)`, `(0,1,0)` and translation `(0,0,128)`.

## Contents of `minimap.1s`

This is a genuine 3D scene, but it does not render track geometry from above.
It renders a textured map plane and separate marker geometry:

- node `minimap`: position `(128,128,0)`, scale `(1,1,1)`, texture
  `xt_minimap`; quad corners `(-1024,1024,0)`, `(-1024,-1024,0)`,
  `(1024,1024,0)`, `(1024,-1024,0)` and UVs `(-3.5,-3.5)`, `(-3.5,4.5)`,
  `(4.5,-3.5)`, `(4.5,4.5)`;
- node `me`: triangle vertices `(16.587,-16.396,0)`,
  `(0.274,21.517,0)`, `(-16.587,-16.396,0)`;
- node `other`: ten-vertex round marker;
- nodes `kart00` through `kart07`: 15x15 sprite quads using sequential
  regions of texture `aw_01`.

The complete decoded hierarchy and property values are in
`analysis/reports/minimap-scene.txt`.

## Track mapping payload (`the::ToMinimap`)

Serialization and accessors establish this exact layout:

```text
+0x10 float origin_x
+0x14 float origin_y
+0x18 float scale
+0x1c uint32 width
+0x20 uint32 height
+0x24 uint32 serialized field (4 in every shipped track; unused by update)
```

`FUN_00450f60`, `FUN_0049e880`, `FUN_0044bcf0`, and `FUN_0049e8c0` return
the origin, scale, width, and height respectively. Exact per-track values,
including full float precision, are in
`analysis/reports/minimap-track-values.json`. Width and height are 256 for
all 13 demo tracks.

For a player world position `P`, `FUN_00467550` and `FUN_00467840` both use:

```text
C = (width * 0.5, height * 0.5)
Q = C + (P.xy - origin) * scale
```

The marker translation is `(Q.x,Q.y,0.1)`. World Z is explicitly discarded.
The camera base point uses the same `Q`.

## Player fields and marker direction

The caller of `FUN_00467550` copies these fields by value before the call:

```text
stage + 0x5c + player_index * 0x0c : 3-float world position
stage + 0xbc + player_index * 0x24 : 3x3 orientation matrix (9 floats)
```

These meanings follow from their exact sizes and downstream vector/matrix
operations, not from accessor-name guesses. The update extracts column 1 of
the orientation matrix, sets its Z to zero, normalizes it, and negates it for
marker basis column 1. It then constructs the remaining orthonormal columns
with cross products against global `(0,0,1)`.

The marker scale evaluates exactly to `(1,1,1)`: constructor field 0x248 is
96.0, the global scale vector is `(1,1,1)`, and the update computes
`global_scale * 96 / field_0x248`.

## Camera following

Player zero's position and orientation are retained at Minimap offsets
0x24c and 0x258. `FUN_00467840` converts the orientation to a quaternion and
interpolates the retained quaternion toward it with:

```text
t = min((frame_time_ms - previous_frame_time_ms) / 1500.0, 1.0)
```

It flips one quaternion when their dot product is negative before
interpolation. The camera translation is the mapped point `Q` plus the
smoothed forward/basis direction multiplied by 96.0. Evidence for the exact
decompiled operations is retained in `analysis/reports/minimap-marker-basis.c`
and `analysis/reports/minimap-coordinate-path.c`; an implementation should
preserve their matrix column convention.

## Consequence for implementation

The original path needs no guessed track bounds and no guessed top-down
track camera. Its inputs are the shipped `xt_minimap`, the per-track
`ToMinimap` origin/scale/size, the 192x192 perspective camera above, the
player world-position/orientation arrays, the recovered 96-unit following
offset, and the geometry/materials in `minimap.1s`.

