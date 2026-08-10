"""Derive the C track constants in Scripts/Runtime/Gameplay/kart_demo_data.c from the KTRK exports.

Reads Assets/Tracks/meshes/track_<code>.ktrk (produced by
DeveloperTools/AssetPipeline/export_track_meshes.ps1) and prints the TRACKS[] AABB arguments and the
TRACK_SCENES[] start-line table.

Every value comes from one source so that scene placement, the safety wall, the
minimap normalization and the spawn all agree:

  AABB       full mesh-vertex bounds, exactly as recorded in the KTRK header
  start x/y  centroid of the road-flagged mesh whose texture contains "start"
  ground z   that quad's Z (all start quads are flat)
  axis       the quad's SHORT horizontal span; the stripe crosses the road, so
             the racing direction is perpendicular to its long edge

The +/- sign of the racing direction is NOT derivable from the mesh data. It is
assumed to be the +axis and must be confirmed against the original game; see
AXIS_CONFIDENCE below and docs/TRACK_ASSET_PIPELINE.md.

Usage:  python DeveloperTools/AssetPipeline/derive_track_constants.py
"""

import glob
import os
import struct

MESH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "analysis", "track-assets", "meshes")

# A real start stripe is a long thin quad across the road. Candidates below this
# long/short ratio are too square for the axis to be read off the geometry.
CONFIDENT_ASPECT = 3.0


def load(path):
    b = open(path, "rb").read()
    if b[0:4] != b"KTRK":
        raise ValueError("not a KTRK file: " + path)
    version, mesh_count = struct.unpack_from("<2I", b, 4)
    if version != 1:
        raise ValueError("unsupported KTRK version %d" % version)
    off = 20
    minimum = struct.unpack_from("<3f", b, off); off += 12
    maximum = struct.unpack_from("<3f", b, off); off += 12
    meshes = []
    for _ in range(mesh_count):
        name = b[off:off + 96].split(b"\0")[0].decode("utf-8", "replace"); off += 96
        texture = b[off:off + 96].split(b"\0")[0].decode("utf-8", "replace"); off += 96
        flags, vertex_count, index_count = struct.unpack_from("<3I", b, off); off += 12
        raw = struct.unpack_from("<%df" % (vertex_count * 5), b, off)
        off += vertex_count * 20
        off += index_count * 4
        meshes.append((name, texture, flags,
                       [raw[i * 5:i * 5 + 3] for i in range(vertex_count)]))
    return minimum, maximum, meshes


def start_quad(meshes):
    """Best start-line candidate: a flat "start"-textured quad, most elongated.

    The road flag is deliberately not required. It comes from a substring match
    on the node name, which misses Korean node names: ice_I02's start floor is
    r_<hangul>+@s1, textured ice_i02_start, and is flagged as scenery. Requiring
    the flag left that track with no start line at all. Flatness is checked
    instead, since every start quad in the 13 tracks lies in one plane.
    """
    best = None
    for name, texture, flags, verts in meshes:
        if "start" not in texture.lower() or not verts:
            continue
        span_x = max(v[0] for v in verts) - min(v[0] for v in verts)
        span_y = max(v[1] for v in verts) - min(v[1] for v in verts)
        span_z = max(v[2] for v in verts) - min(v[2] for v in verts)
        if span_x <= 0.0 or span_y <= 0.0 or span_z > 0.5:
            continue
        aspect = max(span_x, span_y) / min(span_x, span_y)
        if best is None or aspect > best[0]:
            centroid = tuple(sum(c) / len(verts) for c in zip(*verts))
            # The stripe's long edge crosses the road, so racing runs along the
            # short edge.
            axis = "Y" if span_x > span_y else "X"
            best = (aspect, name, texture, centroid, axis)
    return best


def f(value):
    return repr(float("%.7g" % value)) + "f"


def main():
    rows = []
    for path in sorted(glob.glob(os.path.join(MESH_DIR, "track_*.ktrk"))):
        code = os.path.basename(path)[len("track_"):-len(".ktrk")]
        minimum, maximum, meshes = load(path)
        rows.append((code, minimum, maximum, start_quad(meshes)))

    print("/* AABB arguments for TRACKS[] */")
    for code, minimum, maximum, _ in rows:
        print('    /* %-12s */ %s, %s, %s, %s, %s, %s,' % (
            code, f(minimum[0]), f(minimum[1]), f(minimum[2]),
            f(maximum[0]), f(maximum[1]), f(maximum[2])))

    print()
    print("/* TRACK_SCENES[] start-line table */")
    for code, minimum, _, best in rows:
        if best is None:
            print('    { "%s", false, false, 0.0f, 0.0f, %s, KART_START_AXIS_NONE },'
                  '  /* no road-flagged start mesh */' % (code, f(minimum[2])))
            continue
        aspect, name, texture, centroid, axis = best
        confident = aspect >= CONFIDENT_ASPECT
        print('    { "%s", true, %s, %s, %s, %s, KART_START_AXIS_%s },'
              '  /* %s / %s, aspect %.2f */' % (
                  code, "true" if confident else "false",
                  f(centroid[0]), f(centroid[1]), f(centroid[2]),
                  axis, name, texture, aspect))


if __name__ == "__main__":
    main()
