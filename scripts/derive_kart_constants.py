"""Verify (or regenerate) the kart constants in src/kart_demo_data.c against the
original demo assets.

Inputs, both produced from the demo's own `Data/kart.rho`:

  analysis/kart-assets/extracted/<kart>/parameter.xml   Dynamics values
  analysis/kart-assets/meshes/<kart>.ktrk               exported model.1s

To produce them from an installed 2004 demo:

  dotnet asset_tools/rho-safe-index/bin/Release/net9.0/RhoSafeIndex.dll \
      extract-selected "<demo>/Data/kart.rho" analysis/kart-assets/extracted \
      analysis/kart-assets/kart.extracted.json

  dotnet asset_tools/track-mesh-exporter/bin/Release/net9.0/TrackMeshExporter.dll \
      export-kart analysis/kart-assets/extracted/<kart>/model.1s \
      analysis/kart-assets/meshes/<kart>

Each model.1s holds the body as mesh 0 followed by the wheels and small parts.
The geometry constants come from the **body mesh only** — the rear wheels stick
out wider than the body, so the whole-model AABB does not reproduce them:

  half_width   = (max_x - min_x) / 2
  half_length  = (max_y - min_y) / 2
  model_height = max_z - min(0, min_z)

The clamp in model_height only matters for saber1, the one body whose lowest
vertex sits slightly below z = 0.

Usage:  python scripts/derive_kart_constants.py [--emit]
"""

import os
import re
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH_DIR = os.path.join(ROOT, 'analysis', 'kart-assets', 'meshes')
PARAM_DIR = os.path.join(ROOT, 'analysis', 'kart-assets', 'extracted')

DYNAMICS_ORDER = [
    'Mass', 'AirFriction', 'DragFactor', 'ForwardAccelForce',
    'BackwardAccelForce', 'GripBrakeForce', 'SlipBrakeForce', 'MaxSteerAngle',
    'SteerConstraint', 'FrontGripFactor', 'RearGripFactor',
    'DriftTriggerFactor', 'DriftTriggerTime', 'DriftSlipFactor',
    'DriftEscapeForce', 'CornerDrawFactor',
]

TOLERANCE = 2e-5


def body_vertices(path):
    """Vertices of mesh 0 of a KTRK export."""
    data = open(path, 'rb').read()
    if data[0:4] != b'KTRK':
        raise ValueError('not a KTRK file: ' + path)
    offset = 44  # magic, version, counts, bounds
    offset += 96 + 96          # name, texture
    _flags, vertex_count, _index_count = struct.unpack_from('<3I', data, offset)
    offset += 12
    raw = struct.unpack_from('<%df' % (vertex_count * 5), data, offset)
    return [raw[i * 5:i * 5 + 3] for i in range(vertex_count)]


def geometry(path):
    verts = body_vertices(path)
    min_x = min(v[0] for v in verts)
    max_x = max(v[0] for v in verts)
    min_y = min(v[1] for v in verts)
    max_y = max(v[1] for v in verts)
    min_z = min(v[2] for v in verts)
    max_z = max(v[2] for v in verts)
    return ((max_x - min_x) / 2.0,
            (max_y - min_y) / 2.0,
            max_z - min(0.0, min_z))


def main():
    emit = '--emit' in sys.argv
    src = open(os.path.join(ROOT, 'src', 'kart_demo_data.c'),
               encoding='utf-8').read()

    macros = {}
    for m in re.finditer(r'#define (\w+_DYNAMICS)\s*\\\s*\n\s*DYNAMICS\(([^)]*)\)', src):
        body = m.group(2).replace('\\', ' ').replace('\n', ' ')
        macros[m.group(1)] = [float(x.strip().rstrip('f')) for x in body.split(',')]

    rows = re.findall(
        r'KART\("(\w+)",\s*(\w+_DYNAMICS),\s*([\d.]+)f,\s*([\d.]+)f,\s*([\d.]+)f\)',
        src)

    mismatches = 0
    for name, macro, hw, hl, h in rows:
        mesh_path = os.path.join(MESH_DIR, name + '.ktrk')
        param_path = os.path.join(PARAM_DIR, name, 'parameter.xml')

        if os.path.exists(mesh_path):
            got = geometry(mesh_path)
            for label, value, ours in zip(('half_width', 'half_length', 'model_height'),
                                          got, (float(hw), float(hl), float(h))):
                if abs(value - ours) > TOLERANCE:
                    print('%-11s %-13s asset=%.7f ours=%.7f MISMATCH'
                          % (name, label, value, ours))
                    mismatches += 1
            if emit:
                print('    KART("%s", %s, %.7ff, %.8ff, %.8ff),'
                      % (name, macro, got[0], got[1], got[2]))
        else:
            print('%-11s mesh missing: %s' % (name, mesh_path))
            mismatches += 1

        if os.path.exists(param_path):
            xml = open(param_path, encoding='utf-8', errors='replace').read()
            asset = {k: float(v) for k, v in re.findall(r"(\w+)='([-\d.]+)'", xml)}
            ours = dict(zip(DYNAMICS_ORDER, macros[macro]))
            for key in DYNAMICS_ORDER:
                # An attribute the asset omits falls back to zero.
                value = asset.get(key, 0.0)
                if abs(value - ours[key]) > 1e-6:
                    print('%-11s %-20s asset=%g ours=%g MISMATCH'
                          % (name, key, value, ours[key]))
                    mismatches += 1
        else:
            print('%-11s parameter.xml missing: %s' % (name, param_path))
            mismatches += 1

    print()
    print('%d karts checked -> %d mismatches' % (len(rows), mismatches))
    return 1 if mismatches else 0


if __name__ == '__main__':
    sys.exit(main())
