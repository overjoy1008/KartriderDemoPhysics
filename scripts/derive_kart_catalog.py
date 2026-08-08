"""Build a per-kart catalogue of design, geometry and bounding volumes from the
2004 demo's own `Data/kart.rho`.

This is a reporting pass, not a verification pass: `derive_kart_constants.py`
already checks the 26 constants in `src/kart_demo_data.c` against the assets.
This script instead records everything the assets say about each kart:

  design    the RGBA body/detail skins and the shadow blob, with the dominant
            colours of the skin that actually covers the body
  modelling the submesh breakdown of `model.1s` - body, wheels, small parts
  bounds    three different boxes, which are genuinely different numbers:
              full     every vertex of every submesh
              body     submesh 0 only, which is what the physics uses
              wheels   the four wheel submeshes, which stick out wider

Inputs, both produced from an installed demo (see derive_kart_constants.py for
the two extraction commands):

  analysis/kart-assets/extracted/<kart>/{model.1s,parameter.xml,*.png}
  analysis/kart-assets/meshes/<kart>.ktrk

Outputs:

  analysis/reports/kart-catalog.json   full detail, committed
  docs/KART_MODEL_CATALOG.md           the readable catalogue

Usage:  python scripts/derive_kart_catalog.py
"""

import hashlib
import json
import os
import re
import struct
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH_DIR = os.path.join(ROOT, 'analysis', 'kart-assets', 'meshes')
ASSET_DIR = os.path.join(ROOT, 'analysis', 'kart-assets', 'extracted')
JSON_OUT = os.path.join(ROOT, 'analysis', 'reports', 'kart-catalog.json')
DOC_OUT = os.path.join(ROOT, 'docs', 'KART_MODEL_CATALOG.md')

# Order follows src/kart_demo_data.c so the catalogue lines up with the table
# a reader is most likely to have open next to it.
KART_ORDER = [
    'practice1',
    'burst1', 'burst2', 'burst3', 'burst4', 'burst5',
    'cotten1', 'cotten2', 'cotten3', 'cotten4', 'cotten5',
    'marathon1', 'marathon2', 'marathon3', 'marathon4', 'marathon5',
    'saber1', 'saber2', 'saber3', 'saber4', 'saber5',
    'solid1', 'solid2', 'solid3', 'solid4', 'solid5',
]

MESH_HEADER = 44          # magic, version, 3 counts, min, max
MESH_ENTRY_HEADER = 204   # name 96, texture 96, flags 4, vertex 4, index 4
VERTEX_STRIDE = 20        # position 12, uv 8


# --------------------------------------------------------------------------
# KTRK

def read_ktrk(path):
    """Every submesh of a KTRK export, with its own bounds."""
    data = open(path, 'rb').read()
    if data[0:4] != b'KTRK':
        raise ValueError('not a KTRK file: ' + path)

    version, mesh_count, vertex_total, triangle_total = struct.unpack_from(
        '<4I', data, 4)
    header_min = struct.unpack_from('<3f', data, 20)
    header_max = struct.unpack_from('<3f', data, 32)

    meshes = []
    offset = MESH_HEADER
    for _ in range(mesh_count):
        name = data[offset:offset + 96].split(b'\0')[0].decode('utf-8', 'replace')
        texture = data[offset + 96:offset + 192].split(b'\0')[0].decode('utf-8', 'replace')
        flags, vertex_count, index_count = struct.unpack_from(
            '<3I', data, offset + 192)
        offset += MESH_ENTRY_HEADER

        raw = struct.unpack_from('<%df' % (vertex_count * 5), data, offset)
        positions = [raw[i * 5:i * 5 + 3] for i in range(vertex_count)]
        offset += vertex_count * VERTEX_STRIDE
        offset += index_count * 4

        meshes.append({
            'name': name,
            'texture': texture,
            'flags': flags,
            'vertices': vertex_count,
            'triangles': index_count // 3,
            'bounds': bounds_of(positions),
        })

    return {
        'version': version,
        'mesh_count': mesh_count,
        'vertex_total': vertex_total,
        'triangle_total': triangle_total,
        'header_bounds': {'min': list(header_min), 'max': list(header_max)},
        'meshes': meshes,
    }


def bounds_of(positions):
    axes = list(zip(*positions))
    lo = [min(a) for a in axes]
    hi = [max(a) for a in axes]
    return {
        'min': lo,
        'max': hi,
        'size': [hi[i] - lo[i] for i in range(3)],
        'center': [(hi[i] + lo[i]) / 2.0 for i in range(3)],
    }


def merge_bounds(box_list):
    lo = [min(b['min'][i] for b in box_list) for i in range(3)]
    hi = [max(b['max'][i] for b in box_list) for i in range(3)]
    return {
        'min': lo,
        'max': hi,
        'size': [hi[i] - lo[i] for i in range(3)],
        'center': [(hi[i] + lo[i]) / 2.0 for i in range(3)],
    }


def wheel_meshes(meshes):
    """The four wheels: the largest group of submeshes past the body that share
    a vertex count. Every demo kart has exactly four such submeshes."""
    groups = {}
    for index, mesh in enumerate(meshes[1:], start=1):
        groups.setdefault(mesh['vertices'], []).append(index)
    fours = [v for v in groups.values() if len(v) == 4]
    if not fours:
        return []
    return max(fours, key=lambda v: meshes[v[0]]['vertices'])


# --------------------------------------------------------------------------
# PNG

def read_png(path):
    """Size, colour type and - for the RGBA skins - the dominant colours."""
    data = open(path, 'rb').read()
    width, height, depth, color_type = struct.unpack_from('>IIBB', data, 16)

    idat = b''
    offset = 8
    while offset < len(data):
        length, kind = struct.unpack_from('>I4s', data, offset)
        if kind == b'IDAT':
            idat += data[offset + 8:offset + 8 + length]
        offset += 12 + length

    info = {
        'file': os.path.basename(path),
        'width': width,
        'height': height,
        'bit_depth': depth,
        'color_type': color_type,
        'bytes': len(data),
    }

    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(color_type)
    if channels is None or depth != 8:
        return info

    pixels = unfilter(zlib.decompress(idat), width, height, channels)
    info['pixel_sha256'] = hashlib.sha256(bytes(pixels)).hexdigest().upper()
    info['opaque_ratio'] = opaque_ratio(pixels, channels)
    info.update(paint_survey(pixels, channels))
    return info


def unfilter(raw, width, height, channels):
    """PNG defilter for 8-bit images. Returns one flat bytearray."""
    stride = width * channels
    out = bytearray(stride * height)
    prior = bytearray(stride)
    pos = 0
    for row in range(height):
        method = raw[pos]
        pos += 1
        line = bytearray(raw[pos:pos + stride])
        pos += stride
        for i in range(stride):
            left = line[i - channels] if i >= channels else 0
            up = prior[i]
            upleft = prior[i - channels] if i >= channels else 0
            if method == 1:
                line[i] = (line[i] + left) & 0xFF
            elif method == 2:
                line[i] = (line[i] + up) & 0xFF
            elif method == 3:
                line[i] = (line[i] + ((left + up) >> 1)) & 0xFF
            elif method == 4:
                line[i] = (line[i] + paeth(left, up, upleft)) & 0xFF
        out[row * stride:(row + 1) * stride] = line
        prior = line
    return out


def paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def opaque_ratio(pixels, channels):
    if channels not in (2, 4):
        return 1.0
    alpha = pixels[channels - 1::channels]
    return round(sum(1 for a in alpha if a >= 128) / float(len(alpha)), 4)


def is_color_key(r, g, b):
    """Pure magenta, the era's standard 'this texel is unused' fill."""
    return r > 240 and g < 15 and b > 240


def is_blue_panel(r, g, b):
    """Pure blue. Every kart skin carries a patch of it at the same place in
    the atlas, so it is a marker rather than paint."""
    return r < 10 and g < 10 and b > 245


def paint_survey(pixels, channels):
    """What the skin is actually made of.

    Written this way because the obvious 'top colours by area' answer is
    useless here: the panels are painted white and grey, so a ranked list is
    all greys for 25 of the 26 karts and says nothing. The interesting split
    is padding / achromatic panel / marker / real paint."""
    if channels < 3:
        return {}

    opaque = key = achromatic = blue = painted = 0
    paint_counts = {}
    for i in range(0, len(pixels), channels):
        if channels == 4 and pixels[i + 3] < 128:
            continue
        opaque += 1
        r, g, b = pixels[i], pixels[i + 1], pixels[i + 2]
        if is_color_key(r, g, b):
            key += 1
        elif is_blue_panel(r, g, b):
            blue += 1
        elif max(r, g, b) - min(r, g, b) < 20:
            achromatic += 1
        else:
            painted += 1
            bucket = (r >> 4, g >> 4, b >> 4)
            paint_counts[bucket] = paint_counts.get(bucket, 0) + 1

    body = opaque - key
    ranked = sorted(paint_counts.items(), key=lambda kv: -kv[1])[:3]
    return {
        'opaque_px': opaque,
        'color_key_px': key,
        'body_px': body,
        'achromatic_px': achromatic,
        'blue_marker_px': blue,
        'painted_px': painted,
        'achromatic_ratio': round(achromatic / float(body), 4) if body else 0.0,
        'paint_colors': [{'hex': '#%02X%02X%02X' % (r * 17, g * 17, b * 17),
                          'px': n} for (r, g, b), n in ranked],
    }


# --------------------------------------------------------------------------
# parameter.xml

def read_parameter(path):
    xml = open(path, encoding='utf-8', errors='replace').read()
    return {k: float(v) for k, v in re.findall(r"(\w+)='([-\d.]+)'", xml)}


PRESETS = {
    'PRACTICE': (0.740, 2000.0, 22.0),
    'STANDARD': (0.725, 3300.0, 28.0),
    'MARATHON': (0.622, 3000.0, 26.0),
    'SABER': (0.786, 3550.0, 29.0),
    'SOLID': (0.855, 3800.0, 30.0),
}


def preset_of(params):
    key = (params.get('DragFactor'), params.get('ForwardAccelForce'),
           params.get('SteerConstraint'))
    for name, signature in PRESETS.items():
        if all(abs(key[i] - signature[i]) < 1e-6 for i in range(3)):
            return name
    return 'UNKNOWN'


# --------------------------------------------------------------------------

def build(kart):
    mesh_path = os.path.join(MESH_DIR, kart + '.ktrk')
    asset_dir = os.path.join(ASSET_DIR, kart)

    model = read_ktrk(mesh_path)
    meshes = model['meshes']
    body = meshes[0]
    wheels = wheel_meshes(meshes)

    body_box = body['bounds']
    full_box = merge_bounds([m['bounds'] for m in meshes])
    wheel_box = merge_bounds([meshes[i]['bounds'] for i in wheels]) if wheels else None

    textures = []
    for name in sorted(os.listdir(asset_dir)):
        if name.lower().endswith('.png'):
            textures.append(read_png(os.path.join(asset_dir, name)))

    params = read_parameter(os.path.join(asset_dir, 'parameter.xml'))

    return {
        'name': kart,
        'preset': preset_of(params),
        'mass': params.get('Mass'),
        'model': {
            'submeshes': model['mesh_count'],
            'vertices': model['vertex_total'],
            'triangles': model['triangle_total'],
            'body_index': 0,
            'wheel_indices': wheels,
            'part_indices': [i for i in range(1, len(meshes)) if i not in wheels],
            'meshes': meshes,
        },
        'bounds': {
            'full': full_box,
            'body': body_box,
            'wheels': wheel_box,
        },
        'physics': {
            'half_width': body_box['size'][0] / 2.0,
            'half_length': body_box['size'][1] / 2.0,
            'model_height': body_box['max'][2] - min(0.0, body_box['min'][2]),
            'height_clamped': body_box['min'][2] < 0.0,
            'wheels_wider_by': (full_box['size'][0] - body_box['size'][0]) / 2.0,
        },
        'textures': textures,
    }


def f(value, digits=4):
    return ('%.*f' % (digits, value)).rstrip('0').rstrip('.')


def write_doc(karts):
    out = []
    w = out.append

    w('# 카트 모델 카탈로그\n')
    w('26개 카트의 디자인·모델링·바운딩 박스를 원본 데모 에셋에서 직접 뽑은 것이다.')
    w('생성은 `python scripts/derive_kart_catalog.py`, 원본 데이터는')
    w('`analysis/reports/kart-catalog.json`에 그대로 들어 있다.\n')
    w('출처는 `C:\\Program Files (x86)\\Nexon\\KartRider Demo\\Data\\kart.rho`')
    w('(134 엔트리)이며, 값 검증은 `docs/KART_ASSET_VERIFICATION.md`가 따로 맡는다.')
    w('아카이브는 Cotton을 `cotten`으로 적는다.\n')

    w('## 아카이브에 있는 것\n')
    w('`kart.rho`의 최상위는 카트 이름 폴더 27개 + `common/` + `kartlist.xml`이다.')
    w('26개는 아래 카탈로그의 카트이고, 나머지 하나가 `mine1`이다.\n')
    w('- `mine1`은 **`model.1s` 하나뿐**이다. `parameter.xml`도 텍스처도 없어서')
    w('  물리 값도 외형도 존재하지 않는다. 이름대로 아이템 지뢰 모델로 보이며,')
    w('  카트 26대에 들어가지 않는 이유도 이것이다.')
    w('- `kartlist.xml`에는 `burst3` 한 항목만 있다. 데모가 burst3로 시작하는 것과')
    w('  일치하며, 나머지 25대는 아카이브에만 있고 목록에는 없다.\n')

    w('## 바운딩 박스가 세 개인 이유\n')
    w('| 박스 | 범위 | 쓰임 |')
    w('|---|---|---|')
    w('| full | 전 서브메쉬 정점 | KTRK 헤더에 기록되는 값, 렌더 컬링용 |')
    w('| body | 서브메쉬 0(차체)만 | **물리가 쓰는 값.** `half_width`/`half_length`/`model_height`의 출처 |')
    w('| wheels | 바퀴 4개 서브메쉬 | 차체보다 옆으로 튀어나오는 폭의 정체 |')
    w('')
    wider = sum(1 for k in karts if k['physics']['wheels_wider_by'] > 2e-5)
    w('셋이 다른 값이라는 게 핵심이다. %d/%d 카트는 바퀴가 차체보다 옆으로 더'
      % (wider, len(karts)))
    w('나와서 full로는 물리 상수가 재현되지 않는다. 나머지 %d대는 차체가 더 넓어'
      % (len(karts) - wider))
    w('full 폭과 body 폭이 같다. 아래 `바퀴 초과` 열이 한쪽당 초과폭이다.\n')
    w('```text')
    w('half_width   = (body.max_x - body.min_x) / 2')
    w('half_length  = (body.max_y - body.min_y) / 2')
    w('model_height = body.max_z - min(0, body.min_z)')
    w('```\n')

    w('## 물리 치수와 바운딩 박스\n')
    w('길이 단위는 원본 모델 단위 그대로다.\n')
    w('| 카트 | 프리셋 | half_width | half_length | model_height | full 폭 | full 길이 | full 높이 | 바퀴 초과 |')
    w('|---|---|---|---|---|---|---|---|---|')
    for k in karts:
        p, b = k['physics'], k['bounds']['full']
        w('| `%s` | %s | %s | %s | %s%s | %s | %s | %s | %s |' % (
            k['name'], k['preset'],
            f(p['half_width'], 7), f(p['half_length'], 7), f(p['model_height'], 7),
            ' ⚠' if p['height_clamped'] else '',
            f(b['size'][0]), f(b['size'][1]), f(b['size'][2]),
            f(p['wheels_wider_by'])))
    w('')
    w('⚠ = 차체 최저점이 z 0보다 아래라 `model_height`에 0 클램프가 실제로 걸리는 카트.\n')

    w('## 차체 AABB 원값\n')
    w('| 카트 | min (x, y, z) | max (x, y, z) |')
    w('|---|---|---|')
    for k in karts:
        b = k['bounds']['body']
        w('| `%s` | %s, %s, %s | %s, %s, %s |' % (
            k['name'],
            f(b['min'][0], 7), f(b['min'][1], 7), f(b['min'][2], 7),
            f(b['max'][0], 7), f(b['max'][1], 7), f(b['max'][2], 7)))
    w('')

    w('## 모델링\n')
    w('`model.1s`는 `Relement` 루트에 `ReToonRigid` 지오메트리다. 서브메쉬 0이')
    w('차체, 그다음이 바퀴 4개, 나머지가 잔부품이다. 어느 서브메쉬에도 이름이나')
    w('텍스처 문자열이 없어서, 차체/바퀴 구분은 순서와 정점 수로 한다.\n')
    w('| 카트 | 서브메쉬 | 정점 | 삼각형 | 차체 v/t | 바퀴 인덱스 | 바퀴 1개 v/t | 잔부품 |')
    w('|---|---|---|---|---|---|---|---|')
    for k in karts:
        m = k['model']
        meshes = m['meshes']
        wheel_ids = m['wheel_indices']
        wheel_cell = ('%s' % ','.join(str(i) for i in wheel_ids)) if wheel_ids else '—'
        wheel_size = ('%d / %d' % (meshes[wheel_ids[0]]['vertices'],
                                   meshes[wheel_ids[0]]['triangles'])) if wheel_ids else '—'
        parts = m['part_indices']
        part_cell = ', '.join('#%d (%dv/%dt)' % (i, meshes[i]['vertices'],
                                                 meshes[i]['triangles'])
                              for i in parts) or '—'
        w('| `%s` | %d | %d | %d | %d / %d | %s | %s | %s |' % (
            k['name'], m['submeshes'], m['vertices'], m['triangles'],
            meshes[0]['vertices'], meshes[0]['triangles'],
            wheel_cell, wheel_size, part_cell))
    w('')

    w('### 모델의 앞은 -y다\n')
    w('에셋 어디에도 축 이름이 없다. 서로 무관한 근거 셋이 모두 -y를 앞으로')
    w('가리킨다.\n')
    w('- y -0.36에 있는 9정점 8삼각형 원반은 **핸들**이고, 평균 법선이')
    w('  (0, +0.65, +0.76)이다. 위쪽과 +y를 향한다 — 운전자가 +y에 있다는 뜻이다.')
    w('- +y 쪽 바퀴가 큰 쪽이다. 카트는 뒷바퀴가 크다.')
    w('- 차체 높이가 +y 끝에서 0.69, -y 끝에서 0.50이다. 앞이 낮고 뒤가 높은')
    w('  통상적인 실루엣이다.\n')
    w('그래서 시뮬레이터는 모델을 z축 기준 180도 돌려 놓는다. x와 y를 같이')
    w('뒤집으므로 거울상이 아니라 회전이다 — 한쪽만 뒤집으면 좌우가 바뀐다.\n')

    w('### 바퀴 배치\n')
    w('바퀴 서브메쉬의 AABB 중심이다. 좌우 대칭이고, y 부호로 앞뒤가 갈린다.\n')
    w('| 카트 | 바퀴 중심 (x, y, z) × 4 | 바퀴 지름(x, z) |')
    w('|---|---|---|')
    for k in karts:
        m = k['model']
        if not m['wheel_indices']:
            w('| `%s` | — | — |' % k['name'])
            continue
        centers = []
        for i in m['wheel_indices']:
            c = m['meshes'][i]['bounds']['center']
            centers.append('(%s, %s, %s)' % (f(c[0], 3), f(c[1], 3), f(c[2], 3)))
        size = m['meshes'][m['wheel_indices'][0]]['bounds']['size']
        w('| `%s` | %s | %s × %s |' % (
            k['name'], ' '.join(centers), f(size[0], 3), f(size[2], 3)))
    w('')

    w('## 디자인\n')
    w('카트마다 PNG 세 장이 붙는다.\n')
    w('| 파일 | 형식 | 크기 | 실제 내용 |')
    w('|---|---|---|---|')
    w('| `1.png` | RGBA8 | 256×128 | 카트의 전부. 도색·데칼·명암이 다 여기 있다 |')
    w('| `0.png` | RGBA8 | 256×128 | **사실상 빈 파일** (아래 참조) |')
    w('| `shadow.png` | RGB8 | 64×64 | 지면 그림자 블롭 |')
    w('')
    w('`common/`에 `number.png`, `plate.png`가 전 카트 공용으로 따로 있다.\n')

    blank = [k for k in karts
             if any(t['file'] == '0.png' and t.get('opaque_ratio', 1) < 0.001
                    for t in k['textures'])]
    zero_hashes = {t['pixel_sha256'] for k in karts for t in k['textures']
                   if t['file'] == '0.png' and 'pixel_sha256' in t}
    w('### `0.png`는 내용이 없다\n')
    w('%d/%d 카트의 `0.png`가 32768픽셀 중 **2픽셀만 불투명**하고 나머지는 전부'
      % (len(blank), len(karts)))
    w('알파 0이다. 그 2픽셀은 (255, 0)과 (0, 127), 즉 텍스처의 대각 두 모서리이며')
    w('색은 순백이다. UV 랩 모드를 잡아두려는 상투적 수법으로 보인다.')
    w('픽셀 해시가 %s 종류뿐이라 %d장 전부가 내용상 같은 파일이다. 아카이브의 파일'
      % ('한' if len(zero_hashes) == 1 else '%d' % len(zero_hashes), len(karts)))
    w('바이트 해시는 26개가 서로 다르지만 압축 결과의 차이일 뿐이다.\n')
    w('따라서 카트 외형을 논할 때 볼 것은 `1.png` 한 장뿐이다.\n')

    w('### `1.png`은 패널 아틀라스이고, 카트는 무채색이다\n')
    w('평평한 패널 조각을 격자로 늘어놓은 아틀라스다. 조각 사이의 빈 자리는')
    w('순마젠타 색키로 메워져 있고, 알파 0인 여백이 또 따로 있다.\n')
    w('불투명 텍셀에서 색키를 뺀 것이 실제 차체 면적인데, **그 면적의 대부분이')
    w('무채색**(채널 최대-최소 < 20)이다. 즉 데모 카트의 도색은 흰색·회색·검정')
    w('명암뿐이고, 카트끼리 다른 것은 색이 아니라 패널 모양이다.\n')
    w('예외가 두 가지 있다.\n')
    w('- **청색 마커.** 26대 전부가 아틀라스의 같은 자리에 순청색(0, 0, 255)')
    w('  패치를 900텍셀 안팎으로 갖고 있다. 명암 단계가 전혀 없는 단색이라')
    w('  도색이 아니라 표식이며, 런타임에서 무엇으로 대체되는지는 이 에셋만으로는')
    w('  알 수 없다. 같은 자리에 순시안(0, 255, 255)이 1~2텍셀 더 붙는다.')
    w('- **`practice1`.** 유일하게 진짜 도색이 있다. 노랑·검정 위험 표시 줄무늬가')
    w('  들어간 연습용 카트다.\n')
    w('| 카트 | 불투명 | 색키 | 차체 텍셀 | 무채색 | 청색 마커 | 유채색 텍셀 | 유채색 상위 |')
    w('|---|---|---|---|---|---|---|---|')
    for k in karts:
        by_name = {t['file']: t for t in k['textures']}
        skin = by_name.get('1.png')
        if not skin or 'body_px' not in skin:
            w('| `%s` | — | — | — | — | — | — | — |' % k['name'])
            continue
        colors = ' '.join('`%s` %d' % (c['hex'], c['px'])
                          for c in skin['paint_colors'][:3]) or '—'
        w('| `%s` | %.0f%% | %.0f%% | %d | %.1f%% | %d | %d | %s |' % (
            k['name'], skin['opaque_ratio'] * 100,
            100.0 * skin['color_key_px'] / skin['opaque_px'],
            skin['body_px'], skin['achromatic_ratio'] * 100,
            skin['blue_marker_px'], skin['painted_px'], colors))
    w('')

    w('## 시뮬레이터에서 보기\n')
    w('`build-win/kart.exe`가 위 26개 모델을 전부 내장한다. 트랙 씬과 같은 KTKZ')
    w('컨테이너로 압축해 RCDATA로 넣었고(합쳐서 130KB), 같은 로더가 읽는다.\n')
    w('| 키 | 하는 일 |')
    w('|---|---|')
    w('| `M` | 복구한 메쉬 ↔ 예전 8정점 상자 전환 |')
    w('| `B` | 바운딩 박스 표시 — 노랑 full, 초록 body, 자홍 wheels |')
    w('| `K` | 카트 교체. 26대 모두 각자의 메쉬로 그려진다 |')
    w('')
    w('상자 쪽도 그대로 남겨 뒀다. 그게 물리가 실제로 다루는 형상이라 둘을')
    w('나란히 비교할 수 있어야 한다. 상자의 지붕 계수 `0.8`/`0.75`는 어느 원본')
    w('에셋에도 없는 값이고, 메쉬 쪽에는 그런 임의값이 없다.\n')
    w('도색은 원본 스킨이 아니라 데모의 상태 색(기본 빨강, 드리프트 하늘색,')
    w('부스트 주황)이다. 위에서 본 대로 스킨에 쓸 색이 없어서, 텍스처를 입히면')
    w('회색 카트가 나온다.\n')
    w('`tests/test_kart_model_assets.c`가 26개 모델을 전부 열어 서브메쉬 6개와')
    w('바퀴 4개를 확인하고, 차체 박스에서 `half_width`/`half_length`/')
    w('`model_height`를 다시 계산해 `src/kart_demo_data.c`의 상수와 대조한다.')
    w('그리는 형상과 시뮬레이션하는 형상이 갈라지지 않게 묶어 두는 장치다.')

    open(DOC_OUT, 'w', encoding='utf-8').write('\n'.join(out) + '\n')


def main():
    karts = [build(name) for name in KART_ORDER]

    with open(JSON_OUT, 'w', encoding='utf-8') as handle:
        json.dump({'source': 'Data/kart.rho', 'karts': karts}, handle,
                  indent=2, ensure_ascii=False)

    write_doc(karts)

    print('%d karts -> %s' % (len(karts), os.path.relpath(JSON_OUT, ROOT)))
    print('%d karts -> %s' % (len(karts), os.path.relpath(DOC_OUT, ROOT)))


if __name__ == '__main__':
    main()
