"""Packs the tracks' texture assets into the KTEX table the demo embeds.

The meshes already carry everything the lookup needs: track.1s names a texture
per material, and the exporter wrote that name into every KTRK mesh along with
the per-vertex UVs. What was missing was the pixels, which live beside the
meshes in the theme archives as DDS (DXT1/DXT3/DXT5) and PNG.

This reads the referenced names straight out of the packed KTRK files, resolves
each against the extracted archives in the same order DeveloperTools/AssetPipeline/map_track_textures.ps1
does (the track's own theme, then theme_common, then track_common), decodes it,
box-filters it down to a cap the software rasterizer can sample cheaply, and
writes one shared table for all 13 tracks.

Nothing here interprets the art: the pixels are the asset's own, the key is the
name track.1s wrote, and a name the archives do not hold is simply left out.
"""

import argparse
import hashlib
import os
import struct
import sys
import zlib

KTEX_VERSION = 2
KEY_BYTES = 112

THEMES = ("desert", "forest", "ice", "village")

# 0x01: the source carried an alpha channel, so the stored 1-bit mask is
# meaningful and the rasterizer must test it. Without it every texel is opaque.
KTEX_FLAG_MASKED = 1
# 0x02: an 8-bit alpha plane follows the 1-bit mask. Only the kart images carry
# it, because 0x00417160 composites them over a solid colour at load time and
# needs the asset's real coverage, not a cutout.
KTEX_FLAG_ALPHA8 = 2


# --- DDS ------------------------------------------------------------------

def _unpack_565(value):
    r = (value >> 11) & 0x1F
    g = (value >> 5) & 0x3F
    b = value & 0x1F
    return (r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)


def _dxt_colour_block(data, offset, opaque):
    c0, c1 = struct.unpack_from("<HH", data, offset)
    bits = struct.unpack_from("<I", data, offset + 4)[0]
    r0, g0, b0 = _unpack_565(c0)
    r1, g1, b1 = _unpack_565(c1)
    if c0 > c1 or opaque:
        colours = [
            (r0, g0, b0, 255),
            (r1, g1, b1, 255),
            ((2 * r0 + r1) // 3, (2 * g0 + g1) // 3, (2 * b0 + b1) // 3, 255),
            ((r0 + 2 * r1) // 3, (g0 + 2 * g1) // 3, (b0 + 2 * b1) // 3, 255),
        ]
    else:
        colours = [
            (r0, g0, b0, 255),
            (r1, g1, b1, 255),
            ((r0 + r1) // 2, (g0 + g1) // 2, (b0 + b1) // 2, 255),
            (0, 0, 0, 0),
        ]
    return [colours[(bits >> (2 * i)) & 3] for i in range(16)]


def _dxt5_alpha_block(data, offset):
    a0, a1 = data[offset], data[offset + 1]
    bits = int.from_bytes(data[offset + 2:offset + 8], "little")
    if a0 > a1:
        table = [a0, a1] + [((7 - i) * a0 + (i + 1) * a1) // 7 for i in range(6)]
    else:
        table = [a0, a1] + [((5 - i) * a0 + (i + 1) * a1) // 5 for i in range(4)]
        table += [0, 255]
    return [table[(bits >> (3 * i)) & 7] for i in range(16)]


def decode_dds(data):
    """Returns (width, height, RGBA bytes, has_alpha)."""
    if data[:4] != b"DDS ":
        raise ValueError("not a DDS file")
    height, width = struct.unpack_from("<II", data, 12)
    pixel_flags = struct.unpack_from("<I", data, 80)[0]
    fourcc = data[84:88]
    offset = 128
    pixels = bytearray(width * height * 4)

    def put(bx, by, texels, alphas=None):
        for i, (r, g, b, a) in enumerate(texels):
            x = bx + (i & 3)
            y = by + (i >> 2)
            if x >= width or y >= height:
                continue
            base = (y * width + x) * 4
            pixels[base] = r
            pixels[base + 1] = g
            pixels[base + 2] = b
            pixels[base + 3] = alphas[i] if alphas is not None else a

    if pixel_flags & 0x4:
        block = {b"DXT1": 8, b"DXT3": 16, b"DXT5": 16}.get(fourcc)
        if block is None:
            raise ValueError("unsupported DDS fourcc %r" % fourcc)
        for by in range(0, height, 4):
            for bx in range(0, width, 4):
                if fourcc == b"DXT1":
                    put(bx, by, _dxt_colour_block(data, offset, False))
                elif fourcc == b"DXT3":
                    raw = int.from_bytes(data[offset:offset + 8], "little")
                    alphas = [((raw >> (4 * i)) & 0xF) * 17 for i in range(16)]
                    put(bx, by, _dxt_colour_block(data, offset + 8, True), alphas)
                else:
                    alphas = _dxt5_alpha_block(data, offset)
                    put(bx, by, _dxt_colour_block(data, offset + 8, True), alphas)
                offset += block
        has_alpha = fourcc != b"DXT1" or any(
            pixels[i] != 255 for i in range(3, len(pixels), 4))
        return width, height, bytes(pixels), has_alpha

    # Uncompressed. Only the layouts the archives actually use are handled.
    bits = struct.unpack_from("<I", data, 88)[0]
    masks = struct.unpack_from("<4I", data, 92)
    if bits not in (16, 24, 32):
        raise ValueError("unsupported DDS bit depth %d" % bits)
    step = bits // 8
    shifts = []
    for mask in masks:
        if mask == 0:
            shifts.append(None)
            continue
        shift = (mask & -mask).bit_length() - 1
        width_bits = bin(mask >> shift).count("1")
        shifts.append((shift, width_bits))
    for index in range(width * height):
        raw = int.from_bytes(data[offset:offset + step], "little")
        offset += step
        channels = []
        for entry in shifts:
            if entry is None:
                channels.append(255)
                continue
            shift, width_bits = entry
            value = (raw >> shift) & ((1 << width_bits) - 1)
            channels.append(value * 255 // ((1 << width_bits) - 1))
        base = index * 4
        pixels[base] = channels[0]
        pixels[base + 1] = channels[1]
        pixels[base + 2] = channels[2]
        pixels[base + 3] = channels[3]
    return width, height, bytes(pixels), shifts[3] is not None


# --- PNG ------------------------------------------------------------------

def decode_png(data):
    """Returns (width, height, RGBA bytes, has_alpha)."""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG file")
    offset = 8
    header = None
    palette = b""
    transparency = b""
    stream = bytearray()
    while offset < len(data):
        length, kind = struct.unpack_from(">I4s", data, offset)
        body = data[offset + 8:offset + 8 + length]
        offset += 12 + length
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", body)
        elif kind == b"PLTE":
            palette = body
        elif kind == b"tRNS":
            transparency = body
        elif kind == b"IDAT":
            stream += body
        elif kind == b"IEND":
            break
    if header is None:
        raise ValueError("PNG without IHDR")
    width, height, depth, colour, compression, filt, interlace = header
    if interlace != 0:
        raise ValueError("interlaced PNG")
    if depth not in (1, 2, 4, 8, 16):
        raise ValueError("unsupported PNG bit depth %d" % depth)
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[colour]
    raw = zlib.decompress(bytes(stream))
    bits_per_pixel = depth * channels
    stride = (width * bits_per_pixel + 7) // 8
    step = max(1, bits_per_pixel // 8)

    lines = []
    previous = bytearray(stride)
    position = 0
    for _ in range(height):
        method = raw[position]
        line = bytearray(raw[position + 1:position + 1 + stride])
        position += 1 + stride
        for i in range(stride):
            a = line[i - step] if i >= step else 0
            b = previous[i]
            c = previous[i - step] if i >= step else 0
            if method == 1:
                line[i] = (line[i] + a) & 0xFF
            elif method == 2:
                line[i] = (line[i] + b) & 0xFF
            elif method == 3:
                line[i] = (line[i] + (a + b) // 2) & 0xFF
            elif method == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pred = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pred) & 0xFF
        lines.append(line)
        previous = line

    def samples(line):
        if depth == 8:
            return list(line)
        if depth == 16:
            return [line[i] for i in range(0, len(line), 2)]
        out = []
        maximum = (1 << depth) - 1
        for byte in line:
            for slot in range(8 // depth):
                shift = 8 - depth * (slot + 1)
                out.append((byte >> shift) & maximum)
        return out

    pixels = bytearray(width * height * 4)
    has_alpha = colour in (4, 6) or (colour == 3 and len(transparency) > 0)
    scale = 255 // ((1 << depth) - 1) if depth < 8 else 1
    for y, line in enumerate(lines):
        values = samples(line)
        for x in range(width):
            base = (y * width + x) * 4
            index = x * channels
            if colour == 3:
                entry = values[index]
                pixels[base] = palette[entry * 3]
                pixels[base + 1] = palette[entry * 3 + 1]
                pixels[base + 2] = palette[entry * 3 + 2]
                pixels[base + 3] = (
                    transparency[entry] if entry < len(transparency) else 255)
            elif colour == 0:
                grey = values[index] * scale
                pixels[base] = pixels[base + 1] = pixels[base + 2] = grey
                pixels[base + 3] = 255
            elif colour == 4:
                grey = values[index] * scale
                pixels[base] = pixels[base + 1] = pixels[base + 2] = grey
                pixels[base + 3] = values[index + 1] * scale
            else:
                pixels[base] = values[index] * scale
                pixels[base + 1] = values[index + 1] * scale
                pixels[base + 2] = values[index + 2] * scale
                pixels[base + 3] = (
                    values[index + 3] * scale if colour == 6 else 255)
    return width, height, bytes(pixels), has_alpha


# --- resampling -----------------------------------------------------------

def resize_box(width, height, pixels, target_width, target_height):
    """Box filter, which is what a power-of-two downscale of these wants. RGB is
    averaged weighted by alpha so a cutout's transparent black does not bleed
    into the visible edge."""
    out = bytearray(target_width * target_height * 4)
    for ty in range(target_height):
        y0 = ty * height // target_height
        y1 = max(y0 + 1, (ty + 1) * height // target_height)
        for tx in range(target_width):
            x0 = tx * width // target_width
            x1 = max(x0 + 1, (tx + 1) * width // target_width)
            r = g = b = a = 0
            weight = 0
            count = 0
            for y in range(y0, y1):
                row = y * width
                for x in range(x0, x1):
                    base = (row + x) * 4
                    alpha = pixels[base + 3]
                    r += pixels[base] * alpha
                    g += pixels[base + 1] * alpha
                    b += pixels[base + 2] * alpha
                    a += alpha
                    weight += alpha
                    count += 1
            base = (ty * target_width + tx) * 4
            if weight == 0:
                out[base + 3] = 0
            else:
                out[base] = min(255, r // weight)
                out[base + 1] = min(255, g // weight)
                out[base + 2] = min(255, b // weight)
                out[base + 3] = a // count
    return bytes(out)


def fit_power_of_two(value, cap):
    size = 1
    while size * 2 <= value and size * 2 <= cap:
        size *= 2
    return size


# --- asset resolution -----------------------------------------------------

def build_source_index(shared_root):
    """Stems lowercased, which is the match DeveloperTools/AssetPipeline/map_track_textures.ps1 makes."""
    index = {}
    for source in os.listdir(shared_root):
        root = os.path.join(shared_root, source)
        if not os.path.isdir(root):
            continue
        table = {}
        for base, _, files in os.walk(root):
            for name in files:
                stem, extension = os.path.splitext(name)
                if extension.lower() not in (".dds", ".png", ".tga"):
                    continue
                table.setdefault(stem.lower(), os.path.join(base, name))
        index[source] = table
    return index


def texture_lookup_name(raw):
    """The KTRK export wrote the material's texture name as UTF-8, and a few of
    them are Korean. cp949 is the fallback for a name the mesh exporter copied
    out of track.1s without converting, and latin-1 always decodes."""
    for encoding in ("utf-8", "cp949"):
        try:
            return raw.decode(encoding).lower()
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1").lower()


def read_ktrk_textures(path):
    data = open(path, "rb").read()
    if data[:4] != b"KTRK":
        raise ValueError("%s is not a KTRK export" % path)
    offset = 4
    version, mesh_count = struct.unpack_from("<II", data, offset)
    offset += 16 + 24
    names = []
    for _ in range(mesh_count):
        offset += 96
        texture = data[offset:offset + 96].split(b"\0")[0]
        offset += 96
        _, vertex_count, index_count = struct.unpack_from("<3I", data, offset)
        offset += 12 + vertex_count * 20 + index_count * 4
        if texture:
            names.append(texture)
    return version, names


# --- packing --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh-directory", default="Assets/Tracks/meshes")
    parser.add_argument(
        "--skydome-directory", default="Assets/Tracks/skydome")
    parser.add_argument(
        "--kart-directory", default="Assets/Models/Karts/extracted")
    parser.add_argument("--shared-directory", default="Assets/Tracks/shared")
    parser.add_argument(
        "--output", default="Assets/Tracks/packed/track_textures.ktxz")
    parser.add_argument("--max-size", type=int, default=64)
    parser.add_argument("--skydome-max-size", type=int, default=256)
    parser.add_argument("--kart-max-size", type=int, default=256)
    arguments = parser.parse_args()

    workspace = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    mesh_root = os.path.join(workspace, arguments.mesh_directory)
    shared_root = os.path.join(workspace, arguments.shared_directory)
    output = os.path.join(workspace, arguments.output)

    index = build_source_index(shared_root)
    images = []
    image_by_digest = {}
    alpha8_images = set()
    entries = {}
    missing = set()

    # The skydomes are separate exports beside the tracks and name their texture
    # the same way, so they go through exactly the same resolution. They are
    # walked first because a dome fills the whole screen at once, unlike a track
    # surface seen at a distance, and so is stored at its own larger cap.
    sources = []
    skydome_root = os.path.join(workspace, arguments.skydome_directory)
    if os.path.isdir(skydome_root):
        sources += [(skydome_root, name, arguments.skydome_max_size)
                    for name in sorted(os.listdir(skydome_root))]
    sources += [(mesh_root, name, arguments.max_size)
                for name in sorted(os.listdir(mesh_root))]

    for root, name, cap in sources:
        if not name.endswith(".ktrk"):
            continue
        track = name[:-len(".ktrk")]
        if track.endswith(".skydome"):
            track = track[:-len(".skydome")]
        theme = track.split("_")[1] if track.startswith("track_") else ""
        _, textures = read_ktrk_textures(os.path.join(root, name))
        for texture in sorted(set(textures)):
            # The key is the mesh's own texture bytes behind the theme, so the
            # loader matches KartTrackSceneMesh.texture without re-encoding it.
            key = theme.encode("ascii") + b"/" + texture
            if key in entries or key in missing:
                continue
            stem = texture_lookup_name(texture)
            path = None
            for source in ("theme_%s" % theme, "theme_common", "track_common"):
                path = index.get(source, {}).get(stem)
                if path is not None:
                    break
            if path is None:
                missing.add(key)
                continue
            data = open(path, "rb").read()
            try:
                if path.lower().endswith(".dds"):
                    width, height, pixels, has_alpha = decode_dds(data)
                elif path.lower().endswith(".png"):
                    width, height, pixels, has_alpha = decode_png(data)
                else:
                    missing.add(key)
                    continue
            except ValueError as error:
                sys.stderr.write("skipping %s: %s\n" % (path, error))
                missing.add(key)
                continue
            if width == 0 or height == 0:
                missing.add(key)
                continue
            target_width = fit_power_of_two(width, cap)
            target_height = fit_power_of_two(height, cap)
            if (target_width, target_height) != (width, height):
                pixels = resize_box(
                    width, height, pixels, target_width, target_height)
            digest = hashlib.sha1(
                struct.pack("<IIB", target_width, target_height, has_alpha)
                + pixels).digest()
            image = image_by_digest.get(digest)
            if image is None:
                image = len(images)
                image_by_digest[digest] = image
                images.append((target_width, target_height, pixels, has_alpha))
            entries[key] = image

    # The kart models. Their KTRK meshes name no texture at all - the exporter
    # finds no material on a ReToonRigid node - but docs/KART_MODEL_CATALOG.md
    # established that a kart's whole body is the single 256x128 `1.png` beside
    # its model.1s. So the key is the theme "kart" and the kart's own asset name,
    # and the demo looks it up that way rather than from the mesh.
    kart_root = os.path.join(workspace, arguments.kart_directory)
    if os.path.isdir(kart_root):
        kart_sources = [(kart, os.path.join(kart_root, kart, "1.png"),
                         b"kart/" + kart.encode("latin-1"))
                        for kart in sorted(os.listdir(kart_root))]
        # The two shared images 0x00417160 stamps onto every skin.
        kart_sources += [
            ("common", os.path.join(kart_root, "common", "plate.png"),
             b"kart/@plate"),
            ("common", os.path.join(kart_root, "common", "number.png"),
             b"kart/@number"),
        ]
        for kart, path, key in kart_sources:
            if not os.path.isfile(path):
                continue
            if key in entries:
                continue
            try:
                width, height, pixels, has_alpha = decode_png(open(path, "rb").read())
            except ValueError as error:
                sys.stderr.write("skipping %s: %s\n" % (path, error))
                missing.add(key)
                continue
            # Kart images are stamped and composited at their own texel
            # coordinates, so they are stored at the asset's exact size - the
            # plate is 45x20 and the number strip 100x17, neither a power of
            # two, and neither is ever sampled by a UV.
            digest = hashlib.sha1(
                struct.pack("<IIB", width, height, has_alpha) + pixels).digest()
            image = image_by_digest.get(digest)
            if image is None:
                image = len(images)
                image_by_digest[digest] = image
                images.append((width, height, pixels, has_alpha))
            alpha8_images.add(image)
            entries[key] = image

    payload = bytearray()
    payload += b"KTEX"
    payload += struct.pack("<III", KTEX_VERSION, len(entries), len(images))
    for key in sorted(entries):
        raw = key[:KEY_BYTES - 1]
        payload += raw + b"\0" * (KEY_BYTES - len(raw))
        payload += struct.pack("<I", entries[key])
    for image_index, (width, height, pixels, has_alpha) in enumerate(images):
        keep_alpha = image_index in alpha8_images
        flags = KTEX_FLAG_MASKED if has_alpha else 0
        if keep_alpha:
            flags |= KTEX_FLAG_ALPHA8
        payload += struct.pack("<III", width, height, flags)
        colour = bytearray()
        mask = bytearray((width * height + 7) // 8)
        for i in range(width * height):
            base = i * 4
            r, g, b, a = pixels[base:base + 4]
            colour += struct.pack(
                "<H", ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
            # A texel counts as present at the same half-way point the original
            # art is authored against; there is no blending in the rasterizer.
            if a >= 128:
                mask[i >> 3] |= 1 << (i & 7)
        payload += colour
        payload += mask
        if keep_alpha:
            payload += bytes(pixels[i * 4 + 3] for i in range(width * height))

    compressed = zlib.compressobj(9, zlib.DEFLATED, -15)
    body = compressed.compress(bytes(payload)) + compressed.flush()
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "wb") as handle:
        handle.write(b"KTXZ")
        handle.write(struct.pack("<I", len(payload)))
        handle.write(body)

    print("textures: %d keys, %d images, %d missing" % (
        len(entries), len(images), len(missing)))
    if missing:
        # The names come from the asset and are not all ASCII, so they are
        # escaped rather than handed to whatever the console codepage is.
        print("unresolved: %s" % ", ".join(
            sorted(texture_lookup_name(name).encode("unicode_escape")
                   .decode("ascii") for name in missing)))
    print("wrote %s (%d bytes raw, %d packed)" % (
        output, len(payload), len(body) + 8))


if __name__ == "__main__":
    main()
