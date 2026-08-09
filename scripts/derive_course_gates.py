"""Extract the original's course checkpoint gates from the track assets.

Reads analysis/track-assets/extracted/track_<code>/track.1s and writes the
generated tables in src/kart_course_data.c. See docs/ORIGINAL_COURSE.md for the
decompilation these formats and rules come from.

Two things are pulled out of each track:

  the `course` tag   the `track` object's property block carries one, holding
                     `road` and `branch` children with name/start/end/final/
                     reverse attributes. 0x00424e00 walks exactly this.

  `the::ToRoad`      the object a `road` tag names. It is NOT the RoadObj01@sN
                     render mesh in the scene tree; it is a separate object in
                     the container's tail section, found by its class stamp
                     (Adler32 of "ToRoad" = 07de0249). Its elements each carry
                     one road slice: the gate quad, the side walls, and the
                     centreline records.

`the::ToRoad` on-disk element, confirmed against all 13 tracks:

    wstr  name                       "" except at start/end/final/branch points
    int   vertexCount                4 or 12
    vec3  vertices[vertexCount]
    int   gateTriangleCount          always 2 -> the two halves of the gate quad
    u16   gateTriangles[][3]
    wstr  extra                      "" except one "warpnext" in ice_R01
    int   wallTriangleCount          0, 4, 6, ... -> side walls, unused here
    u16   wallTriangles[][3]
    int   recordCount
    record records[recordCount]      vec3 position, vec3 direction, vec3 up

The memory layout the accessors imply matches field for field: name at +0x00,
vertices at +0x04 (0x00426d00, stride 0xc), gate triangles at +0x10
(0x00426d20, stride 6), the extra string at +0x1c, walls at +0x20, records at
+0x2c (0x00426d60, stride 0x24), total 0x38 (0x00426d80).

Usage:  python scripts/derive_course_gates.py
"""

import glob
import math
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXTRACTED = os.path.join(ROOT, "analysis", "track-assets", "extracted")
MESHES = os.path.join(ROOT, "analysis", "track-assets", "meshes")
OUTPUT = os.path.join(ROOT, "src", "kart_course_data.c")

# [aa 47] object marker followed by the class stamp, Adler32 of "ToRoad".
TOROAD_STAMP = bytes.fromhex("aa47" + "4902de07")
# The `track` object the course tag hangs off, by its own class stamp.
TRACK_STAMP = bytes.fromhex("aa47" + "4c045319")


# --- track.1s primitives ---------------------------------------------------

def u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def wstr(data, offset):
    length = u32(data, offset)
    offset += 4
    return data[offset:offset + length * 2].decode("utf-16-le"), offset + length * 2


def read_tag(data, offset):
    """One binary-XML tag: name, text, attributes, children."""
    name, offset = wstr(data, offset)
    text, offset = wstr(data, offset)
    count = u32(data, offset)
    offset += 4
    attributes = {}
    for _ in range(count):
        key, offset = wstr(data, offset)
        value, offset = wstr(data, offset)
        attributes[key] = value
    count = u32(data, offset)
    offset += 4
    children = []
    for _ in range(count):
        child, offset = read_tag(data, offset)
        children.append(child)
    return {"name": name, "text": text, "attributes": attributes,
            "children": children}, offset


def read_toroad(data, offset):
    offset += 6
    index = struct.unpack_from("<H", data, offset)[0]
    offset += 2
    name, offset = wstr(data, offset)
    has_property = data[offset]
    offset += 1
    if has_property:
        _, offset = read_tag(data, offset)
    offset += 1
    count = u32(data, offset)
    offset += 4
    elements = []
    for _ in range(count):
        element_name, offset = wstr(data, offset)
        vertex_count = u32(data, offset)
        offset += 4
        vertices = [struct.unpack_from("<3f", data, offset + i * 12)
                    for i in range(vertex_count)]
        offset += vertex_count * 12
        gate_count = u32(data, offset)
        offset += 4
        gates = [struct.unpack_from("<3H", data, offset + i * 6)
                 for i in range(gate_count)]
        offset += gate_count * 6
        extra, offset = wstr(data, offset)
        wall_count = u32(data, offset)
        offset += 4 + wall_count * 6
        record_count = u32(data, offset)
        offset += 4
        records = [struct.unpack_from("<9f", data, offset + i * 36)
                   for i in range(record_count)]
        offset += record_count * 36
        elements.append({"name": element_name, "vertices": vertices,
                         "gates": gates, "extra": extra, "records": records})
    return {"index": index, "name": name, "elements": elements}, offset


def read_track(path):
    """Every ToRoad object keyed by name, plus the outermost `course` tag."""
    data = open(path, "rb").read()
    roads = {}
    search = 0
    while True:
        found = data.find(TOROAD_STAMP, search)
        if found < 0:
            break
        search = found + 1
        try:
            road, _ = read_toroad(data, found)
        except (struct.error, UnicodeDecodeError):
            # A byte sequence that only looks like an object header. A real one
            # always parses to the end of its element array.
            continue
        if road["elements"]:
            roads[road["name"]] = road["elements"]

    # 0x004240f0 looks the scene object named "track" up, then asks its property
    # block for the child tag named "course". Nothing else in the file is that
    # tag: `courseReverse` is the reverse-mode variant the demo never selects,
    # and the `course` tags inside a `branch` are its alternatives.
    candidates = []
    search = 0
    while True:
        found = data.find(TRACK_STAMP, search)
        if found < 0:
            break
        search = found + 1
        offset = found + 8
        name, offset = wstr(data, offset)
        has_property = data[offset]
        offset += 1
        if name != "track" or not has_property:
            continue
        try:
            tag, _ = read_tag(data, offset + 4)
        except (struct.error, UnicodeDecodeError):
            continue
        for child in tag["children"]:
            if child["name"] == "course":
                candidates.append(child)
    # desert_I02 carries a second `track` object whose course names roads that
    # are not in the file at all. Resolving against the ToRoad objects present
    # is what tells the two apart.
    resolved = [tag for tag in candidates if course_resolves(tag, roads)]
    if not resolved:
        raise ValueError("%s: no `course` tag whose roads are all in the file "
                         "(%d candidates)" % (path, len(candidates)))
    return roads, resolved[0], len(candidates)


def course_resolves(tag, roads):
    for child in tag["children"]:
        if child["name"] == "road":
            if child["attributes"].get("name") not in roads:
                return False
        elif child["name"] == "branch":
            if not all(course_resolves(course, roads) for course in child["children"]):
                return False
    return True


# --- world transform -------------------------------------------------------

def load_ktrk_bounds(code):
    """The track AABB and the start-line quad, as src/kart_demo_data.c has them.

    Gates have to land in the same space the collision meshes do, so they go
    through the transform kart_track_scene_world_vertex applies.
    """
    path = os.path.join(MESHES, "track_%s.ktrk" % code)
    data = open(path, "rb").read()
    if data[0:4] != b"KTRK":
        raise ValueError("not a KTRK file: " + path)
    mesh_count = struct.unpack_from("<I", data, 8)[0]
    offset = 20
    minimum = struct.unpack_from("<3f", data, offset)
    offset += 12
    maximum = struct.unpack_from("<3f", data, offset)
    offset += 12
    start_quad = None
    for _ in range(mesh_count):
        offset += 96
        texture = data[offset:offset + 96].split(b"\0")[0].decode("utf-8", "replace")
        offset += 96
        flags, vertex_count, index_count = struct.unpack_from("<3I", data, offset)
        offset += 12
        raw = struct.unpack_from("<%df" % (vertex_count * 5), data, offset)
        offset += vertex_count * 20 + index_count * 4
        points = [raw[i * 5:i * 5 + 3] for i in range(vertex_count)]
        if flags & 1 and "start" in texture.lower():
            span = [(min(p[i] for p in points), max(p[i] for p in points))
                    for i in range(3)]
            area = (span[0][1] - span[0][0]) * (span[1][1] - span[1][0])
            if start_quad is None or area > start_quad[0]:
                start_quad = (area, span)
    return minimum, maximum, (start_quad[1] if start_quad else None)


class Space:
    """kart_track_scene_world_vertex, and the same map for a direction.

    Every scene mirrors X (kart_demo_track_mirror_x), which is a reflection, so
    a direction's X flips with it and nothing else does.
    """

    def __init__(self, minimum, maximum, ground_z):
        self.centre_x = (minimum[0] + maximum[0]) * 0.5
        self.centre_y = (minimum[1] + maximum[1]) * 0.5
        self.ground_z = ground_z

    def point(self, v):
        return (self.centre_x - v[0], v[1] - self.centre_y, v[2] - self.ground_z)

    def direction(self, v):
        return (-v[0], v[1], v[2])


# --- emission --------------------------------------------------------------

def collect_sections(tag, roads, path):
    """Flatten the course tag into the section list the runtime walks.

    `road` and `branch` are the only children these 13 tracks use; `plane`,
    which 0x00424e00 also accepts, appears in none of them and is not emitted.
    """
    sections = []
    for child in tag["children"]:
        if child["name"] == "road":
            attributes = child["attributes"]
            name = attributes["name"]
            if name not in roads:
                raise ValueError("%s: course names missing ToRoad %r" % (path, name))
            sections.append({"kind": "road", "road": name,
                             "start": attributes.get("start", ""),
                             "end": attributes.get("end", ""),
                             "final": attributes.get("final", ""),
                             "reverse": parse_bool(attributes.get("reverse"))})
        elif child["name"] == "branch":
            sections.append({"kind": "branch",
                             "alternatives": [collect_sections(course, roads, path)
                                              for course in child["children"]]})
        else:
            raise ValueError("%s: unhandled course child %r" % (path, child["name"]))
    return sections


def parse_bool(value):
    """0x0047a470's attribute reader. `1` and `true` are both used in the assets."""
    if value is None:
        return False
    return value.strip().lower() in ("1", "true")


def identifier(text):
    return "".join(c if c.isalnum() else "_" for c in text)


def format_float(value):
    text = "%.9g" % value
    # `19f` is an integer with a bad suffix; `19.0f` is the float meant.
    if not any(c in text for c in ".eEn"):
        text += ".0"
    return text + "f"


class Emitter:
    def __init__(self):
        self.lines = []
        self.roads = {}

    def road_symbol(self, code, name):
        key = (code, name)
        if key not in self.roads:
            self.roads[key] = "ROAD_%s_%s" % (identifier(code), identifier(name))
        return self.roads[key]

    def emit_road(self, code, name, elements, space):
        symbol = self.road_symbol(code, name)
        for index, element in enumerate(elements):
            if not element["records"]:
                continue
            self.lines.append(
                "static const KartCourseRecord %s_R%u[] = {" % (symbol, index))
            for record in element["records"]:
                position = space.point(record[0:3])
                direction = space.direction(record[3:6])
                self.lines.append("    {{%s, %s, %s}, {%s, %s, %s}}," % (
                    format_float(position[0]), format_float(position[1]),
                    format_float(position[2]), format_float(direction[0]),
                    format_float(direction[1]), format_float(direction[2])))
            self.lines.append("};")
        self.lines.append("static const KartCourseElement %s[] = {" % symbol)
        for index, element in enumerate(elements):
            faces = []
            for face in range(2):
                triangle = element["gates"][face]
                for corner in range(3):
                    point = space.point(element["vertices"][triangle[corner]])
                    faces.append("{%s, %s, %s}" % (
                        format_float(point[0]), format_float(point[1]),
                        format_float(point[2])))
            records = ("%s_R%u" % (symbol, index)) if element["records"] else "NULL"
            self.lines.append("    {%s, {{%s},\n      {%s}}, %s, %s, %u}," % (
                c_string(element["name"]), ", ".join(faces[0:3]),
                ", ".join(faces[3:6]), c_string(element["extra"]), records,
                len(element["records"])))
        self.lines.append("};")
        self.lines.append("")

    def emit_sections(self, code, sections, symbol):
        for index, section in enumerate(sections):
            if section["kind"] == "branch":
                for alternative, nested in enumerate(section["alternatives"]):
                    self.emit_sections(code, nested, "%s_B%u_%u" % (symbol, index, alternative))
                self.lines.append(
                    "static const KartCourseSection *const %s_B%u[] = {%s};" % (
                        symbol, index,
                        ", ".join("%s_B%u_%u" % (symbol, index, alternative)
                                  for alternative in range(len(section["alternatives"])))))
                self.lines.append(
                    "static const unsigned int %s_B%u_counts[] = {%s};" % (
                        symbol, index,
                        ", ".join("%u" % len(nested)
                                  for nested in section["alternatives"])))
        self.lines.append("static const KartCourseSection %s[] = {" % symbol)
        for index, section in enumerate(sections):
            if section["kind"] == "road":
                road = self.road_symbol(code, section["road"])
                self.lines.append(
                    "    {%s, (unsigned int)(sizeof %s / sizeof *%s), %s, %s, %s, %u,"
                    " NULL, NULL, 0}," % (
                        road, road, road, c_string(section["start"]),
                        c_string(section["end"]), c_string(section["final"]),
                        1 if section["reverse"] else 0))
            else:
                self.lines.append(
                    "    {NULL, 0, NULL, NULL, NULL, 0, %s_B%u, %s_B%u_counts,"
                    " (unsigned int)(sizeof %s_B%u / sizeof *%s_B%u)}," % (
                        symbol, index, symbol, index, symbol, index, symbol, index))
        self.lines.append("};")
        self.lines.append("")


def c_string(text):
    if not text:
        return "NULL"
    return '"%s"' % text.replace("\\", "\\\\").replace('"', '\\"')


# --- validation ------------------------------------------------------------

def validate(code, roads, sections, bounds, report, measurements):
    minimum, maximum = bounds
    used = set()
    # Worst excursion of a gate corner past the mesh AABB, per horizontal axis
    # and vertically. A gate is a wall standing across the road, so its top
    # legitimately reaches above the scenery; a corner far outside in X or Y
    # would instead mean the vertices were read wrong.
    excursion = [0.0, 0.0, 0.0]

    def walk(items):
        for section in items:
            if section["kind"] == "road":
                used.add(section["road"])
                check_road(section)
            else:
                for nested in section["alternatives"]:
                    walk(nested)

    def check_road(section):
        elements = roads[section["road"]]
        names = [element["name"] for element in elements]
        for key in ("start", "end", "final"):
            wanted = section[key]
            if wanted and wanted not in names:
                report.append("%-12s %-16s %s=%r is not an element name; "
                              "0x00424e00 falls back to its default"
                              % (code, section["road"], key, wanted))
        for index, element in enumerate(elements):
            if len(element["gates"]) < 2:
                report.append("%-12s %-16s element %u has %d gate triangles, "
                              "0x00424e00 reads two"
                              % (code, section["road"], index, len(element["gates"])))
            for record in element["records"]:
                length = math.sqrt(sum(c * c for c in record[3:6]))
                if abs(length - 1.0) > 1.0e-3:
                    report.append("%-12s %-16s element %u direction is not a unit "
                                  "vector: |v|=%.6f"
                                  % (code, section["road"], index, length))
            for triangle in element["gates"][:2]:
                for corner in triangle:
                    v = element["vertices"][corner]
                    for axis in range(3):
                        past = max(minimum[axis] - v[axis], v[axis] - maximum[axis])
                        excursion[axis] = max(excursion[axis], past)

    walk(sections)
    if max(excursion[0], excursion[1]) > 1.0:
        report.append("%-12s a gate corner is %.2f/%.2f outside the mesh AABB "
                      "horizontally" % (code, excursion[0], excursion[1]))
    measurements.append("%-12s %-3d gates, corner reach past the mesh AABB "
                        "x %.2f  y %.2f  z %.2f"
                        % (code, sum(len(roads[name]) for name in used),
                           excursion[0], excursion[1], excursion[2]))
    return used


def start_gate_check(code, roads, sections, start_quad):
    """Where the `start` element's gate falls relative to the painted stripe.

    The two are separate assets, so they are not required to agree; printing
    the distance is what confirms the element naming was read off the file in
    the right order rather than shifted by one.
    """
    for section in sections:
        if section["kind"] != "road" or not section["start"]:
            continue
        elements = roads[section["road"]]
        for element in elements:
            if element["name"] != section["start"] or not element["records"]:
                continue
            position = element["records"][0][0:3]
            if start_quad is None:
                return "%-12s start gate at (%.2f, %.2f, %.2f), no stripe mesh" % (
                    (code,) + tuple(position))
            inside = all(start_quad[axis][0] - 0.01 <= position[axis]
                         <= start_quad[axis][1] + 0.01 for axis in range(2))
            return ("%-12s start gate (%.2f, %.2f, %.2f)  stripe x[%.2f %.2f] "
                    "y[%.2f %.2f]  %s" % (
                        code, position[0], position[1], position[2],
                        start_quad[0][0], start_quad[0][1],
                        start_quad[1][0], start_quad[1][1],
                        "on stripe" if inside else "OFF STRIPE"))
    return "%-12s no named start element" % code


# --- driver ----------------------------------------------------------------

def main():
    paths = sorted(glob.glob(os.path.join(EXTRACTED, "track_*", "track.1s")))
    if not paths:
        raise SystemExit("no extracted tracks under " + EXTRACTED)
    emitter = Emitter()
    entries = []
    report = []
    stripes = []
    measurements = []
    for path in paths:
        code = os.path.basename(os.path.dirname(path))[len("track_"):]
        roads, tag, candidates = read_track(path)
        if candidates > 1:
            report.append("%-12s %d `track` objects carry a course tag; used the "
                          "first whose roads resolve" % (code, candidates))
        sections = collect_sections(tag, roads, path)
        minimum, maximum, start_quad = load_ktrk_bounds(code)
        used = validate(code, roads, sections, (minimum, maximum), report,
                        measurements)
        stripes.append(start_gate_check(code, roads, sections, start_quad))
        space = Space(minimum, maximum,
                      start_quad[2][0] if start_quad else minimum[2])
        for name in sorted(used):
            emitter.emit_road(code, name, roads[name], space)
        symbol = "COURSE_%s" % identifier(code)
        emitter.emit_sections(code, sections, symbol)
        entries.append((code, symbol))

    with open(OUTPUT, "w", encoding="utf-8") as out:
        out.write(HEADER)
        out.write("\n".join(emitter.lines))
        out.write("\nstatic const KartCourseAsset COURSES[] = {\n")
        for code, symbol in entries:
            out.write('    {"%s", %s, (unsigned int)(sizeof %s / sizeof *%s)},\n'
                      % (code, symbol, symbol, symbol))
        out.write("};\n")
        out.write(FOOTER)

    print("wrote %s: %d tracks, %d road objects"
          % (os.path.relpath(OUTPUT, ROOT), len(entries), len(emitter.roads)))
    print("\ngate geometry:")
    for line in measurements:
        print("  " + line)
    print("\nstart element gate vs painted start stripe (asset space):")
    for line in stripes:
        print("  " + line)
    print("\nvalidation:")
    if report:
        for line in report:
            print("  " + line)
    else:
        print("  clean")
    return 0


HEADER = '''/* Generated by scripts/derive_course_gates.py. Do not edit.

   The original's checkpoint gates, read out of each track.1s. See that script
   and docs/ORIGINAL_COURSE.md for the on-disk format and the decompilation.

   Positions are in the simulator's world space: the same transform
   kart_track_scene_world_vertex applies to scene vertices, so a gate lines up
   with the road under it. Directions carry the scene's X mirror and nothing
   else. */

#include "kart_course.h"

#include <stddef.h>
#include <string.h>

'''

FOOTER = '''
unsigned int kart_course_asset_count(void)
{
    return (unsigned int)(sizeof COURSES / sizeof *COURSES);
}

const KartCourseAsset *kart_course_asset_at(unsigned int index)
{
    return index < kart_course_asset_count() ? &COURSES[index] : NULL;
}

const KartCourseAsset *kart_course_find_asset(const char *asset_name)
{
    unsigned int index;
    if (asset_name == NULL) return NULL;
    for (index = 0; index < kart_course_asset_count(); ++index) {
        if (strcmp(COURSES[index].track, asset_name) == 0) return &COURSES[index];
    }
    return NULL;
}
'''


if __name__ == "__main__":
    sys.exit(main())
