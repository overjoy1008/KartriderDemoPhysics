using System.Numerics;
using System.Text;
using System.Text.Json;
using KartLibrary.Game.Engine.Relements;
using KartLibrary.Game.Engine.Track;
using KartLibrary.IO;
using KartLibrary.Xml;

// "export" reads a track.1s, whose root is a TrackContainer.
// "export-kart" reads a kart model.1s, whose root is the Relement scene itself.
if (args.Length != 3 || (args[0] != "export" && args[0] != "export-kart"))
{
    Console.Error.WriteLine("Usage: track-mesh-exporter export      <track.1s> <output-prefix-below-workspace>");
    Console.Error.WriteLine("       track-mesh-exporter export-kart <model.1s> <output-prefix-below-workspace>");
    return 2;
}

// Mesh flag bit 0: this geometry is in the original's collision set. It used to
// be a guess from the node name and is now read from the asset.
const uint MeshFlagCollidable = 1u;
// KTRK 2 differs from 1 only in what the flags field means. The bump is here so
// a stale version-1 export fails to load instead of quietly reporting that
// nothing in the track is solid.
const uint KtrkVersion = 2u;

bool kartModel = args[0] == "export-kart";
string input = Path.GetFullPath(args[1]);
string outputPrefix = Path.GetFullPath(args[2]);
string workspace = Path.GetFullPath(Directory.GetCurrentDirectory()) + Path.DirectorySeparatorChar;
if (!outputPrefix.StartsWith(workspace, StringComparison.OrdinalIgnoreCase))
    throw new IOException("Output must remain inside the current workspace.");
if (!File.Exists(input)) throw new FileNotFoundException("input .1s not found", input);
Directory.CreateDirectory(Path.GetDirectoryName(outputPrefix)!);

KartObjectManager.Initialize();
Relement scene;
using (FileStream stream = new(input, FileMode.Open, FileAccess.Read, FileShare.Read))
using (BinaryReader reader = new(stream))
{
    Dictionary<short, KartObject> objects = new();
    Dictionary<short, object> fields = new();
    scene = kartModel
        ? reader.ReadKartObject<Relement>(objects, fields)
        : reader.ReadKartObject<TrackContainer>(objects, fields).TrackScene;
}

List<MeshData> meshes = new();
// KartRider track coordinates already use X/Y as the ground plane and Z as height.
// Keeping identity also reproduces the AABBs previously measured from demo track.1s.
Matrix4x4 axis = Matrix4x4.Identity;
Collect(scene, axis, false, meshes);

Vector3 min = new(float.PositiveInfinity);
Vector3 max = new(float.NegativeInfinity);
foreach (MeshData mesh in meshes)
foreach (Vertex vertex in mesh.Vertices)
{
    min = Vector3.Min(min, vertex.Position);
    max = Vector3.Max(max, vertex.Position);
}

string binaryPath = outputPrefix + ".ktrk";
if (File.Exists(binaryPath)) throw new IOException($"Refusing to overwrite: {binaryPath}");
using (FileStream stream = new(binaryPath, FileMode.CreateNew, FileAccess.Write, FileShare.None))
using (BinaryWriter writer = new(stream, Encoding.UTF8))
{
    writer.Write(Encoding.ASCII.GetBytes("KTRK"));
    writer.Write(KtrkVersion);
    writer.Write((uint)meshes.Count);
    writer.Write((uint)meshes.Sum(m => m.Vertices.Count));
    writer.Write((uint)meshes.Sum(m => m.Indices.Count / 3));
    WriteVector(writer, min);
    WriteVector(writer, max);
    foreach (MeshData mesh in meshes)
    {
        WriteFixedUtf8(writer, mesh.Name, 96);
        WriteFixedUtf8(writer, mesh.Texture ?? "", 96);
        writer.Write(mesh.Flags);
        writer.Write((uint)mesh.Vertices.Count);
        writer.Write((uint)mesh.Indices.Count);
        foreach (Vertex vertex in mesh.Vertices)
        {
            WriteVector(writer, vertex.Position);
            writer.Write(vertex.UV.X);
            writer.Write(vertex.UV.Y);
        }
        foreach (uint index in mesh.Indices) writer.Write(index);
    }
}

var report = new
{
    Source = input,
    Output = binaryPath,
    MeshCount = meshes.Count,
    VertexCount = meshes.Sum(m => m.Vertices.Count),
    TriangleCount = meshes.Sum(m => m.Indices.Count / 3),
    Bounds = new { Min = new[] { min.X, min.Y, min.Z }, Max = new[] { max.X, max.Y, max.Z } },
    Textures = meshes.Select(m => m.Texture).Where(t => !string.IsNullOrWhiteSpace(t)).Distinct().Order().ToArray(),
    CollidableMeshCount = meshes.Count(m => (m.Flags & MeshFlagCollidable) != 0),
    CollidableTriangleCount = meshes.Where(m => (m.Flags & MeshFlagCollidable) != 0).Sum(m => m.Indices.Count / 3),
    Collidable = meshes.Where(m => (m.Flags & MeshFlagCollidable) != 0).Select(m => new { m.Name, Triangles = m.Indices.Count / 3 }).ToArray(),
    Meshes = meshes.Select(m => new { m.Name, m.Texture, m.Flags, m.Kind, m.Determinant, Vertices = m.Vertices.Count, Triangles = m.Indices.Count / 3 }).ToArray()
};
string reportPath = outputPrefix + ".mesh.json";
if (File.Exists(reportPath)) throw new IOException($"Refusing to overwrite: {reportPath}");
File.WriteAllText(reportPath, JsonSerializer.Serialize(report, new JsonSerializerOptions { WriteIndented = true }));
Console.WriteLine($"Exported {meshes.Count} meshes, {report.VertexCount} vertices, {report.TriangleCount} triangles");
Console.WriteLine($"Bounds: ({min.X}, {min.Y}, {min.Z}) .. ({max.X}, {max.Y}, {max.Z})");
return 0;

// Whether this node's geometry belongs in the collision set, following the rule
// at 0x00432390: a node whose property block carries a `road` child is
// collidable, a node without one inherits its parent's answer, and the answer
// propagates to children.
//
// The demo builds its collision grid from nothing else. Geometry that never
// sees a `road` tag is scenery the original drives straight through.
//
// Two places are checked because the asset reader splits the node's serialized
// property block between Relement.Additional and the node's TexProperty; in
// these tracks it lands on the latter. Either one carrying `road` means the
// same thing.
static bool HasRoadTag(BinaryXmlTag? tag) => tag is not null && tag["road"].Any();

static bool IsCollidable(Relement node, bool inherited) =>
    inherited || HasRoadTag(node.Additional) || HasRoadTag(node.Tex?.Additional);

static void Collect(Relement node, Matrix4x4 parent, bool collidable, List<MeshData> meshes)
{
    Matrix4x4 transform = Matrix4x4.CreateScale(node.Scale) * node.Transform *
                          Matrix4x4.CreateTranslation(node.Position) * parent;
    collidable = IsCollidable(node, collidable);
    // Kart models store geometry as ReToonRigid: separate vertex, normal and
    // texcoord arrays, each indexed independently per face corner. A corner is
    // therefore a (vertex, texcoord) pair and not a vertex on its own, so the
    // only faithful export is one output vertex per corner. Pairing TexCoords[i]
    // with Vertices[i] instead - which is what this did before - produced UVs
    // that were not the asset's: every u came out 0.
    if (node is ReToonRigid toon && toon.Vertices is not null && toon.MeshFaces is not null)
    {
        List<Vector3> positions = new(toon.MeshFaces.Length * 3);
        List<Vector2> corners = new(toon.MeshFaces.Length * 3);
        List<uint> indices = new(toon.MeshFaces.Length * 3);
        Vector3[]? texCoords = toon.TexCoords;
        foreach (ReToonRigidMeshFace face in toon.MeshFaces)
        {
            int[] vertexIndices = { face.VertexIndex1, face.VertexIndex2, face.VertexIndex3 };
            int[] uvIndices = { face.TexCoordIndex1, face.TexCoordIndex2, face.TexCoordIndex3 };
            for (int corner = 0; corner < 3; corner++)
            {
                int vertexIndex = vertexIndices[corner];
                if (vertexIndex < 0 || vertexIndex >= toon.Vertices.Length) { indices.Clear(); break; }
                int uvIndex = uvIndices[corner];
                // The reader hands each texcoord back as a Vector3, but only the
                // last two components are the coordinate. The first is a 4-byte
                // tag whose bits are the entry's own index shifted left 16 - it
                // reads back as a denormal float stepping by 9.18e-41, never as
                // anything in 0..1 - so u is Y and v is Z.
                Vector2 uv = texCoords is not null && uvIndex >= 0 && uvIndex < texCoords.Length
                    ? new Vector2(texCoords[uvIndex].Y, texCoords[uvIndex].Z)
                    : Vector2.Zero;
                indices.Add((uint)positions.Count);
                positions.Add(toon.Vertices[vertexIndex]);
                corners.Add(uv);
            }
            if (indices.Count == 0) break;
        }
        if (indices.Count >= 3)
        {
            Vector2[,] uvs = new Vector2[corners.Count, 1];
            for (int i = 0; i < corners.Count; i++) uvs[i, 0] = corners[i];
            AddIndexed(node, positions.ToArray(), uvs, indices, transform, collidable, meshes, "ReToonRigid");
        }
    }
    else if (node is ReTriList triList && triList.Vertex?.Vertices is not null && triList.Vertex.Indexes is not null)
        AddIndexed(node, triList.Vertex.Vertices, triList.Vertex.TextureUVs, triList.Vertex.Indexes.Select(i => (uint)(ushort)i), transform, collidable, meshes, "ReTriList");
    else if (node is ReTriStrip triStrip && triStrip.Vertex?.Vertices is not null && triStrip.Vertex.Indexes is not null)
    {
        List<uint> indices = new();
        for (int i = 2; i < triStrip.Vertex.Indexes.Length; i++)
        {
            uint a = (uint)(ushort)triStrip.Vertex.Indexes[i - 2];
            uint b = (uint)(ushort)triStrip.Vertex.Indexes[i - 1];
            uint c = (uint)(ushort)triStrip.Vertex.Indexes[i];
            if ((i & 1) != 0) (a, b) = (b, a);
            if (a != b && b != c && a != c) indices.AddRange(new[] { a, b, c });
        }
        AddIndexed(node, triStrip.Vertex.Vertices, triStrip.Vertex.TextureUVs, indices, transform, collidable, meshes, "ReTriStrip");
    }
    foreach (Relement child in node) Collect(child, transform, collidable, meshes);
}

static void AddIndexed(Relement node, Vector3[] positions, Vector2[,]? textureUvs,
                       IEnumerable<uint> sourceIndices, Matrix4x4 transform, bool collidable,
                       List<MeshData> meshes, string kind)
{
    List<Vertex> vertices = new(positions.Length);
    for (int i = 0; i < positions.Length; i++)
    {
        Vector2 uv = textureUvs is not null && textureUvs.GetLength(1) > 0 ? textureUvs[i, 0] : Vector2.Zero;
        vertices.Add(new Vertex(Vector3.Transform(positions[i], transform), uv));
    }
    List<uint> indices = sourceIndices.ToList();
    if (indices.Count < 3 || indices.Any(i => i >= vertices.Count)) return;

    // A node placed by a mirroring transform reverses its triangles' orientation.
    // Transforming the vertices without also reversing the index order leaves the
    // face normal - normalize(cross(v1 - v0, v2 - v0)), which is what the
    // original's collision builder computes at 0x00432390 - pointing into the
    // surface instead of out of it. That is not a cosmetic difference: the
    // suspension response at 0x0042f460 scales its force by
    // dot(contact normal, chassis up), so a road whose normal points down pulls
    // the kart through the deck rather than holding it up. village_R01's
    // overpass (RoadObj09@s1/@s2, determinant -8) is where it shows.
    //
    // Reversing the winding under a negative determinant is the same
    // compensation kart_track_collision.c's read_triangle already applies for
    // the scene-wide X mirror; this is the per-node case it could not see.
    float determinant = transform.GetDeterminant();
    if (determinant < 0.0f)
    {
        for (int i = 0; i + 2 < indices.Count; i += 3)
            (indices[i + 1], indices[i + 2]) = (indices[i + 2], indices[i + 1]);
    }

    string? texture = node.Tex?.u3;
    meshes.Add(new MeshData(node.Name ?? "", texture, collidable ? MeshFlagCollidable : 0u, vertices, indices, kind, determinant));
}

static void WriteVector(BinaryWriter writer, Vector3 value)
{
    writer.Write(value.X); writer.Write(value.Y); writer.Write(value.Z);
}

static void WriteFixedUtf8(BinaryWriter writer, string value, int size)
{
    byte[] encoded = Encoding.UTF8.GetBytes(value);
    if (encoded.Length >= size) Array.Resize(ref encoded, size - 1);
    writer.Write(encoded);
    writer.Write(new byte[size - encoded.Length]);
}

internal sealed record Vertex(Vector3 Position, Vector2 UV);
internal sealed record MeshData(string Name, string? Texture, uint Flags, List<Vertex> Vertices, List<uint> Indices, string Kind, float Determinant);
