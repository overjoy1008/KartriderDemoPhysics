using System.Numerics;
using System.Text;
using System.Text.Json;
using KartLibrary.Game.Engine.Relements;
using KartLibrary.Game.Engine.Track;
using KartLibrary.IO;

// "export" reads a track.1s, whose root is a TrackContainer.
// "export-kart" reads a kart model.1s, whose root is the Relement scene itself.
if (args.Length != 3 || (args[0] != "export" && args[0] != "export-kart"))
{
    Console.Error.WriteLine("Usage: track-mesh-exporter export      <track.1s> <output-prefix-below-workspace>");
    Console.Error.WriteLine("       track-mesh-exporter export-kart <model.1s> <output-prefix-below-workspace>");
    return 2;
}

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
Collect(scene, axis, meshes);

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
    writer.Write(1u);
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
    CollisionCandidates = meshes.Where(m => m.Flags != 0).Select(m => new { m.Name, m.Flags, Triangles = m.Indices.Count / 3 }).ToArray(),
    Meshes = meshes.Select(m => new { m.Name, m.Texture, m.Flags, Vertices = m.Vertices.Count, Triangles = m.Indices.Count / 3 }).ToArray()
};
string reportPath = outputPrefix + ".mesh.json";
if (File.Exists(reportPath)) throw new IOException($"Refusing to overwrite: {reportPath}");
File.WriteAllText(reportPath, JsonSerializer.Serialize(report, new JsonSerializerOptions { WriteIndented = true }));
Console.WriteLine($"Exported {meshes.Count} meshes, {report.VertexCount} vertices, {report.TriangleCount} triangles");
Console.WriteLine($"Bounds: ({min.X}, {min.Y}, {min.Z}) .. ({max.X}, {max.Y}, {max.Z})");
return 0;

static void Collect(Relement node, Matrix4x4 parent, List<MeshData> meshes)
{
    Matrix4x4 transform = Matrix4x4.CreateScale(node.Scale) * node.Transform *
                          Matrix4x4.CreateTranslation(node.Position) * parent;
    // Kart models store geometry as ReToonRigid: separate vertex, normal and
    // texcoord arrays indexed per face corner. Only the vertex indices matter
    // for the mesh itself.
    if (node is ReToonRigid toon && toon.Vertices is not null && toon.MeshFaces is not null)
    {
        List<uint> indices = new(toon.MeshFaces.Length * 3);
        foreach (ReToonRigidMeshFace face in toon.MeshFaces)
        {
            indices.Add((uint)face.VertexIndex1);
            indices.Add((uint)face.VertexIndex2);
            indices.Add((uint)face.VertexIndex3);
        }
        Vector2[,]? uv = null;
        if (toon.TexCoords is not null && toon.TexCoords.Length >= toon.Vertices.Length)
        {
            uv = new Vector2[toon.Vertices.Length, 1];
            for (int i = 0; i < toon.Vertices.Length; i++)
                uv[i, 0] = new Vector2(toon.TexCoords[i].X, toon.TexCoords[i].Y);
        }
        AddIndexed(node, toon.Vertices, uv, indices, transform, meshes);
    }
    else if (node is ReTriList triList && triList.Vertex?.Vertices is not null && triList.Vertex.Indexes is not null)
        AddIndexed(node, triList.Vertex.Vertices, triList.Vertex.TextureUVs, triList.Vertex.Indexes.Select(i => (uint)(ushort)i), transform, meshes);
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
        AddIndexed(node, triStrip.Vertex.Vertices, triStrip.Vertex.TextureUVs, indices, transform, meshes);
    }
    foreach (Relement child in node) Collect(child, transform, meshes);
}

static void AddIndexed(Relement node, Vector3[] positions, Vector2[,]? textureUvs,
                       IEnumerable<uint> sourceIndices, Matrix4x4 transform, List<MeshData> meshes)
{
    List<Vertex> vertices = new(positions.Length);
    for (int i = 0; i < positions.Length; i++)
    {
        Vector2 uv = textureUvs is not null && textureUvs.GetLength(1) > 0 ? textureUvs[i, 0] : Vector2.Zero;
        vertices.Add(new Vertex(Vector3.Transform(positions[i], transform), uv));
    }
    List<uint> indices = sourceIndices.ToList();
    if (indices.Count < 3 || indices.Any(i => i >= vertices.Count)) return;
    string? texture = node.Tex?.u3;
    meshes.Add(new MeshData(node.Name ?? "", texture, ClassifyNode(node.Name ?? ""), vertices, indices));
}

static uint ClassifyNode(string name)
{
    string lower = name.ToLowerInvariant();
    uint flags = 0;
    if (lower.Contains("road") || lower.Contains("drive")) flags |= 1u; // drive-surface candidate
    if (lower.Contains("wall") || lower.Contains("fence") || lower.Contains("bound")) flags |= 2u; // barrier candidate
    return flags;
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
internal sealed record MeshData(string Name, string? Texture, uint Flags, List<Vertex> Vertices, List<uint> Indices);
