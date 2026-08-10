using System.Security.Cryptography;
using System.Text.Json;
using KartLibrary.File;

if (args.Length < 2 || (args[0] != "list" && args[0] != "extract-selected"))
{
    Console.Error.WriteLine("Usage:");
    Console.Error.WriteLine("  rho-safe-index list <archive.rho> [manifest.json]");
    Console.Error.WriteLine("  rho-safe-index extract-selected <archive.rho> <workspace-output-dir> [manifest.json]");
    return 2;
}

string command = args[0];
string archivePath = Path.GetFullPath(args[1]);
if (!File.Exists(archivePath))
{
    Console.Error.WriteLine($"Archive not found: {archivePath}");
    return 2;
}

FileInfo before = new(archivePath);
string hashBefore = Sha256(archivePath);
List<Entry> entries = new();

using (RhoArchive archive = new())
{
    archive.Open(archivePath);
    Walk(archive.RootFolder, entries);
    if (command == "extract-selected")
    {
        if (args.Length < 3)
            throw new ArgumentException("extract-selected requires an output directory.");
        string outputRoot = ValidateOutputRoot(args[2], archivePath);
        ExtractSelected(archive.RootFolder, outputRoot, entries);
    }
}

FileInfo after = new(archivePath);
string hashAfter = Sha256(archivePath);
if (before.Length != after.Length || before.LastWriteTimeUtc != after.LastWriteTimeUtc || hashBefore != hashAfter)
    throw new IOException("Safety check failed: the source archive changed while being indexed.");

var manifest = new Manifest(
    archivePath,
    before.Length,
    hashBefore,
    entries.Count,
    entries.OrderBy(e => e.Path, StringComparer.OrdinalIgnoreCase).ToArray());

JsonSerializerOptions jsonOptions = new() { WriteIndented = true };
string json = JsonSerializer.Serialize(manifest, jsonOptions);
int manifestArg = command == "list" ? 2 : 3;
if (args.Length > manifestArg)
{
    string outputPath = Path.GetFullPath(args[manifestArg]);
    Directory.CreateDirectory(Path.GetDirectoryName(outputPath)!);
    File.WriteAllText(outputPath, json);
    Console.WriteLine($"Indexed {entries.Count} entries -> {outputPath}");
}
else
{
    Console.WriteLine(json);
}
Console.WriteLine($"Source SHA-256 unchanged: {hashAfter}");
return 0;

static void Walk(RhoFolder folder, List<Entry> entries)
{
    foreach (RhoFile file in folder.Files)
    {
        string path = file.FullName.TrimStart('/');
        entries.Add(new Entry(path, file.Size, Classify(path)));
    }
    foreach (RhoFolder child in folder.Folders)
        Walk(child, entries);
}

static string ValidateOutputRoot(string requestedPath, string archivePath)
{
    string workspaceRoot = Path.GetFullPath(Directory.GetCurrentDirectory()) + Path.DirectorySeparatorChar;
    string outputRoot = Path.GetFullPath(requestedPath) + Path.DirectorySeparatorChar;
    if (!outputRoot.StartsWith(workspaceRoot, StringComparison.OrdinalIgnoreCase))
        throw new IOException("Extraction output must remain inside the current workspace.");
    string archiveDirectory = Path.GetDirectoryName(archivePath)! + Path.DirectorySeparatorChar;
    if (outputRoot.StartsWith(archiveDirectory, StringComparison.OrdinalIgnoreCase))
        throw new IOException("Extraction output must not be inside the source archive directory.");
    Directory.CreateDirectory(outputRoot);
    return outputRoot;
}

static void ExtractSelected(RhoFolder folder, string outputRoot, List<Entry> entries)
{
    const int maxFileSize = 256 * 1024 * 1024;
    foreach (RhoFile file in folder.Files)
    {
        string path = file.FullName.TrimStart('/');
        string kind = Classify(path);
        if (kind == "other") continue;
        if (file.Size < 0 || file.Size > maxFileSize)
            throw new IOException($"Refusing unexpectedly large entry ({file.Size} bytes): {path}");

        string normalized = path.Replace('/', Path.DirectorySeparatorChar);
        string target = Path.GetFullPath(Path.Combine(outputRoot, normalized));
        if (!target.StartsWith(outputRoot, StringComparison.OrdinalIgnoreCase))
            throw new IOException($"Unsafe archive path: {path}");
        Directory.CreateDirectory(Path.GetDirectoryName(target)!);
        using (FileStream destination = new(target, FileMode.CreateNew, FileAccess.Write, FileShare.None))
            file.WriteTo(destination);
        string extractedHash = Sha256(target);
        int index = entries.FindIndex(e => e.Path == path);
        entries[index] = entries[index] with { ExtractedSha256 = extractedHash };
    }
    foreach (RhoFolder child in folder.Folders)
        ExtractSelected(child, outputRoot, entries);
}

static string Classify(string path)
{
    string lower = path.ToLowerInvariant();
    string extension = Path.GetExtension(lower);
    if (lower.EndsWith("track.1s")) return "track-mesh";
    if (lower.Contains("collision") || lower.Contains("collider") || lower.Contains("bound") ||
        lower.Contains("wall") || lower.Contains("drive") || lower.Contains("road")) return "collision-candidate";
    if (lower.Contains("minimap") || lower.Contains("mini_map") || lower.Contains("bigmap") ||
        lower.Contains("coursemap") || lower.Contains("trackmap") || lower.Contains("mapinfo")) return "map-image";
    if (lower.Contains("trackcard") || lower.Contains("trackthumb")) return "track-ui-image";
    if (extension is ".dds" or ".tga" or ".png" or ".jpg" or ".jpeg" or ".bmp" or ".bml") return "texture";
    if (extension is ".1s" or ".arb") return "model";
    /* Plain-text descriptors such as kart parameter.xml, which carry the model
       dimensions the simulator's presets are checked against. */
    if (extension is ".xml" or ".txt" or ".ini") return "config";
    if (extension is ".wav" or ".ogg" or ".mp3") return "sound";
    return "other";
}

static string Sha256(string path)
{
    using FileStream stream = new(path, FileMode.Open, FileAccess.Read, FileShare.Read);
    return Convert.ToHexString(SHA256.HashData(stream));
}

internal sealed record Entry(string Path, int Size, string Kind, string? ExtractedSha256 = null);
internal sealed record Manifest(string Archive, long ArchiveSize, string Sha256, int EntryCount, Entry[] Entries);
