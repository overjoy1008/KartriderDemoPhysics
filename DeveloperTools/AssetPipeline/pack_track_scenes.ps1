# Packs the KTRK exports into KTKZ containers for embedding as RCDATA.
#
# Container layout, matching kart_track_scene_load_compressed():
#   char   magic[4] = "KTKZ"
#   uint32 uncompressed KTRK size, little-endian
#   uint8  raw DEFLATE stream (RFC 1951, no zlib or gzip wrapper)
#
# .NET's DeflateStream emits exactly that raw stream, so no framing has to be
# stripped afterwards.

param(
    [string]$InputDirectory = 'Assets\Tracks\meshes',
    [string]$OutputDirectory = 'Assets\Tracks\packed'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$inputRoot = [IO.Path]::GetFullPath((Join-Path $workspace $InputDirectory))
$outputRoot = [IO.Path]::GetFullPath((Join-Path $workspace $OutputDirectory))
$workspacePrefix = [IO.Path]::GetFullPath($workspace).TrimEnd('\') + '\'
if (-not $outputRoot.StartsWith($workspacePrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Output directory must remain below the workspace.'
}

New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null

$rawTotal = 0L
$packedTotal = 0L
Get-ChildItem -LiteralPath $inputRoot -Filter '*.ktrk' | Sort-Object Name | ForEach-Object {
    $raw = [IO.File]::ReadAllBytes($_.FullName)
    if ($raw.Length -lt 8 -or
        [Text.Encoding]::ASCII.GetString($raw, 0, 4) -ne 'KTRK') {
        throw "Not a KTRK file: $($_.FullName)"
    }

    $memory = New-Object IO.MemoryStream
    $deflate = New-Object IO.Compression.DeflateStream(
        $memory, [IO.Compression.CompressionLevel]::Optimal, $true)
    $deflate.Write($raw, 0, $raw.Length)
    $deflate.Dispose()
    $compressed = $memory.ToArray()
    $memory.Dispose()

    $target = Join-Path $outputRoot ($_.BaseName + '.ktkz')
    $output = New-Object IO.MemoryStream
    $output.Write([Text.Encoding]::ASCII.GetBytes('KTKZ'), 0, 4)
    $output.Write([BitConverter]::GetBytes([uint32]$raw.Length), 0, 4)
    $output.Write($compressed, 0, $compressed.Length)
    [IO.File]::WriteAllBytes($target, $output.ToArray())
    $output.Dispose()

    $rawTotal += $raw.Length
    $packedTotal += (8 + $compressed.Length)
    '{0,-24} {1,10:N0} -> {2,9:N0} bytes ({3,5:N1}%)' -f `
        $_.BaseName, $raw.Length, (8 + $compressed.Length),
        (100.0 * (8 + $compressed.Length) / $raw.Length)
}

if ($rawTotal -eq 0) { throw "No .ktrk files found in $inputRoot" }
'{0,-24} {1,10:N0} -> {2,9:N0} bytes ({3,5:N1}%)' -f `
    'TOTAL', $rawTotal, $packedTotal, (100.0 * $packedTotal / $rawTotal)
