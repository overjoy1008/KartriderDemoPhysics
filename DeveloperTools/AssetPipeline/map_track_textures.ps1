param(
    [string]$MeshDirectory = 'Assets\Tracks\meshes',
    [string]$SharedDirectory = 'Assets\Tracks\shared',
    [string]$OutputFile = 'Assets\Tracks\texture-resolution.json'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$meshRoot = [IO.Path]::GetFullPath((Join-Path $workspace $MeshDirectory))
$sharedRoot = [IO.Path]::GetFullPath((Join-Path $workspace $SharedDirectory))
$output = [IO.Path]::GetFullPath((Join-Path $workspace $OutputFile))

$sourceIndexes = @{}
foreach ($source in @('theme_common', 'theme_desert', 'theme_forest', 'theme_ice', 'theme_village', 'track_common')) {
    $root = Join-Path $sharedRoot $source
    $index = @{}
    if (Test-Path -LiteralPath $root) {
        Get-ChildItem -LiteralPath $root -Recurse -File | ForEach-Object {
            $stem = [IO.Path]::GetFileNameWithoutExtension($_.Name).ToLowerInvariant()
            if (-not $index.ContainsKey($stem)) { $index[$stem] = @() }
            $index[$stem] += $_.FullName.Substring($workspace.Length).TrimStart('\')
        }
    }
    $sourceIndexes[$source] = $index
}

$tracks = @()
Get-ChildItem -LiteralPath $meshRoot -Filter '*.mesh.json' | Sort-Object Name | ForEach-Object {
    $meshReport = Get-Content -LiteralPath $_.FullName -Raw | ConvertFrom-Json
    $track = $_.BaseName.Replace('.mesh', '').Replace('track_', '')
    $theme = $track.Split('_')[0]
    $resolved = @()
    foreach ($texture in @($meshReport.Textures | Where-Object { $_ } | Sort-Object -Unique)) {
        $key = $texture.ToLowerInvariant()
        $candidates = @()
        foreach ($source in @("theme_$theme", 'theme_common', 'track_common')) {
            if ($sourceIndexes[$source].ContainsKey($key)) {
                $candidates += $sourceIndexes[$source][$key] | ForEach-Object {
                    [pscustomobject]@{ Source = $source; Path = $_ }
                }
            }
        }
        $resolved += [pscustomobject]@{
            Texture = $texture
            Status = if ($candidates.Count -gt 0) { 'resolved' } else { 'unmatched' }
            Candidates = $candidates
        }
    }
    $tracks += [pscustomobject]@{
        Track = $track
        ReferenceCount = $resolved.Count
        ResolvedCount = @($resolved | Where-Object Status -eq 'resolved').Count
        UnmatchedCount = @($resolved | Where-Object Status -eq 'unmatched').Count
        Textures = $resolved
    }
}

$result = [pscustomobject]@{
    GeneratedFrom = 'track.1s material texture names plus extracted theme/track-common archives'
    Tracks = $tracks
}
New-Item -ItemType Directory -Path (Split-Path -Parent $output) -Force | Out-Null
$result | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $output -Encoding UTF8
Write-Output "Wrote texture resolution report: $output"
