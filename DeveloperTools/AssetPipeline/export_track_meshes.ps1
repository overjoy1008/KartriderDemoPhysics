param(
    [string]$InputDirectory = 'Assets\Tracks\extracted',
    [string]$OutputDirectory = 'Assets\Tracks\meshes',
    [string]$SkydomeDirectory = 'Assets\Tracks\skydome'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$tool = Join-Path $workspace 'DeveloperTools\AssetImporters\track-mesh-exporter\bin\Release\net9.0\TrackMeshExporter.dll'
$inputRoot = [IO.Path]::GetFullPath((Join-Path $workspace $InputDirectory))
$outputRoot = [IO.Path]::GetFullPath((Join-Path $workspace $OutputDirectory))
$skydomeRoot = [IO.Path]::GetFullPath((Join-Path $workspace $SkydomeDirectory))
$workspacePrefix = [IO.Path]::GetFullPath($workspace).TrimEnd('\') + '\'
foreach ($root in @($outputRoot, $skydomeRoot)) {
    if (-not $root.StartsWith($workspacePrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Output directory must remain below the workspace.'
    }
}

New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
New-Item -ItemType Directory -Path $skydomeRoot -Force | Out-Null
Get-ChildItem -LiteralPath $inputRoot -Directory | Sort-Object Name | ForEach-Object {
    $trackFile = Join-Path $_.FullName 'track.1s'
    if (-not (Test-Path -LiteralPath $trackFile -PathType Leaf)) { return }
    $prefix = Join-Path $outputRoot $_.Name
    if ((Test-Path -LiteralPath "$prefix.ktrk") -or (Test-Path -LiteralPath "$prefix.mesh.json")) {
        Write-Warning "Skipping existing export: $prefix"
        return
    }
    & dotnet $tool export $trackFile $prefix
    if ($LASTEXITCODE -ne 0) { throw "Mesh export failed: $trackFile" }
}

# The skydome sits beside the track in its own .1s, and its root is the Relement
# scene rather than a TrackContainer, so it goes through the kart model path.
#
# Six of the thirteen do not read: the desert and forest domes carry class stamp
# 1f4b04fc, which KartLibrary has no type for. Those tracks keep an empty sky,
# so a failure here is reported and skipped rather than fatal.
Get-ChildItem -LiteralPath $inputRoot -Directory | Sort-Object Name | ForEach-Object {
    $skyFile = Join-Path $_.FullName 'skydome.1s'
    if (-not (Test-Path -LiteralPath $skyFile -PathType Leaf)) { return }
    $prefix = Join-Path $skydomeRoot ($_.Name + '.skydome')
    if ((Test-Path -LiteralPath "$prefix.ktrk") -or (Test-Path -LiteralPath "$prefix.mesh.json")) {
        Write-Warning "Skipping existing skydome export: $prefix"
        return
    }
    # The failing domes throw, and a .NET stack trace on stderr would otherwise
    # be promoted to a terminating error by $ErrorActionPreference.
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & dotnet $tool export-kart $skyFile $prefix 2>$null | Out-Null
    $failed = $LASTEXITCODE -ne 0
    $ErrorActionPreference = $previous
    if ($failed) {
        Write-Warning "Skydome export failed (unsupported class): $skyFile"
    }
}
