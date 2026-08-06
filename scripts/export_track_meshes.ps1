param(
    [string]$InputDirectory = 'analysis\track-assets\extracted',
    [string]$OutputDirectory = 'analysis\track-assets\meshes'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$tool = Join-Path $workspace 'asset_tools\track-mesh-exporter\bin\Release\net9.0\TrackMeshExporter.dll'
$inputRoot = [IO.Path]::GetFullPath((Join-Path $workspace $InputDirectory))
$outputRoot = [IO.Path]::GetFullPath((Join-Path $workspace $OutputDirectory))
$workspacePrefix = [IO.Path]::GetFullPath($workspace).TrimEnd('\') + '\'
if (-not $outputRoot.StartsWith($workspacePrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Output directory must remain below the workspace.'
}

New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
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
