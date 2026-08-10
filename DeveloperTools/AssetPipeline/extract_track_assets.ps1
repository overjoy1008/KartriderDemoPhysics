param(
    [string]$DataDirectory = 'C:\Program Files (x86)\TCGAME\TCGameApps\kart\Data',
    [string]$OutputDirectory = 'Assets\Tracks\extracted'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$tool = Join-Path $workspace 'DeveloperTools\AssetImporters\rho-safe-index\bin\Release\net9.0\RhoSafeIndex.dll'
$output = [IO.Path]::GetFullPath((Join-Path $workspace $OutputDirectory))
$workspacePrefix = [IO.Path]::GetFullPath($workspace).TrimEnd('\') + '\'

if (-not $output.StartsWith($workspacePrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Output directory must remain below the workspace.'
}
if (-not (Test-Path -LiteralPath $tool -PathType Leaf)) {
    throw "Build the safe indexer first: dotnet build DeveloperTools\AssetImporters\rho-safe-index\RhoSafeIndex.csproj -c Release"
}

$trackCodes = @(
    'desert_I01', 'desert_I02', 'desert_R01',
    'forest_I01', 'forest_I02', 'forest_R02',
    'ice_I01', 'ice_I02', 'ice_R01',
    'village_I01', 'village_I02', 'village_R01', 'village_R03'
)

New-Item -ItemType Directory -Path $output -Force | Out-Null
foreach ($trackCode in $trackCodes) {
    $archive = Join-Path $DataDirectory "track_$trackCode.rho"
    if (-not (Test-Path -LiteralPath $archive -PathType Leaf)) {
        Write-Warning "Archive not present: $archive"
        continue
    }
    $trackOutput = Join-Path $output "track_$trackCode"
    $manifest = Join-Path (Split-Path -Parent $output) "track_$trackCode.extracted.json"
    if (Test-Path -LiteralPath $trackOutput) {
        throw "Refusing to overwrite an existing extraction directory: $trackOutput"
    }
    & dotnet $tool extract-selected $archive $trackOutput $manifest
    if ($LASTEXITCODE -ne 0) { throw "Extraction failed: $archive" }
}
