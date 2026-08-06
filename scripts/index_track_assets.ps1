param(
    [string]$DataDirectory = 'C:\Program Files (x86)\TCGAME\TCGameApps\kart\Data',
    [string]$OutputDirectory = 'analysis\track-assets'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$tool = Join-Path $workspace 'asset_tools\rho-safe-index\bin\Release\net9.0\RhoSafeIndex.dll'
$output = [IO.Path]::GetFullPath((Join-Path $workspace $OutputDirectory))

if (-not (Test-Path -LiteralPath $tool -PathType Leaf)) {
    throw "Build the safe indexer first: dotnet build asset_tools\rho-safe-index\RhoSafeIndex.csproj -c Release"
}
if (-not (Test-Path -LiteralPath $DataDirectory -PathType Container)) {
    throw "Data directory not found: $DataDirectory"
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
    $manifest = Join-Path $output "track_$trackCode.manifest.json"
    if (Test-Path -LiteralPath $archive -PathType Leaf) {
        & dotnet $tool list $archive $manifest
        if ($LASTEXITCODE -ne 0) { throw "Indexing failed: $archive" }
    }
    else {
        Write-Warning "Archive not present: $archive"
    }
}
