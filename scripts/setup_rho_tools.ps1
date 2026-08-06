$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$upstream = Join-Path $workspace 'analysis\tools\rho-reader-upstream'
$expectedCommit = '37fab6415360afa83e8034f8cedb2cc6766d2fe0'

if (-not (Test-Path -LiteralPath (Join-Path $upstream '.git') -PathType Container)) {
    New-Item -ItemType Directory -Path (Split-Path -Parent $upstream) -Force | Out-Null
    & git clone https://github.com/xpoi5010/Kartrider-File-Reader.git $upstream
    if ($LASTEXITCODE -ne 0) { throw 'Failed to clone Kartrider-File-Reader.' }
}

& git -c "safe.directory=$($upstream.Replace('\','/'))" -C $upstream checkout --detach $expectedCommit
if ($LASTEXITCODE -ne 0) { throw "Failed to select audited upstream commit $expectedCommit." }

$env:APPDATA = Join-Path $workspace 'analysis\ghidra-home\AppData\Roaming'
$env:NUGET_PACKAGES = Join-Path $workspace '.nuget-packages'
New-Item -ItemType Directory -Path $env:APPDATA -Force | Out-Null

$nugetConfig = Join-Path $workspace 'asset_tools\rho-safe-index\NuGet.Config'
& dotnet restore (Join-Path $workspace 'asset_tools\rho-safe-index\RhoSafeIndex.csproj') --configfile $nugetConfig
if ($LASTEXITCODE -ne 0) { throw 'RHO indexer restore failed.' }
& dotnet restore (Join-Path $workspace 'asset_tools\track-mesh-exporter\TrackMeshExporter.csproj') --configfile $nugetConfig
if ($LASTEXITCODE -ne 0) { throw 'Track exporter restore failed.' }

& dotnet build (Join-Path $workspace 'asset_tools\rho-safe-index\RhoSafeIndex.csproj') -c Release --no-restore
if ($LASTEXITCODE -ne 0) { throw 'RHO indexer build failed.' }
& dotnet build (Join-Path $workspace 'asset_tools\track-mesh-exporter\TrackMeshExporter.csproj') -c Release --no-restore
if ($LASTEXITCODE -ne 0) { throw 'Track exporter build failed.' }
