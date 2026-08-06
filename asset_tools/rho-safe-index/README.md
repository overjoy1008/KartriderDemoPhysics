# Safe RHO indexer

This CLI opens a KartRider `.rho` archive read-only through the GPL-3.0
[`Kartrider-File-Reader`](https://github.com/xpoi5010/Kartrider-File-Reader)
library and writes only a JSON file list. It records the source SHA-256 before
and after indexing and aborts if the archive changed.

The upstream checkout is intentionally kept under
`analysis/tools/rho-reader-upstream` and is not part of this repository.

```powershell
dotnet run --project asset_tools/rho-safe-index -- `
  list "C:\path\to\track_forest_I01.rho" `
  "analysis\track-assets\track_forest_I01.manifest.json"
```

The `list` command has no extraction or archive-writing behavior.

After reviewing a JSON list, selected map/model/image entries can be copied to
an empty directory below the current workspace. The command rejects path
traversal, files over 256 MiB, existing output files, and output paths outside
the workspace. It verifies the source archive hash again afterward.

```powershell
dotnet run --project asset_tools/rho-safe-index -- `
  extract-selected "C:\path\to\track_forest_I01.rho" `
  "analysis\track-assets\extracted\track_forest_I01" `
  "analysis\track-assets\track_forest_I01.extracted.json"
```
