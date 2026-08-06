# Verified Windows build

This directory contains the checked Windows simulator executables built from
the repository's current source with MinGW:

- `kart_topdown.exe`: top-down renderer
- `kart_3d.exe`: software-rendered 3D chase view

Both executables use the same recovered physics implementation. The remaining
CMake cache, object, test, and comparison files in this local directory are
reproducible build artifacts and are intentionally excluded from Git.

To rebuild and run the verification suite:

```powershell
cmake -S . -B .build-win -G "MinGW Makefiles"
cmake --build .build-win
ctest --test-dir .build-win --output-on-failure
```
