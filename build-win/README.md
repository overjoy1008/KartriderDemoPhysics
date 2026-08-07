# Verified Windows build

This directory contains the checked Windows simulator executable built from the
repository's current source with MinGW:

- `kart.exe`: both renderers in one binary, V switches between the software
  3D chase view and the top-down view

The remaining CMake cache, object, test, and comparison files in this local
directory are reproducible build artifacts and are intentionally excluded from
Git.

To rebuild and run the verification suite:

```powershell
cmake -S . -B .build-win -G "MinGW Makefiles"
cmake --build .build-win
ctest --test-dir .build-win --output-on-failure
```
