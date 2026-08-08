# Verified Windows build

This directory contains the checked Windows simulator executable built from the
repository's current source with MinGW:

- `kart.exe`: both renderers in one binary, V switches between the software
  3D chase view and the top-down view. It embeds the 13 track scenes and all
  26 kart models; M switches the kart between the recovered mesh and the box
  built from its physics constants, and B draws the model's bounding volumes.

The remaining CMake cache, object, test, and comparison files in this local
directory are reproducible build artifacts and are intentionally excluded from
Git.

To rebuild and run the verification suite:

```powershell
cmake -S . -B .build-win -G "MinGW Makefiles"
cmake --build .build-win
ctest --test-dir .build-win --output-on-failure
```
