# Scripts

The native code is organized in Unity-style, responsibility-based folders:

- `Runtime/Physics`: vehicle dynamics, fixed-step simulation, and collision.
- `Runtime/Gameplay`: course progress, countdown, and recovered game data.
- `Runtime/Input`, `Runtime/Audio`, `Runtime/Rendering`, `Runtime/Assets`: focused runtime systems.
- `Platform/Windows` and `Platform/macOS`: presentation and OS integration.
- `Tests`: unit, differential, and asset integration tests.

Each runtime module keeps its public header beside its implementation. CMake exposes all
runtime module directories through the `kart_dynamics` target, so existing `#include
"kart_*.h"` directives remain stable.
