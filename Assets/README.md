# Assets

Runtime and source assets are grouped by purpose:

- `Audio/`: music and sound effects extracted from the original archives.
- `Models/Karts/`: kart source data, exported meshes, and packed runtime models.
- `Tracks/`: track source data, minimaps, textures, exported scenes, and packed runtime data.

The large generated/extracted payloads remain ignored by Git. Build scripts and CMake use
these paths directly, so a local asset workspace can be moved without changing behavior.
