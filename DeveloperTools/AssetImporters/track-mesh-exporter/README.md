# Track mesh exporter

Decodes an extracted KartRider `track.1s` and writes:

- `.ktrk`: a little-endian, C-friendly mesh container (`KTRK`, version 1)
- `.mesh.json`: bounds, mesh counts, texture references, and conservative
  road/wall collision candidates inferred only from original node names

The exporter does not claim that every node containing `road` or `wall` is an
authoritative collision primitive. Those flags are kept as candidates until
the original runtime's collision-node selection is recovered.
