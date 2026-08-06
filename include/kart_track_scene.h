#ifndef KART_TRACK_SCENE_H
#define KART_TRACK_SCENE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct KartTrackSceneVertex {
    float x;
    float y;
    float z;
    float u;
    float v;
} KartTrackSceneVertex;

typedef struct KartTrackSceneMesh {
    char name[97];
    char texture[97];
    uint32_t flags;
    uint32_t vertex_count;
    uint32_t index_count;
    /* Asset-space bounds, derived at load time rather than stored in the file.
       Collision queries reject whole meshes with these before touching any
       triangle, which is what makes querying every mesh affordable. */
    float minimum[3];
    float maximum[3];
    KartTrackSceneVertex *vertices;
    uint32_t *indices;
} KartTrackSceneMesh;

typedef struct KartTrackScene {
    uint32_t mesh_count;
    uint32_t total_vertex_count;
    uint32_t total_triangle_count;
    float minimum[3];
    float maximum[3];
    KartTrackSceneMesh *meshes;
} KartTrackScene;

bool kart_track_scene_load_memory(
    KartTrackScene *scene,
    const void *data,
    size_t size);

/* Loads a KTKZ container: the 4-byte magic "KTKZ", the uint32 little-endian
   size of the KTRK payload, then that payload as a raw DEFLATE stream.

   Embedding the scenes compressed keeps all 13 tracks in the executable at
   roughly 30% of their raw size. */
bool kart_track_scene_load_compressed(
    KartTrackScene *scene,
    const void *data,
    size_t size);

/* Recomputes every mesh's bounds from its vertices. Both loaders call this, so
   a scene that came from a file is ready to query. A scene assembled by hand
   must call it before any collision query, or its meshes will be rejected by
   their zeroed bounds. */
void kart_track_scene_compute_bounds(KartTrackScene *scene);

void kart_track_scene_free(KartTrackScene *scene);

#endif
