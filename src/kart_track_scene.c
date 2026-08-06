#include "kart_track_scene.h"

#include "kart_inflate.h"

#include <stdlib.h>
#include <string.h>

/* Matches the loader's own limits, so a corrupt header cannot ask for a huge
   allocation before kart_track_scene_load_memory gets to validate it. */
#define KART_TRACK_SCENE_MAX_BYTES (256u * 1024u * 1024u)

typedef struct SceneReader {
    const unsigned char *cursor;
    size_t remaining;
} SceneReader;

static bool read_bytes(SceneReader *reader, void *destination, size_t size)
{
    if (size > reader->remaining) {
        return false;
    }
    memcpy(destination, reader->cursor, size);
    reader->cursor += size;
    reader->remaining -= size;
    return true;
}

static bool read_u32(SceneReader *reader, uint32_t *value)
{
    unsigned char bytes[4];
    if (!read_bytes(reader, bytes, sizeof(bytes))) {
        return false;
    }
    *value = (uint32_t)bytes[0] |
             ((uint32_t)bytes[1] << 8) |
             ((uint32_t)bytes[2] << 16) |
             ((uint32_t)bytes[3] << 24);
    return true;
}

static bool read_float(SceneReader *reader, float *value)
{
    uint32_t bits;
    if (!read_u32(reader, &bits)) {
        return false;
    }
    memcpy(value, &bits, sizeof(bits));
    return true;
}

void kart_track_scene_free(KartTrackScene *scene)
{
    uint32_t mesh_index;
    if (scene == NULL) {
        return;
    }
    if (scene->meshes != NULL) {
        for (mesh_index = 0; mesh_index < scene->mesh_count; ++mesh_index) {
            free(scene->meshes[mesh_index].vertices);
            free(scene->meshes[mesh_index].indices);
        }
    }
    free(scene->meshes);
    memset(scene, 0, sizeof(*scene));
}

bool kart_track_scene_load_memory(
    KartTrackScene *scene,
    const void *data,
    size_t size)
{
    SceneReader reader;
    char magic[4];
    uint32_t version;
    uint32_t mesh_index;
    uint32_t coordinate;

    if (scene == NULL || data == NULL) {
        return false;
    }
    memset(scene, 0, sizeof(*scene));
    reader.cursor = (const unsigned char *)data;
    reader.remaining = size;
    if (!read_bytes(&reader, magic, sizeof(magic)) ||
        memcmp(magic, "KTRK", sizeof(magic)) != 0 ||
        !read_u32(&reader, &version) || version != 1 ||
        !read_u32(&reader, &scene->mesh_count) ||
        !read_u32(&reader, &scene->total_vertex_count) ||
        !read_u32(&reader, &scene->total_triangle_count) ||
        scene->mesh_count > 100000 ||
        scene->total_vertex_count > 10000000 ||
        scene->total_triangle_count > 10000000) {
        goto fail;
    }
    for (coordinate = 0; coordinate < 3; ++coordinate) {
        if (!read_float(&reader, &scene->minimum[coordinate])) {
            goto fail;
        }
    }
    for (coordinate = 0; coordinate < 3; ++coordinate) {
        if (!read_float(&reader, &scene->maximum[coordinate])) {
            goto fail;
        }
    }
    if (scene->mesh_count != 0) {
        scene->meshes = (KartTrackSceneMesh *)calloc(
            scene->mesh_count, sizeof(*scene->meshes));
        if (scene->meshes == NULL) {
            goto fail;
        }
    }
    for (mesh_index = 0; mesh_index < scene->mesh_count; ++mesh_index) {
        KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        uint32_t vertex_index;
        size_t vertex_bytes;
        size_t index_bytes;
        if (!read_bytes(&reader, mesh->name, 96) ||
            !read_bytes(&reader, mesh->texture, 96) ||
            !read_u32(&reader, &mesh->flags) ||
            !read_u32(&reader, &mesh->vertex_count) ||
            !read_u32(&reader, &mesh->index_count) ||
            mesh->vertex_count > scene->total_vertex_count ||
            mesh->index_count > scene->total_triangle_count * 3u ||
            mesh->index_count % 3u != 0u) {
            goto fail;
        }
        mesh->name[96] = '\0';
        mesh->texture[96] = '\0';
        if (mesh->vertex_count > SIZE_MAX / sizeof(*mesh->vertices) ||
            mesh->index_count > SIZE_MAX / sizeof(*mesh->indices)) {
            goto fail;
        }
        vertex_bytes = (size_t)mesh->vertex_count * sizeof(*mesh->vertices);
        index_bytes = (size_t)mesh->index_count * sizeof(*mesh->indices);
        mesh->vertices = (KartTrackSceneVertex *)malloc(vertex_bytes);
        mesh->indices = (uint32_t *)malloc(index_bytes);
        if ((vertex_bytes != 0 && mesh->vertices == NULL) ||
            (index_bytes != 0 && mesh->indices == NULL)) {
            goto fail;
        }
        for (vertex_index = 0; vertex_index < mesh->vertex_count; ++vertex_index) {
            KartTrackSceneVertex *vertex = &mesh->vertices[vertex_index];
            if (!read_float(&reader, &vertex->x) ||
                !read_float(&reader, &vertex->y) ||
                !read_float(&reader, &vertex->z) ||
                !read_float(&reader, &vertex->u) ||
                !read_float(&reader, &vertex->v)) {
                goto fail;
            }
        }
        for (vertex_index = 0; vertex_index < mesh->index_count; ++vertex_index) {
            if (!read_u32(&reader, &mesh->indices[vertex_index]) ||
                mesh->indices[vertex_index] >= mesh->vertex_count) {
                goto fail;
            }
        }
    }
    kart_track_scene_compute_bounds(scene);
    return true;

fail:
    kart_track_scene_free(scene);
    return false;
}

void kart_track_scene_compute_bounds(KartTrackScene *scene)
{
    uint32_t mesh_index;
    if (scene == NULL) {
        return;
    }
    for (mesh_index = 0; mesh_index < scene->mesh_count; ++mesh_index) {
        KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        uint32_t vertex_index;
        int coordinate;
        for (coordinate = 0; coordinate < 3; ++coordinate) {
            /* Inverted, so a mesh with no vertices intersects nothing. */
            mesh->minimum[coordinate] = 1.0e30f;
            mesh->maximum[coordinate] = -1.0e30f;
        }
        for (vertex_index = 0; vertex_index < mesh->vertex_count; ++vertex_index) {
            const float *position = &mesh->vertices[vertex_index].x;
            for (coordinate = 0; coordinate < 3; ++coordinate) {
                if (position[coordinate] < mesh->minimum[coordinate]) {
                    mesh->minimum[coordinate] = position[coordinate];
                }
                if (position[coordinate] > mesh->maximum[coordinate]) {
                    mesh->maximum[coordinate] = position[coordinate];
                }
            }
        }
    }
}

bool kart_track_scene_load_compressed(
    KartTrackScene *scene,
    const void *data,
    size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    uint32_t payload_size;
    unsigned char *payload;
    size_t produced;
    bool loaded;

    if (scene == NULL || data == NULL || size < 8u) return false;
    if (memcmp(bytes, "KTKZ", 4) != 0) return false;
    payload_size = (uint32_t)bytes[4] |
                   ((uint32_t)bytes[5] << 8) |
                   ((uint32_t)bytes[6] << 16) |
                   ((uint32_t)bytes[7] << 24);
    if (payload_size == 0u || payload_size > KART_TRACK_SCENE_MAX_BYTES) {
        return false;
    }
    payload = (unsigned char *)malloc(payload_size);
    if (payload == NULL) return false;
    produced = kart_inflate_raw(payload, payload_size, bytes + 8, size - 8u);
    /* The recorded size is part of the container, so a short or long stream is
       a corrupt resource rather than something to load partially. */
    loaded = produced == (size_t)payload_size &&
             kart_track_scene_load_memory(scene, payload, produced);
    free(payload);
    return loaded;
}
