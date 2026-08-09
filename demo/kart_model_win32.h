#ifndef KART_MODEL_WIN32_H
#define KART_MODEL_WIN32_H

/* Structure of a kart's model.1s, worked out once at load time.

   The exported KTRK carries the submeshes in the order the original file did
   and nothing else: no submesh has a name or a texture string, so the parts
   have to be told apart by position and size. Every one of the 26 karts has the
   same six-submesh layout, which makes the rule short and exact:

     index 0        the body, and the only submesh that varies between karts
     four submeshes sharing a vertex count      the wheels
     whatever is left (one submesh, 9v/8t)      the steering wheel

   The three bounding boxes this yields are genuinely different volumes and the
   demo draws all three, because only the middle one drives the physics:

     full    every vertex, which is what the KTRK header records
     body    submesh 0, the source of half_width/half_length/model_height
     wheels  the four wheel submeshes, which on most karts are wider than
             the body and so are the reason `full` does not reproduce the
             physics constants

   See docs/KART_MODEL_CATALOG.md for the per-kart numbers. */

#include "kart_track_scene.h"

#include <stdbool.h>
#include <stdint.h>

#define KART_MODEL_WHEEL_COUNT 4

typedef struct KartModelBox {
    bool valid;
    float minimum[3];
    float maximum[3];
} KartModelBox;

typedef struct KartModelParts {
    bool valid;
    uint32_t body_index;
    uint32_t wheel_indices[KART_MODEL_WHEEL_COUNT];
    uint32_t wheel_count;
    KartModelBox full;
    KartModelBox body;
    KartModelBox wheels;
} KartModelParts;

static void kart_model_box_reset(KartModelBox *box)
{
    box->valid = false;
    box->minimum[0] = box->minimum[1] = box->minimum[2] = 0.0f;
    box->maximum[0] = box->maximum[1] = box->maximum[2] = 0.0f;
}

static void kart_model_box_add(KartModelBox *box, const KartTrackSceneMesh *mesh)
{
    int axis;
    for (axis = 0; axis < 3; ++axis) {
        if (!box->valid || mesh->minimum[axis] < box->minimum[axis]) {
            box->minimum[axis] = mesh->minimum[axis];
        }
        if (!box->valid || mesh->maximum[axis] > box->maximum[axis]) {
            box->maximum[axis] = mesh->maximum[axis];
        }
    }
    box->valid = true;
}

/* True for the submeshes that make up the wheels: the largest group past the
   body whose members share a vertex count, when that group has exactly four
   members. Falls back to no wheels rather than guessing, which costs only the
   wheel box - the model still draws in full. */
static void kart_model_find_wheels(
    const KartTrackScene *scene,
    KartModelParts *parts)
{
    uint32_t candidate;
    parts->wheel_count = 0;
    for (candidate = 1; candidate < scene->mesh_count; ++candidate) {
        const uint32_t vertex_count = scene->meshes[candidate].vertex_count;
        uint32_t matches[KART_MODEL_WHEEL_COUNT];
        uint32_t match_count = 0;
        uint32_t other;
        for (other = 1; other < scene->mesh_count; ++other) {
            if (scene->meshes[other].vertex_count != vertex_count) continue;
            if (match_count == KART_MODEL_WHEEL_COUNT) {
                match_count = KART_MODEL_WHEEL_COUNT + 1u;
                break;
            }
            matches[match_count++] = other;
        }
        if (match_count != KART_MODEL_WHEEL_COUNT) continue;
        /* Ties are impossible in the demo's models, but preferring the larger
           group keeps the rule deterministic if a future asset has one. */
        if (parts->wheel_count == KART_MODEL_WHEEL_COUNT &&
            vertex_count <= scene->meshes[parts->wheel_indices[0]].vertex_count) {
            continue;
        }
        for (other = 0; other < KART_MODEL_WHEEL_COUNT; ++other) {
            parts->wheel_indices[other] = matches[other];
        }
        parts->wheel_count = KART_MODEL_WHEEL_COUNT;
    }
}

static void kart_model_parts_build(
    const KartTrackScene *scene,
    KartModelParts *parts)
{
    uint32_t i;
    parts->valid = false;
    parts->body_index = 0;
    parts->wheel_count = 0;
    kart_model_box_reset(&parts->full);
    kart_model_box_reset(&parts->body);
    kart_model_box_reset(&parts->wheels);
    if (scene == NULL || scene->mesh_count == 0) {
        return;
    }

    for (i = 0; i < scene->mesh_count; ++i) {
        kart_model_box_add(&parts->full, &scene->meshes[i]);
    }
    kart_model_box_add(&parts->body, &scene->meshes[0]);

    kart_model_find_wheels(scene, parts);
    for (i = 0; i < parts->wheel_count; ++i) {
        kart_model_box_add(&parts->wheels, &scene->meshes[parts->wheel_indices[i]]);
    }
    parts->valid = true;
}

static bool kart_model_is_wheel(const KartModelParts *parts, uint32_t mesh_index)
{
    uint32_t i;
    for (i = 0; i < parts->wheel_count; ++i) {
        if (parts->wheel_indices[i] == mesh_index) return true;
    }
    return false;
}

#endif
