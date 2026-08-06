#include "kart_track_collision.h"

#include <math.h>

/* Every mesh in the scene is solid, not just the road/wall named candidates:
   scenery meshes are the rocks, ledges and tree trunks the kart would otherwise
   drive through. What a face is used for comes from its normal, not its name.

   Whole meshes are rejected by their bounds first, so including scenery costs
   far less than the triangle counts suggest. */
#define KART_TRACK_ROAD_MIN_NORMAL_Z 0.20f
#define KART_TRACK_WALL_MAX_NORMAL_Z 0.55f
#define KART_TRACK_CONTACT_MARGIN 0.05f

static KartVec3 add(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static KartVec3 subtract(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static KartVec3 scale(KartVec3 value, float amount)
{
    return (KartVec3){value.x * amount, value.y * amount, value.z * amount};
}

static float dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static KartVec3 cross(KartVec3 a, KartVec3 b)
{
    return (KartVec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

static KartVec3 normalize(KartVec3 value)
{
    const float length = sqrtf(dot(value, value));
    return length > 1.0e-8f ? scale(value, 1.0f / length) : (KartVec3){0};
}

static void orientation_axes(
    KartQuat q,
    KartVec3 *right,
    KartVec3 *forward)
{
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;
    *right = (KartVec3){
        1.0f - 2.0f * (yy + zz),
        2.0f * (xy + wz),
        2.0f * (xz - wy),
    };
    *forward = (KartVec3){
        -2.0f * (xy - wz),
        -(1.0f - 2.0f * (xx + zz)),
        -2.0f * (yz + wx),
    };
}

KartVec3 kart_track_scene_world_vertex(
    const KartTrackSceneVertex *vertex,
    const KartDemoTrackSpec *track)
{
    const float center_x = (track->minimum.x + track->maximum.x) * 0.5f;
    const float center_y = (track->minimum.y + track->maximum.y) * 0.5f;
    return (KartVec3){
        kart_demo_track_mirror_x(track)
            ? center_x - vertex->x
            : vertex->x - center_x,
        vertex->y - center_y,
        vertex->z - kart_demo_track_scene_ground_z(track),
    };
}

/* The mesh's asset-space bounds in world space. Y and Z keep their ordering;
   the scene's X mirror swaps that pair of corners. */
static void mesh_world_bounds(
    const KartTrackSceneMesh *mesh,
    const KartDemoTrackSpec *track,
    KartVec3 *minimum,
    KartVec3 *maximum)
{
    const KartTrackSceneVertex low = {
        mesh->minimum[0], mesh->minimum[1], mesh->minimum[2], 0.0f, 0.0f};
    const KartTrackSceneVertex high = {
        mesh->maximum[0], mesh->maximum[1], mesh->maximum[2], 0.0f, 0.0f};
    const KartVec3 a = kart_track_scene_world_vertex(&low, track);
    const KartVec3 b = kart_track_scene_world_vertex(&high, track);
    minimum->x = a.x < b.x ? a.x : b.x;
    maximum->x = a.x < b.x ? b.x : a.x;
    minimum->y = a.y;
    maximum->y = b.y;
    minimum->z = a.z;
    maximum->z = b.z;
}

static bool boxes_overlap(
    KartVec3 a_min,
    KartVec3 a_max,
    KartVec3 b_min,
    KartVec3 b_max)
{
    return a_min.x <= b_max.x && a_max.x >= b_min.x &&
           a_min.y <= b_max.y && a_max.y >= b_min.y &&
           a_min.z <= b_max.z && a_max.z >= b_min.z;
}

static bool mesh_can_be_skipped(
    const KartTrackSceneMesh *mesh,
    const KartDemoTrackSpec *track,
    KartVec3 query_min,
    KartVec3 query_max)
{
    KartVec3 mesh_min;
    KartVec3 mesh_max;
    if (mesh->vertex_count == 0 || mesh->index_count < 3) return true;
    mesh_world_bounds(mesh, track, &mesh_min, &mesh_max);
    return !boxes_overlap(mesh_min, mesh_max, query_min, query_max);
}

static bool segment_triangle_intersection(
    KartVec3 start,
    KartVec3 delta,
    KartVec3 a,
    KartVec3 b,
    KartVec3 c,
    float *fraction,
    KartVec3 *normal)
{
    const float epsilon = 1.0e-6f;
    const KartVec3 edge_ab = subtract(b, a);
    const KartVec3 edge_ac = subtract(c, a);
    const KartVec3 p = cross(delta, edge_ac);
    const float determinant = dot(edge_ab, p);
    KartVec3 from_a;
    KartVec3 q;
    float inverse;
    float u;
    float v;
    float t;
    if (fabsf(determinant) < epsilon) return false;
    inverse = 1.0f / determinant;
    from_a = subtract(start, a);
    u = dot(from_a, p) * inverse;
    if (u < -epsilon || u > 1.0f + epsilon) return false;
    q = cross(from_a, edge_ab);
    v = dot(delta, q) * inverse;
    if (v < -epsilon || u + v > 1.0f + epsilon) return false;
    t = dot(edge_ac, q) * inverse;
    if (t < -epsilon || t > 1.0f + epsilon) return false;
    *fraction = fmaxf(0.0f, fminf(t, 1.0f));
    *normal = normalize(cross(edge_ab, edge_ac));
    return dot(*normal, *normal) > 0.0f;
}

bool kart_track_scene_query_ground(
    const KartTrackScene *scene,
    const KartDemoTrackSpec *track,
    KartVec3 ray_start,
    KartVec3 ray_delta,
    KartGroundHit *hit)
{
    float nearest = 2.0f;
    KartVec3 nearest_normal = {0};
    const KartVec3 ray_end = add(ray_start, ray_delta);
    KartVec3 query_min;
    KartVec3 query_max;
    uint32_t mesh_index;
    if (scene == NULL || track == NULL || hit == NULL) return false;
    query_min = (KartVec3){
        fminf(ray_start.x, ray_end.x),
        fminf(ray_start.y, ray_end.y),
        fminf(ray_start.z, ray_end.z),
    };
    query_max = (KartVec3){
        fmaxf(ray_start.x, ray_end.x),
        fmaxf(ray_start.y, ray_end.y),
        fmaxf(ray_start.z, ray_end.z),
    };
    for (mesh_index = 0; mesh_index < scene->mesh_count; ++mesh_index) {
        const KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        uint32_t index;
        if (mesh_can_be_skipped(mesh, track, query_min, query_max)) continue;
        for (index = 0; index + 2u < mesh->index_count; index += 3u) {
            const KartVec3 a = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[index]], track);
            const KartVec3 b = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[index + 1u]], track);
            const KartVec3 c = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[index + 2u]], track);
            float fraction;
            KartVec3 normal;
            if (segment_triangle_intersection(
                    ray_start, ray_delta, a, b, c, &fraction, &normal) &&
                fabsf(normal.z) >= KART_TRACK_ROAD_MIN_NORMAL_Z &&
                fraction < nearest) {
                nearest = fraction;
                nearest_normal = normal.z < 0.0f ? scale(normal, -1.0f) : normal;
            }
        }
    }
    if (nearest > 1.0f) return false;
    hit->point = add(ray_start, scale(ray_delta, nearest));
    hit->normal = nearest_normal;
    hit->surface_id = 3;
    return true;
}

static bool point_in_triangle(
    KartVec3 point,
    KartVec3 a,
    KartVec3 b,
    KartVec3 c)
{
    const float epsilon = -1.0e-4f;
    const KartVec3 v0 = subtract(b, a);
    const KartVec3 v1 = subtract(c, a);
    const KartVec3 v2 = subtract(point, a);
    const float d00 = dot(v0, v0);
    const float d01 = dot(v0, v1);
    const float d11 = dot(v1, v1);
    const float d20 = dot(v2, v0);
    const float d21 = dot(v2, v1);
    const float denominator = d00 * d11 - d01 * d01;
    float v;
    float w;
    if (fabsf(denominator) < 1.0e-8f) return false;
    v = (d11 * d20 - d01 * d21) / denominator;
    w = (d00 * d21 - d01 * d20) / denominator;
    return v >= epsilon && w >= epsilon && v + w <= 1.0f - epsilon;
}

static bool contact_is_duplicate(
    const KartBodyContact *contacts,
    unsigned int count,
    KartVec3 normal)
{
    unsigned int i;
    for (i = 0; i < count; ++i) {
        if (dot(contacts[i].normal, normal) > 0.95f) return true;
    }
    return false;
}

unsigned int kart_track_scene_query_body_collisions(
    const KartTrackScene *scene,
    const KartDemoTrackSpec *track,
    const KartSimulationState *state,
    KartBodyContact *contacts,
    unsigned int capacity)
{
    KartVec3 body_right;
    KartVec3 body_forward;
    KartVec3 query_min;
    KartVec3 query_max;
    float reach;
    uint32_t mesh_index;
    unsigned int count = 0;
    if (scene == NULL || track == NULL || state == NULL ||
        contacts == NULL || capacity == 0) return 0;
    orientation_axes(state->orientation, &body_right, &body_forward);
    /* The widest the body can reach in any direction, plus the contact margin. */
    reach = state->geometry.half_width + state->geometry.half_length +
            KART_TRACK_CONTACT_MARGIN;
    query_min = (KartVec3){
        state->position.x - reach,
        state->position.y - reach,
        state->position.z - reach,
    };
    query_max = (KartVec3){
        state->position.x + reach,
        state->position.y + reach,
        state->position.z + reach,
    };
    for (mesh_index = 0; mesh_index < scene->mesh_count && count < capacity; ++mesh_index) {
        const KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        uint32_t index;
        if (mesh_can_be_skipped(mesh, track, query_min, query_max)) continue;
        for (index = 0; index + 2u < mesh->index_count && count < capacity; index += 3u) {
            const KartVec3 a = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[index]], track);
            const KartVec3 b = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[index + 1u]], track);
            const KartVec3 c = kart_track_scene_world_vertex(
                &mesh->vertices[mesh->indices[index + 2u]], track);
            KartVec3 normal = normalize(cross(subtract(b, a), subtract(c, a)));
            float distance;
            float support;
            KartVec3 projected;
            if (dot(normal, normal) == 0.0f ||
                fabsf(normal.z) > KART_TRACK_WALL_MAX_NORMAL_Z) continue;
            normal.z = 0.0f;
            normal = normalize(normal);
            distance = dot(subtract(state->position, a), normal);
            if (distance < 0.0f) normal = scale(normal, -1.0f);
            distance = fabsf(distance);
            support = fabsf(dot(normal, body_right)) * state->geometry.half_width +
                      fabsf(dot(normal, body_forward)) * state->geometry.half_length;
            if (distance > support + KART_TRACK_CONTACT_MARGIN) continue;
            projected = add(state->position, scale(normal, -distance));
            if (!point_in_triangle(projected, a, b, c) ||
                contact_is_duplicate(contacts, count, normal)) continue;
            contacts[count].normal = normal;
            contacts[count].sweep_fraction = 0.5f;
            contacts[count].surface_id = 4;
            ++count;
        }
    }
    return count;
}
