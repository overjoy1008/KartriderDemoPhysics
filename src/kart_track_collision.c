#include "kart_track_collision.h"

#include <math.h>

/* The two queries the original runs against its triangle set, reproduced here.
   See docs/ORIGINAL_COLLISION.md for the decompilation they come from.

   Ground is a per-wheel ray (0x00432fc0) and the body is an oriented box
   (0x00433310 -> 0x00434d40). They are separate mechanisms in the original and
   they are separate here.

   Only meshes the asset marks collidable are considered. The original builds
   its collision grid from the subtrees carrying a `property/road` block and
   from nothing else (0x00432390), so everything else is scenery it drives
   through. The exporter bakes that tag into mesh flag bit 0. */

/* _DAT_00571d48. The ray query skips any triangle whose normal is steeper than
   this, so walls are invisible to the wheels; the response uses the same
   number to tell a wall contact from a landing. */
#define KART_TRACK_ROAD_MIN_NORMAL_Z 0.6499999761581421f
/* 0x3f333333, the z half-extent 0x00430830 builds the body box with. Unlike the
   other two extents it is not per-kart. */
#define KART_BODY_HALF_HEIGHT 0.699999988079071f
/* Two normals this close are the same surface. The original has no such test —
   it simply iterates every overlapping triangle — but after the first contact
   the resolver's approach guard makes repeats of one normal no-ops, so folding
   them together changes nothing and stops a road from crowding a wall out of a
   fixed-size buffer. */
#define KART_TRACK_CONTACT_NORMAL_EPSILON 0.999f

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
    KartVec3 *forward,
    KartVec3 *up)
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
    *up = (KartVec3){
        2.0f * (xz + wy),
        2.0f * (yz - wx),
        1.0f - 2.0f * (xx + yy),
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

/* One triangle in world space, with the scene mirror compensated.

   kart_track_scene_world_vertex negates X for every scene. Mirroring a single
   axis reverses a triangle's winding, so a normal taken straight from the
   mirrored corners points the opposite way to the one the original computed at
   0x00432390. Swapping two corners puts the winding back.

   Both queries go through here because both now report the face normal as it
   is, rather than turning it toward the kart or toward the sky. Getting the
   sign wrong would flip every floor into a ceiling. */
static void read_triangle(
    const KartTrackSceneMesh *mesh,
    const KartDemoTrackSpec *track,
    uint32_t index,
    KartVec3 *a,
    KartVec3 *b,
    KartVec3 *c)
{
    const bool mirrored = kart_demo_track_mirror_x(track);
    *a = kart_track_scene_world_vertex(&mesh->vertices[mesh->indices[index]], track);
    *b = kart_track_scene_world_vertex(
        &mesh->vertices[mesh->indices[index + (mirrored ? 2u : 1u)]], track);
    *c = kart_track_scene_world_vertex(
        &mesh->vertices[mesh->indices[index + (mirrored ? 1u : 2u)]], track);
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
    if ((mesh->flags & KART_TRACK_SCENE_MESH_COLLIDABLE) == 0) return true;
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
            KartVec3 a;
            KartVec3 b;
            KartVec3 c;
            float fraction;
            KartVec3 normal;
            read_triangle(mesh, track, index, &a, &b, &c);
            /* 0x00432fc0 rejects on fabs(normal.z) and reports the face normal
               as it is. It does not turn a downward-facing face upward, so a
               face wound the wrong way pushes the suspension down rather than
               holding the kart up — that is the original's behaviour and the
               reason this is not "corrected" here. */
            if (segment_triangle_intersection(
                    ray_start, ray_delta, a, b, c, &fraction, &normal) &&
                fabsf(normal.z) >= KART_TRACK_ROAD_MIN_NORMAL_Z &&
                fraction < nearest) {
                nearest = fraction;
                nearest_normal = normal;
            }
        }
    }
    if (nearest > 1.0f) return false;
    hit->point = add(ray_start, scale(ray_delta, nearest));
    hit->normal = nearest_normal;
    hit->surface_id = 3;
    return true;
}

/* One separating-axis test against an axis that is the cross product of a box
   axis and a triangle edge. `p0`/`p1` are the two distinct projections of the
   triangle's vertices onto that axis and `radius` is the box's extent along it;
   the third vertex always projects onto one of the two, which is why the
   original only ever computes two. */
static bool axis_separates(float p0, float p1, float radius)
{
    const float low = p0 < p1 ? p0 : p1;
    const float high = p0 < p1 ? p1 : p0;
    return low > radius || high < -radius;
}

/* Does an axis-aligned box centred on the origin overlap the triangle?

   This is the test at 0x00434d40: the nine box-axis-cross-edge axes first, then
   the three box axes, then the triangle's own plane. It is Akenine-Moller's
   published routine, which is what the original implements.

   The box is axis-aligned because the caller has already rotated the triangle
   into the box's frame, which is how the original gets an oriented box out of
   an axis-aligned test. */
static bool box_triangle_overlap(
    KartVec3 half,
    KartVec3 v0,
    KartVec3 v1,
    KartVec3 v2)
{
    const KartVec3 e0 = subtract(v1, v0);
    const KartVec3 e1 = subtract(v2, v1);
    const KartVec3 e2 = subtract(v0, v2);
    const KartVec3 edges[3] = {e0, e1, e2};
    const KartVec3 corners[3] = {v0, v1, v2};
    /* Which two of the three corners each test compares. The third always
       projects onto one of them, and which two those are changes with the edge:
       0x00434d40 reads v0/v2 for the first two edges on the X and Y axes but
       v0/v1 for the third, and shifts the Z axis the other way. Using one fixed
       pair for all nine looks plausible and reports false separations. */
    static const int first[3][3] = {{0, 0, 1}, {0, 0, 0}, {0, 0, 1}};
    static const int second[3][3] = {{2, 2, 2}, {2, 2, 1}, {1, 1, 2}};
    KartVec3 normal;
    float plane_offset;
    float plane_radius;
    KartVec3 low;
    KartVec3 high;
    int i;

    /* Nine cross-product axes. Crossing a unit basis vector with an edge makes
       most terms vanish, which is why each test is two multiplies. */
    for (i = 0; i < 3; ++i) {
        const KartVec3 e = edges[i];
        const float ax = fabsf(e.x);
        const float ay = fabsf(e.y);
        const float az = fabsf(e.z);
        const KartVec3 px = corners[first[i][0]];
        const KartVec3 qx = corners[second[i][0]];
        const KartVec3 py = corners[first[i][1]];
        const KartVec3 qy = corners[second[i][1]];
        const KartVec3 pz = corners[first[i][2]];
        const KartVec3 qz = corners[second[i][2]];
        /* axis = X cross e */
        if (axis_separates(
                e.z * px.y - e.y * px.z,
                e.z * qx.y - e.y * qx.z,
                az * half.y + ay * half.z)) return false;
        /* axis = Y cross e */
        if (axis_separates(
                -e.z * py.x + e.x * py.z,
                -e.z * qy.x + e.x * qy.z,
                az * half.x + ax * half.z)) return false;
        /* axis = Z cross e */
        if (axis_separates(
                e.y * pz.x - e.x * pz.y,
                e.y * qz.x - e.x * qz.y,
                ay * half.x + ax * half.y)) return false;
    }

    /* The three box axes: a plain AABB overlap against the triangle's bounds. */
    low.x = fminf(v0.x, fminf(v1.x, v2.x));
    high.x = fmaxf(v0.x, fmaxf(v1.x, v2.x));
    low.y = fminf(v0.y, fminf(v1.y, v2.y));
    high.y = fmaxf(v0.y, fmaxf(v1.y, v2.y));
    low.z = fminf(v0.z, fminf(v1.z, v2.z));
    high.z = fmaxf(v0.z, fmaxf(v1.z, v2.z));
    if (low.x > half.x || high.x < -half.x) return false;
    if (low.y > half.y || high.y < -half.y) return false;
    if (low.z > half.z || high.z < -half.z) return false;

    /* The triangle's plane against the box. */
    normal = cross(e0, e1);
    plane_offset = -dot(normal, v0);
    plane_radius = fabsf(normal.x) * half.x +
                   fabsf(normal.y) * half.y +
                   fabsf(normal.z) * half.z;
    return fabsf(plane_offset) <= plane_radius;
}

static bool contact_is_duplicate(
    const KartBodyContact *contacts,
    unsigned int count,
    KartVec3 normal)
{
    unsigned int i;
    for (i = 0; i < count; ++i) {
        if (dot(contacts[i].normal, normal) > KART_TRACK_CONTACT_NORMAL_EPSILON) {
            return true;
        }
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
    KartVec3 body_up;
    KartVec3 centre;
    KartVec3 half;
    KartVec3 query_min;
    KartVec3 query_max;
    float reach;
    uint32_t mesh_index;
    unsigned int count = 0;
    if (scene == NULL || track == NULL || state == NULL ||
        contacts == NULL || capacity == 0) return 0;
    orientation_axes(state->orientation, &body_right, &body_forward, &body_up);

    /* 0x00430830 builds the box from the kart's two plan-view half-extents and
       a fixed half-height, centred one unit up the chassis axis from the
       origin the wheels hang off. */
    half = (KartVec3){
        state->geometry.half_width,
        state->geometry.half_length,
        KART_BODY_HALF_HEIGHT,
    };
    centre = add(state->position, body_up);

    reach = sqrtf(dot(half, half));
    query_min = (KartVec3){centre.x - reach, centre.y - reach, centre.z - reach};
    query_max = (KartVec3){centre.x + reach, centre.y + reach, centre.z + reach};

    for (mesh_index = 0; mesh_index < scene->mesh_count && count < capacity; ++mesh_index) {
        const KartTrackSceneMesh *mesh = &scene->meshes[mesh_index];
        uint32_t index;
        if (mesh_can_be_skipped(mesh, track, query_min, query_max)) continue;
        for (index = 0; index + 2u < mesh->index_count && count < capacity; index += 3u) {
            KartVec3 a;
            KartVec3 b;
            KartVec3 c;
            KartVec3 offset_a;
            KartVec3 offset_b;
            KartVec3 offset_c;
            /* Into the box's frame: the rows of the rotation are the body axes. */
            KartVec3 local_a;
            KartVec3 local_b;
            KartVec3 local_c;
            KartVec3 normal;
            read_triangle(mesh, track, index, &a, &b, &c);
            offset_a = subtract(a, centre);
            offset_b = subtract(b, centre);
            offset_c = subtract(c, centre);
            local_a = (KartVec3){
                dot(offset_a, body_right),
                dot(offset_a, body_forward),
                dot(offset_a, body_up)};
            local_b = (KartVec3){
                dot(offset_b, body_right),
                dot(offset_b, body_forward),
                dot(offset_b, body_up)};
            local_c = (KartVec3){
                dot(offset_c, body_right),
                dot(offset_c, body_forward),
                dot(offset_c, body_up)};
            if (!box_triangle_overlap(half, local_a, local_b, local_c)) continue;
            /* The true face normal, with no flattening and no steepness filter:
               a shallow face reaches the resolver's landing branch and a steep
               one reaches its wall branch, which is the split the original
               makes. */
            normal = normalize(cross(subtract(b, a), subtract(c, a)));
            if (dot(normal, normal) == 0.0f ||
                contact_is_duplicate(contacts, count, normal)) continue;
            contacts[count].normal = normal;
            contacts[count].point = scale(add(add(a, b), c), 1.0f / 3.0f);
            contacts[count].sweep_fraction = 0.5f;
            contacts[count].surface_id = 4;
            ++count;
        }
    }
    return count;
}
